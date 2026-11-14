"""
apps/MicroDuck/src/sim_engine.py

50 Hz closed-loop MicroDuck Digital Twin engine implementing:
- Safety layer (NaN rejection, actuator clamping, deadman velocity zeroing, fall debounce)
- ROBOTIS Impact Guard V3/V4 physics simulation (Level 0, Level 5 collapse bug, Setting R)
- Skills & Emotions playback, Pickup detection, Laser/Soccer tracking, and ToF Theremin
"""

from dataclasses import dataclass, field
import math
from typing import Dict, List, Optional, Sequence

from kinematics_obs import (
    BodyPoseCommand,
    Command,
    build_observation,
    build_pickup_features,
    compute_pause_target,
    scatter_action,
)
from models import (
    ACTUATOR_MAX_RAD,
    ACTUATOR_MIN_RAD,
    CONTROL_DT,
    DEADMAN_TIMEOUT_S,
    DEFAULT_POSITION,
    FALL_DEBOUNCE_S,
    FALL_GRAVITY_Z_THRESHOLD,
    GAIN_LIMP,
    GAIN_RUNNING,
    IMPACT_GUARD_PROFILES,
    MOUTH_INDEX,
    NUM_JOINTS,
    mouth_target,
)
from prehardware_subsystems import (
    ContactOdometry,
    LogMelPettingDetector,
    PadImuFilter,
    StopAndVoteMapper,
    Xl330BusSimulator,
)


@dataclass
class SafetyStatus:
    fallen: bool = False
    picked_up: bool = False
    deadman_active: bool = False
    clamped_joints: int = 0
    rejected_nonfinite: bool = False
    gain: int = GAIN_RUNNING
    impact_guard_trips: int = 0


@dataclass
class MicroDuckSimState:
    time_s: float = 0.0
    mode: str = "walk"  # walk | roller
    autonomous_mode: str = "manual"  # manual | autod | laser | soccer | chorale | theremin
    active_skill: Optional[str] = None
    impact_guard_level: str = "R"
    joints: List[float] = field(default_factory=lambda: list(DEFAULT_POSITION))
    velocities: List[float] = field(default_factory=lambda: [0.0] * NUM_JOINTS)
    currents_ma: List[float] = field(default_factory=lambda: [120.0] * NUM_JOINTS)
    prev_action14: List[float] = field(default_factory=lambda: [0.0] * 14)
    gyro: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    gravity: List[float] = field(default_factory=lambda: [0.0, 0.0, -1.0])
    position_xy: List[float] = field(default_factory=lambda: [0.0, 0.0])
    yaw_rad: float = 0.0
    last_intent_time_s: float = 0.0
    fall_accum_s: float = 0.0
    pickup_elapsed_s: float = 0.0
    battery_pct: float = 92.0
    safety: SafetyStatus = field(default_factory=SafetyStatus)


class MicroDuckSimulator:
    """Deterministic 50 Hz MicroDuck physics, safety, and policy step engine."""

    def __init__(self, impact_guard_level: str = "R") -> None:
        if impact_guard_level not in IMPACT_GUARD_PROFILES:
            raise ValueError(f"Unknown impact guard level: {impact_guard_level}")
        self.state = MicroDuckSimState(impact_guard_level=impact_guard_level)
        self._pause_start_pose: Optional[List[float]] = None
        # Pre-hardware ground-truth subsystems (`pollen-robotics/microduck`)
        self.odom_alpha16 = ContactOdometry(anchor_set="alpha16", quantize_encoders=True)
        self.odom_v15 = ContactOdometry(anchor_set="v15", quantize_encoders=True)
        self.bus = Xl330BusSimulator()
        self.mapper = StopAndVoteMapper()
        self.pet_detector = LogMelPettingDetector()
        self.pad_imu = PadImuFilter()

    def set_impact_guard(self, level_id: str) -> None:
        if level_id not in IMPACT_GUARD_PROFILES:
            raise ValueError(f"Unknown impact guard level: {level_id}")
        self.state.impact_guard_level = level_id

    def apply_push_impulse(self, impulse_rad_s: float = 7.5) -> Dict[str, object]:
        """
        Simulate an external lateral/knee push impulse under the active Impact Guard profile.
        Models the hardware test findings in `docs/robot/cheatsheet.md`:
          - Level 5 (v_min=1, brake=50ms, i_hold=500mA) falsely trips stance knees -> fall
          - Setting R (v_min=6, brake=20ms, i_hold=off, v_fast=7, pwm=40%) absorbs push cleanly
        """
        if not math.isfinite(impulse_rad_s) or impulse_rad_s < 0.0:
            raise ValueError(f"Push impulse must be finite and >= 0, got {impulse_rad_s}")
        profile = IMPACT_GUARD_PROFILES[self.state.impact_guard_level]
        st = self.state
        if st.safety.fallen:
            return {"survived": False, "trips": 0, "peak_arrival_rad_s": impulse_rad_s}
        if (
            profile.enabled
            and profile.v_min_rad_s is not None
            and profile.v_min_rad_s <= 2.0
            and (profile.brake_ms or 0) >= 40
            and impulse_rad_s >= profile.v_min_rad_s
        ):
            # Stance knee false-trip + soft hold collapse
            st.safety.impact_guard_trips += 6
            st.gravity = [0.82, 0.35, -0.35]
            st.fall_accum_s = FALL_DEBOUNCE_S + 0.05
            st.safety.fallen = True
            st.safety.gain = GAIN_LIMP
            return {"survived": False, "trips": 6, "peak_arrival_rad_s": impulse_rad_s}

        arrival = (
            min(impulse_rad_s, profile.v_fast_rad_s)
            if (profile.enabled and profile.v_fast_rad_s is not None)
            else impulse_rad_s
        )
        trips = 1 if (profile.enabled and profile.v_min_rad_s is not None and impulse_rad_s >= profile.v_min_rad_s) else 0
        st.safety.impact_guard_trips += trips
        st.gravity = [0.12, 0.05, -0.99]
        return {"survived": True, "trips": trips, "peak_arrival_rad_s": arrival}

    def set_picked_up(self, held: bool) -> None:
        if held and not self.state.safety.picked_up:
            self._pause_start_pose = list(self.state.joints)
            self.state.pickup_elapsed_s = 0.0
        elif not held:
            self._pause_start_pose = None
            self.state.pickup_elapsed_s = 0.0
        self.state.safety.picked_up = held

    def compute_theremin(self, hand_distance_m: float, status_code: int = 5) -> Dict[str, float]:
        """Compute ToF theremin frequency (Hz) and mouth opening [0..1] from hand distance."""
        valid_statuses = {4, 5, 9, 13}
        if (
            not math.isfinite(hand_distance_m)
            or status_code not in valid_statuses
            or hand_distance_m < 0.08
            or hand_distance_m > 0.60
        ):
            return {"active": 0.0, "freq_hz": 0.0, "mouth_open": 0.0, "mouth_rad": mouth_target(0.0)}
        norm = max(0.0, min(1.0, (0.60 - hand_distance_m) / (0.60 - 0.08)))
        freq_hz = 220.0 * (2.0 ** (norm * 1.5))  # A3 (220 Hz) to ~E5 (622 Hz)
        return {
            "active": 1.0,
            "freq_hz": round(freq_hz, 2),
            "mouth_open": round(norm, 3),
            "mouth_rad": round(mouth_target(norm), 4),
        }

    def validate_and_clamp_targets(self, raw_targets15: Sequence[float]) -> List[float]:
        """Enforce `duck-control/src/safety.rs` non-finite rejection & [-pi, pi] clamping."""
        if len(raw_targets15) != NUM_JOINTS:
            raise ValueError(f"Expected {NUM_JOINTS} targets")
        for val in raw_targets15:
            if not math.isfinite(val):
                self.state.safety.rejected_nonfinite = True
                self.state.safety.clamped_joints = 0
                return list(self.state.joints)
        self.state.safety.rejected_nonfinite = False
        clamped = []
        clamp_count = 0
        for val in raw_targets15:
            if val < ACTUATOR_MIN_RAD:
                clamped.append(ACTUATOR_MIN_RAD)
                clamp_count += 1
            elif val > ACTUATOR_MAX_RAD:
                clamped.append(ACTUATOR_MAX_RAD)
                clamp_count += 1
            else:
                clamped.append(float(val))
        self.state.safety.clamped_joints = clamp_count
        return clamped

    def step(
        self,
        command: Optional[Command] = None,
        mouth_open: float = 0.0,
        dt: float = CONTROL_DT,
        refresh_intent: bool = True,
    ) -> Dict[str, object]:
        """Advance simulation by one 50 Hz tick (20 ms) and return observation + telemetry."""
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError(f"dt must be finite and > 0, got {dt}")
        st = self.state
        st.time_s += dt
        if command is None:
            command = Command()
        elif refresh_intent:
            st.last_intent_time_s = st.time_s

        # Deadman timeout check
        stale = (st.time_s - st.last_intent_time_s) > DEADMAN_TIMEOUT_S
        st.safety.deadman_active = stale
        eff_cmd = Command(head=command.head, body=command.body) if stale else command

        # Fall debounce check (including B3 non-finite IMU gravity fault protection)
        gravity_finite = len(st.gravity) == 3 and all(math.isfinite(g) for g in st.gravity)
        if not gravity_finite:
            st.safety.fallen = True
            st.safety.gain = GAIN_LIMP
        elif st.gravity[2] > FALL_GRAVITY_Z_THRESHOLD:
            st.fall_accum_s += dt
            if st.fall_accum_s >= FALL_DEBOUNCE_S:
                st.safety.fallen = True
                st.safety.gain = GAIN_LIMP
        else:
            st.fall_accum_s = 0.0
            st.safety.fallen = False
            st.safety.gain = GAIN_RUNNING

        # Compute target pose
        if st.safety.picked_up and self._pause_start_pose is not None:
            st.pickup_elapsed_s += dt
            target15 = compute_pause_target(self._pause_start_pose, st.pickup_elapsed_s)
            target15[MOUTH_INDEX] = mouth_target(mouth_open)
            action14 = [target15[i] - DEFAULT_POSITION[i] for i in range(NUM_JOINTS) if i != MOUTH_INDEX]
        else:
            # Synthesize plausible walking/standing gait cycle offsets
            vx, vy, vyaw = eff_cmd.twist
            twist_finite = all(math.isfinite(v) for v in eff_cmd.twist)
            speed = eff_cmd.twist_magnitude() if twist_finite else 0.0
            phase = st.time_s * 7.5
            swing = min(0.32, speed * 0.65)
            action14 = [0.0] * 14
            if not twist_finite:
                # Propagate non-finite command to target validation without poisoning odometry
                action14[0] = float("nan")
            elif not st.safety.fallen and speed > 0.01:
                s = math.sin(phase)
                c = math.cos(phase)
                # Left leg (0..4)
                action14[0] = vyaw * 0.15 * s
                action14[1] = vy * 0.25 + 0.06 * c
                action14[2] = swing * s
                action14[3] = max(0.0, -swing * c) * 1.1
                action14[4] = -0.5 * swing * s
                # Right leg (9..13) in anti-phase (mirrored polarity)
                action14[9] = -vyaw * 0.15 * s
                action14[10] = vy * 0.25 - 0.06 * c
                action14[11] = swing * s
                action14[12] = -max(0.0, swing * c) * 1.1
                action14[13] = -0.5 * swing * s

            # Head offsets (indices 5..8 in 14-D action)
            action14[5] = eff_cmd.head[0]
            action14[6] = eff_cmd.head[1]
            action14[7] = eff_cmd.head[2]
            action14[8] = eff_cmd.head[3]

            raw15 = scatter_action(action14, mouth_rad=0.0)
            target15 = [DEFAULT_POSITION[i] + raw15[i] for i in range(NUM_JOINTS)]
            target15[MOUTH_INDEX] = mouth_target(mouth_open)

            # Integrate 2D odometry only when upright and twist command is finite (B1)
            if not st.safety.fallen and twist_finite:
                st.yaw_rad += vyaw * dt
                cos_y, sin_y = math.cos(st.yaw_rad), math.sin(st.yaw_rad)
                st.position_xy[0] += (vx * cos_y - vy * sin_y) * dt
                st.position_xy[1] += (vx * sin_y + vy * cos_y) * dt

        safe15 = self.validate_and_clamp_targets(target15)
        for i in range(NUM_JOINTS):
            vel = (safe15[i] - st.joints[i]) / dt
            st.velocities[i] = vel
            st.joints[i] = safe15[i]

        # B2: Store post-clamp, guaranteed-finite action14 and sanitize observation inputs
        st.prev_action14 = [safe15[i] - DEFAULT_POSITION[i] for i in range(NUM_JOINTS) if i != MOUTH_INDEX]
        obs_gyro = [v if math.isfinite(v) else 0.0 for v in st.gyro]
        obs_gravity = list(st.gravity) if gravity_finite else [0.0, 0.0, 1.0]
        safe_cmd = Command(
            twist=tuple(v if math.isfinite(v) else 0.0 for v in eff_cmd.twist),
            head=tuple(v if math.isfinite(v) else 0.0 for v in eff_cmd.head),
            body=BodyPoseCommand(
                z=eff_cmd.body.z if math.isfinite(eff_cmd.body.z) else 0.0,
                roll=eff_cmd.body.roll if math.isfinite(eff_cmd.body.roll) else 0.0,
                pitch=eff_cmd.body.pitch if math.isfinite(eff_cmd.body.pitch) else 0.0,
            ),
        )
        obs61 = build_observation(
            gyro=obs_gyro,
            gravity=obs_gravity,
            positions15=st.joints,
            velocities15=st.velocities,
            previous_action14=st.prev_action14,
            command=safe_cmd,
        )
        pickup63 = build_pickup_features(
            gyro=obs_gyro,
            gravity=obs_gravity,
            positions15=st.joints,
            velocities15=st.velocities,
            currents_ma15=st.currents_ma,
            previous_target15=safe15,
            previous_paused=st.safety.picked_up,
        )

        # Update pre-hardware subsystems (`odometry`, `bus.rs`)
        cy, sy = math.cos(st.yaw_rad * 0.5), math.sin(st.yaw_rad * 0.5)
        quat_wxyz = [cy, 0.0, 0.0, sy]
        odom16_snap = self.odom_alpha16.update(st.joints, quat_wxyz)
        odom15_snap = self.odom_v15.update(st.joints, quat_wxyz)
        bus_snap = self.bus.step_bus(
            target_positions15=safe15,
            actual_positions15=st.joints,
            actual_velocities15=st.velocities,
            dt=dt,
            kp_gain=st.safety.gain,
        )
        st.currents_ma = list(bus_snap["currents_ma"])

        return {
            "time_s": round(st.time_s, 4),
            "obs61": obs61,
            "pickup63": pickup63,
            "joints15": list(st.joints),
            "position_xy": list(st.position_xy),
            "yaw_rad": st.yaw_rad,
            "safety": st.safety,
            "odometry_alpha16": odom16_snap,
            "odometry_v15": odom15_snap,
            "bus": bus_snap,
        }
