"""
apps/MicroDuck/src/models.py

Domain constants, hardware tables, daemon topology, and Impact Guard specifications
derived directly from `pollen-robotics/microduck` (`duck-control`, `duck-ipc-proto`,
`docs/design/architecture.md`, `docs/robot/cheatsheet.md`).
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import math

NUM_JOINTS: int = 15
OBS_LEN: int = 61
ACTION_LEN: int = 14
PICKUP_FEAT_LEN: int = 63
PICKUP_WINDOW: int = 50
MOUTH_INDEX: int = 9
CONTROL_HZ: int = 50
CONTROL_DT: float = 1.0 / CONTROL_HZ

JOINT_NAMES: Tuple[str, ...] = (
    "left_hip_yaw",
    "left_hip_roll",
    "left_hip_pitch",
    "left_knee",
    "left_ankle",
    "neck_pitch",
    "head_pitch",
    "head_yaw",
    "head_roll",
    "mouth",
    "right_hip_yaw",
    "right_hip_roll",
    "right_hip_pitch",
    "right_knee",
    "right_ankle",
)

# XL330 Dynamixel IDs matching duck_ipc_proto::JOINT_IDS
JOINT_IDS: Tuple[int, ...] = (
    11, 12, 13, 14, 15,  # left leg
    21, 22, 23, 24, 25,  # neck, head, mouth
    31, 32, 33, 34, 35,  # right leg
)

# Home pose (radians) matching duck-control/src/model.rs::DEFAULT_POSITION
DEFAULT_POSITION: Tuple[float, ...] = (
    0.0, -0.0873, -0.4579, -0.0049, 0.4530,
    0.3491, 0.3491, 0.0, 0.0, 0.0,
    0.0, 0.0873, 0.4579, 0.0049, -0.4530,
)

# Untorqued rest seat pose matching duck-control/src/model.rs::REST_POSITION
REST_POSITION: Tuple[float, ...] = (
    0.3129, 0.4050, -0.2338, 1.5493, 0.5522,
    0.9572, 1.4787, -0.0129, -0.0522, -0.0121,
    0.0353, -0.5660, 0.2853, -1.4696, -0.4096,
)

# Pickup pause stance offsets (14 joints, mouth excluded) from duck-control/src/pickup.rs::PAUSE_POSE
PAUSE_POSE_OFFSETS: Tuple[float, ...] = (
    0.0, -0.100, 0.038, -0.145, 0.044,
    -0.096, 0.008, 0.0, 0.0,
    0.0, 0.100, -0.038, 0.145, -0.044,
)

MOUTH_CLOSED_RAD: float = -5.0 * math.pi / 180.0
MOUTH_OPEN_RAD: float = 30.0 * math.pi / 180.0

ACTUATOR_MIN_RAD: float = -math.pi
ACTUATOR_MAX_RAD: float = math.pi
FALL_GRAVITY_Z_THRESHOLD: float = -0.5
FALL_DEBOUNCE_S: float = 0.200
DEADMAN_TIMEOUT_S: float = 0.500
GAIN_RUNNING: int = 200
GAIN_LIMP: int = 50


def mouth_target(open_fraction: float) -> float:
    """Convert [0, 1] mouth opening fraction to radians (-5 deg .. +30 deg)."""
    if not math.isfinite(open_fraction):
        open_fraction = 0.0
    clamped = max(0.0, min(1.0, open_fraction))
    return MOUTH_CLOSED_RAD + clamped * (MOUTH_OPEN_RAD - MOUTH_CLOSED_RAD)


@dataclass(frozen=True)
class ImpactGuardProfile:
    level_id: str
    name: str
    enabled: bool
    v_min_rad_s: Optional[float]
    brake_ms: Optional[int]
    decel_rad_s2: Optional[int]
    i_hold_ma: Optional[int]
    v_fast_rad_s: Optional[float]
    pwm_fast_pct: Optional[int]
    notes: str


IMPACT_GUARD_PROFILES: Dict[str, ImpactGuardProfile] = {
    "0": ImpactGuardProfile(
        level_id="0",
        name="Off (Current Limit 1000 mA only)",
        enabled=False,
        v_min_rad_s=None,
        brake_ms=None,
        decel_rad_s2=None,
        i_hold_ma=None,
        v_fast_rad_s=None,
        pwm_fast_pct=None,
        notes="Walks steadily; arrivals reach ~8.5 rad/s on hard landings.",
    ),
    "5": ImpactGuardProfile(
        level_id="5",
        name="Factory Impact Guard (V3/V4 Default)",
        enabled=True,
        v_min_rad_s=1.0,
        brake_ms=50,
        decel_rad_s2=400,
        i_hold_ma=500,
        v_fast_rad_s=9.0,
        pwm_fast_pct=50,
        notes="Trips ~6x/s on stance knees (<3 rad/s) and latches soft hold -> stance collapse.",
    ),
    "7": ImpactGuardProfile(
        level_id="7",
        name="Late Short Brake",
        enabled=True,
        v_min_rad_s=6.0,
        brake_ms=20,
        decel_rad_s2=400,
        i_hold_ma=32767,
        v_fast_rad_s=9.0,
        pwm_fast_pct=50,
        notes="Brakes only on arrivals >6 rad/s without soft hold; survived 20/20 pushes.",
    ),
    "R": ImpactGuardProfile(
        level_id="R",
        name="Setting R (Hardened Default)",
        enabled=True,
        v_min_rad_s=6.0,
        brake_ms=20,
        decel_rad_s2=400,
        i_hold_ma=32767,
        v_fast_rad_s=7.0,
        pwm_fast_pct=40,
        notes="Caps arrival velocity near 7 rad/s with 40% PWM taper and zero stance-knee false trips.",
    ),
}


@dataclass(frozen=True)
class DaemonSpec:
    name: str
    owns: str
    socket_path: str
    survives_dead_robotd: bool
    notes: str


DAEMON_TOPOLOGY: Tuple[DaemonSpec, ...] = (
    DaemonSpec("robotd", "50 Hz control loop, Dynamixel XL330 bus, ONNX policies, safety, vetbook", "/run/robotd.sock", False, "Authoritative safety & sole motor writer"),
    DaemonSpec("configd", "Wi-Fi (NM D-Bus), identity, BLE pairing PIN, gamepad bonding, reboot", "/run/configd.sock", True, "Recovery path daemon"),
    DaemonSpec("updaterd", "Signed release verification, atomic symlink swap, health-gate rollback, HF account", "/run/updaterd.sock", True, "Recovery path daemon"),
    DaemonSpec("btd", "BLE GATT server transport adapter (subset of API)", "BLE GATT", True, "Recovery path daemon; owns zero state"),
    DaemonSpec("padd", "Bluetooth/USB gamepad reader + IMU head control + raw tap", "/run/padd/pad.sock", False, "Intent client"),
    DaemonSpec("netpadd", "UDP LAN gamepad transport", "UDP port", False, "Intent client"),
    DaemonSpec("autod", "Autonomous behavior state machine (wander, groom, nap, laser, soccer)", "—", False, "Pure brain + socket intent client"),
    DaemonSpec("mediad", "Camera/audio pipeline, WebRTC gateway, laser/ball/duck vision", "/run/mediad/media.sock", False, "TCP :8080 console & :8443 signaling"),
    DaemonSpec("tofd", "VL53L5CX/VL53L8CX 8x8 ToF depth stream (+ zero3 BMI088 IMU)", "/run/tofd/tof.sock", True, "Publishes depth frames; reads nothing"),
    DaemonSpec("mapd", "2D occupancy grid SLAM & relocalization", "/run/mapd/map.sock", False, "Subscriber only; commands nothing"),
)
