"""
apps/MicroDuck/src/kinematics_obs.py

Exact Python implementation of:
1. `duck-control/src/obs.rs` — 61-D Observation vector & 14->15 joint action scatter
2. `duck-control/src/pickup.rs` — 63-D Pickup Detector feature row & 300 ms PAUSE_POSE ramp
3. `kinematics/src/lib.rs` — MJCF-driven SE(3) forward kinematics for all bodies & sites
"""

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

from models import (
    ACTION_LEN,
    DEFAULT_POSITION,
    MOUTH_INDEX,
    NUM_JOINTS,
    OBS_LEN,
    PAUSE_POSE_OFFSETS,
    PICKUP_FEAT_LEN,
)

PAUSE_RAMP_S: float = 0.300


def policy_joints(values15: Sequence[float]) -> List[float]:
    """Extract the 14 policy-visible joints (all 15 joints minus MOUTH_INDEX=9)."""
    if len(values15) != NUM_JOINTS:
        raise ValueError(f"Expected {NUM_JOINTS} joints, got {len(values15)}")
    return [float(values15[i]) for i in range(NUM_JOINTS) if i != MOUTH_INDEX]


def scatter_action(action14: Sequence[float], mouth_rad: float = 0.0) -> List[float]:
    """Map 14 policy actions back into 15-joint order, placing mouth_rad at MOUTH_INDEX."""
    if len(action14) != ACTION_LEN:
        raise ValueError(f"Expected {ACTION_LEN} actions, got {len(action14)}")
    out = [0.0] * NUM_JOINTS
    src = 0
    for i in range(NUM_JOINTS):
        if i == MOUTH_INDEX:
            out[i] = float(mouth_rad)
        else:
            out[i] = float(action14[src])
            src += 1
    return out


@dataclass
class BodyPoseCommand:
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0


@dataclass
class Command:
    twist: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # vx, vy, vyaw
    head: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)  # neck_pitch, head_pitch, head_yaw, head_roll
    body: BodyPoseCommand = field(default_factory=BodyPoseCommand)

    def twist_magnitude(self) -> float:
        return math.sqrt(sum(v * v for v in self.twist))


def build_observation(
    gyro: Sequence[float],
    gravity: Sequence[float],
    positions15: Sequence[float],
    velocities15: Sequence[float],
    previous_action14: Sequence[float],
    command: Command,
) -> List[float]:
    """
    Build the exact 61-D observation vector matching `duck-control/src/obs.rs`:
      0..3   gyro (rad/s)
      3..6   projected gravity (unit vector)
      6..20  joint positions - DEFAULT_POSITION (mouth excluded)
      20..34 joint velocities (mouth excluded)
      34..48 previous action (mouth excluded)
      48..51 twist [vx, vy, vyaw]
      51..55 head [neck_pitch, head_pitch, head_yaw, head_roll]
      55..57 [0.0, 0.0] (unbound body x, y)
      57..60 [body.z, body.roll, body.pitch] (NOTE: roll before pitch!)
      60     0.0 (unbound body yaw)
    """
    if len(gyro) != 3 or len(gravity) != 3:
        raise ValueError("gyro and gravity must be 3-vectors")
    if len(previous_action14) != ACTION_LEN:
        raise ValueError(f"previous_action14 must have length {ACTION_LEN}")

    home14 = policy_joints(DEFAULT_POSITION)
    pos14 = policy_joints(positions15)
    vel14 = policy_joints(velocities15)

    obs = [0.0] * OBS_LEN
    obs[0:3] = [float(x) for x in gyro]
    obs[3:6] = [float(x) for x in gravity]
    obs[6:20] = [pos14[j] - home14[j] for j in range(ACTION_LEN)]
    obs[20:34] = [vel14[j] for j in range(ACTION_LEN)]
    obs[34:48] = [float(previous_action14[j]) for j in range(ACTION_LEN)]
    obs[48:51] = [float(x) for x in command.twist]
    obs[51:55] = [float(x) for x in command.head]
    obs[55] = 0.0
    obs[56] = 0.0
    obs[57] = float(command.body.z)
    obs[58] = float(command.body.roll)
    obs[59] = float(command.body.pitch)
    obs[60] = 0.0
    return obs


def build_pickup_features(
    gyro: Sequence[float],
    gravity: Sequence[float],
    positions15: Sequence[float],
    velocities15: Sequence[float],
    currents_ma15: Sequence[float],
    previous_target15: Sequence[float],
    previous_paused: bool,
) -> List[float]:
    """Build one 63-D feature row matching `duck-control/src/pickup.rs::features`."""
    home14 = policy_joints(DEFAULT_POSITION)
    angles14 = policy_joints(positions15)
    rates14 = policy_joints(velocities15)
    target14 = policy_joints(previous_target15)
    amps14 = policy_joints(currents_ma15)

    row = [0.0] * PICKUP_FEAT_LEN
    row[0:3] = [float(x) for x in gyro]
    row[3:6] = [float(x) for x in gravity]
    for j in range(ACTION_LEN):
        row[6 + j] = angles14[j] - home14[j]
        row[20 + j] = rates14[j]
        row[34 + j] = target14[j] - home14[j]
        row[48 + j] = abs(amps14[j]) / 1000.0
    row[62] = 1.0 if previous_paused else 0.0
    return row


def compute_pause_target(start_pose15: Sequence[float], elapsed_s: float) -> List[float]:
    """Compute the 300 ms ramp from start_pose15 to DEFAULT_POSITION + PAUSE_POSE_OFFSETS."""
    if len(start_pose15) != NUM_JOINTS:
        raise ValueError(f"Expected {NUM_JOINTS} joints")
    t = max(0.0, min(1.0, elapsed_s / PAUSE_RAMP_S))
    offsets15 = scatter_action(PAUSE_POSE_OFFSETS, mouth_rad=0.0)
    out = [0.0] * NUM_JOINTS
    for i in range(NUM_JOINTS):
        if i == MOUTH_INDEX:
            out[i] = float(start_pose15[i])
        else:
            goal = DEFAULT_POSITION[i] + offsets15[i]
            out[i] = float(start_pose15[i]) + (goal - float(start_pose15[i])) * t
    return out


def _quat_to_rot3(q: Sequence[float]) -> List[List[float]]:
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    w, x, y, z = w / n, x / n, y / n, z / n
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
    ]


def _axis_to_rot3(axis: Sequence[float], angle: float) -> List[List[float]]:
    ax, ay, az = axis
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n < 1e-12 or abs(angle) < 1e-12:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    x, y, z = ax / n, ay / n, az / n
    s, c = math.sin(angle), math.cos(angle)
    ic = 1.0 - c
    return [
        [c + x * x * ic, x * y * ic - z * s, x * z * ic + y * s],
        [y * x * ic + z * s, c + y * y * ic, y * z * ic - x * s],
        [z * x * ic - y * s, z * y * ic + x * s, c + z * z * ic],
    ]


def _mat3_mul(a: List[List[float]], b: List[List[float]]) -> List[List[float]]:
    return [
        [sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def _mat3_vec(a: List[List[float]], v: Sequence[float]) -> List[float]:
    return [a[i][0] * v[0] + a[i][1] * v[1] + a[i][2] * v[2] for i in range(3)]


_DEFAULT_BODIES = [
    {"parent": -1, "joint": -1, "joint_name": None, "pos": [0.0, 0.0, 0.12], "quat": [1.0, 0.0, 0.0, 0.0], "axis": [0.0, 0.0, 0.0]},
    {"parent": 0, "joint": 0, "joint_name": "left_hip_yaw", "pos": [0.006, 0.0175, -0.005], "quat": [0.0, -0.707107, -0.707107, -0.0], "axis": [0.0, 0.0, 1.0]},
    {"parent": 1, "joint": 1, "joint_name": "left_hip_roll", "pos": [-0.0, 0.0165, 0.0125], "quat": [0.707107, -0.707107, -0.0, 0.0], "axis": [0.0, 0.0, 1.0]},
    {"parent": 2, "joint": 2, "joint_name": "left_hip_pitch", "pos": [0.025, 0.0, -0.0185], "quat": [0.5, -0.5, 0.5, -0.5], "axis": [0.0, 0.0, 1.0]},
    {"parent": 3, "joint": 3, "joint_name": "left_knee", "pos": [0.022, 0.035777, -0.004], "quat": [0.0, 0.707107, 0.707107, -0.0], "axis": [0.0, 0.0, 1.0]},
    {"parent": 4, "joint": 4, "joint_name": "left_ankle", "pos": [-0.0, 0.042, -0.026], "quat": [0.0, 1.0, 0.0, -0.0], "axis": [0.0, 0.0, 1.0]},
    {"parent": 0, "joint": 5, "joint_name": "neck_pitch", "pos": [0.026, 0.014501, 0.032442], "quat": [0.0, -0.0, -0.707107, 0.707107], "axis": [0.0, 0.0, 1.0]},
    {"parent": 6, "joint": 6, "joint_name": "head_pitch", "pos": [0.0, -0.05, 0.0], "quat": [0.0, 1.0, 0.0, 0.0], "axis": [0.0, 0.0, 1.0]},
    {"parent": 7, "joint": 7, "joint_name": "head_yaw", "pos": [-0.0, 0.018693, -0.0145], "quat": [0.0, -0.0, -0.707107, -0.707107], "axis": [0.0, 0.0, 1.0]},
    {"parent": 8, "joint": 8, "joint_name": "head_roll", "pos": [-0.0179, 0.0, 0.0145], "quat": [0.707107, -0.0, -0.707107, 0.0], "axis": [0.0, 0.0, 1.0]},
    {"parent": 0, "joint": 10, "joint_name": "right_hip_yaw", "pos": [0.006, -0.0175, -0.005], "quat": [0.0, -0.707107, -0.707107, -0.0], "axis": [0.0, 0.0, 1.0]},
    {"parent": 10, "joint": 11, "joint_name": "right_hip_roll", "pos": [-0.0, 0.0165, 0.0125], "quat": [0.0, -0.0, 0.707107, 0.707107], "axis": [0.0, 0.0, 1.0]},
    {"parent": 11, "joint": 12, "joint_name": "right_hip_pitch", "pos": [0.025, 0.0, -0.0185], "quat": [0.5, -0.5, 0.5, -0.5], "axis": [0.0, 0.0, 1.0]},
    {"parent": 12, "joint": 13, "joint_name": "right_knee", "pos": [-0.022, 0.035777, -0.004], "quat": [0.0, 1.0, -0.0, -0.0], "axis": [0.0, 0.0, 1.0]},
    {"parent": 13, "joint": 14, "joint_name": "right_ankle", "pos": [-0.042, -0.0, -0.026], "quat": [0.0, -0.707107, 0.707107, 0.0], "axis": [0.0, 0.0, 1.0]},
]


class ForwardKinematics:
    """SE(3) Forward Kinematics engine backed by extracted `microduck_cad_rig.json` (with built-in kinematic fallback)."""

    def __init__(self, rig_json_path: Optional[Path] = None) -> None:
        if rig_json_path is None:
            rig_json_path = Path(__file__).resolve().parent.parent / "web" / "assets" / "models" / "microduck_cad_rig.json"
        if rig_json_path.exists():
            data = json.loads(rig_json_path.read_text(encoding="utf-8"))
            self.bodies = data["bodies"]
            self.joint_names = data["joint_names"]
        else:
            from models import JOINT_NAMES
            self.bodies = list(_DEFAULT_BODIES)
            self.joint_names = list(JOINT_NAMES)

    def solve_bodies(self, joint_angles15: Sequence[float]) -> List[Tuple[List[List[float]], List[float]]]:
        """Compute world (R, t) in robot trunk frame for every body in the CAD rig."""
        if len(joint_angles15) != NUM_JOINTS:
            raise ValueError(f"Expected {NUM_JOINTS} joint angles")
        poses: List[Tuple[List[List[float]], List[float]]] = []
        for b in self.bodies:
            parent_idx = b["parent"]
            if parent_idx >= 0:
                pr, pt = poses[parent_idx]
            else:
                pr = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
                pt = [0.0, 0.0, 0.0]

            lr = _quat_to_rot3(b["quat"])
            lt = b["pos"]
            r = _mat3_mul(pr, lr)
            t_offset = _mat3_vec(pr, lt)
            t = [pt[0] + t_offset[0], pt[1] + t_offset[1], pt[2] + t_offset[2]]

            j_idx = b["joint"]
            if 0 <= j_idx < NUM_JOINTS:
                r = _mat3_mul(r, _axis_to_rot3(b["axis"], float(joint_angles15[j_idx])))

            poses.append((r, t))
        return poses
