"""
apps/MicroDuck/tests/test_microduck_sim.py

Deterministic pytest verification suite for the MicroDuck digital twin:
1. Ground-truth extraction integrity (`microduck_cad_rig.json`, `microduck_cad_articulated.glb`, `microduck_mjcf.json`, 13 emotions, scores).
2. 61-D observation vector layout & mouth index 9 exclusion (`duck-control/src/obs.rs`).
3. 63-D pickup feature row & 300 ms PAUSE_POSE ramp (`duck-control/src/pickup.rs`).
4. Safety invariants: NaN rejection, [-pi, pi] range clamp, 500 ms deadman velocity zeroing, 200 ms fall debounce.
5. Impact Guard Level 5 stance-collapse vs Setting R survival (`docs/robot/cheatsheet.md`).
6. SE(3) forward kinematics across all 15 CAD bodies & ToF Theremin pitch/mouth mapping.
"""

import json
import math
from pathlib import Path
import sys
import pytest

SRC_DIR = Path(__file__).resolve().parent.parent / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from kinematics_obs import (
    BodyPoseCommand,
    Command,
    ForwardKinematics,
    build_observation,
    build_pickup_features,
    compute_pause_target,
    policy_joints,
    scatter_action,
)
from models import (
    ACTION_LEN,
    ACTUATOR_MAX_RAD,
    ACTUATOR_MIN_RAD,
    DEFAULT_POSITION,
    GAIN_LIMP,
    MOUTH_CLOSED_RAD,
    MOUTH_INDEX,
    MOUTH_OPEN_RAD,
    NUM_JOINTS,
    OBS_LEN,
    PAUSE_POSE_OFFSETS,
    PICKUP_FEAT_LEN,
    mouth_target,
)
from sim_engine import MicroDuckSimulator

PROJECT_ROOT = Path(__file__).resolve().parent.parent
WEB_ASSETS = PROJECT_ROOT / "web" / "assets"


def test_extracted_ground_truth_assets_exist():
    rig_path = WEB_ASSETS / "models" / "microduck_cad_rig.json"
    if not rig_path.exists():
        pytest.skip("Upstream assets not fetched yet; run `bash scripts/fetch_upstream_assets.sh`")
    glb_path = WEB_ASSETS / "models" / "microduck_cad_articulated.glb"
    mjcf_path = WEB_ASSETS / "models" / "microduck_mjcf.json"
    assert rig_path.exists() and rig_path.stat().st_size > 50_000
    assert glb_path.exists() and glb_path.stat().st_size > 50_000
    assert mjcf_path.exists()

    rig = json.loads(rig_path.read_text(encoding="utf-8"))
    assert len(rig["meshes"]) == 29
    assert len(rig["bodies"]) == 15
    assert len(rig["parts"]) == 59
    assert rig["home_pose_stats"]["height_m"] > 0.24

    emotions_idx = json.loads((WEB_ASSETS / "emotions" / "index.json").read_text(encoding="utf-8"))
    assert len(emotions_idx) == 13
    for name in emotions_idx:
        assert (WEB_ASSETS / "emotions" / f"{name}.json").exists()
        assert (WEB_ASSETS / "emotions" / f"{name}.wav").exists()
    assert (WEB_ASSETS / "scores" / "wistful.duckscore").exists()


def test_observation_layout_and_mouth_exclusion():
    pos = list(DEFAULT_POSITION)
    pos[MOUTH_INDEX] = 0.42  # Mouth angle must be invisible to 61-D obs
    pos[0] += 0.25  # left_hip_yaw offset
    pos[14] -= 0.15  # right_ankle offset
    vel = [0.1 * i for i in range(NUM_JOINTS)]
    prev_act = [0.05] * ACTION_LEN
    cmd = Command(
        twist=(0.3, -0.1, 0.4),
        head=(0.1, 0.2, -0.3, 0.05),
        body=BodyPoseCommand(z=-0.02, roll=0.11, pitch=-0.07),
    )

    obs = build_observation(
        gyro=(0.01, 0.02, -0.03),
        gravity=(0.0, 0.0, -1.0),
        positions15=pos,
        velocities15=vel,
        previous_action14=prev_act,
        command=cmd,
    )
    assert len(obs) == OBS_LEN
    assert math.isclose(obs[6], 0.25, abs_tol=1e-9)
    assert math.isclose(obs[19], -0.15, abs_tol=1e-9)
    # Mouth index 9 is skipped: slot 9 in policy_joints is right_hip_yaw (joint 10)
    assert math.isclose(obs[6 + 9], 0.0, abs_tol=1e-9)
    assert math.isclose(obs[20 + 9], vel[10], abs_tol=1e-9)
    # Command block: 48..51 twist, 51..55 head, 55..57 zero, 57..60 [z, roll, pitch], 60 zero
    assert obs[48:51] == [0.3, -0.1, 0.4]
    assert obs[51:55] == [0.1, 0.2, -0.3, 0.05]
    assert obs[55:57] == [0.0, 0.0]
    assert obs[57:60] == [-0.02, 0.11, -0.07]
    assert obs[60] == 0.0


def test_scatter_and_mouth_clamping():
    act = [float(i + 1) for i in range(ACTION_LEN)]
    joints15 = scatter_action(act, mouth_rad=0.22)
    assert len(joints15) == NUM_JOINTS
    assert math.isclose(joints15[MOUTH_INDEX], 0.22)
    assert policy_joints(joints15) == act

    assert math.isclose(mouth_target(0.0), MOUTH_CLOSED_RAD)
    assert math.isclose(mouth_target(1.0), MOUTH_OPEN_RAD)
    assert math.isclose(mouth_target(float("nan")), MOUTH_CLOSED_RAD)
    assert math.isclose(mouth_target(5.0), MOUTH_OPEN_RAD)


def test_pickup_features_and_pause_ramp():
    row = build_pickup_features(
        gyro=(0.1, 0.2, 0.3),
        gravity=(0.0, 0.0, -1.0),
        positions15=DEFAULT_POSITION,
        velocities15=[0.0] * NUM_JOINTS,
        currents_ma15=[500.0] * NUM_JOINTS,
        previous_target15=DEFAULT_POSITION,
        previous_paused=True,
    )
    assert len(row) == PICKUP_FEAT_LEN
    assert math.isclose(row[48], 0.5)
    assert row[62] == 1.0

    start = list(DEFAULT_POSITION)
    start[MOUTH_INDEX] = 0.25
    end = compute_pause_target(start, elapsed_s=0.35)
    assert math.isclose(end[MOUTH_INDEX], 0.25)
    assert math.isclose(end[1], DEFAULT_POSITION[1] + PAUSE_POSE_OFFSETS[1], abs_tol=1e-9)


def test_safety_deadman_clamp_and_nonfinite_rejection():
    sim = MicroDuckSimulator(impact_guard_level="R")
    out = sim.validate_and_clamp_targets([5.0] + [0.0] * 14)
    assert math.isclose(out[0], ACTUATOR_MAX_RAD)
    assert sim.state.safety.clamped_joints == 1

    out_neg = sim.validate_and_clamp_targets([-9.0] + [0.0] * 14)
    assert math.isclose(out_neg[0], ACTUATOR_MIN_RAD)

    before = list(sim.state.joints)
    out_nan = sim.validate_and_clamp_targets([float("nan")] + [0.0] * 14)
    assert sim.state.safety.rejected_nonfinite is True
    assert out_nan == before

    # Advance >0.5s without refreshing intents -> deadman zeroes velocity
    for _ in range(30):
        res = sim.step(Command(twist=(0.3, 0.0, 0.0)), refresh_intent=False)
    assert res["safety"].deadman_active is True
    assert res["obs61"][48:51] == [0.0, 0.0, 0.0]


def test_impact_guard_level5_vs_setting_r():
    sim_factory = MicroDuckSimulator(impact_guard_level="5")
    r_factory = sim_factory.apply_push_impulse(7.5)
    assert r_factory["survived"] is False
    assert sim_factory.state.safety.fallen is True

    sim_r = MicroDuckSimulator(impact_guard_level="R")
    r_hardened = sim_r.apply_push_impulse(7.5)
    assert r_hardened["survived"] is True
    assert r_hardened["peak_arrival_rad_s"] <= 7.0
    assert sim_r.state.safety.fallen is False


def test_forward_kinematics_and_theremin():
    fk = ForwardKinematics()
    poses = fk.solve_bodies(DEFAULT_POSITION)
    assert len(poses) == 15
    # Root trunk_base sits at MJCF standing height z = 0.12 m
    assert math.isclose(poses[0][1][2], 0.12, abs_tol=1e-5)

    sim = MicroDuckSimulator()
    t_near = sim.compute_theremin(0.15, status_code=4)  # Status 4 consistency-ok beyond 30cm
    t_far = sim.compute_theremin(0.50, status_code=13)
    assert t_near["active"] == 1.0 and t_far["active"] == 1.0
    assert t_near["freq_hz"] > t_far["freq_hz"]
    assert t_near["mouth_open"] > t_far["mouth_open"]

    t_invalid = sim.compute_theremin(0.25, status_code=255)
    assert t_invalid["active"] == 0.0
    t_nan = sim.compute_theremin(float("nan"), status_code=5)
    assert t_nan["active"] == 0.0 and t_nan["freq_hz"] == 0.0


def test_xgoduck_hardware_stl_catalog_and_assets():
    models = PROJECT_ROOT / "web" / "assets" / "models"
    catalog_path = models / "xgoduck_stl_catalog.json"
    if not catalog_path.exists():
        pytest.skip("Upstream STL assets not fetched yet; run `bash scripts/fetch_upstream_assets.sh`")
    catalog = json.loads(catalog_path.read_text(encoding="utf-8"))
    summary = catalog["summary"]
    assert summary["v2_standard_count"] == 47
    assert summary["v5_audio_video_count"] == 50
    assert summary["total_stl_count"] == 97
    assert (models / "microduck_3d_print.stl").stat().st_size > 100_000
    assert (models / "xgoduck_v5_stl_kit.glb").stat().st_size > 100_000
    tire = json.loads((models / "xgoduck_v5_tire.json").read_text(encoding="utf-8"))
    assert len(tire["faces"]) > 0 and tire["radius_m"] > 0.005


def test_claude_endgegner_adversarial_nan_guards():
    sim = MicroDuckSimulator(impact_guard_level="R")
    # B1: NaN twist must reject frame and never poison odometry or obs61
    for _ in range(10):
        res = sim.step(Command(twist=(float("nan"), 0.0, 0.0)))
    assert res["safety"].rejected_nonfinite is True
    assert all(math.isfinite(v) for v in res["position_xy"])
    assert math.isfinite(res["yaw_rad"])
    assert all(math.isfinite(v) for v in res["obs61"])

    # B2: NaN head command rejects frame, holds joints, and keeps prev_action14 & obs[34:48] finite
    prev_joints = list(sim.state.joints)
    res_head = sim.step(Command(head=(float("nan"), 0.0, 0.0, 0.0)))
    assert res_head["safety"].rejected_nonfinite is True
    assert res_head["joints15"] == prev_joints
    assert all(math.isfinite(v) for v in sim.state.prev_action14)
    assert all(math.isfinite(v) for v in res_head["obs61"][34:48])

    # B3: NaN gravity fault latches fallen + GAIN_LIMP and keeps obs61 finite
    sim.state.gravity = [float("nan"), 0.0, -1.0]
    res_grav = sim.step(Command())
    assert res_grav["safety"].fallen is True
    assert res_grav["safety"].gain == GAIN_LIMP
    assert all(math.isfinite(v) for v in res_grav["obs61"])

    # dt validation
    for bad_dt in (0.0, -0.02, float("nan")):
        try:
            sim.step(Command(), dt=bad_dt)
            assert False, f"Expected ValueError for dt={bad_dt}"
        except ValueError:
            pass
