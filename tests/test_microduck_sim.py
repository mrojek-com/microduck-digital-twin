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


def test_closed_loop_codelab_and_sim_gap_invariants():
    sim_js = (PROJECT_ROOT / "web" / "simulator.js").read_text(encoding="utf-8")
    index_html = (PROJECT_ROOT / "web" / "index.html").read_text(encoding="utf-8")

    # 1. Verify TEXTAREA keyboard guard prevents spacebar/WASD hijacking in code editor
    assert "isEditableTarget" in sim_js
    assert 'tag === "TEXTAREA"' in sim_js

    # 2. Verify all 5 closed-loop presets & compositional synthesizer exist
    for preset_key in ("pd_reflex", "laser_servo", "soccer_fsm", "tof_beatbox", "python_style"):
        assert f"{preset_key}:" in sim_js
        assert f'value="{preset_key}"' in index_html

    # 3. Verify ground-truth closed-loop API & Sim-to-Real Reality Gap hooks
    for api_fn in (
        "readImu()",
        "readTarget()",
        "readTof()",
        "readOdometry()",
        "readBusSlow()",
        "readPetting()",
        "readPadImu()",
        "setJointOffset(",
        "recover()",
        "watch(",
        "assert(",
    ):
        assert api_fn in sim_js
    assert "select-sim-gap" in index_html
    assert "code-watch-strip" in index_html


def test_pillar1_alpha16_vs_v15_contact_odometry():
    from prehardware_subsystems import ALPHA16_LEFT, ALPHA16_RIGHT, ContactOdometry, SWITCH_CONFIRM_TICKS

    assert len(ALPHA16_LEFT) == 16 and len(ALPHA16_RIGHT) == 16
    assert SWITCH_CONFIRM_TICKS == 2

    odom16 = ContactOdometry(anchor_set="alpha16", quantize_encoders=True)
    odom15 = ContactOdometry(anchor_set="v15", quantize_encoders=True)

    # Initial standing frame seeds anchor_xy so trunk starts at (0, 0)
    s0 = odom16.update(DEFAULT_POSITION, [1.0, 0.0, 0.0, 0.0])
    assert math.isclose(s0["position"][0], 0.0, abs_tol=1e-6)
    assert math.isclose(s0["position"][1], 0.0, abs_tol=1e-6)

    # Simulate alternating right/left stance rolls that dip the opposite sole below -0.010 m
    sim = MicroDuckSimulator()
    for _ in range(30):
        res = sim.step(Command(twist=(0.25, 0.0, 0.1)))
    assert "odometry_alpha16" in res and "odometry_v15" in res

    # Settle standing for 10 ticks (`odometry/src/lib.rs::the_anchor_switches_only_after_the_claim_holds`)
    for _ in range(10):
        s_settled = odom16.update(DEFAULT_POSITION, [1.0, 0.0, 0.0, 0.0])
    settled_foot = s_settled["anchor_foot"]
    settled_switches = s_settled["switches"]

    # Roll the trunk so the OTHER foot is clearly lower — for one tick (must NOT switch yet)
    roll_rad = 0.35 if settled_foot == "left" else -0.35
    q_roll = [math.cos(roll_rad / 2.0), math.sin(roll_rad / 2.0), 0.0, 0.0]
    s_bid = odom16.update(DEFAULT_POSITION, q_roll)
    assert s_bid["anchor_foot"] == settled_foot
    assert s_bid["switches"] == settled_switches

    # 2nd consecutive tick confirms switch (pending_ticks=2 -> switch!) without teleporting pose
    s_sw = odom16.update(DEFAULT_POSITION, q_roll)
    assert s_sw["switches"] == settled_switches + 1
    assert s_sw["anchor_foot"] != settled_foot
    assert math.isfinite(s_sw["position"][0]) and math.isfinite(s_sw["position"][1])
    assert abs(s_sw["position"][0] - s_bid["position"][0]) < 1e-6


def test_pillar2_remote_io_tcp_json_protocol():
    import socket
    from remote_io_bridge import PROTOCOL_VERSION, RemoteIoSessionHandler, RemoteIoTcpServer

    assert PROTOCOL_VERSION == 1

    # 1. Verify handshake gate rejects pre-handshake operations (Claude Endgegner directive)
    handler = RemoteIoSessionHandler()
    pre_err = json.loads(handler.handle_line('{"op":"read"}'))
    assert "error" in pre_err

    # 2. Verify protocol mismatch is flagged clearly
    bad_proto = json.loads(handler.handle_line('{"op":"hello","protocol":99,"joints":15}'))
    assert bad_proto["protocol"] == 1 and "error" in bad_proto
    assert handler.handshaken is False

    # 3. Verify live loopback TCP server (`TCP_NODELAY`, hello -> read -> write -> gain -> torque -> slow -> stateless reconnect)
    srv = RemoteIoTcpServer()
    srv.start()
    try:
        def rpc(sock_file, sock_obj, payload_dict):
            sock_obj.sendall((json.dumps(payload_dict) + "\n").encode("utf-8"))
            return json.loads(sock_file.readline())

        with socket.create_connection(srv.address, timeout=2.0) as s1:
            f1 = s1.makefile("r", encoding="utf-8", newline="\n")
            h = rpc(f1, s1, {"op": "hello", "protocol": 1, "joints": 15})
            assert h == {"protocol": 1}

            r_frame = rpc(f1, s1, {"op": "read"})
            assert len(r_frame["positions"]) == 15
            assert len(r_frame["velocities"]) == 15
            assert len(r_frame["currents_ma"]) == 15
            assert len(r_frame["imu"]["quat"]) == 4

            targets = list(DEFAULT_POSITION)
            targets[0] = 0.25
            w_ack = rpc(f1, s1, {"op": "write", "targets": targets})
            assert w_ack == {}

            g_ack = rpc(f1, s1, {"op": "gain", "kp": 180})
            assert g_ack == {}

            t_ack = rpc(f1, s1, {"op": "torque", "on": True})
            assert t_ack == {}

            slow = rpc(f1, s1, {"op": "slow"})
            assert 6.0 <= slow["volts"] <= 8.5
            assert len(slow["temps_c"]) == 15

        # Stateless reconnect on a new TCP socket after first connection closed
        with socket.create_connection(srv.address, timeout=2.0) as s2:
            f2 = s2.makefile("r", encoding="utf-8", newline="\n")
            h2 = rpc(f2, s2, {"op": "hello", "protocol": 1, "joints": 15})
            assert h2 == {"protocol": 1}
            r2 = rpc(f2, s2, {"op": "read"})
            assert math.isclose(r2["positions"][0], 0.25, abs_tol=0.005)
    finally:
        srv.stop()


def test_pillar3_xl330_fast_sync_read_and_slow_sensors():
    import struct
    from prehardware_subsystems import (
        RAD_PER_COUNT,
        RAD_PER_SEC_PER_COUNT,
        STALE_RUN_WARN,
        StaleImuTracker,
        Xl330BusSimulator,
        quantize_xl330_position_rad,
        quantize_xl330_velocity_rad_s,
    )

    # 1. Verify exact 0.229 rpm/count velocity quantization and 4096-count position quantization
    q_vel = quantize_xl330_velocity_rad_s(RAD_PER_SEC_PER_COUNT * 7.3)
    assert math.isclose(q_vel, RAD_PER_SEC_PER_COUNT * 7.0, abs_tol=1e-9)
    q_pos = quantize_xl330_position_rad(RAD_PER_COUNT * 12.4)
    assert math.isclose(q_pos, RAD_PER_COUNT * 12.0, abs_tol=1e-9)

    # 2. Verify 12-byte Fast Sync Read (`124..136`) and 3-byte Slow Sensors (`144..146`) binary packing
    bus = Xl330BusSimulator(nominal_volts=8.2, ambient_temp_c=31.0)
    stalled_targets = [p + 0.8 for p in DEFAULT_POSITION]
    for _ in range(50):
        snap = bus.step_bus(stalled_targets, DEFAULT_POSITION, [0.0] * 15, dt=0.02, kp_gain=200)
    assert snap["bus_volts"] < 8.2  # 2S Li-Ion voltage sag under stall current
    assert max(snap["temps_c"]) >= 31.0  # Joule coil heating

    fast_blk = bus.pack_fast_sync_read_block(3)
    assert len(fast_blk) == 12
    pwm, cur, vel_cnt, pos_cnt = struct.unpack("<hhii", fast_blk)
    assert -885 <= pwm <= 885 and cur > 0

    slow_blk = bus.pack_slow_sensors_block(3)
    assert len(slow_blk) == 3
    v_cnt, t_cnt = struct.unpack("<HB", slow_blk)
    assert 60 <= v_cnt <= 84 and t_cnt >= 31

    # 3. Verify StaleImuTracker (`STALE_RUN_WARN = 25` identical 12-byte SFLP blocks)
    tracker = StaleImuTracker()
    sample_block = b"\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0a\x0b\x0c"
    assert tracker.observe(sample_block) == 0
    for _ in range(STALE_RUN_WARN):
        tracker.observe(sample_block)
    assert tracker.is_frozen_warn is True
    assert tracker.observe(b"\x00" * 12) == 0
    assert tracker.is_frozen_warn is False


def test_pillar4_maploc_stop_and_vote_and_pet_detect_log_mel():
    from prehardware_subsystems import (
        LOG_EPS,
        N_MELS,
        WINDOW_FRAMES,
        WINDOW_SAMPLES,
        LogMelPettingDetector,
        StopAndVoteMapper,
        build_mel_filterbank,
    )

    # 1. Verify Stop-and-Vote mapper ignores scans while moving and locks keyframe only during stops
    mapper = StopAndVoteMapper()
    scan64 = [0.45] * 64
    for _ in range(20):
        m_mov = mapper.observe_tick([0.1, 0.0], 0.0, speed_m_s=0.25, gyro_norm_rad_s=0.3, tof_zones64_m=scan64)
        assert m_mov["committed_keyframe"] is False
    assert mapper.keyframes == []

    for _ in range(15):
        mapper.observe_tick([0.2, 0.0], 0.0, speed_m_s=0.0, gyro_norm_rad_s=0.01, tof_zones64_m=scan64)
    assert len(mapper.keyframes) == 1
    assert mapper.keyframes[0]["valid_zones"] == 64

    # 2. Verify pinned `pet-detect/src/lib.rs` log-mel contract (`16,240 samples`, `40 x 100 = 4,000 floats`)
    assert WINDOW_SAMPLES == 16_240
    assert N_MELS * WINDOW_FRAMES == 4_000
    filters = build_mel_filterbank()
    assert len(filters) == 40 and all(len(f) > 0 for f in filters)

    det = LogMelPettingDetector()
    silence = [0.0] * WINDOW_SAMPLES
    mel_quiet = det.extract_log_mel(silence)
    assert len(mel_quiet) == 4_000
    assert all(abs(v - math.log(LOG_EPS)) < 1e-3 for v in mel_quiet)

    # SoundSentry gate skips quiet windows when not petting
    res_gated = det.evaluate_window(silence, audible=False)
    assert res_gated["skipped_by_sentry"] is True and res_gated["is_petting"] is False


def test_pillar5_pad_imu_complementary_filter_and_dead_radio_trap():
    from prehardware_subsystems import CLONE_UNINITIALIZED_Z_DRIFT_DPS, PadImuFilter

    pad = PadImuFilter()
    # 1. Feed 0.6s of resting Switch Pro clone IMU samples with +12 deg/s uncalibrated Z gyro bias
    for step in range(35):
        out = pad.push_sample(
            accel_g=(0.0, 0.0, 1.0),
            raw_gyro_dps=(0.1, -0.1, CLONE_UNINITIALIZED_Z_DRIFT_DPS),
            at_s=step * 0.02,
            evdev_seq=step + 1,
        )
    assert out["calibrated"] is True
    assert math.isclose(out["bias_dps"][2], CLONE_UNINITIALIZED_Z_DRIFT_DPS, abs_tol=0.2)
    assert out["dead_radio_trap_detected"] is False

    # 2. Freeze evdev_seq for 14 ticks (280 ms > 250 ms robotd deadman) to trigger dead-radio stale stream trap
    frozen_seq = 100
    for step in range(15):
        out_stale = pad.push_sample(
            accel_g=(0.0, 0.0, 1.0),
            raw_gyro_dps=(0.0, 0.0, CLONE_UNINITIALIZED_Z_DRIFT_DPS),
            at_s=1.0 + step * 0.02,
            evdev_seq=frozen_seq,
        )
    assert out_stale["dead_radio_trap_detected"] is True


def test_hf_visual_and_inline_roller_upgrades():
    sim_js = (PROJECT_ROOT / "web" / "simulator.js").read_text(encoding="utf-8")
    index_html = (PROJECT_ROOT / "web" / "index.html").read_text(encoding="utf-8")
    fetch_sh = (PROJECT_ROOT / "scripts" / "fetch_upstream_assets.sh").read_text(encoding="utf-8")

    # 1. 36-deg creased vertex normals + 16-slot PBR material table + physical jaw hinge
    assert "computeCreasedGeometry" in sim_js
    assert "applyCreasedNormalsToBufferGeometry" in sim_js
    assert "highResGlbGeometries" in sim_js
    assert "PART_SLOTS" in sim_js
    assert "setupPhysicalJawPivot" in sim_js

    # 2. Authentic inline roller-blade assembly & Crouch Glide skill (`roller_crouch`)
    for token in (
        "ankle_l_v1.stl",
        "ankle_r_v1.stl",
        "roller_blade.stl",
        "passive_LF_wheel",
        "passive_LR_wheel",
        "passive_RF_wheel",
        "passive_RR_wheel",
        "roller_crouch",
        "RVEL_FWD = 0.60",
    ):
        assert token in sim_js
    assert 'data-skill="roller_crouch"' in index_html

    # 3. Upstream asset fetch bootstrap includes Apache-2.0 HF simulator assets
    assert "pollen-robotics/microduck-simulator" in fetch_sh
    assert "BEST_roller_crouch.onnx" in fetch_sh


