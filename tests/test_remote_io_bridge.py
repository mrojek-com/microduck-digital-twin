"""
apps/MicroDuck/tests/test_remote_io_bridge.py

Integration & protocol tests for the clean-room `RemoteIo` TCP JSON Line bridge
(`src/remote_io_bridge.py`, PROTOCOL = 1 matching `duck-control/src/sim.rs`).
"""

import json
import socket
import sys
from pathlib import Path

SRC_DIR = Path(__file__).resolve().parent.parent / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from models import DEFAULT_POSITION, NUM_JOINTS
from remote_io_bridge import PROTOCOL_VERSION, RemoteIoSessionHandler, RemoteIoTcpServer


def _tcp_exchange(host: str, port: int, payloads: list[dict]) -> list[dict]:
    replies = []
    with socket.create_connection((host, port), timeout=2.0) as sock:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        reader = sock.makefile("r", encoding="utf-8", newline="\n")
        for req in payloads:
            sock.sendall((json.dumps(req) + "\n").encode("utf-8"))
            line = reader.readline()
            assert line, f"EOF waiting for reply to {req}"
            replies.append(json.loads(line))
    return replies


def test_handshake_required_before_bus_ops() -> None:
    session = RemoteIoSessionHandler()
    r_read = json.loads(session.handle_line(json.dumps({"op": "read"})))
    assert "error" in r_read
    assert "handshake" in r_read["error"].lower()

    r_bad_proto = json.loads(
        session.handle_line(json.dumps({"op": "hello", "protocol": 99, "joints": NUM_JOINTS}))
    )
    assert r_bad_proto["protocol"] == PROTOCOL_VERSION
    assert "error" in r_bad_proto
    assert not session.handshaken

    r_bad_joints = json.loads(
        session.handle_line(json.dumps({"op": "hello", "protocol": PROTOCOL_VERSION, "joints": 12}))
    )
    assert "error" in r_bad_joints
    assert not session.handshaken


def test_tcp_loopback_full_protocol_roundtrip() -> None:
    server = RemoteIoTcpServer(host="127.0.0.1", port=0)
    server.start()
    try:
        host, port = server.address
        target = list(DEFAULT_POSITION)
        target[5] = -0.5  # neck_pitch

        replies = _tcp_exchange(
            host,
            port,
            [
                {"op": "hello", "protocol": PROTOCOL_VERSION, "joints": NUM_JOINTS},
                {"op": "gain", "kp": 640},
                {"op": "torque", "on": True},
                {"op": "write", "targets": target},
                {"op": "read"},
                {"op": "slow"},
            ],
        )

        assert replies[0] == {"protocol": PROTOCOL_VERSION}
        assert replies[1] == {}
        assert replies[2] == {}
        assert replies[3] == {}

        read_frame = replies[4]
        assert len(read_frame["positions"]) == NUM_JOINTS
        assert len(read_frame["velocities"]) == NUM_JOINTS
        assert len(read_frame["currents_ma"]) == NUM_JOINTS
        assert len(read_frame["imu"]["gyro"]) == 3
        assert len(read_frame["imu"]["gravity"]) == 3
        assert len(read_frame["imu"]["quat"]) == 4
        assert abs(read_frame["positions"][5] - (-0.5)) < 0.01

        slow_frame = replies[5]
        assert 6.0 <= slow_frame["volts"] <= 9.0
        assert len(slow_frame["temps_c"]) == NUM_JOINTS
    finally:
        server.stop()


def test_write_rejects_nan_and_wrong_length() -> None:
    session = RemoteIoSessionHandler()
    assert json.loads(
        session.handle_line(json.dumps({"op": "hello", "protocol": PROTOCOL_VERSION, "joints": NUM_JOINTS}))
    ) == {"protocol": PROTOCOL_VERSION}

    r_short = json.loads(session.handle_line(json.dumps({"op": "write", "targets": [0.0] * 10})))
    assert "error" in r_short

    # Raw JSON with NaN / non-numeric
    r_bad_val = json.loads(session.handle_line('{"op": "write", "targets": ["bad",' + ",".join(["0.0"] * 14) + "]}"))
    assert "error" in r_bad_val

    r_bad_gain = json.loads(session.handle_line(json.dumps({"op": "gain", "kp": -5})))
    assert "error" in r_bad_gain

    r_bad_torque = json.loads(session.handle_line(json.dumps({"op": "torque", "on": "yes"})))
    assert "error" in r_bad_torque
