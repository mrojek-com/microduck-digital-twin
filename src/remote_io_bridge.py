"""
apps/MicroDuck/src/remote_io_bridge.py

Clean-room implementation of the `RemoteIo` TCP JSON Line simulator wire protocol
referenced from `pollen-robotics/microduck` (`duck-control/src/sim.rs`, PROTOCOL = 1).

Why this exists:
The upstream Rust `robotd` daemon includes a native external simulator backend (`RemoteIo`)
that connects over TCP at 50 Hz (~50 KB/s) using newline-delimited JSON. By speaking this
exact wire contract (`PROTOCOL = 1`), developers waiting for physical MicroDuck hardware
can launch the real upstream Rust `robotd` and `robotctl` binaries against our Python /
WebGL Digital Twin with zero code changes to `robotd`.

Wire Protocol Contract (`duck-control/src/sim.rs`):
- Transport: TCP stream with `TCP_NODELAY = 1` (Nagle disabled to avoid 40 ms tick delay).
- Framing: One UTF-8 JSON object per line (`\\n`), stateless reconnect on any framing/version error.
- Requests (`{"op": ...}`):
  1. `{"op": "hello", "protocol": 1, "joints": 15}`
     -> Reply: `{"protocol": 1}` (or error if protocol/joints mismatch)
  2. `{"op": "read"}`
     -> Reply: `{"positions": [f64; 15], "velocities": [f64; 15], "currents_ma": [f64; 15], "imu": {"gyro": [f64; 3], "gravity": [f64; 3], "quat": [f64; 4]}}`
  3. `{"op": "write", "targets": [f64; 15]}`
     -> Reply: `{}` on success or `{"error": "<reason>"}` on non-finite / wrong length / untorqued
  4. `{"op": "gain", "kp": u16}`
     -> Reply: `{}` or `{"error": "<reason>"}`
  5. `{"op": "torque", "on": bool}`
     -> Reply: `{}`
  6. `{"op": "slow"}`
     -> Reply: `{"volts": f64, "temps_c": [f64; 15]}`
"""

from __future__ import annotations

import json
import math
import socket
import threading
from typing import Dict, List, Optional, Tuple

from models import CONTROL_DT, DEFAULT_POSITION, GAIN_RUNNING, NUM_JOINTS
from prehardware_subsystems import Xl330BusSimulator
from sim_engine import MicroDuckSimulator

PROTOCOL_VERSION: int = 1


def _rpy_to_quat_wxyz(roll: float, pitch: float, yaw: float) -> List[float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return [round(w, 6), round(x, 6), round(y, 6), round(z, 6)]


class RemoteIoSessionHandler:
    """
    Stateful per-connection handler for `duck-control/src/sim.rs::RemoteIo`.
    Requires a valid `hello` handshake before `read`/`write`/`gain`/`torque`/`slow` are accepted
    (per Claude Endgegner directive).
    """

    def __init__(
        self,
        sim: Optional[MicroDuckSimulator] = None,
        bus: Optional[Xl330BusSimulator] = None,
    ) -> None:
        self.sim = sim or MicroDuckSimulator()
        self.bus = bus or Xl330BusSimulator()
        self.handshaken: bool = False
        self.torque_enabled: bool = True
        self.kp_gain: int = GAIN_RUNNING
        self.last_targets: List[float] = list(DEFAULT_POSITION)

    def handle_line(self, raw_line: str) -> str:
        """Process a single JSON request line and return a JSON response line (without trailing newline)."""
        line = raw_line.strip()
        if not line:
            return json.dumps({"error": "empty request line"})
        try:
            req = json.loads(line)
        except Exception as exc:
            return json.dumps({"error": f"invalid json: {exc}"})

        if not isinstance(req, dict) or "op" not in req:
            return json.dumps({"error": "missing 'op' tag"})

        op = req.get("op")
        if op == "hello":
            peer_proto = req.get("protocol")
            peer_joints = req.get("joints", NUM_JOINTS)
            # Always answer with our protocol version so `RemoteIo::connect` can report version mismatches cleanly,
            # and only mark session handshaken when both protocol==1 and joints==15 match.
            if peer_proto == PROTOCOL_VERSION and peer_joints == NUM_JOINTS:
                self.handshaken = True
                return json.dumps({"protocol": PROTOCOL_VERSION})
            self.handshaken = False
            if peer_proto != PROTOCOL_VERSION:
                return json.dumps({"protocol": PROTOCOL_VERSION, "error": f"expected protocol {PROTOCOL_VERSION}, got {peer_proto}"})
            return json.dumps({"protocol": 0, "error": f"expected {NUM_JOINTS} joints, got {peer_joints}"})

        if not self.handshaken:
            return json.dumps({"error": "handshake required before bus operations (send op='hello' first)"})

        if op == "read":
            st = self.sim.state
            bus_snap = self.bus.step_bus(
                target_positions15=self.last_targets if self.torque_enabled else st.joints,
                actual_positions15=st.joints,
                actual_velocities15=st.velocities,
                dt=CONTROL_DT,
                kp_gain=self.kp_gain if self.torque_enabled else 0,
            )
            # Compute trunk roll/pitch from gravity vector
            gx, gy, gz = st.gravity
            roll = math.atan2(-gy, -gz)
            pitch = math.atan2(gx, math.sqrt(gy * gy + gz * gz))
            quat = _rpy_to_quat_wxyz(roll, pitch, st.yaw_rad)
            frame = {
                "positions": [round(float(x), 6) for x in bus_snap["quantized_positions"]],
                "velocities": [round(float(x), 6) for x in bus_snap["quantized_velocities"]],
                "currents_ma": [round(float(x), 2) for x in bus_snap["currents_ma"]],
                "imu": {
                    "gyro": [round(float(x), 6) for x in st.gyro],
                    "gravity": [round(float(x), 6) for x in st.gravity],
                    "quat": quat,
                },
            }
            return json.dumps(frame)

        if op == "write":
            targets = req.get("targets")
            if not isinstance(targets, list) or len(targets) != NUM_JOINTS:
                return json.dumps({"error": f"expected {NUM_JOINTS} joint targets"})
            for val in targets:
                if not isinstance(val, (int, float)) or not math.isfinite(float(val)):
                    return json.dumps({"error": "non-finite joint target rejected"})
            clamped = self.sim.validate_and_clamp_targets([float(x) for x in targets])
            self.last_targets = list(clamped)
            if self.torque_enabled:
                st = self.sim.state
                st.time_s += CONTROL_DT
                for i in range(NUM_JOINTS):
                    st.velocities[i] = (clamped[i] - st.joints[i]) / CONTROL_DT
                    st.joints[i] = clamped[i]
                self.bus.step_bus(
                    target_positions15=clamped,
                    actual_positions15=st.joints,
                    actual_velocities15=st.velocities,
                    dt=CONTROL_DT,
                    kp_gain=self.kp_gain,
                )
                st.currents_ma = list(self.bus.currents_ma)
            return json.dumps({})

        if op == "gain":
            kp = req.get("kp")
            if not isinstance(kp, int) or kp < 0 or kp > 16383:
                return json.dumps({"error": f"invalid kp gain: {kp}"})
            self.kp_gain = kp
            self.sim.state.safety.gain = kp
            return json.dumps({})

        if op == "torque":
            on = req.get("on")
            if not isinstance(on, bool):
                return json.dumps({"error": f"invalid torque flag: {on}"})
            self.torque_enabled = on
            return json.dumps({})

        if op == "slow":
            return json.dumps(
                {
                    "volts": float(self.bus.bus_volts),
                    "temps_c": [float(round(t, 1)) for t in self.bus.temps_c],
                }
            )

        return json.dumps({"error": f"unknown op: {op}"})


class RemoteIoTcpServer:
    """
    Loopback TCP server speaking `duck-control/src/sim.rs::RemoteIo` (`PROTOCOL = 1`).
    Supports stateless per-tick reconnects and sets `TCP_NODELAY = 1`.
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 0,
        sim: Optional[MicroDuckSimulator] = None,
        bus: Optional[Xl330BusSimulator] = None,
    ) -> None:
        self.sim = sim or MicroDuckSimulator()
        self.bus = bus or Xl330BusSimulator()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, port))
        self._sock.listen(4)
        self.address: Tuple[str, int] = self._sock.getsockname()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    @property
    def addr_str(self) -> str:
        return f"{self.address[0]}:{self.address[1]}"

    def start(self) -> None:
        self._thread = threading.Thread(target=self._serve_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        try:
            self._sock.close()
        except Exception:
            pass
        if self._thread is not None:
            self._thread.join(timeout=1.5)

    def _serve_loop(self) -> None:
        self._sock.settimeout(0.2)
        while not self._stop_event.is_set():
            try:
                conn, _ = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            self._handle_client(conn)

    def _handle_client(self, conn: socket.socket) -> None:
        with conn:
            try:
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                conn.settimeout(1.0)
                session = RemoteIoSessionHandler(sim=self.sim, bus=self.bus)
                reader = conn.makefile("r", encoding="utf-8", newline="\n")
                for line in reader:
                    if self._stop_event.is_set():
                        break
                    reply = session.handle_line(line)
                    conn.sendall((reply + "\n").encode("utf-8"))
                    # As in `duck-control/src/sim.rs`, if an error is returned before handshake, close link so peer reconnects cleanly
                    if not session.handshaken and '"error"' in reply:
                        break
            except Exception:
                pass
