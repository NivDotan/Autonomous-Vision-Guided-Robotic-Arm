from __future__ import annotations

import json
import time
from collections import deque
from pathlib import Path

from .config import (
    GRIP_LOAD_THRESHOLD,
    GRIPPER_ROT_90_POS,
    MOTOR_IDS,
    MOTOR_NAMES,
    PORT,
)


class SimpleMotor:
    def __init__(self, motor_id: int, model: str = "sts3215"):
        self.id = motor_id
        self.model = model


class FeetechHardware:
    """Small wrapper around LeRobot's Feetech bus.

    The app can run without this class connecting. In that case camera, SAM2,
    RF-DETR, and PyBullet sim still work.
    """

    def __init__(self, port: str = PORT):
        self.port = port
        self.bus = None

    @property
    def connected(self) -> bool:
        return self.bus is not None

    def connect(self) -> bool:
        try:
            from lerobot.motors.feetech import FeetechMotorsBus

            motors = {f"motor_{i}": SimpleMotor(i) for i in MOTOR_IDS}
            self.bus = FeetechMotorsBus(port=self.port, motors=motors)
            self.bus.connect()
            print("Hardware connected.")
            return True
        except Exception as exc:
            self.bus = None
            print(f"Hardware unavailable: {exc}")
            return False

    def disconnect(self) -> None:
        if self.bus is not None:
            self.bus.disconnect()
            self.bus = None

    def read_ticks(self) -> dict[str, int] | None:
        if self.bus is None:
            return None
        try:
            return {
                name: int(self.bus.read("Present_Position", f"motor_{motor_id}", normalize=False))
                for name, motor_id in zip(MOTOR_NAMES, MOTOR_IDS)
            }
        except Exception:
            return None

    def write_ticks(self, ticks: dict[str, int]) -> None:
        if self.bus is None:
            return
        try:
            for name, motor_id in zip(MOTOR_NAMES, MOTOR_IDS):
                self.bus.write("Goal_Position", f"motor_{motor_id}", int(ticks[name]), normalize=False)
        except Exception:
            pass

    def load_home(self, path: Path) -> dict[int, int]:
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            return {int(k): int(v) for k, v in data.items()}
        except Exception:
            return {}

    def write_home(self, home: dict[int, int]) -> None:
        if self.bus is None:
            return
        for motor_id, tick in home.items():
            self.bus.write("Goal_Position", f"motor_{motor_id}", int(tick), normalize=False)

    def write_goal_velocity(self, motor_name: str, velocity: int) -> None:
        if self.bus is None:
            return
        try:
            idx = list(MOTOR_NAMES).index(motor_name)
            motor_id = MOTOR_IDS[idx]
            self.bus.write("Goal_Velocity", f"motor_{motor_id}", int(velocity), normalize=False)
        except Exception:
            pass

    def set_torque(self, enabled: bool, motor_ids: list[int] | None = None) -> None:
        if self.bus is None:
            return
        ids = motor_ids if motor_ids is not None else list(MOTOR_IDS)
        val = 1 if enabled else 0
        for mid in ids:
            try:
                self.bus.write("Torque_Enable", f"motor_{mid}", val, normalize=False)
            except Exception as exc:
                print(f"[HW] set_torque motor_{mid} failed: {exc}")

    def read_gripper_load(self) -> int | None:
        if self.bus is None:
            return None
        try:
            raw = abs(int(self.bus.read("Present_Load", "motor_6", normalize=False)))
            return raw - 1024 if raw > 1024 else raw
        except Exception:
            return None

    def read_gripper_current(self) -> int | None:
        if self.bus is None:
            return None
        try:
            return int(self.bus.read("Present_Current", "motor_6", normalize=False))
        except Exception:
            return None

    def read_gripper_state(self) -> tuple[int | None, int | None]:
        return self.read_gripper_load(), self.read_gripper_current()

    def gripper_load_detected(self) -> bool:
        if self.bus is None:
            return False
        try:
            raw = abs(int(self.bus.read("Present_Load", "motor_6", normalize=False)))
            load = raw - 1024 if raw > 1024 else raw
            if load > GRIP_LOAD_THRESHOLD:
                time.sleep(1)
                return True
        except Exception:
            return False
        return False


# ── DaemonHardware — drop-in replacement using C++ motor_daemon ─────────────

class DaemonHardware:
    """
    Drop-in replacement for FeetechHardware that communicates with the
    C++ motor_daemon process over ZeroMQ + MessagePack.

    The daemon must be running before connect() is called:
        motor_daemon.exe --port COM4 --zmq-port 5555
    """

    def __init__(self, endpoint: str = "tcp://localhost:5555"):
        self._endpoint = endpoint
        self._socket   = None
        self._ctx      = None
        self._current_buffer: deque[int] = deque(maxlen=10)

    @property
    def connected(self) -> bool:
        return self._socket is not None

    def connect(self) -> bool:
        try:
            import zmq
            import msgpack
            self._zmq     = zmq
            self._msgpack = msgpack
            self._ctx    = zmq.Context()
            self._socket = self._ctx.socket(zmq.REQ)
            self._socket.setsockopt(zmq.RCVTIMEO, 100)   # 100ms timeout
            self._socket.setsockopt(zmq.LINGER, 0)
            self._socket.connect(self._endpoint)
            # Ping with STATUS command.
            self._socket.send(msgpack.packb({"cmd": 0xFF}))
            resp = msgpack.unpackb(self._socket.recv(), raw=False)
            if resp.get("status", resp.get(b"status", 1)) != 0:
                raise RuntimeError("daemon STATUS response not OK")
            print(f"DaemonHardware connected to {self._endpoint}")
            return True
        except Exception as exc:
            print(f"DaemonHardware unavailable: {exc}")
            self._socket = None
            self._ctx    = None
            return False

    def disconnect(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._ctx.term()
            self._socket = None
            self._ctx    = None

    def read_ticks(self) -> dict[str, int] | None:
        if self._socket is None:
            return None
        try:
            self._socket.send(self._msgpack.packb({"cmd": 0x02}))
            resp = self._msgpack.unpackb(self._socket.recv(), raw=False)
            ticks_raw = resp.get("ticks") or resp.get(b"ticks")
            if ticks_raw is None:
                return None
            return {name: int(ticks_raw[i]) for i, name in enumerate(MOTOR_NAMES)}
        except Exception:
            return None

    def write_ticks(self, ticks: dict[str, int]) -> None:
        if self._socket is None:
            return
        try:
            payload = [int(ticks.get(n, 2048)) for n in MOTOR_NAMES]
            self._socket.send(self._msgpack.packb({"cmd": 0x01, "ticks": payload}))
            self._socket.recv()  # consume ACK
        except Exception:
            pass

    def load_home(self, path) -> dict[int, int]:
        # Delegate to FeetechHardware-style JSON loading.
        try:
            import json
            from pathlib import Path
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            return {int(k): int(v) for k, v in data.items()}
        except Exception:
            return {}

    def write_home(self, home: dict[int, int]) -> None:
        ticks = home_ticks_to_state(home)
        self.write_ticks(ticks)

    def write_goal_velocity(self, motor_name: str, velocity: int) -> None:
        pass  # daemon protocol doesn't expose Goal_Velocity — silently ignored

    def set_torque(self, enabled: bool, motor_ids: list[int] | None = None) -> None:
        print(f"[HW] set_torque({'ON' if enabled else 'OFF'}) — daemon mode: stop sending commands to free motors")

    def reset_gripper_current_buffer(self) -> None:
        self._current_buffer.clear()

    def _call_cmd03(self) -> dict | None:
        """One ZMQ call for cmd 0x03 — returns full response or None on error."""
        if self._socket is None:
            return None
        try:
            self._socket.send(self._msgpack.packb({"cmd": 0x03}))
            return self._msgpack.unpackb(self._socket.recv(), raw=False)
        except Exception:
            return None

    def read_gripper_state(self) -> tuple[int | None, int | None]:
        """Returns (load, current) in one ZMQ call."""
        resp = self._call_cmd03()
        if resp is None:
            return None, None
        load = resp.get("load", resp.get(b"load", None))
        curr = resp.get("current", resp.get(b"current", None))
        return (int(load) if load is not None else None,
                int(curr) if curr is not None else None)

    def read_gripper_load(self) -> int | None:
        load, _ = self.read_gripper_state()
        return load

    def read_gripper_current(self) -> int | None:
        _, curr = self.read_gripper_state()
        return curr

    def gripper_load_detected(self) -> bool:
        from . import config as cfg
        resp = self._call_cmd03()
        if resp is None:
            return False
        raw_curr = resp.get("current", resp.get(b"current", None))
        if raw_curr is not None:
            # Current-based detection: stable AND above threshold
            current = int(raw_curr)
            self._current_buffer.append(current)
            if len(self._current_buffer) < cfg.CURRENT_STABLE_COUNT:
                return False
            last_n = list(self._current_buffer)[-cfg.CURRENT_STABLE_COUNT:]
            stable = (max(last_n) - min(last_n)) <= cfg.CURRENT_STABLE_WINDOW
            above  = min(last_n) > cfg.CURRENT_GRIP_THRESHOLD
            return stable and above
        # Fallback if daemon hasn't been rebuilt yet: use raw load
        raw_load = resp.get("load", resp.get(b"load", None))
        if raw_load is not None:
            return int(raw_load) > cfg.GRIP_LOAD_THRESHOLD
        return bool(resp.get("detected", False))


def make_hardware(use_daemon: bool = False, **kwargs):
    """Factory — returns DaemonHardware or FeetechHardware based on config."""
    if use_daemon:
        from . import config as cfg
        endpoint = kwargs.pop("endpoint", cfg.DAEMON_ENDPOINT)
        return DaemonHardware(endpoint=endpoint, **kwargs)
    port = kwargs.pop("port", None)
    from . import config as cfg
    return FeetechHardware(port=port or cfg.PORT)


def home_ticks_to_state(home: dict[int, int]) -> dict[str, int]:
    return {
        "base": home.get(1, 2048),
        "shoulder": home.get(2, 2048),
        "elbow": home.get(3, 2048),
        "palm": home.get(4, 2048),
        "wrist": home.get(5, GRIPPER_ROT_90_POS),
        "gripper": home.get(6, 3000),
    }

