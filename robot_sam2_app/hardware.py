from __future__ import annotations

import json
import time
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


def home_ticks_to_state(home: dict[int, int]) -> dict[str, int]:
    return {
        "base": home.get(1, 2048),
        "shoulder": home.get(2, 2048),
        "elbow": home.get(3, 2048),
        "palm": home.get(4, 2048),
        "wrist": home.get(5, GRIPPER_ROT_90_POS),
        "gripper": home.get(6, 3000),
    }

