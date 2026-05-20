from __future__ import annotations

import csv
import time
from datetime import datetime
from pathlib import Path

from .tracking import TrackingResult

LOG_INTERVAL = 0.5  # seconds between rows

HEADER = [
    # Time
    "timestamp", "time_ms",
    # Motor — current (actual hardware position)
    "cur_base", "cur_shoulder", "cur_elbow", "cur_palm", "cur_wrist", "cur_gripper",
    # Motor — target (commanded position)
    "tgt_base", "tgt_shoulder", "tgt_elbow", "tgt_palm", "tgt_wrist", "tgt_gripper",
    # Vision / object tracking
    "tracking_mode", "obj_tracked",
    "obj_x", "obj_y", "obj_area", "obj_w", "obj_h",
    "err_x", "err_y",
    # Sensors
    "vl53_dist_mm", "gripper_load", "gripper_current",
    # State machine flags
    "motors_enabled", "approach_mode", "arm_locked", "pre_grasp_palm",
    "gripper_closed", "returning_home", "retreat_mode", "is_frozen", "is_centered",
    # Grip retry counters
    "grip_attempt", "gripper_closed_frames",
    # Mode
    "free_mode",
]


class DataLogger:
    """Records full robot state to CSV at a fixed interval (default 0.5 s).

    Starts automatically on setup(). Press L to stop/restart mid-session.
    Each run writes a new timestamped file under log_dir/.
    """

    def __init__(self, log_dir: Path, hardware=None, interval: float = LOG_INTERVAL):
        self.log_dir = Path(log_dir)
        self._hardware = hardware
        self._interval = interval
        self._file = None
        self._writer = None
        self.active = False
        self.path: Path | None = None
        self._last_write: float = 0.0

    def start(self) -> None:
        self.log_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = self.log_dir / f"robot_log_{ts}.csv"
        self._file = open(self.path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(HEADER)
        self._last_write = 0.0
        self.active = True
        print(f"[LOG] Recording → {self.path}")

    def record(self, state, result: TrackingResult,
               frame_w: int, frame_h: int, free_mode: bool,
               vl53_dist_mm: int | None = None) -> None:
        if not self.active or self._writer is None:
            return
        now = time.monotonic()
        if now - self._last_write < self._interval:
            return
        self._last_write = now

        # Read gripper load + current in one ZMQ call (2 Hz)
        gripper_load, gripper_current = None, None
        if self._hardware is not None and hasattr(self._hardware, "read_gripper_state"):
            gripper_load, gripper_current = self._hardware.read_gripper_state()

        c = state.curr
        t = state.target
        ts_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        time_ms = int(time.time() * 1000)

        # Vision fields
        if result.success and frame_w > 0 and frame_h > 0:
            aim_x = state.current_aim_x if (state.approach_mode and state.current_aim_x is not None) else 0.5
            aim_y = state.current_aim_y if (state.approach_mode and state.current_aim_y is not None) else 0.5
            err_x = round((frame_w * aim_x - result.center_x) / frame_w, 4)
            err_y = round((frame_h * aim_y - result.center_y) / frame_h, 4)
            vision = [
                1, result.center_x, result.center_y,
                result.area, result.width, result.height,
                err_x, err_y,
            ]
        else:
            vision = [0, "", "", "", "", "", "", ""]

        row = [
            ts_str, time_ms,
            c["base"], c["shoulder"], c["elbow"], c["palm"], c["wrist"], c["gripper"],
            t["base"], t["shoulder"], t["elbow"], t["palm"], t["wrist"], t["gripper"],
            state.tracking_mode, *vision,
            vl53_dist_mm, gripper_load, gripper_current,
            int(state.motors_enabled),
            int(state.approach_mode),
            int(state.arm_locked),
            int(state.pre_grasp_palm),
            int(state.gripper_closed),
            int(state.returning_home),
            int(state.retreat_mode),
            int(state.is_frozen),
            int(state.is_centered),
            state.grip_attempt,
            state.gripper_closed_frames,
            int(free_mode),
        ]
        self._writer.writerow(row)
        self._file.flush()

    def stop(self) -> None:
        self.active = False
        if self._file is not None:
            self._file.flush()
            self._file.close()
            self._file = None
            self._writer = None
        print(f"[LOG] Stopped → {self.path}")
