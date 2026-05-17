from __future__ import annotations

import csv
import time
from datetime import datetime
from pathlib import Path

from .tracking import TrackingResult


class DataLogger:
    """Records motor positions + vision data to a CSV file each frame."""

    HEADER = [
        "time_ms",
        "base", "shoulder", "elbow", "palm", "wrist", "gripper",
        "obj_x", "obj_y", "obj_area", "cell",
        "err_x", "err_y",
        "approach_mode", "free_mode",
    ]

    def __init__(self, log_dir: Path):
        self.log_dir = Path(log_dir)
        self._file = None
        self._writer = None
        self.active = False
        self.path: Path | None = None

    def start(self) -> None:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = self.log_dir / f"robot_log_{ts}.csv"
        self._file = open(self.path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.HEADER)
        self.active = True
        print(f"[LOG] Recording → {self.path}")

    def record(self, state, result: TrackingResult,
               frame_w: int, frame_h: int, free_mode: bool) -> None:
        if not self.active or self._writer is None:
            return
        t = int(time.time() * 1000)
        c = state.curr
        if result.success:
            err_x = round((frame_w * 0.5 - result.center_x) / frame_w, 4)
            err_y = round((frame_h * 0.5 - result.center_y) / frame_h, 4)
            col = min(int(result.center_x * 3 / frame_w), 2)
            row = min(int(result.center_y * 3 / frame_h), 2)
            cell = row * 3 + col + 1
            self._writer.writerow([
                t,
                c["base"], c["shoulder"], c["elbow"],
                c["palm"], c["wrist"], c["gripper"],
                result.center_x, result.center_y, result.area, cell,
                err_x, err_y,
                int(state.approach_mode), int(free_mode),
            ])
        else:
            self._writer.writerow([
                t,
                c["base"], c["shoulder"], c["elbow"],
                c["palm"], c["wrist"], c["gripper"],
                "", "", "", "", "", "",
                int(state.approach_mode), int(free_mode),
            ])

    def stop(self) -> None:
        self.active = False
        if self._file is not None:
            self._file.flush()
            self._file.close()
            self._file = None
            self._writer = None
        print(f"[LOG] Stopped. File saved → {self.path}")
