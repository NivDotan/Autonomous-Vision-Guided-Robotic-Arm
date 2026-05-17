"""
Camera-to-robot-base coordinate transform.

After hand-eye calibration, load the result JSON.
Until then, the identity transform is used (camera coincident with base).
"""
from __future__ import annotations

import json
import numpy as np


class CoordTransform:
    """
    Converts 3D points from camera frame to robot base frame.

    The transform is R * p + t where (R, t) come from a hand-eye calibration.
    Without a calibration file the identity is used, which is a useful
    placeholder until the physical camera is mounted and calibrated.

    Calibration file format (JSON):
      {
        "R": [[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]],
        "t": [tx, ty, tz]
      }
    """

    def __init__(self, calib_path: str | None = None):
        self._R = np.eye(3, dtype=float)
        self._t = np.zeros(3, dtype=float)
        if calib_path:
            self._load(calib_path)

    def camera_to_base(
        self,
        p_cam: tuple[float, float, float] | np.ndarray,
    ) -> tuple[float, float, float]:
        """Transform a 3D point from camera frame to robot base frame."""
        p = np.asarray(p_cam, dtype=float)
        p_base = self._R @ p + self._t
        return float(p_base[0]), float(p_base[1]), float(p_base[2])

    def base_to_camera(
        self,
        p_base: tuple[float, float, float] | np.ndarray,
    ) -> tuple[float, float, float]:
        """Inverse transform: robot base frame → camera frame."""
        p = np.asarray(p_base, dtype=float)
        p_cam = self._R.T @ (p - self._t)
        return float(p_cam[0]), float(p_cam[1]), float(p_cam[2])

    def save(self, path: str) -> None:
        data = {"R": self._R.tolist(), "t": self._t.tolist()}
        with open(path, "w") as f:
            json.dump(data, f, indent=2)

    def _load(self, path: str) -> None:
        try:
            with open(path) as f:
                data = json.load(f)
            self._R = np.array(data["R"], dtype=float)
            self._t = np.array(data["t"], dtype=float)
        except Exception as e:
            print(f"[CoordTransform] Could not load calibration from {path}: {e}")
            self._R = np.eye(3)
            self._t = np.zeros(3)
