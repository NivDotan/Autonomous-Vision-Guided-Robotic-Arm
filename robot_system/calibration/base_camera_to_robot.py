"""
Step 12b — Base-camera to robot-frame calibration.

Maps pixel coordinates (u, v) from a static overhead camera to robot-frame
Cartesian coordinates (x, y) on the table plane (z = table_z).

Algorithm: planar homography (4+ point correspondences).
  1. Collect N calibration pairs: click pixel (u,v) <-> robot position (x,y).
  2. cv2.findHomography (or numpy fallback) computes the 3x3 H matrix.
  3. pixel_to_robot(u, v) applies H to get (x, y, table_z).

Persistence: save/load as JSON so calibration survives restarts.

Usage (dry-run, no camera or robot):
    from calibration.base_camera_to_robot import CameraRobotCalibration
    cal = CameraRobotCalibration(table_z=0.02)
    cal.add_point(pixel=(100, 200), robot_xy=(0.10, 0.05))
    cal.add_point(pixel=(540, 200), robot_xy=(0.10,-0.05))
    cal.add_point(pixel=(100, 380), robot_xy=(0.30, 0.05))
    cal.add_point(pixel=(540, 380), robot_xy=(0.30,-0.05))
    cal.fit()
    x, y, z = cal.pixel_to_robot(320, 290)
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional


@dataclass
class CalibPoint:
    pixel:    tuple[float, float]    # (u, v) in the camera image
    robot_xy: tuple[float, float]    # (x, y) in metres, robot base frame


class CalibrationError(Exception):
    """Raised when calibration data is insufficient or degenerate."""


class CameraRobotCalibration:
    """
    Homography-based pixel→robot calibration for a static overhead camera.

    Requires at least 4 non-collinear point correspondences before fit().

    Args:
        table_z:   height of the table surface in robot frame (metres)
        min_points: minimum correspondences required to fit
    """

    MIN_POINTS = 4

    def __init__(self, table_z: float = 0.0, min_points: int = 4) -> None:
        self.table_z    = table_z
        self.min_points = max(min_points, self.MIN_POINTS)
        self._points: list[CalibPoint] = []
        self._H: Optional[list[list[float]]] = None   # 3x3 homography (row-major)

    # ------------------------------------------------------------------
    # Data collection
    # ------------------------------------------------------------------

    def add_point(
        self,
        pixel:    tuple[float, float],
        robot_xy: tuple[float, float],
    ) -> None:
        """Add one pixel ↔ robot correspondence."""
        self._points.append(CalibPoint(pixel=pixel, robot_xy=robot_xy))
        self._H = None   # invalidate fitted model

    def clear_points(self) -> None:
        self._points.clear()
        self._H = None

    @property
    def n_points(self) -> int:
        return len(self._points)

    # ------------------------------------------------------------------
    # Fitting
    # ------------------------------------------------------------------

    def fit(self) -> float:
        """
        Fit the homography from the collected point correspondences.

        Returns:
            RMS reprojection error in pixels.

        Raises:
            CalibrationError: fewer than min_points, or degenerate geometry.
        """
        if len(self._points) < self.min_points:
            raise CalibrationError(
                f"Need at least {self.min_points} points, got {len(self._points)}"
            )

        src = [(p.pixel[0],    p.pixel[1])    for p in self._points]
        dst = [(p.robot_xy[0], p.robot_xy[1]) for p in self._points]

        try:
            import numpy as np
            self._H = _fit_homography_numpy(
                np.array(src, dtype=float),
                np.array(dst, dtype=float),
            )
            rms = _rms_error(src, dst, self._H)
            return rms
        except Exception as exc:
            raise CalibrationError(f"Homography fit failed: {exc}") from exc

    # ------------------------------------------------------------------
    # Inference
    # ------------------------------------------------------------------

    def pixel_to_robot(
        self,
        u: float,
        v: float,
    ) -> tuple[float, float, float]:
        """
        Map a pixel coordinate to robot frame (x, y, table_z).

        Raises:
            CalibrationError: if fit() has not been called.
        """
        if self._H is None:
            raise CalibrationError("Call fit() before pixel_to_robot()")
        x, y = _apply_homography(self._H, u, v)
        return x, y, self.table_z

    def is_fitted(self) -> bool:
        return self._H is not None

    # ------------------------------------------------------------------
    # Persistence
    # ------------------------------------------------------------------

    def save(self, path: str | Path) -> None:
        """Save calibration (points + homography) to a JSON file."""
        data = {
            "table_z": self.table_z,
            "points": [
                {"pixel": list(p.pixel), "robot_xy": list(p.robot_xy)}
                for p in self._points
            ],
            "homography": self._H,
        }
        Path(path).write_text(json.dumps(data, indent=2))

    @classmethod
    def load(cls, path: str | Path) -> "CameraRobotCalibration":
        """Load a previously saved calibration."""
        data = json.loads(Path(path).read_text())
        cal = cls(table_z=data["table_z"])
        for pt in data["points"]:
            cal.add_point(tuple(pt["pixel"]), tuple(pt["robot_xy"]))  # type: ignore[arg-type]
        if data.get("homography"):
            cal._H = data["homography"]
        return cal

    # ------------------------------------------------------------------
    # Reprojection diagnostics
    # ------------------------------------------------------------------

    def reprojection_errors(self) -> list[float]:
        """Return per-point reprojection error (metres) in robot frame."""
        if self._H is None:
            raise CalibrationError("Not fitted yet.")
        errors = []
        for pt in self._points:
            px, py = _apply_homography(self._H, *pt.pixel)
            dx = px - pt.robot_xy[0]
            dy = py - pt.robot_xy[1]
            errors.append(math.hypot(dx, dy))
        return errors


# ------------------------------------------------------------------
# Pure-numpy homography (no OpenCV required)
# ------------------------------------------------------------------

def _fit_homography_numpy(
    src: "np.ndarray",   # (N,2) pixel coords
    dst: "np.ndarray",   # (N,2) robot coords
) -> list[list[float]]:
    """Direct linear transform (DLT) homography estimation."""
    import numpy as np

    N = len(src)
    A = np.zeros((2 * N, 9))
    for i in range(N):
        u, v   = src[i]
        x, y   = dst[i]
        A[2*i]   = [-u, -v, -1,  0,  0,  0, x*u, x*v, x]
        A[2*i+1] = [ 0,  0,  0, -u, -v, -1, y*u, y*v, y]

    _, _, Vt = np.linalg.svd(A)
    H = Vt[-1].reshape(3, 3)
    H /= H[2, 2]   # normalise
    return H.tolist()


def _apply_homography(
    H: list[list[float]],
    u: float,
    v: float,
) -> tuple[float, float]:
    import numpy as np
    Hm  = np.array(H)
    src = np.array([u, v, 1.0])
    dst = Hm @ src
    return float(dst[0] / dst[2]), float(dst[1] / dst[2])


def _rms_error(
    src: list[tuple[float, float]],
    dst: list[tuple[float, float]],
    H: list[list[float]],
) -> float:
    import numpy as np
    errs = []
    for (u, v), (x, y) in zip(src, dst):
        px, py = _apply_homography(H, u, v)
        errs.append((px - x) ** 2 + (py - y) ** 2)
    return float(np.sqrt(np.mean(errs)))
