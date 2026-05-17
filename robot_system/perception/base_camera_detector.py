"""
Step 12a — Object detector for the static base (overhead) camera.

Detects objects on the table surface and returns their pixel centroids and
bounding boxes.  Supports two modes:

  colour   — HSV threshold (fast, tunable, no model)
  contour  — finds the N largest blobs in a mask (used after colour filter)

In dry-run mode a synthetic object list is returned.

Usage:
    from perception.base_camera_detector import BaseDetector, DetectionConfig
    det = BaseDetector(dry_run=True)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    objects = det.detect(frame)
    for obj in objects:
        print(obj.centroid_px, obj.area_px)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class DetectedObject:
    """One object found in the base-camera frame."""
    centroid_px: tuple[float, float]   # (x, y) in pixels
    area_px:     int
    bbox:        tuple[int, int, int, int]   # x_min, y_min, x_max, y_max
    label:       str = "object"

    @property
    def x(self) -> float:
        return self.centroid_px[0]

    @property
    def y(self) -> float:
        return self.centroid_px[1]


@dataclass
class DetectionConfig:
    """Tuning for the colour-threshold detector."""
    # HSV range for the target object colour (default: yellow-orange)
    hsv_lower: tuple[int, int, int] = (15,  80, 80)
    hsv_upper: tuple[int, int, int] = (35, 255, 255)

    min_area_px: int = 500     # ignore blobs smaller than this
    max_objects: int = 5       # return at most this many objects


class BaseDetector:
    """
    Detect objects in a base-camera frame.

    Args:
        config:   colour and size thresholds
        dry_run:  if True, detect() returns a synthetic detection at frame centre
    """

    def __init__(
        self,
        config: DetectionConfig = DetectionConfig(),
        dry_run: bool = True,
    ) -> None:
        self.cfg     = config
        self.dry_run = dry_run

    def detect(
        self,
        frame: np.ndarray,
        max_objects: Optional[int] = None,
    ) -> list[DetectedObject]:
        """
        Detect objects in the given BGR frame.

        Returns a list sorted by area (largest first).
        """
        if self.dry_run:
            return self._detect_dry_run(frame)
        return self._detect_colour(frame, max_objects or self.cfg.max_objects)

    # ------------------------------------------------------------------
    # Dry-run
    # ------------------------------------------------------------------

    def _detect_dry_run(self, frame: np.ndarray) -> list[DetectedObject]:
        H, W = frame.shape[:2]
        cx, cy = W / 2.0, H / 2.0
        return [DetectedObject(
            centroid_px=(cx, cy),
            area_px=2500,
            bbox=(int(cx) - 25, int(cy) - 25, int(cx) + 25, int(cy) + 25),
        )]

    # ------------------------------------------------------------------
    # Colour-threshold detector
    # ------------------------------------------------------------------

    def _detect_colour(self, frame: np.ndarray, max_obj: int) -> list[DetectedObject]:
        try:
            import cv2  # type: ignore[import-untyped]
        except ImportError:
            raise ImportError("opencv-python required for live detection. "
                              "Run: pip install opencv-python")

        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lo   = np.array(self.cfg.hsv_lower, dtype=np.uint8)
        hi   = np.array(self.cfg.hsv_upper, dtype=np.uint8)
        mask = cv2.inRange(hsv, lo, hi)

        # Morphological clean-up
        k    = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        objects: list[DetectedObject] = []
        for cnt in contours:
            area = int(cv2.contourArea(cnt))
            if area < self.cfg.min_area_px:
                continue
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx = M["m10"] / M["m00"]
            cy = M["m01"] / M["m00"]
            x, y, w, h = cv2.boundingRect(cnt)
            objects.append(DetectedObject(
                centroid_px=(cx, cy),
                area_px=area,
                bbox=(x, y, x + w, y + h),
            ))

        objects.sort(key=lambda o: o.area_px, reverse=True)
        return objects[:max_obj]
