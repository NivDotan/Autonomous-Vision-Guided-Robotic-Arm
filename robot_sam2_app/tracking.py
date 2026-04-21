from __future__ import annotations

from dataclasses import dataclass

import cv2

from .config import APPROACH_THRESHOLD, SEG_EVERY_N_FRAMES


@dataclass
class TrackingResult:
    success: bool
    center_x: int | None = None
    center_y: int | None = None
    area: int = 0
    width: int = 0
    height: int = 0


class ObjectTracker:
    """SAM2 initializes a bbox, OpenCV CSRT tracks it frame-to-frame."""

    def __init__(self):
        self.clicked_point: tuple[int, int] | None = None
        self.pending_auto_bbox: tuple[int, int, int, int] | None = None
        self.tracker = None
        self.active = False
        self.bbox = None

    def request_click(self, x: int, y: int) -> None:
        self.clicked_point = (x, y)
        self.pending_auto_bbox = None
        self.tracker = None
        self.active = False

    def request_bbox(self, box_xyxy: tuple[int, int, int, int]) -> None:
        self.pending_auto_bbox = box_xyxy
        self.clicked_point = None
        self.tracker = None
        self.active = False

    def reset(self) -> None:
        self.clicked_point = None
        self.pending_auto_bbox = None
        self.tracker = None
        self.active = False
        self.bbox = None

    def process(self, frame_bgr, segmenter, frame_index: int, approach_mode: bool) -> TrackingResult:
        if (self.clicked_point is not None or self.pending_auto_bbox is not None) and not self.active:
            if frame_index % SEG_EVERY_N_FRAMES == 0:
                self._initialize_from_sam2(frame_bgr, segmenter)

        if not self.active or self.tracker is None:
            return TrackingResult(False)

        ok, bbox = self.tracker.update(frame_bgr)
        if not ok:
            self.reset()
            return TrackingResult(False)

        x, y, w, h = map(int, bbox)
        self.bbox = (x, y, w, h)
        cx, cy = x + w // 2, y + h // 2
        area = w * h
        color = (0, 255, 0) if approach_mode and area >= APPROACH_THRESHOLD else (0, 255, 255)
        cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), color, 2)
        cv2.circle(frame_bgr, (cx, cy), 5, (0, 0, 255), -1)
        cv2.putText(frame_bgr, f"Area: {area}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        return TrackingResult(True, cx, cy, area, w, h)

    def _initialize_from_sam2(self, frame_bgr, segmenter) -> None:
        if self.pending_auto_bbox is not None:
            x0, y0, x1, y1 = self.pending_auto_bbox
            point = ((x0 + x1) // 2, (y0 + y1) // 2)
            bbox = segmenter.segment_bbox(frame_bgr, point, self.pending_auto_bbox)
            if bbox is None:
                bbox = (x0, y0, x1 - x0, y1 - y0)
        else:
            bbox = segmenter.segment_bbox(frame_bgr, self.clicked_point)

        if bbox is None:
            return
        self.tracker = self._make_csrt_tracker()
        self.tracker.init(frame_bgr, bbox)
        self.active = True
        self.clicked_point = None
        self.pending_auto_bbox = None

    @staticmethod
    def _make_csrt_tracker():
        if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerCSRT_create"):
            return cv2.legacy.TrackerCSRT_create()
        return cv2.TrackerCSRT_create()

