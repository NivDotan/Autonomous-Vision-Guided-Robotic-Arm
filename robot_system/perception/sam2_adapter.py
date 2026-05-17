"""
Step 8 — SAM2 adapter with dry-run support.

Wraps sam2.build_sam / SAM2ImagePredictor behind a stable interface so the
rest of the pipeline doesn't touch SAM2 internals directly.

Dry-run mode generates a synthetic elliptical mask around the click point —
no GPU, no model weights needed for testing.

Usage (dry-run, no model required):
    import numpy as np
    from perception.sam2_adapter import Sam2Adapter

    adapter = Sam2Adapter(dry_run=True)
    frame   = np.zeros((480, 640, 3), dtype=np.uint8)
    adapter.set_image(frame)
    result = adapter.predict(point_xy=(320, 240))
    print(result.features.area_px, result.features.centroid_norm)

Usage (live, SAM2 installed):
    from sam2.build_sam import build_sam2
    from sam2.sam2_image_predictor import SAM2ImagePredictor

    sam  = build_sam2(config_file, checkpoint, device="cuda")
    pred = SAM2ImagePredictor(sam)
    adapter = Sam2Adapter(predictor=pred, dry_run=False)
    ...
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Optional

import numpy as np

from .mask_features import MaskFeatures, extract_features


@dataclass
class PredictionResult:
    """Output of a single SAM2 prediction call."""
    mask:     np.ndarray       # bool H×W
    score:    float            # confidence in [0, 1]
    features: MaskFeatures     # pre-computed geometric features


class Sam2Adapter:
    """
    Unified SAM2 interface for image-mode (single-frame) segmentation.

    Parameters
    ----------
    predictor:
        A live ``SAM2ImagePredictor`` instance.  Required when dry_run=False.
    dry_run:
        If True, ``predict()`` returns a synthetic elliptical mask centred
        on the click point.  No model or GPU needed.
    dry_run_radius:
        Radius (px) of the synthetic ellipse in dry-run mode.
    """

    def __init__(
        self,
        predictor: Any = None,
        dry_run: bool = True,
        dry_run_radius: int = 60,
    ) -> None:
        if not dry_run and predictor is None:
            raise ValueError("predictor must be provided when dry_run=False")
        self.predictor      = predictor
        self.dry_run        = dry_run
        self.dry_run_radius = dry_run_radius

        self._frame:  Optional[np.ndarray] = None   # current RGB frame
        self._H:      int = 0
        self._W:      int = 0
        self._image_set: bool = False

    # ------------------------------------------------------------------
    # Image management
    # ------------------------------------------------------------------

    def set_image(self, frame: np.ndarray) -> None:
        """
        Feed a new frame.  Must be called before predict().

        Args:
            frame: H×W×3 uint8 RGB (or BGR — SAM2 expects RGB; convert first).
        """
        if frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError(f"frame must be H×W×3, got {frame.shape}")

        self._frame = frame
        self._H, self._W = frame.shape[:2]
        self._image_set = True

        if not self.dry_run:
            self.predictor.set_image(frame)

    def reset(self) -> None:
        """Clear the current frame (call between unrelated frames)."""
        self._frame     = None
        self._image_set = False

    # ------------------------------------------------------------------
    # Prediction
    # ------------------------------------------------------------------

    def predict_box(
        self,
        box_xyxy: tuple[float, float, float, float],
        multimask: bool = True,
    ) -> PredictionResult:
        """
        Segment using a bounding box prompt (x_min, y_min, x_max, y_max).

        More stable than point prompts for tracking — the box constrains the
        search region so the mask doesn't jump to unrelated objects.
        """
        if not self._image_set:
            raise RuntimeError("Call set_image() before predict_box()")

        if self.dry_run:
            cx = (box_xyxy[0] + box_xyxy[2]) / 2.0
            cy = (box_xyxy[1] + box_xyxy[3]) / 2.0
            rx = int((box_xyxy[2] - box_xyxy[0]) / 2)
            ry = int((box_xyxy[3] - box_xyxy[1]) / 2)
            Y, X = np.ogrid[: self._H, : self._W]
            mask = ((X - cx) / max(rx, 1)) ** 2 + ((Y - cy) / max(ry, 1)) ** 2 <= 1.0
            features = extract_features(mask, frame_shape=(self._H, self._W))
            return PredictionResult(mask=mask, score=1.0, features=features)

        import torch
        box = np.array([box_xyxy], dtype=np.float32)
        with torch.inference_mode():
            masks, scores, _ = self.predictor.predict(
                point_coords=None,
                point_labels=None,
                box=box,
                multimask_output=multimask,
            )
        best  = int(np.argmax(scores))
        mask  = masks[best].astype(bool)
        score = float(scores[best])
        features = extract_features(mask, frame_shape=(self._H, self._W))
        return PredictionResult(mask=mask, score=score, features=features)

    def predict(
        self,
        point_xy: tuple[float, float],
        label: int = 1,
        multimask: bool = True,
    ) -> PredictionResult:
        """
        Segment the object at the given click point.

        Args:
            point_xy:  (x, y) click position in pixel coords.
            label:     1 = foreground click, 0 = background click.
            multimask: if True, SAM2 returns 3 masks; we pick the highest-score one.

        Returns:
            PredictionResult with mask, score, and MaskFeatures.

        Raises:
            RuntimeError: if set_image() has not been called yet.
        """
        if not self._image_set:
            raise RuntimeError("Call set_image() before predict()")

        if self.dry_run:
            return self._predict_dry_run(point_xy)
        return self._predict_live(point_xy, label, multimask)

    # ------------------------------------------------------------------
    # Dry-run implementation
    # ------------------------------------------------------------------

    def _predict_dry_run(
        self,
        point_xy: tuple[float, float],
    ) -> PredictionResult:
        """Generate a synthetic ellipse mask around the click point."""
        cx, cy = float(point_xy[0]), float(point_xy[1])
        rx = self.dry_run_radius
        ry = int(rx * 0.75)   # slightly taller than wide, like a real object

        Y, X = np.ogrid[: self._H, : self._W]
        mask = ((X - cx) / rx) ** 2 + ((Y - cy) / ry) ** 2 <= 1.0

        # Clip to image bounds (click near edge)
        mask = mask & np.ones((self._H, self._W), dtype=bool)

        features = extract_features(mask, frame_shape=(self._H, self._W))
        return PredictionResult(mask=mask, score=1.0, features=features)

    # ------------------------------------------------------------------
    # Live SAM2 implementation
    # ------------------------------------------------------------------

    def _predict_live(
        self,
        point_xy: tuple[float, float],
        label: int,
        multimask: bool,
    ) -> PredictionResult:
        import torch

        points = np.array([[point_xy[0], point_xy[1]]], dtype=np.float32)
        labels = np.array([label], dtype=np.int32)

        with torch.inference_mode():
            masks, scores, _ = self.predictor.predict(
                point_coords=points,
                point_labels=labels,
                multimask_output=multimask,
            )

        # masks: (N, H, W) bool  scores: (N,)
        best = int(np.argmax(scores))
        mask = masks[best].astype(bool)
        score = float(scores[best])

        features = extract_features(mask, frame_shape=(self._H, self._W))
        return PredictionResult(mask=mask, score=score, features=features)

    # ------------------------------------------------------------------
    # Visualisation helper
    # ------------------------------------------------------------------

    def overlay(
        self,
        frame: np.ndarray,
        result: PredictionResult,
        color: tuple[int, int, int] = (0, 255, 0),
        alpha: float = 0.4,
    ) -> np.ndarray:
        """
        Return a copy of frame with the mask blended in.

        Args:
            frame:  H×W×3 uint8
            result: PredictionResult from predict()
            color:  BGR overlay colour
            alpha:  blend weight for mask region
        """
        out = frame.copy()
        fg  = result.mask
        for c, val in enumerate(color):
            channel = out[:, :, c].astype(np.float32)
            channel[fg] = channel[fg] * (1 - alpha) + val * alpha
            out[:, :, c] = channel.clip(0, 255).astype(np.uint8)

        # Draw centroid cross
        cx, cy = result.features.centroid_px
        cx, cy = int(round(cx)), int(round(cy))
        r = 8
        out[max(0, cy - r): cy + r + 1, max(0, cx - 1): cx + 2] = color
        out[max(0, cy - 1): cy + 2, max(0, cx - r): cx + r + 1] = color

        return out
