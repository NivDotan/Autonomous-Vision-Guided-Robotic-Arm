"""
Step 7 — Extract geometric features from a SAM2 binary mask.

Input:  a boolean / uint8 H×W numpy array (1 = object, 0 = background)
Output: MaskFeatures dataclass with centroid, area, bbox, normalised coords

No SAM2 model or robot required — pure numpy.

Usage:
    import numpy as np
    from perception.mask_features import extract_features

    mask = np.zeros((480, 640), dtype=bool)
    mask[200:300, 280:360] = True
    f = extract_features(mask)
    print(f.centroid_px, f.area_px, f.fill_ratio)
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class BoundingBox:
    x_min: int
    y_min: int
    x_max: int
    y_max: int

    @property
    def width(self) -> int:
        return self.x_max - self.x_min

    @property
    def height(self) -> int:
        return self.y_max - self.y_min

    @property
    def area(self) -> int:
        return self.width * self.height

    @property
    def center(self) -> tuple[float, float]:
        return (self.x_min + self.x_max) / 2.0, (self.y_min + self.y_max) / 2.0

    @property
    def aspect_ratio(self) -> float:
        return self.width / self.height if self.height > 0 else 0.0


@dataclass(frozen=True)
class MaskFeatures:
    """
    Geometric summary of a single object mask.

    Pixel coordinates: origin = top-left, x = column, y = row.
    Normalised coordinates: range [-1, 1] relative to image centre.
    """

    # ── pixel space ───────────────────────────────────────────────────
    centroid_px: tuple[float, float]    # (x, y) in pixels
    area_px:     int                    # number of foreground pixels
    bbox:        BoundingBox

    # ── normalised (relative to image centre, range ≈ [-1, 1]) ───────
    centroid_norm: tuple[float, float]  # (x, y) in [-1, 1]

    # ── shape descriptors ─────────────────────────────────────────────
    fill_ratio:    float    # area_px / bbox.area  (how solid the mask is)
    frame_fill:    float    # area_px / (H*W)      (fraction of full frame)
    compactness:   float    # 4π·area / perimeter²  (1 = circle, →0 = elongated)

    # ── image dims for reference ──────────────────────────────────────
    frame_w: int
    frame_h: int

    @property
    def is_empty(self) -> bool:
        return self.area_px == 0

    def offset_from_centre(self) -> tuple[float, float]:
        """Signed (dx, dy) in normalised coords from frame centre to centroid."""
        return self.centroid_norm

    def centroid_in_frame(self, margin: float = 0.15) -> bool:
        """True if centroid is within the central (1-2*margin) × (1-2*margin) region."""
        nx, ny = self.centroid_norm
        lim = 1.0 - 2.0 * margin
        return abs(nx) <= lim and abs(ny) <= lim


def _perimeter(mask: np.ndarray) -> int:
    """Count edge pixels via 4-connectivity erosion difference."""
    from numpy.lib.stride_tricks import sliding_window_view  # noqa: F401 — fallback below

    # simple: a foreground pixel is on the perimeter if any 4-neighbour is background
    fg = mask.astype(bool)
    interior = (
        np.pad(fg[1:, :],  ((0, 1), (0, 0)), constant_values=False) &
        np.pad(fg[:-1, :], ((1, 0), (0, 0)), constant_values=False) &
        np.pad(fg[:, 1:],  ((0, 0), (0, 1)), constant_values=False) &
        np.pad(fg[:, :-1], ((0, 0), (1, 0)), constant_values=False)
    )
    return int(np.sum(fg & ~interior))


def extract_features(
    mask: np.ndarray,
    frame_shape: Optional[tuple[int, int]] = None,
) -> MaskFeatures:
    """
    Compute geometric features from a binary mask.

    Args:
        mask:        H×W array, truthy = object pixel.
        frame_shape: (H, W) of the original frame.  Defaults to mask.shape.

    Returns:
        MaskFeatures.  If the mask is empty, all numeric fields are 0/0.0.
    """
    mask = np.asarray(mask, dtype=bool)
    if mask.ndim != 2:
        raise ValueError(f"mask must be 2-D, got shape {mask.shape}")

    H, W = mask.shape
    fH, fW = frame_shape if frame_shape is not None else (H, W)

    area = int(mask.sum())

    if area == 0:
        empty_bb = BoundingBox(0, 0, 0, 0)
        return MaskFeatures(
            centroid_px=(0.0, 0.0),
            area_px=0,
            bbox=empty_bb,
            centroid_norm=(0.0, 0.0),
            fill_ratio=0.0,
            frame_fill=0.0,
            compactness=0.0,
            frame_w=fW,
            frame_h=fH,
        )

    # Centroid via first moments
    rows, cols = np.where(mask)
    cx = float(cols.mean())
    cy = float(rows.mean())

    # Bounding box
    x_min, x_max = int(cols.min()), int(cols.max()) + 1
    y_min, y_max = int(rows.min()), int(rows.max()) + 1
    bb = BoundingBox(x_min, y_min, x_max, y_max)

    # Normalised centroid: map [0, W] -> [-1, 1]
    nx = (cx - fW / 2.0) / (fW / 2.0)
    ny = (cy - fH / 2.0) / (fH / 2.0)

    # Fill ratio
    bb_area      = bb.area
    fill_ratio   = area / bb_area   if bb_area > 0 else 0.0
    frame_fill   = area / (fW * fH) if (fW * fH) > 0 else 0.0

    # Compactness  4π·A / P²
    perim = _perimeter(mask)
    compactness = (4.0 * math.pi * area / (perim ** 2)) if perim > 0 else 0.0

    return MaskFeatures(
        centroid_px=(cx, cy),
        area_px=area,
        bbox=bb,
        centroid_norm=(nx, ny),
        fill_ratio=fill_ratio,
        frame_fill=frame_fill,
        compactness=compactness,
        frame_w=fW,
        frame_h=fH,
    )
