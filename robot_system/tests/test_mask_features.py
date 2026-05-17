"""
Tests for mask_features.py — Step 7.

Run:
    python tests/test_mask_features.py
or:
    python -m pytest tests/test_mask_features.py -v
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from perception.mask_features import extract_features, BoundingBox

TOL = 1e-6


# ------------------------------------------------------------------
# Empty mask
# ------------------------------------------------------------------

def test_empty_mask():
    mask = np.zeros((480, 640), dtype=bool)
    f = extract_features(mask)
    assert f.is_empty
    assert f.area_px == 0
    assert f.centroid_px == (0.0, 0.0)
    assert f.centroid_norm == (0.0, 0.0)
    print("PASS  test_empty_mask")


# ------------------------------------------------------------------
# Centroid
# ------------------------------------------------------------------

def test_centroid_centre_pixel():
    """Single pixel at exact frame centre → normalised (0, 0)."""
    H, W = 480, 640
    mask = np.zeros((H, W), dtype=bool)
    mask[H // 2, W // 2] = True
    f = extract_features(mask)
    assert abs(f.centroid_px[0] - W // 2) < TOL
    assert abs(f.centroid_px[1] - H // 2) < TOL
    assert abs(f.centroid_norm[0]) < 1e-3    # near 0
    assert abs(f.centroid_norm[1]) < 1e-3
    print("PASS  test_centroid_centre_pixel")


def test_centroid_rectangle():
    """Centroid of a filled rectangle == geometric centre."""
    mask = np.zeros((100, 100), dtype=bool)
    mask[20:60, 30:70] = True   # rows 20-59, cols 30-69
    f = extract_features(mask)
    assert abs(f.centroid_px[0] - 49.5) < TOL   # (30+69)/2
    assert abs(f.centroid_px[1] - 39.5) < TOL   # (20+59)/2
    print("PASS  test_centroid_rectangle")


def test_centroid_top_left_object():
    """Object in top-left → normalised coords should be negative."""
    mask = np.zeros((480, 640), dtype=bool)
    mask[0:50, 0:50] = True
    f = extract_features(mask)
    assert f.centroid_norm[0] < 0   # left of centre
    assert f.centroid_norm[1] < 0   # above centre
    print("PASS  test_centroid_top_left_object")


# ------------------------------------------------------------------
# Area
# ------------------------------------------------------------------

def test_area_exact():
    mask = np.zeros((200, 200), dtype=bool)
    mask[50:150, 50:150] = True   # 100×100 = 10000 px
    f = extract_features(mask)
    assert f.area_px == 10_000
    print("PASS  test_area_exact")


# ------------------------------------------------------------------
# Bounding box
# ------------------------------------------------------------------

def test_bbox_values():
    mask = np.zeros((300, 400), dtype=bool)
    mask[10:60, 20:80] = True
    f = extract_features(mask)
    assert f.bbox == BoundingBox(x_min=20, y_min=10, x_max=80, y_max=60)
    assert f.bbox.width  == 60
    assert f.bbox.height == 50
    print("PASS  test_bbox_values")


def test_bbox_single_pixel():
    mask = np.zeros((100, 100), dtype=bool)
    mask[42, 17] = True
    f = extract_features(mask)
    assert f.bbox.width  == 1
    assert f.bbox.height == 1
    assert f.bbox.area   == 1
    print("PASS  test_bbox_single_pixel")


# ------------------------------------------------------------------
# Fill ratio
# ------------------------------------------------------------------

def test_fill_ratio_solid_rectangle():
    """A fully filled rectangle has fill_ratio == 1.0."""
    mask = np.zeros((200, 200), dtype=bool)
    mask[10:90, 10:90] = True
    f = extract_features(mask)
    assert abs(f.fill_ratio - 1.0) < TOL
    print("PASS  test_fill_ratio_solid_rectangle")


def test_fill_ratio_hollow():
    """A hollow ring has fill_ratio < 1."""
    mask = np.zeros((100, 100), dtype=bool)
    mask[10:90, 10:90] = True
    mask[20:80, 20:80] = False   # hole
    f = extract_features(mask)
    assert f.fill_ratio < 1.0
    print("PASS  test_fill_ratio_hollow")


# ------------------------------------------------------------------
# Frame fill
# ------------------------------------------------------------------

def test_frame_fill():
    H, W = 100, 100
    mask = np.zeros((H, W), dtype=bool)
    mask[25:75, 25:75] = True   # 50×50 = 2500 px out of 10000
    f = extract_features(mask)
    assert abs(f.frame_fill - 0.25) < TOL
    print("PASS  test_frame_fill")


# ------------------------------------------------------------------
# Compactness
# ------------------------------------------------------------------

def test_compactness_circle_approx():
    """A rasterised circle should have compactness close to 1."""
    H, W = 200, 200
    Y, X = np.ogrid[:H, :W]
    mask = (X - 100) ** 2 + (Y - 100) ** 2 <= 80 ** 2
    f = extract_features(mask)
    # Rasterised circle is not perfect, but should be > 0.9
    assert f.compactness > 0.9, f"compactness={f.compactness:.3f}"
    print(f"PASS  test_compactness_circle_approx  (compactness={f.compactness:.3f})")


def test_compactness_thin_line():
    """A very thin horizontal line has low compactness."""
    mask = np.zeros((200, 200), dtype=bool)
    mask[100, :] = True   # 1-pixel-tall line
    f = extract_features(mask)
    assert f.compactness < 0.1, f"compactness={f.compactness:.4f}"
    print(f"PASS  test_compactness_thin_line  (compactness={f.compactness:.4f})")


# ------------------------------------------------------------------
# centroid_in_frame
# ------------------------------------------------------------------

def test_centred_object_in_frame():
    """Object at frame centre is detected as centred."""
    H, W = 480, 640
    mask = np.zeros((H, W), dtype=bool)
    mask[220:260, 300:340] = True   # near centre
    f = extract_features(mask)
    assert f.centroid_in_frame(margin=0.15)
    print("PASS  test_centred_object_in_frame")


def test_corner_object_not_in_frame():
    """Object in corner is not considered centred."""
    mask = np.zeros((480, 640), dtype=bool)
    mask[0:20, 0:20] = True
    f = extract_features(mask)
    assert not f.centroid_in_frame(margin=0.15)
    print("PASS  test_corner_object_not_in_frame")


# ------------------------------------------------------------------
# frame_shape override
# ------------------------------------------------------------------

def test_frame_shape_override():
    """Normalised coords use frame_shape, not mask shape."""
    mask = np.zeros((100, 100), dtype=bool)
    mask[50, 50] = True
    # Override: mask is a crop from a 480×640 frame; pixel (50,50) in the crop
    # maps differently depending on frame_shape
    f_crop  = extract_features(mask, frame_shape=(100, 100))
    f_frame = extract_features(mask, frame_shape=(480, 640))
    # In the 100×100 case, pixel 50 is at the centre → norm ≈ 0
    assert abs(f_crop.centroid_norm[0]) < 0.02
    # In the 640-wide frame, pixel 50 is well left of centre → norm < 0
    assert f_frame.centroid_norm[0] < 0
    print("PASS  test_frame_shape_override")


if __name__ == "__main__":
    test_empty_mask()
    test_centroid_centre_pixel()
    test_centroid_rectangle()
    test_centroid_top_left_object()
    test_area_exact()
    test_bbox_values()
    test_bbox_single_pixel()
    test_fill_ratio_solid_rectangle()
    test_fill_ratio_hollow()
    test_frame_fill()
    test_compactness_circle_approx()
    test_compactness_thin_line()
    test_centred_object_in_frame()
    test_corner_object_not_in_frame()
    test_frame_shape_override()
    print("\nAll mask_features tests passed.")
