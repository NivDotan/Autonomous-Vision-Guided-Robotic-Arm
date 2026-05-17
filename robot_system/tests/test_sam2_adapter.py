"""
Tests for Sam2Adapter — Step 8.

Run:
    python tests/test_sam2_adapter.py
or:
    python -m pytest tests/test_sam2_adapter.py -v

All tests use dry_run=True — no SAM2 model or GPU required.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from perception.sam2_adapter import Sam2Adapter, PredictionResult

H, W = 480, 640


def _blank_frame() -> np.ndarray:
    return np.zeros((H, W, 3), dtype=np.uint8)


# ------------------------------------------------------------------
# Construction guard
# ------------------------------------------------------------------

def test_no_predictor_live_raises():
    try:
        Sam2Adapter(predictor=None, dry_run=False)
        assert False, "Should have raised"
    except ValueError as e:
        print(f"PASS  test_no_predictor_live_raises  ({e})")


def test_predict_before_set_image_raises():
    adapter = Sam2Adapter(dry_run=True)
    try:
        adapter.predict((320, 240))
        assert False, "Should have raised"
    except RuntimeError as e:
        print(f"PASS  test_predict_before_set_image_raises  ({e})")


# ------------------------------------------------------------------
# Basic dry-run
# ------------------------------------------------------------------

def test_set_image_valid():
    adapter = Sam2Adapter(dry_run=True)
    adapter.set_image(_blank_frame())
    assert adapter._image_set
    assert adapter._H == H and adapter._W == W
    print("PASS  test_set_image_valid")


def test_wrong_frame_shape_raises():
    adapter = Sam2Adapter(dry_run=True)
    try:
        adapter.set_image(np.zeros((480, 640), dtype=np.uint8))   # missing channel dim
        assert False, "Should have raised"
    except ValueError as e:
        print(f"PASS  test_wrong_frame_shape_raises  ({e})")


def test_predict_returns_result():
    adapter = Sam2Adapter(dry_run=True)
    adapter.set_image(_blank_frame())
    result = adapter.predict((320, 240))
    assert isinstance(result, PredictionResult)
    assert result.mask.shape == (H, W)
    assert result.mask.dtype == bool
    assert 0.0 <= result.score <= 1.0
    print("PASS  test_predict_returns_result")


# ------------------------------------------------------------------
# Ellipse geometry
# ------------------------------------------------------------------

def test_ellipse_centred_on_click():
    """Dry-run mask centroid should be close to the click point."""
    adapter = Sam2Adapter(dry_run=True, dry_run_radius=60)
    adapter.set_image(_blank_frame())
    cx, cy = 320, 240
    result = adapter.predict((cx, cy))
    f = result.features
    assert abs(f.centroid_px[0] - cx) < 2.0, f"cx off: {f.centroid_px[0]}"
    assert abs(f.centroid_px[1] - cy) < 2.0, f"cy off: {f.centroid_px[1]}"
    print("PASS  test_ellipse_centred_on_click")


def test_ellipse_area_scales_with_radius():
    """Larger dry_run_radius → larger mask area."""
    adapter_small = Sam2Adapter(dry_run=True, dry_run_radius=30)
    adapter_large = Sam2Adapter(dry_run=True, dry_run_radius=80)
    frame = _blank_frame()
    for a in (adapter_small, adapter_large):
        a.set_image(frame)
    r_small = adapter_small.predict((320, 240)).features.area_px
    r_large = adapter_large.predict((320, 240)).features.area_px
    assert r_large > r_small, f"large={r_large} should > small={r_small}"
    print(f"PASS  test_ellipse_area_scales_with_radius  (small={r_small}, large={r_large})")


def test_different_click_positions():
    """Different click points produce different centroids."""
    adapter = Sam2Adapter(dry_run=True)
    adapter.set_image(_blank_frame())
    r1 = adapter.predict((100, 100))
    r2 = adapter.predict((500, 350))
    assert r1.features.centroid_px != r2.features.centroid_px
    print("PASS  test_different_click_positions")


def test_click_near_edge_clipped():
    """Click near image edge should not crash; mask is clipped to frame."""
    adapter = Sam2Adapter(dry_run=True, dry_run_radius=80)
    adapter.set_image(_blank_frame())
    result = adapter.predict((5, 5))   # top-left corner
    assert result.mask.shape == (H, W)
    assert result.features.area_px > 0
    print("PASS  test_click_near_edge_clipped")


# ------------------------------------------------------------------
# Score
# ------------------------------------------------------------------

def test_dry_run_score_is_one():
    adapter = Sam2Adapter(dry_run=True)
    adapter.set_image(_blank_frame())
    assert adapter.predict((320, 240)).score == 1.0
    print("PASS  test_dry_run_score_is_one")


# ------------------------------------------------------------------
# reset()
# ------------------------------------------------------------------

def test_reset_clears_image():
    adapter = Sam2Adapter(dry_run=True)
    adapter.set_image(_blank_frame())
    adapter.reset()
    assert not adapter._image_set
    try:
        adapter.predict((320, 240))
        assert False, "Should have raised"
    except RuntimeError:
        print("PASS  test_reset_clears_image")


# ------------------------------------------------------------------
# overlay()
# ------------------------------------------------------------------

def test_overlay_same_shape():
    """overlay() must return an array with the same shape as the input."""
    adapter = Sam2Adapter(dry_run=True)
    frame   = np.zeros((H, W, 3), dtype=np.uint8)
    adapter.set_image(frame)
    result  = adapter.predict((320, 240))
    out     = adapter.overlay(frame, result)
    assert out.shape == frame.shape
    assert out.dtype == np.uint8
    print("PASS  test_overlay_same_shape")


def test_overlay_colours_mask_region():
    """Pixels inside the mask should be tinted; outside should stay black."""
    adapter = Sam2Adapter(dry_run=True, dry_run_radius=60)
    frame   = np.zeros((H, W, 3), dtype=np.uint8)
    adapter.set_image(frame)
    result  = adapter.predict((320, 240))
    out     = adapter.overlay(frame, result, color=(0, 255, 0), alpha=1.0)

    mask = result.mask
    # Green channel inside mask should be 255 (alpha=1.0, colour=green)
    assert out[mask, 1].mean() > 200, "green channel not set inside mask"
    # Outside mask should stay 0 (excluding centroid cross pixels)
    outside = ~mask
    # zero out a small central strip where the cross is drawn
    outside[230:250, 310:330] = False
    assert out[outside].max() == 0, "pixels outside mask were modified"
    print("PASS  test_overlay_colours_mask_region")


if __name__ == "__main__":
    test_no_predictor_live_raises()
    test_predict_before_set_image_raises()
    test_set_image_valid()
    test_wrong_frame_shape_raises()
    test_predict_returns_result()
    test_ellipse_centred_on_click()
    test_ellipse_area_scales_with_radius()
    test_different_click_positions()
    test_click_near_edge_clipped()
    test_dry_run_score_is_one()
    test_reset_clears_image()
    test_overlay_same_shape()
    test_overlay_colours_mask_region()
    print("\nAll Sam2Adapter tests passed.")
