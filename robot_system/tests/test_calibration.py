"""
Tests for base_camera_detector + base_camera_to_robot — Step 12.

Run:
    python tests/test_calibration.py
or:
    python -m pytest tests/test_calibration.py -v
"""

from __future__ import annotations

import math
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from perception.base_camera_detector import BaseDetector, DetectionConfig
from calibration.base_camera_to_robot import (
    CameraRobotCalibration, CalibrationError,
)

H, W = 480, 640


# ================================================================
# BaseDetector (dry-run)
# ================================================================

def test_detector_dry_run_returns_one_object():
    det = BaseDetector(dry_run=True)
    frame = np.zeros((H, W, 3), dtype=np.uint8)
    objs = det.detect(frame)
    assert len(objs) == 1
    print("PASS  test_detector_dry_run_returns_one_object")


def test_detector_centroid_at_frame_centre():
    det = BaseDetector(dry_run=True)
    frame = np.zeros((H, W, 3), dtype=np.uint8)
    obj = det.detect(frame)[0]
    assert abs(obj.x - W / 2) < 1.0
    assert abs(obj.y - H / 2) < 1.0
    print("PASS  test_detector_centroid_at_frame_centre")


def test_detector_has_positive_area():
    det = BaseDetector(dry_run=True)
    frame = np.zeros((H, W, 3), dtype=np.uint8)
    obj = det.detect(frame)[0]
    assert obj.area_px > 0
    print("PASS  test_detector_has_positive_area")


def test_detector_bbox_contains_centroid():
    det = BaseDetector(dry_run=True)
    frame = np.zeros((H, W, 3), dtype=np.uint8)
    obj = det.detect(frame)[0]
    x0, y0, x1, y1 = obj.bbox
    assert x0 <= obj.x <= x1
    assert y0 <= obj.y <= y1
    print("PASS  test_detector_bbox_contains_centroid")


# ================================================================
# CameraRobotCalibration
# ================================================================

def _four_point_cal(table_z=0.02):
    """Standard 4-corner calibration for a 640x480 frame."""
    cal = CameraRobotCalibration(table_z=table_z)
    cal.add_point((100, 100), (0.35,  0.15))
    cal.add_point((540, 100), (0.35, -0.15))
    cal.add_point((100, 380), (0.20,  0.15))
    cal.add_point((540, 380), (0.20, -0.15))
    return cal


def test_fit_requires_4_points():
    cal = CameraRobotCalibration()
    cal.add_point((0, 0), (0.0, 0.0))
    cal.add_point((1, 0), (0.1, 0.0))
    try:
        cal.fit()
        assert False, "Should have raised CalibrationError"
    except CalibrationError as e:
        print(f"PASS  test_fit_requires_4_points  ({e})")


def test_fit_succeeds_with_4_points():
    cal = _four_point_cal()
    rms = cal.fit()
    assert cal.is_fitted()
    assert rms >= 0.0
    print(f"PASS  test_fit_succeeds_with_4_points  (rms={rms*1000:.3f} mm)")


def test_pixel_to_robot_before_fit_raises():
    cal = CameraRobotCalibration()
    try:
        cal.pixel_to_robot(320, 240)
        assert False, "Should have raised CalibrationError"
    except CalibrationError as e:
        print(f"PASS  test_pixel_to_robot_before_fit_raises  ({e})")


def test_pixel_to_robot_returns_table_z():
    cal = _four_point_cal(table_z=0.03)
    cal.fit()
    _, _, z = cal.pixel_to_robot(320, 240)
    assert abs(z - 0.03) < 1e-9
    print("PASS  test_pixel_to_robot_returns_table_z")


def test_reprojection_low_error():
    """Calibration points should reproject to near-zero error."""
    cal = _four_point_cal()
    cal.fit()
    errs = cal.reprojection_errors()
    assert all(e < 1e-6 for e in errs), f"errors={errs}"
    print(f"PASS  test_reprojection_low_error  (max={max(errs)*1000:.4f} mm)")


def test_corner_pixel_maps_to_known_robot():
    """Top-left pixel (100,100) should map to approximately (0.35, 0.15)."""
    cal = _four_point_cal()
    cal.fit()
    x, y, _ = cal.pixel_to_robot(100, 100)
    assert abs(x - 0.35) < 1e-4, f"x={x}"
    assert abs(y - 0.15) < 1e-4, f"y={y}"
    print(f"PASS  test_corner_pixel_maps_to_known_robot  (x={x:.4f}, y={y:.4f})")


def test_centre_pixel_interpolates():
    """Centre pixel should map inside the robot workspace rectangle."""
    cal = _four_point_cal()
    cal.fit()
    x, y, _ = cal.pixel_to_robot(320, 240)
    assert 0.20 <= x <= 0.35, f"x={x}"
    assert -0.15 <= y <= 0.15, f"y={y}"
    print(f"PASS  test_centre_pixel_interpolates  (x={x:.4f}, y={y:.4f})")


def test_save_and_load():
    """save() then load() must produce identical pixel_to_robot results."""
    cal = _four_point_cal()
    cal.fit()
    x0, y0, z0 = cal.pixel_to_robot(320, 240)

    with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as f:
        path = f.name
    cal.save(path)

    cal2 = CameraRobotCalibration.load(path)
    x1, y1, z1 = cal2.pixel_to_robot(320, 240)

    assert abs(x0 - x1) < 1e-9
    assert abs(y0 - y1) < 1e-9
    assert abs(z0 - z1) < 1e-9
    print("PASS  test_save_and_load")


def test_n_points_tracked():
    cal = CameraRobotCalibration()
    assert cal.n_points == 0
    cal.add_point((0, 0), (0.0, 0.0))
    assert cal.n_points == 1
    cal.clear_points()
    assert cal.n_points == 0
    print("PASS  test_n_points_tracked")


if __name__ == "__main__":
    test_detector_dry_run_returns_one_object()
    test_detector_centroid_at_frame_centre()
    test_detector_has_positive_area()
    test_detector_bbox_contains_centroid()
    test_fit_requires_4_points()
    test_fit_succeeds_with_4_points()
    test_pixel_to_robot_before_fit_raises()
    test_pixel_to_robot_returns_table_z()
    test_reprojection_low_error()
    test_corner_pixel_maps_to_known_robot()
    test_centre_pixel_interpolates()
    test_save_and_load()
    test_n_points_tracked()
    print("\nAll calibration tests passed.")
