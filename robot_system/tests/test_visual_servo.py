"""
Tests for VisualServo — Step 9.

Run:
    python tests/test_visual_servo.py
or:
    python -m pytest tests/test_visual_servo.py -v

All tests use dry_run=True — no robot, no SAM2 model required.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from perception.sam2_adapter import Sam2Adapter
from control.lerobot_adapter import LeRobotAdapter
from control.visual_servo import VisualServo, ServoConfig, _dead, _clamp

H, W = 480, 640


def _make_servo(
    approach: bool = True,
    config: ServoConfig = ServoConfig(),
) -> tuple[VisualServo, Sam2Adapter, LeRobotAdapter]:
    sam     = Sam2Adapter(dry_run=True)
    adapter = LeRobotAdapter(dry_run=True)
    servo   = VisualServo(sam, adapter, config=config, approach_enabled=approach)
    return servo, sam, adapter


def _blank() -> np.ndarray:
    return np.zeros((H, W, 3), dtype=np.uint8)


# ------------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------------

def test_dead_zone():
    assert _dead(0.03, 0.05) == 0.0          # inside zone
    assert abs(_dead(0.10, 0.05) - 0.05) < 1e-9   # outside zone, reduced
    assert abs(_dead(-0.10, 0.05) + 0.05) < 1e-9  # negative side
    print("PASS  test_dead_zone")


def test_clamp():
    assert _clamp(5.0,  (0.0, 3.0)) == 3.0
    assert _clamp(-5.0, (0.0, 3.0)) == 0.0
    assert _clamp(1.5,  (0.0, 3.0)) == 1.5
    print("PASS  test_clamp")


# ------------------------------------------------------------------
# Lifecycle
# ------------------------------------------------------------------

def test_not_tracking_before_start():
    servo, _, _ = _make_servo()
    assert not servo.is_tracking
    print("PASS  test_not_tracking_before_start")


def test_step_before_start_returns_not_tracking():
    servo, _, _ = _make_servo()
    state = servo.step(_blank())
    assert not state.tracking
    print("PASS  test_step_before_start_returns_not_tracking")


def test_start_tracking_sets_flag():
    servo, _, _ = _make_servo()
    servo.start_tracking(_blank(), click_xy=(320, 240))
    assert servo.is_tracking
    print("PASS  test_start_tracking_sets_flag")


def test_stop_tracking_clears_flag():
    servo, _, _ = _make_servo()
    servo.start_tracking(_blank(), click_xy=(320, 240))
    servo.stop_tracking()
    assert not servo.is_tracking
    print("PASS  test_stop_tracking_clears_flag")


def test_step_while_tracking_returns_tracking_true():
    servo, _, _ = _make_servo()
    servo.start_tracking(_blank(), click_xy=(320, 240))
    state = servo.step(_blank())
    assert state.tracking
    print("PASS  test_step_while_tracking_returns_tracking_true")


# ------------------------------------------------------------------
# Centred object — minimal corrections
# ------------------------------------------------------------------

def test_centred_click_near_zero_correction():
    """Object at frame centre → errors ≈ 0 → corrections ≈ 0."""
    servo, _, _ = _make_servo()
    servo.start_tracking(_blank(), click_xy=(W // 2, H // 2))
    state = servo.step(_blank())

    # Normalised centroid should be near 0
    assert abs(state.centroid_norm[0]) < 0.05, state.centroid_norm
    assert abs(state.centroid_norm[1]) < 0.05, state.centroid_norm
    # Corrections should be 0 (inside dead zone)
    assert state.pan_correction  == 0.0, state.pan_correction
    assert state.tilt_correction == 0.0, state.tilt_correction
    print("PASS  test_centred_click_near_zero_correction")


def test_centred_click_is_centred_flag():
    servo, _, _ = _make_servo()
    servo.start_tracking(_blank(), click_xy=(W // 2, H // 2))
    state = servo.step(_blank())
    assert state.is_centred, f"centroid_norm={state.centroid_norm}"
    print("PASS  test_centred_click_is_centred_flag")


# ------------------------------------------------------------------
# Off-centre object — correct correction sign
# ------------------------------------------------------------------

def test_left_click_produces_negative_pan():
    """Object left of centre → centroid_norm.x < 0 → d_pan < 0 (pan left to track)."""
    servo, _, adapter = _make_servo()
    servo.start_tracking(_blank(), click_xy=(50, H // 2))   # far left
    state = servo.step(_blank())
    assert state.pan_correction < 0, f"pan={state.pan_correction:.4f}"
    print(f"PASS  test_left_click_produces_negative_pan  (pan={state.pan_correction:.4f})")


def test_right_click_produces_positive_pan():
    """Object right of centre → centroid_norm.x > 0 → d_pan > 0 (pan right to track)."""
    servo, _, adapter = _make_servo()
    servo.start_tracking(_blank(), click_xy=(W - 50, H // 2))
    state = servo.step(_blank())
    assert state.pan_correction > 0, f"pan={state.pan_correction:.4f}"
    print(f"PASS  test_right_click_produces_positive_pan  (pan={state.pan_correction:.4f})")


def test_top_click_produces_negative_tilt():
    """Object above centre → centroid_norm.y < 0 → d_tilt < 0 (tilt up to track)."""
    servo, _, _ = _make_servo()
    servo.start_tracking(_blank(), click_xy=(W // 2, 30))   # top of frame
    state = servo.step(_blank())
    assert state.tilt_correction < 0, f"tilt={state.tilt_correction:.4f}"
    print(f"PASS  test_top_click_produces_negative_tilt  (tilt={state.tilt_correction:.4f})")


# ------------------------------------------------------------------
# Approach
# ------------------------------------------------------------------

def test_approach_disabled_no_reach_correction():
    servo, _, _ = _make_servo(approach=False)
    servo.start_tracking(_blank(), click_xy=(W // 2, H // 2))
    state = servo.step(_blank())
    assert state.reach_correction == 0.0
    print("PASS  test_approach_disabled_no_reach_correction")


def test_approach_small_object_positive_reach():
    """Small object (low fill) → positive reach correction (extend elbow)."""
    cfg = ServoConfig(target_fill=0.20, reached_fill=0.18, dead_reach=0.0)
    servo, _, _ = _make_servo(approach=True, config=cfg)
    # dry_run_radius=20 → small mask → fill << 0.20
    servo.sam.dry_run_radius = 20
    servo.start_tracking(_blank(), click_xy=(W // 2, H // 2))
    state = servo.step(_blank())
    assert state.reach_correction > 0, f"reach={state.reach_correction}"
    print(f"PASS  test_approach_small_object_positive_reach  (reach={state.reach_correction:.4f})")


def test_object_reached_flag():
    """Large synthetic mask should set object_reached=True."""
    cfg = ServoConfig(reached_fill=0.01)   # very low threshold
    servo, _, _ = _make_servo(config=cfg)
    servo.sam.dry_run_radius = 200         # big ellipse → high fill
    servo.start_tracking(_blank(), click_xy=(W // 2, H // 2))
    state = servo.step(_blank())
    assert state.object_reached, f"fill={state.frame_fill:.4f}"
    print(f"PASS  test_object_reached_flag  (fill={state.frame_fill:.4f})")


# ------------------------------------------------------------------
# Joint accumulation
# ------------------------------------------------------------------

def test_corrections_accumulate_in_adapter():
    """After several steps with a left-shifted object, shoulder_pan should decrease."""
    servo, _, adapter = _make_servo()
    servo.start_tracking(_blank(), click_xy=(30, H // 2))   # far left → ex < 0 → d_pan < 0
    pan_vals = []
    for _ in range(5):
        servo.step(_blank())
        pan_vals.append(adapter.observe()["shoulder_pan"])

    # Pan should be monotonically decreasing (panning left to track left object)
    for a, b in zip(pan_vals, pan_vals[1:]):
        assert b <= a + 1e-9, f"pan not decreasing: {pan_vals}"
    print(f"PASS  test_corrections_accumulate_in_adapter  (final pan={pan_vals[-1]:.4f} rad)")


# ------------------------------------------------------------------
# Joint limit clamping
# ------------------------------------------------------------------

def test_joint_limits_respected():
    """Extreme off-centre clicks should not push joints past limits."""
    cfg = ServoConfig(Kp_pan=10.0, pan_limit=(-0.5, 0.5))   # tight limit
    servo, _, adapter = _make_servo(config=cfg)
    servo.start_tracking(_blank(), click_xy=(0, H // 2))    # hard left
    for _ in range(20):
        servo.step(_blank())
    pan = adapter.observe()["shoulder_pan"]
    assert pan <= 0.5 + 1e-9, f"pan exceeded limit: {pan}"
    print(f"PASS  test_joint_limits_respected  (pan={pan:.4f})")


if __name__ == "__main__":
    test_dead_zone()
    test_clamp()
    test_not_tracking_before_start()
    test_step_before_start_returns_not_tracking()
    test_start_tracking_sets_flag()
    test_stop_tracking_clears_flag()
    test_step_while_tracking_returns_tracking_true()
    test_centred_click_near_zero_correction()
    test_centred_click_is_centred_flag()
    test_left_click_produces_negative_pan()
    test_right_click_produces_positive_pan()
    test_top_click_produces_negative_tilt()
    test_approach_disabled_no_reach_correction()
    test_approach_small_object_positive_reach()
    test_object_reached_flag()
    test_corrections_accumulate_in_adapter()
    test_joint_limits_respected()
    print("\nAll VisualServo tests passed.")
