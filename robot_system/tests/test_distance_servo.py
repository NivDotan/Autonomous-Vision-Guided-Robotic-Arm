"""
Tests for DistanceServo — Step 10.

Run:
    python tests/test_distance_servo.py
or:
    python -m pytest tests/test_distance_servo.py -v

All tests use MockDistanceSensor — no hardware required.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from control.distance_servo import (
    DistanceServo, DistanceConfig, MockDistanceSensor, DistanceState,
)
from control.lerobot_adapter import LeRobotAdapter


def _servo(readings, **cfg_kwargs):
    sensor  = MockDistanceSensor(readings)
    config  = DistanceConfig(**cfg_kwargs)
    adapter = LeRobotAdapter(dry_run=True)
    servo   = DistanceServo(sensor, config)
    return servo, adapter


def _run(servo, adapter, n):
    return [servo.step(adapter) for _ in range(n)]


# ------------------------------------------------------------------
# Buffer / data
# ------------------------------------------------------------------

def test_no_data_not_stable():
    """No readings yet → is_stable=False, should_grip=False."""
    servo, adapter = _servo([])
    state = servo.step(adapter)
    assert not state.is_stable
    assert not state.should_grip
    assert state.dist_mm is None
    print("PASS  test_no_data_not_stable")


def test_dist_mm_reported():
    """step() returns the latest sensor reading."""
    servo, adapter = _servo([250])
    state = servo.step(adapter)
    assert state.dist_mm == 250
    print("PASS  test_dist_mm_reported")


def test_fewer_than_n_readings_not_stable():
    """Buffer needs at least stable_n readings before validating."""
    servo, adapter = _servo([70, 71], stable_n=3)
    states = _run(servo, adapter, 2)
    assert not any(s.is_stable for s in states)
    print("PASS  test_fewer_than_n_readings_not_stable")


# ------------------------------------------------------------------
# Stability validation
# ------------------------------------------------------------------

def test_stable_close_readings_trigger_confirm():
    """Three identical close readings → is_stable=True, is_close=True."""
    servo, adapter = _servo([65, 65, 65], grip_dist_mm=75, stable_window=15, stable_n=3)
    states = _run(servo, adapter, 3)
    assert states[-1].is_stable
    assert states[-1].is_close
    print("PASS  test_stable_close_readings_trigger_confirm")


def test_large_spread_not_stable():
    """Readings with spread > stable_window → not stable."""
    servo, adapter = _servo([60, 90, 60], grip_dist_mm=75, stable_window=15, stable_n=3)
    states = _run(servo, adapter, 3)
    assert not states[-1].is_stable
    print("PASS  test_large_spread_not_stable")


def test_jump_in_buffer_not_stable():
    """A big jump anywhere in the buffer → not stable even if last 3 are fine."""
    # First reading causes a jump vs initial buffer state isn't an issue,
    # but a jump between two consecutive readings in the buffer is.
    readings = [200, 65, 65, 65]   # jump of 135 between reading[0] and [1]
    servo, adapter = _servo(
        readings, grip_dist_mm=75, stable_window=15, max_jump=30, stable_n=3
    )
    states = _run(servo, adapter, 4)
    # Even on step 4, the buffer still contains the 200→65 jump
    assert not states[-1].is_stable
    print("PASS  test_jump_in_buffer_not_stable")


def test_readings_above_grip_dist_not_close():
    """Stable readings above grip_dist_mm → is_stable but not is_close."""
    servo, adapter = _servo([120, 120, 120], grip_dist_mm=75, stable_window=15, stable_n=3)
    states = _run(servo, adapter, 3)
    assert states[-1].is_stable
    assert not states[-1].is_close
    print("PASS  test_readings_above_grip_dist_not_close")


# ------------------------------------------------------------------
# Confirmation counter
# ------------------------------------------------------------------

def test_confirm_count_increments():
    """confirm_count must grow each cycle when stable + close."""
    servo, adapter = _servo([65] * 10, grip_dist_mm=75, stable_n=3, grip_confirm_n=5)
    states = _run(servo, adapter, 10)
    counts = [s.confirm_count for s in states]
    # After filling the buffer (step 3 onward), count should keep growing
    assert counts[-1] >= 5, f"counts={counts}"
    print(f"PASS  test_confirm_count_increments  (counts={counts})")


def test_confirm_resets_on_jump():
    """A sudden jump resets the confirmation counter."""
    readings = [65, 65, 65, 65, 250, 65, 65, 65]   # spike at index 4
    servo, adapter = _servo(readings, grip_dist_mm=75, stable_n=3, max_jump=30)
    states = _run(servo, adapter, len(readings))
    # After the spike the confirm count must drop to 0
    spike_idx = 4
    assert states[spike_idx].confirm_count == 0, \
        f"confirm_count after spike: {states[spike_idx].confirm_count}"
    print("PASS  test_confirm_resets_on_jump")


def test_should_grip_after_confirm_n():
    """should_grip becomes True after grip_confirm_n consecutive stable cycles."""
    cfg = dict(grip_dist_mm=75, stable_n=3, grip_confirm_n=3)
    servo, adapter = _servo([65] * 20, **cfg)
    states = _run(servo, adapter, 20)
    # should_grip must be True at some point after enough stable readings
    assert any(s.should_grip for s in states), "should_grip never became True"
    print("PASS  test_should_grip_after_confirm_n")


def test_should_grip_false_for_far_object():
    """Object far away → should never grip."""
    servo, adapter = _servo([300] * 20, grip_dist_mm=75, stable_n=3, grip_confirm_n=3)
    states = _run(servo, adapter, 20)
    assert not any(s.should_grip for s in states)
    print("PASS  test_should_grip_false_for_far_object")


# ------------------------------------------------------------------
# Approach elbow drive
# ------------------------------------------------------------------

def test_elbow_advances_when_far():
    """With object far away, elbow_flex should increase each step."""
    servo, adapter = _servo([300] * 10, grip_dist_mm=75, Kp_elbow=0.002)
    elbow_vals = []
    for _ in range(10):
        servo.step(adapter)
        elbow_vals.append(adapter.observe()["elbow_flex"])
    assert elbow_vals[-1] > elbow_vals[0], f"elbow didn't advance: {elbow_vals}"
    print(f"PASS  test_elbow_advances_when_far  (elbow 0 -> {elbow_vals[-1]:.4f} rad)")


def test_no_elbow_drive_when_close():
    """When object is already close (stable), elbow should not advance."""
    servo, adapter = _servo([65] * 10, grip_dist_mm=75, Kp_elbow=0.002, stable_n=3)
    states = _run(servo, adapter, 10)
    # Once close, elbow_delta should be 0
    close_states = [s for s in states if s.is_close]
    assert all(s.elbow_delta == 0.0 for s in close_states), \
        [s.elbow_delta for s in close_states]
    print("PASS  test_no_elbow_drive_when_close")


def test_elbow_respects_limits():
    """Elbow should not exceed elbow_limit even after many steps."""
    limit = 0.3
    servo, adapter = _servo(
        [500] * 50, grip_dist_mm=75, Kp_elbow=0.01,
        elbow_limit=(-limit, limit),
    )
    _run(servo, adapter, 50)
    elbow = adapter.observe()["elbow_flex"]
    assert elbow <= limit + 1e-9, f"elbow exceeded limit: {elbow}"
    print(f"PASS  test_elbow_respects_limits  (elbow={elbow:.4f}, limit={limit})")


# ------------------------------------------------------------------
# reset()
# ------------------------------------------------------------------

def test_reset_clears_buffer_and_counter():
    servo, adapter = _servo([65] * 20, grip_dist_mm=75, stable_n=3, grip_confirm_n=3)
    _run(servo, adapter, 20)
    servo.reset()
    state = servo.step(adapter)
    assert state.confirm_count == 0
    assert not state.is_stable
    print("PASS  test_reset_clears_buffer_and_counter")


if __name__ == "__main__":
    test_no_data_not_stable()
    test_dist_mm_reported()
    test_fewer_than_n_readings_not_stable()
    test_stable_close_readings_trigger_confirm()
    test_large_spread_not_stable()
    test_jump_in_buffer_not_stable()
    test_readings_above_grip_dist_not_close()
    test_confirm_count_increments()
    test_confirm_resets_on_jump()
    test_should_grip_after_confirm_n()
    test_should_grip_false_for_far_object()
    test_elbow_advances_when_far()
    test_no_elbow_drive_when_close()
    test_elbow_respects_limits()
    test_reset_clears_buffer_and_counter()
    print("\nAll DistanceServo tests passed.")
