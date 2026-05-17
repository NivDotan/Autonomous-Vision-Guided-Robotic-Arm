"""
Tests for HandoffGrasp — Step 11.

Run:
    python tests/test_handoff_grasp.py
or:
    python -m pytest tests/test_handoff_grasp.py -v

All tests use dry-run adapters and mock sensors — no hardware required.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from perception.sam2_adapter import Sam2Adapter
from control.lerobot_adapter import LeRobotAdapter
from control.visual_servo import VisualServo, ServoConfig
from control.distance_servo import DistanceServo, DistanceConfig, MockDistanceSensor
from tasks.handoff_grasp import HandoffGrasp, GraspConfig, GraspPhase

H, W = 480, 640


def _blank():
    return np.zeros((H, W, 3), dtype=np.uint8)


def _make(
    readings=None,
    v_cfg: ServoConfig = None,
    d_cfg: DistanceConfig = None,
    g_cfg: GraspConfig = None,
    centre_click=True,
):
    """Build a complete grasp stack in dry-run mode."""
    if readings is None:
        # default: object starts far, approaches, stabilises close
        readings = [300, 200, 100, 70, 68, 65, 65, 65, 65, 65]

    sam     = Sam2Adapter(dry_run=True, dry_run_radius=60)
    adapter = LeRobotAdapter(dry_run=True)
    vs      = VisualServo(sam, adapter, config=v_cfg or ServoConfig())
    ds      = DistanceServo(MockDistanceSensor(readings), config=d_cfg or DistanceConfig())
    grasp   = HandoffGrasp(vs, ds, adapter, config=g_cfg or GraspConfig(grip_hold_s=0.0))

    click = (W // 2, H // 2) if centre_click else (50, 50)
    return grasp, adapter, click


def _run_to_done(grasp, click, max_steps=200):
    """Drive the grasp loop until done or max_steps exceeded."""
    frame = _blank()
    grasp.start(frame, click)
    results = []
    for _ in range(max_steps):
        r = grasp.step(frame)
        results.append(r)
        if r.done:
            break
    return results


# ------------------------------------------------------------------
# Lifecycle
# ------------------------------------------------------------------

def test_idle_before_start():
    grasp, _, _ = _make()
    assert grasp.phase == GraspPhase.IDLE
    print("PASS  test_idle_before_start")


def test_centering_phase_after_start():
    grasp, _, click = _make()
    grasp.start(_blank(), click)
    assert grasp.phase == GraspPhase.CENTERING
    print("PASS  test_centering_phase_after_start")


def test_step_without_start_returns_not_running():
    grasp, _, _ = _make()
    r = grasp.step(_blank())
    assert not r.done or r.phase == GraspPhase.IDLE
    print("PASS  test_step_without_start_returns_not_running")


# ------------------------------------------------------------------
# Happy path — centred object, stable close readings
# ------------------------------------------------------------------

def test_happy_path_reaches_done():
    """Centred object + stable close readings → DONE within max_steps."""
    readings = [65] * 50   # already close and stable
    d_cfg = DistanceConfig(grip_dist_mm=75, stable_n=3, grip_confirm_n=3,
                           stable_window=15, max_jump=30)
    grasp, adapter, click = _make(readings=readings, d_cfg=d_cfg,
                                  g_cfg=GraspConfig(grip_hold_s=0.0))
    results = _run_to_done(grasp, click, max_steps=100)

    assert results[-1].done, f"Last phase: {results[-1].phase}"
    assert results[-1].success
    assert results[-1].phase == GraspPhase.DONE
    print(f"PASS  test_happy_path_reaches_done  ({len(results)} steps)")


def test_gripper_closed_on_done():
    """adapter.gripper joint should be non-zero after a successful grasp."""
    import math
    readings = [65] * 50
    d_cfg = DistanceConfig(grip_dist_mm=75, stable_n=3, grip_confirm_n=3)
    grasp, adapter, click = _make(readings=readings, d_cfg=d_cfg,
                                  g_cfg=GraspConfig(grip_hold_s=0.0))
    _run_to_done(grasp, click, max_steps=100)
    gripper = adapter.observe()["gripper"]
    assert gripper > 0, f"gripper not closed: {gripper}"
    print(f"PASS  test_gripper_closed_on_done  (gripper={math.degrees(gripper):.1f} deg)")


def test_phase_sequence():
    """Phases must progress through APPROACHING -> DONE (CENTERING may be instant)."""
    readings = [65] * 50
    d_cfg = DistanceConfig(grip_dist_mm=75, stable_n=3, grip_confirm_n=3)
    grasp, _, click = _make(readings=readings, d_cfg=d_cfg,
                            g_cfg=GraspConfig(grip_hold_s=0.0))
    frame = _blank()
    grasp.start(frame, click)

    # The phase right after start() is always CENTERING
    assert grasp.phase == GraspPhase.CENTERING

    results = _run_to_done(grasp, click, max_steps=200)
    phases = [r.phase for r in results]

    assert GraspPhase.APPROACHING in phases, phases
    assert GraspPhase.DONE        in phases, phases
    assert phases[-1] == GraspPhase.DONE
    print(f"PASS  test_phase_sequence  phases={[p.name for p in dict.fromkeys(phases)]}")


# ------------------------------------------------------------------
# Centering timeout
# ------------------------------------------------------------------

def test_centering_timeout_leads_to_failed():
    """With a very short centering timeout and off-centre object → FAILED."""
    g_cfg = GraspConfig(centering_timeout_s=0.0, grip_hold_s=0.0)
    grasp, _, click = _make(g_cfg=g_cfg, centre_click=False)
    grasp.start(_blank(), click)
    # One step should be enough to exceed the 0-second timeout
    import time; time.sleep(0.01)
    r = grasp.step(_blank())
    assert r.phase == GraspPhase.FAILED, r.phase
    assert r.done
    assert not r.success
    print("PASS  test_centering_timeout_leads_to_failed")


# ------------------------------------------------------------------
# Approach timeout
# ------------------------------------------------------------------

def test_approach_timeout_leads_to_failed():
    """Sensor always reports far → approach times out → FAILED."""
    readings = [300] * 1000   # never close
    d_cfg = DistanceConfig(grip_dist_mm=75, stable_n=3, grip_confirm_n=3)
    g_cfg = GraspConfig(approach_timeout_s=0.0, grip_hold_s=0.0)
    grasp, _, click = _make(readings=readings, d_cfg=d_cfg, g_cfg=g_cfg)
    # Force through centering phase first
    grasp.start(_blank(), click)
    grasp._enter_phase_for_test = lambda p: setattr(grasp, '_phase', p)

    import time
    # Manually jump to APPROACHING to test its timeout
    from tasks.handoff_grasp import GraspPhase as GP
    grasp._phase = GP.APPROACHING
    grasp._t_phase = time.monotonic() - 1.0   # simulate 1s already elapsed

    r = grasp.step(_blank())
    assert r.phase == GP.FAILED, r.phase
    print("PASS  test_approach_timeout_leads_to_failed")


# ------------------------------------------------------------------
# Abort
# ------------------------------------------------------------------

def test_abort_returns_failed():
    grasp, _, click = _make()
    grasp.start(_blank(), click)
    r = grasp.abort()
    assert r.phase == GraspPhase.FAILED
    assert r.done
    assert not r.success
    print("PASS  test_abort_returns_failed")


def test_abort_stops_tracking():
    grasp, _, click = _make()
    grasp.start(_blank(), click)
    assert grasp.vs.is_tracking
    grasp.abort()
    assert not grasp.vs.is_tracking
    print("PASS  test_abort_stops_tracking")


# ------------------------------------------------------------------
# Result fields
# ------------------------------------------------------------------

def test_result_has_elapsed():
    grasp, _, click = _make()
    grasp.start(_blank(), click)
    r = grasp.step(_blank())
    assert r.elapsed_s >= 0.0
    print(f"PASS  test_result_has_elapsed  ({r.elapsed_s:.3f}s)")


def test_result_str_contains_phase():
    grasp, _, click = _make()
    grasp.start(_blank(), click)
    r = grasp.step(_blank())
    assert r.phase.name in str(r), str(r)
    print(f"PASS  test_result_str_contains_phase  ({r})")


if __name__ == "__main__":
    test_idle_before_start()
    test_centering_phase_after_start()
    test_step_without_start_returns_not_running()
    test_happy_path_reaches_done()
    test_gripper_closed_on_done()
    test_phase_sequence()
    test_centering_timeout_leads_to_failed()
    test_approach_timeout_leads_to_failed()
    test_abort_returns_failed()
    test_abort_stops_tracking()
    test_result_has_elapsed()
    test_result_str_contains_phase()
    print("\nAll HandoffGrasp tests passed.")
