"""
Tests for Trajectory — Step 6.

Run:
    python tests/test_trajectory.py
or:
    python -m pytest tests/test_trajectory.py -v
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from control.trajectory import Trajectory, EasingCurve, _apply_easing
from control.lerobot_adapter import LeRobotAdapter

TOL = 1e-9


# ------------------------------------------------------------------
# Easing math
# ------------------------------------------------------------------

def test_easing_endpoints():
    """Every easing curve must map 0->0 and 1->1."""
    for curve in EasingCurve:
        assert abs(_apply_easing(0.0, curve)) < TOL, f"{curve} at 0"
        assert abs(_apply_easing(1.0, curve) - 1.0) < TOL, f"{curve} at 1"
    print("PASS  test_easing_endpoints")


def test_easing_clamp():
    """Values outside [0,1] are clamped."""
    for curve in EasingCurve:
        assert _apply_easing(-1.0, curve) == _apply_easing(0.0, curve)
        assert _apply_easing(2.0,  curve) == _apply_easing(1.0, curve)
    print("PASS  test_easing_clamp")


def test_ease_in_out_midpoint():
    """Ease-in-out midpoint should be exactly 0.5 (symmetric)."""
    v = _apply_easing(0.5, EasingCurve.EASE_IN_OUT)
    assert abs(v - 0.5) < TOL, f"mid={v}"
    print("PASS  test_ease_in_out_midpoint")


# ------------------------------------------------------------------
# Trajectory.at()
# ------------------------------------------------------------------

def test_at_start_and_end():
    """at(0) == start,  at(1) == goal  for all easing curves."""
    start = {"elbow_flex": 0.0, "shoulder_lift": 0.0}
    goal  = {"elbow_flex": 1.0, "shoulder_lift": -0.5}
    for curve in EasingCurve:
        traj = Trajectory(dict(start), dict(goal), easing=curve)
        s = traj.at(0.0)
        g = traj.at(1.0)
        for j in goal:
            assert abs(s[j] - start[j]) < TOL, f"{curve} start {j}"
            assert abs(g[j] - goal[j])  < TOL, f"{curve} end {j}"
    print("PASS  test_at_start_and_end")


def test_partial_goal_subset():
    """Goal with subset of joints only interpolates those joints."""
    start = {"shoulder_pan": 0.0, "elbow_flex": 0.5, "wrist_roll": 1.0}
    goal  = {"elbow_flex": -0.5}   # only one joint
    traj  = Trajectory(dict(start), dict(goal))
    mid   = traj.at(0.5)
    assert "shoulder_pan" not in mid    # not in goal → not in output
    assert "elbow_flex" in mid
    assert abs(mid["elbow_flex"] - 0.0) < 1e-6   # midpoint of 0.5 -> -0.5
    print("PASS  test_partial_goal_subset")


def test_monotone_linear():
    """Linear easing must be strictly monotone between start and end."""
    start = {"q": 0.0}
    goal  = {"q": 1.0}
    traj  = Trajectory(dict(start), dict(goal), easing=EasingCurve.LINEAR)
    vals  = [traj.at(i / 10)["q"] for i in range(11)]
    for a, b in zip(vals, vals[1:]):
        assert b >= a - TOL, f"not monotone: {a} -> {b}"
    print("PASS  test_monotone_linear")


# ------------------------------------------------------------------
# steps() generator
# ------------------------------------------------------------------

def test_steps_count():
    """steps() yields exactly ceil(duration*hz)+1 waypoints."""
    traj = Trajectory({"j": 0.0}, {"j": 1.0}, duration=2.0)
    pts  = list(traj.steps(hz=10))
    assert len(pts) == 21, f"expected 21, got {len(pts)}"
    print("PASS  test_steps_count")


def test_steps_first_and_last():
    """First step is start, last step is goal."""
    start = {"elbow_flex": math.radians(10)}
    goal  = {"elbow_flex": math.radians(90)}
    traj  = Trajectory(dict(start), dict(goal), duration=1.0)
    pts   = list(traj.steps(hz=20))
    assert abs(pts[0]["elbow_flex"]  - math.radians(10)) < TOL
    assert abs(pts[-1]["elbow_flex"] - math.radians(90)) < TOL
    print("PASS  test_steps_first_and_last")


# ------------------------------------------------------------------
# Integration with LeRobotAdapter
# ------------------------------------------------------------------

def test_execute_dry_run(capsys=None):
    """execute() against a dry-run adapter ends at goal joints."""
    adapter = LeRobotAdapter(dry_run=True)
    start   = adapter.observe()          # all zeros
    goal    = {"shoulder_lift": math.radians(30), "elbow_flex": math.radians(-20)}

    traj = Trajectory.between(start, goal, duration=0.1, easing=EasingCurve.EASE_OUT)
    traj.execute(adapter, hz=50)

    obs = adapter.observe()
    assert abs(obs["shoulder_lift"] - math.radians(30)) < TOL
    assert abs(obs["elbow_flex"]    - math.radians(-20)) < TOL
    print("PASS  test_execute_dry_run")


def test_between_factory():
    """Trajectory.between() is equivalent to the constructor."""
    s = {"j": 0.0}
    g = {"j": 1.0}
    t1 = Trajectory(dict(s), dict(g), duration=3.0, easing=EasingCurve.EASE_IN)
    t2 = Trajectory.between(s, g, duration=3.0, easing=EasingCurve.EASE_IN)
    for i in range(11):
        t = i / 10
        assert abs(t1.at(t)["j"] - t2.at(t)["j"]) < TOL
    print("PASS  test_between_factory")


if __name__ == "__main__":
    test_easing_endpoints()
    test_easing_clamp()
    test_ease_in_out_midpoint()
    test_at_start_and_end()
    test_partial_goal_subset()
    test_monotone_linear()
    test_steps_count()
    test_steps_first_and_last()
    test_execute_dry_run()
    test_between_factory()
    print("\nAll Trajectory tests passed.")
