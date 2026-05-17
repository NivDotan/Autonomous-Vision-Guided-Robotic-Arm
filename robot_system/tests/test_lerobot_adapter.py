"""
Tests for LeRobotAdapter — Step 5.

Run:
    python tests/test_lerobot_adapter.py
or:
    python -m pytest tests/test_lerobot_adapter.py -v
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from control.lerobot_adapter import LeRobotAdapter


def test_dry_run_observe_zeros():
    """Fresh adapter observe() returns all zeros."""
    a = LeRobotAdapter(dry_run=True)
    obs = a.observe()
    assert set(obs.keys()) == set(LeRobotAdapter.JOINT_NAMES)
    assert all(v == 0.0 for v in obs.values()), f"Expected zeros, got {obs}"
    print("PASS  test_dry_run_observe_zeros")


def test_move_joints_partial_update(capsys=None):
    """move_joints with a subset only changes those joints."""
    a = LeRobotAdapter(dry_run=True)
    a.move_joints({"shoulder_pan": math.radians(30)})
    obs = a.observe()
    assert abs(obs["shoulder_pan"] - math.radians(30)) < 1e-9
    # others stay at zero
    for name in LeRobotAdapter.JOINT_NAMES:
        if name != "shoulder_pan":
            assert obs[name] == 0.0, f"{name} should still be 0"
    print("PASS  test_move_joints_partial_update")


def test_move_joints_remembered():
    """Second move_joints call accumulates on top of previous."""
    a = LeRobotAdapter(dry_run=True)
    a.move_joints({"elbow_flex": math.radians(45)})
    a.move_joints({"wrist_flex": math.radians(-20)})
    obs = a.observe()
    assert abs(obs["elbow_flex"] - math.radians(45)) < 1e-9
    assert abs(obs["wrist_flex"] - math.radians(-20)) < 1e-9
    print("PASS  test_move_joints_remembered")


def test_move_to_home_resets():
    """move_to_home() sets all joints back to zero."""
    a = LeRobotAdapter(dry_run=True)
    a.move_joints({"shoulder_pan": 1.0, "elbow_flex": -0.5})
    a.move_to_home()
    obs = a.observe()
    assert all(v == 0.0 for v in obs.values()), f"Home should be zeros, got {obs}"
    print("PASS  test_move_to_home_resets")


def test_gripper_helpers():
    """open/close gripper set gripper joint correctly."""
    a = LeRobotAdapter(dry_run=True)
    a.close_gripper()
    assert a.observe()["gripper"] == math.radians(45)
    a.open_gripper()
    assert a.observe()["gripper"] == 0.0
    print("PASS  test_gripper_helpers")


def test_no_robot_live_raises():
    """Passing dry_run=False without a robot object must raise ValueError."""
    try:
        LeRobotAdapter(robot=None, dry_run=False)
        assert False, "Should have raised ValueError"
    except ValueError as e:
        print(f"PASS  test_no_robot_live_raises  ({e})")


def test_context_manager():
    """Adapter works as a context manager (no crash in dry-run)."""
    with LeRobotAdapter(dry_run=True) as a:
        a.move_joints({"wrist_roll": math.radians(90)})
        assert abs(a.observe()["wrist_roll"] - math.radians(90)) < 1e-9
    print("PASS  test_context_manager")


if __name__ == "__main__":
    test_dry_run_observe_zeros()
    test_move_joints_partial_update()
    test_move_joints_remembered()
    test_move_to_home_resets()
    test_gripper_helpers()
    test_no_robot_live_raises()
    test_context_manager()
    print("\nAll LeRobotAdapter tests passed.")
