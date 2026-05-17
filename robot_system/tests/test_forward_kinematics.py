"""
Tests for forward_kinematics.py — Step 2 acceptance tests.

Run:
    python -m pytest tests/test_forward_kinematics.py -v
or:
    python tests/test_forward_kinematics.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from kinematics.geometry import ArmGeometry, SO101_GEOMETRY
from kinematics.forward_kinematics import forward_kinematics, fk_from_dict


TOL = 1e-9   # floating-point tolerance


def test_all_zeros_extends_along_x():
    """All joints zero → arm fully extended along +X, z = L0."""
    g = SO101_GEOMETRY
    result = forward_kinematics(0, 0, 0, 0, 0, g)

    expected_x = g.L1 + g.L2 + g.L3
    assert abs(result["x"] - expected_x) < TOL, (
        f"x should be {expected_x:.4f}, got {result['x']:.4f}"
    )
    assert abs(result["y"]) < TOL, f"y should be 0, got {result['y']}"
    assert abs(result["z"] - g.L0) < TOL, (
        f"z should be {g.L0:.4f}, got {result['z']:.4f}"
    )
    assert abs(result["pitch"]) < TOL, "pitch should be 0"
    assert abs(result["roll"]) < TOL, "roll should be 0"
    print("PASS  test_all_zeros_extends_along_x")


def test_q1_90_rotates_to_y():
    """q1=90° → arm points along +Y."""
    g = SO101_GEOMETRY
    q1 = math.pi / 2
    result = forward_kinematics(q1, 0, 0, 0, 0, g)

    expected_y = g.L1 + g.L2 + g.L3
    assert abs(result["x"]) < TOL, f"x should be ~0, got {result['x']}"
    assert abs(result["y"] - expected_y) < TOL, (
        f"y should be {expected_y:.4f}, got {result['y']:.4f}"
    )
    print("PASS  test_q1_90_rotates_to_y")


def test_pitch_equals_sum_of_pitch_joints():
    """pitch == q2 + q3 + q4 for arbitrary angles."""
    q2, q3, q4 = 0.3, -0.5, 0.2
    result = forward_kinematics(0, q2, q3, q4, 0)
    assert abs(result["pitch"] - (q2 + q3 + q4)) < TOL, (
        f"pitch should be {q2+q3+q4:.4f}, got {result['pitch']:.4f}"
    )
    print("PASS  test_pitch_equals_sum_of_pitch_joints")


def test_roll_equals_q5():
    """roll == q5."""
    q5 = 1.2
    result = forward_kinematics(0, 0, 0, 0, q5)
    assert abs(result["roll"] - q5) < TOL
    print("PASS  test_roll_equals_q5")


def test_q2_90_lifts_arm_up():
    """q2=90°, q3=q4=0 → all links point straight up.
    x≈0, z = L0 + L1 + L2 + L3."""
    g = SO101_GEOMETRY
    result = forward_kinematics(0, math.pi / 2, 0, 0, 0, g)
    assert abs(result["x"]) < 1e-9, f"x should be ~0, got {result['x']}"
    expected_z = g.L0 + g.L1 + g.L2 + g.L3
    assert abs(result["z"] - expected_z) < 1e-9, (
        f"z should be {expected_z:.4f}, got {result['z']:.4f}"
    )
    print("PASS  test_q2_90_lifts_arm_up")


def test_fk_from_dict():
    """fk_from_dict is consistent with forward_kinematics."""
    q = {"q1": 0.1, "q2": 0.2, "q3": -0.3, "q4": 0.1, "q5": 0.5}
    r1 = forward_kinematics(**q)
    r2 = fk_from_dict(q)
    for k in ("x", "y", "z", "pitch", "roll"):
        assert abs(r1[k] - r2[k]) < TOL
    print("PASS  test_fk_from_dict")


if __name__ == "__main__":
    test_all_zeros_extends_along_x()
    test_q1_90_rotates_to_y()
    test_pitch_equals_sum_of_pitch_joints()
    test_roll_equals_q5()
    test_q2_90_lifts_arm_up()
    test_fk_from_dict()
    print("\nAll FK tests passed.")
