"""
Tests for inverse_kinematics.py — Step 3 acceptance tests.

Run:
    python tests/test_inverse_kinematics.py
or:
    python -m pytest tests/test_inverse_kinematics.py -v
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from kinematics.geometry import SO101_GEOMETRY
from kinematics.joint_limits import SO101_LIMITS, JointLimits
from kinematics.forward_kinematics import forward_kinematics
from kinematics.inverse_kinematics import (
    inverse_kinematics,
    UnreachableTargetError,
    JointLimitError,
)

XYZ_TOL = 1e-6   # 1 µm — IK→FK round-trip tolerance


def _fk(q: dict[str, float]) -> tuple[float, float, float]:
    r = forward_kinematics(**q)
    return r["x"], r["y"], r["z"]


def test_ik_fk_roundtrip_elbow_up():
    """IK → FK should recover the original target (elbow up)."""
    target = (0.20, 0.05, 0.10)
    x, y, z = target
    phi = math.radians(-30)

    q = inverse_kinematics(x, y, z, phi=phi, elbow_mode="up")
    rx, ry, rz = _fk(q)

    assert abs(rx - x) < XYZ_TOL, f"x error: {abs(rx-x):.2e} m"
    assert abs(ry - y) < XYZ_TOL, f"y error: {abs(ry-y):.2e} m"
    assert abs(rz - z) < XYZ_TOL, f"z error: {abs(rz-z):.2e} m"
    print(f"PASS  test_ik_fk_roundtrip_elbow_up   q={{{', '.join(f'{k}:{math.degrees(v):.1f}°' for k,v in q.items())}}}")


def test_ik_fk_roundtrip_elbow_down():
    """IK → FK should recover the original target (elbow down)."""
    target = (0.18, 0.0, 0.08)
    x, y, z = target
    phi = math.radians(-45)

    q = inverse_kinematics(x, y, z, phi=phi, elbow_mode="down")
    rx, ry, rz = _fk(q)

    assert abs(rx - x) < XYZ_TOL, f"x error: {abs(rx-x):.2e} m"
    assert abs(ry - y) < XYZ_TOL, f"y error: {abs(ry-y):.2e} m"
    assert abs(rz - z) < XYZ_TOL, f"z error: {abs(rz-z):.2e} m"
    print(f"PASS  test_ik_fk_roundtrip_elbow_down  q={{{', '.join(f'{k}:{math.degrees(v):.1f}°' for k,v in q.items())}}}")


def test_elbow_up_vs_down_different_q3():
    """Elbow-up and elbow-down should give different q3 signs."""
    x, y, z, phi = 0.20, 0.0, 0.10, 0.0
    q_up   = inverse_kinematics(x, y, z, phi=phi, elbow_mode="up")
    q_down = inverse_kinematics(x, y, z, phi=phi, elbow_mode="down")
    assert q_up["q3"] * q_down["q3"] < 0 or abs(q_up["q3"] - q_down["q3"]) > 1e-6, \
        "elbow-up and elbow-down q3 should differ"
    print("PASS  test_elbow_up_vs_down_different_q3")


def test_unreachable_raises():
    """Point far beyond arm span must raise UnreachableTargetError."""
    try:
        inverse_kinematics(10.0, 0.0, 0.0, phi=0.0)
        assert False, "Should have raised UnreachableTargetError"
    except UnreachableTargetError as e:
        print(f"PASS  test_unreachable_raises  ({e})")


def test_unreachable_too_close():
    """Point whose wrist centre collapses to zero reach must raise UnreachableTargetError.

    With phi=0, setting x=L3 and z=L0 places the wrist centre exactly at the
    shoulder pivot (rw=0, zw=0) → reach=0 → D = -(L1²+L2²)/(2L1L2) < -1.
    """
    g = SO101_GEOMETRY
    try:
        inverse_kinematics(g.L3, 0.0, g.L0, phi=0.0)
        assert False, "Should have raised UnreachableTargetError"
    except UnreachableTargetError as e:
        print(f"PASS  test_unreachable_too_close  ({e})")


def test_joint_limit_raises():
    """A valid geometric solution that violates limits must raise JointLimitError."""
    # Use a very tight limit set that will reject a normal solution
    tight = JointLimits(
        q1=(0.0, 0.0),    # zero range on base → any q1 != 0 fails
        q2=(-math.pi, math.pi),
        q3=(-math.pi, math.pi),
        q4=(-math.pi, math.pi),
        q5=(-math.pi, math.pi),
    )
    try:
        # non-zero y forces q1 != 0, which violates tight.q1
        inverse_kinematics(0.15, 0.10, 0.10, phi=0.0, limits=tight)
        assert False, "Should have raised JointLimitError"
    except JointLimitError as e:
        print(f"PASS  test_joint_limit_raises  ({e})")


def test_pitch_recovered():
    """FK pitch at IK solution must equal the requested phi."""
    x, y, z = 0.18, 0.0, 0.12
    phi = math.radians(-20)
    q = inverse_kinematics(x, y, z, phi=phi)
    fk = forward_kinematics(**q)
    assert abs(fk["pitch"] - phi) < 1e-9, \
        f"pitch mismatch: got {math.degrees(fk['pitch']):.2f}°, want {math.degrees(phi):.2f}°"
    print("PASS  test_pitch_recovered")


def test_roll_passthrough():
    """q5 must equal the requested roll."""
    roll = math.radians(45)
    q = inverse_kinematics(0.20, 0.0, 0.10, phi=0.0, roll=roll)
    assert abs(q["q5"] - roll) < 1e-12
    print("PASS  test_roll_passthrough")


if __name__ == "__main__":
    test_ik_fk_roundtrip_elbow_up()
    test_ik_fk_roundtrip_elbow_down()
    test_elbow_up_vs_down_different_q3()
    test_unreachable_raises()
    test_unreachable_too_close()
    test_joint_limit_raises()
    test_pitch_recovered()
    test_roll_passthrough()
    print("\nAll IK tests passed.")
