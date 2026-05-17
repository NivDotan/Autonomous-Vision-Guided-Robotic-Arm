"""
Inverse kinematics for SO-101 style 5-DOF arm.

Input:  target (x, y, z) in metres, desired tool pitch phi, wrist roll
Output: joint angles dict {q1..q5} in radians

Algorithm:
    1.  q1  = atan2(y, x)
    2.  r   = sqrt(x²+y²)
    3.  z2  = z - L0
    4.  Wrist centre (remove L3 contribution):
            rw = r  - L3*cos(phi)
            zw = z2 - L3*sin(phi)
    5.  D   = (rw²+zw² - L1²-L2²) / (2·L1·L2)
    6.  D ∉ [-1,1]  → UnreachableTargetError
    7.  q3  = atan2(±√(1-D²), D)   + = elbow-up,  - = elbow-down
    8.  q2  = atan2(zw, rw) - atan2(L2·sin(q3), L1+L2·cos(q3))
    9.  q4  = phi - q2 - q3
    10. q5  = roll
    11. Check joint limits → JointLimitError
"""

from __future__ import annotations

import math
from typing import Literal

from .geometry import ArmGeometry, SO101_GEOMETRY
from .joint_limits import JointLimits, SO101_LIMITS


class UnreachableTargetError(Exception):
    """Target point is outside the arm's reachable workspace."""


class JointLimitError(Exception):
    """IK solution exists but violates at least one joint limit."""


def inverse_kinematics(
    x: float,
    y: float,
    z: float,
    phi: float,                          # desired tool pitch (rad)
    roll: float = 0.0,                   # desired wrist roll (rad)
    geometry: ArmGeometry = SO101_GEOMETRY,
    limits: JointLimits = SO101_LIMITS,
    elbow_mode: Literal["up", "down"] = "up",
) -> dict[str, float]:
    """
    Solve IK for the given Cartesian target.

    Returns:
        {"q1": ..., "q2": ..., "q3": ..., "q4": ..., "q5": ...}  (radians)

    Raises:
        UnreachableTargetError: target is outside workspace.
        JointLimitError: solution violates joint limits.
    """
    L0, L1, L2, L3 = geometry.L0, geometry.L1, geometry.L2, geometry.L3

    # 1. Base yaw
    q1 = math.atan2(y, x)

    # 2. Radial distance from base axis
    r = math.hypot(x, y)

    # 3. Vertical offset above shoulder
    z2 = z - L0

    # 4. Wrist centre (back-project L3 along tool direction)
    rw = r  - L3 * math.cos(phi)
    zw = z2 - L3 * math.sin(phi)

    # 5. Law of cosines
    reach_sq = rw ** 2 + zw ** 2
    D = (reach_sq - L1 ** 2 - L2 ** 2) / (2.0 * L1 * L2)

    # 6. Reachability check
    if D < -1.0 or D > 1.0:
        raise UnreachableTargetError(
            f"Target ({x:.3f}, {y:.3f}, {z:.3f}) unreachable: D={D:.4f} "
            f"(reach={math.sqrt(reach_sq):.3f} m, arm span={L1+L2:.3f} m)"
        )

    # 7. Elbow angle (sign selects up/down configuration)
    sign = +1.0 if elbow_mode == "up" else -1.0
    q3 = math.atan2(sign * math.sqrt(max(0.0, 1.0 - D ** 2)), D)

    # 8. Shoulder angle
    q2 = math.atan2(zw, rw) - math.atan2(L2 * math.sin(q3), L1 + L2 * math.cos(q3))

    # 9. Wrist pitch to achieve desired tool pitch
    q4 = phi - q2 - q3

    # 10. Wrist roll
    q5 = roll

    solution = {"q1": q1, "q2": q2, "q3": q3, "q4": q4, "q5": q5}

    # 11. Joint limit check
    try:
        limits.check(solution)
    except ValueError as exc:
        raise JointLimitError(str(exc)) from exc

    return solution
