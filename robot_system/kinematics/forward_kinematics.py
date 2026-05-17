"""
Forward kinematics for SO-101 style 5-DOF arm.

Joints:
    q1 — base yaw        (rotation around vertical Z)
    q2 — shoulder pitch
    q3 — elbow pitch
    q4 — wrist pitch
    q5 — wrist roll

Equations:
    r  = L1*cos(q2) + L2*cos(q2+q3) + L3*cos(q2+q3+q4)
    z  = L0 + L1*sin(q2) + L2*sin(q2+q3) + L3*sin(q2+q3+q4)
    x  = r * cos(q1)
    y  = r * sin(q1)
"""

from __future__ import annotations

import math
from typing import TypedDict

from .geometry import ArmGeometry, SO101_GEOMETRY


class FKResult(TypedDict):
    x: float
    y: float
    z: float
    pitch: float   # total tool pitch = q2 + q3 + q4
    roll: float    # wrist roll = q5


def forward_kinematics(
    q1: float,
    q2: float,
    q3: float,
    q4: float,
    q5: float,
    geometry: ArmGeometry = SO101_GEOMETRY,
) -> FKResult:
    """Return end-effector pose from joint angles (all in radians)."""
    L0, L1, L2, L3 = geometry.L0, geometry.L1, geometry.L2, geometry.L3

    r = (L1 * math.cos(q2)
         + L2 * math.cos(q2 + q3)
         + L3 * math.cos(q2 + q3 + q4))

    z = (L0
         + L1 * math.sin(q2)
         + L2 * math.sin(q2 + q3)
         + L3 * math.sin(q2 + q3 + q4))

    x = r * math.cos(q1)
    y = r * math.sin(q1)

    return FKResult(
        x=x,
        y=y,
        z=z,
        pitch=q2 + q3 + q4,
        roll=q5,
    )


def fk_from_dict(
    q: dict[str, float],
    geometry: ArmGeometry = SO101_GEOMETRY,
) -> FKResult:
    """Convenience wrapper accepting a joint-name dict."""
    return forward_kinematics(
        q1=q.get("q1", 0.0),
        q2=q.get("q2", 0.0),
        q3=q.get("q3", 0.0),
        q4=q.get("q4", 0.0),
        q5=q.get("q5", 0.0),
        geometry=geometry,
    )
