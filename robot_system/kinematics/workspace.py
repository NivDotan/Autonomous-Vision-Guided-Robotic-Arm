"""
Workspace sampling — generates reachable (x, y, z) points by sweeping
joint angles through their limits using FK.

No robot connection required.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .geometry import ArmGeometry, SO101_GEOMETRY
from .joint_limits import JointLimits, SO101_LIMITS
from .forward_kinematics import forward_kinematics


@dataclass
class WorkspaceSample:
    x: float
    y: float
    z: float
    r: float   # radial distance from base axis


def sample_workspace(
    geometry: ArmGeometry = SO101_GEOMETRY,
    limits: JointLimits = SO101_LIMITS,
    steps_per_joint: int = 20,
) -> list[WorkspaceSample]:
    """
    Sweep q1..q4 through their limits and collect FK end-effector positions.

    q5 (wrist roll) doesn't change the tip position so it's fixed at 0.

    Args:
        steps_per_joint: resolution per joint — total points = steps^4,
                         keep ≤ 25 to avoid long runtimes.
    Returns:
        List of WorkspaceSample with reachable x/y/z points.
    """
    def linspace(lo: float, hi: float, n: int):
        if n <= 1:
            return [(lo + hi) / 2]
        return [lo + (hi - lo) * i / (n - 1) for i in range(n)]

    q1_vals = linspace(*limits.q1, steps_per_joint)
    q2_vals = linspace(*limits.q2, steps_per_joint)
    q3_vals = linspace(*limits.q3, steps_per_joint)
    q4_vals = linspace(*limits.q4, steps_per_joint)

    points: list[WorkspaceSample] = []
    for q1 in q1_vals:
        for q2 in q2_vals:
            for q3 in q3_vals:
                for q4 in q4_vals:
                    fk = forward_kinematics(q1, q2, q3, q4, 0.0, geometry)
                    points.append(WorkspaceSample(
                        x=fk["x"],
                        y=fk["y"],
                        z=fk["z"],
                        r=math.hypot(fk["x"], fk["y"]),
                    ))
    return points
