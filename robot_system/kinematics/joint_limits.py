"""
SO-101 joint limits in radians.

PLACEHOLDER VALUES — replace with actual servo travel limits.
"""

from __future__ import annotations
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class JointLimits:
    q1: tuple[float, float]   # base yaw
    q2: tuple[float, float]   # shoulder pitch
    q3: tuple[float, float]   # elbow pitch
    q4: tuple[float, float]   # wrist pitch
    q5: tuple[float, float]   # wrist roll

    def check(self, q: dict[str, float]) -> None:
        """Raise ValueError if any joint is outside its limits."""
        pairs = [
            ("q1", self.q1, q.get("q1", 0.0)),
            ("q2", self.q2, q.get("q2", 0.0)),
            ("q3", self.q3, q.get("q3", 0.0)),
            ("q4", self.q4, q.get("q4", 0.0)),
            ("q5", self.q5, q.get("q5", 0.0)),
        ]
        for name, (lo, hi), val in pairs:
            if not (lo <= val <= hi):
                raise ValueError(
                    f"Joint {name}={math.degrees(val):.1f}° outside "
                    f"[{math.degrees(lo):.1f}°, {math.degrees(hi):.1f}°]"
                )


def _r(deg: float) -> float:
    return math.radians(deg)


# PLACEHOLDER — replace with real servo limits
SO101_LIMITS = JointLimits(
    q1=(_r(-180), _r(180)),   # base can rotate freely  [PLACEHOLDER]
    q2=(_r(-90),  _r(90)),    # shoulder                [PLACEHOLDER]
    q3=(_r(-135), _r(135)),   # elbow                   [PLACEHOLDER]
    q4=(_r(-135), _r(135)),   # wrist pitch             [PLACEHOLDER]
    q5=(_r(-180), _r(180)),   # wrist roll              [PLACEHOLDER]
)
