"""
SO-101 arm geometry — link lengths in metres.

PLACEHOLDER VALUES — measure your physical robot and replace these.
Use a ruler from joint centre to joint centre.
"""

from __future__ import annotations
from dataclasses import dataclass


@dataclass(frozen=True)
class ArmGeometry:
    L0: float   # base plate to shoulder pivot height  (m)
    L1: float   # shoulder pivot to elbow pivot        (m)
    L2: float   # elbow pivot to wrist pivot           (m)
    L3: float   # wrist pivot to gripper tip           (m)


# PLACEHOLDER — replace with measured values
SO101_GEOMETRY = ArmGeometry(
    L0=0.075,   # ~75 mm base height   [PLACEHOLDER]
    L1=0.130,   # ~130 mm upper arm    [PLACEHOLDER]
    L2=0.125,   # ~125 mm forearm      [PLACEHOLDER]
    L3=0.060,   # ~60 mm wrist+tip     [PLACEHOLDER]
)
