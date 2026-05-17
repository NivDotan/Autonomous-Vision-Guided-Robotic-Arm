"""
Step 6 — Smooth joint-space trajectory interpolation.

Generates a sequence of joint waypoints between a start and goal config,
applying an easing curve so the arm accelerates gently and decelerates
before arriving.

Usage (dry-run, no robot required):
    from control.trajectory import Trajectory, EasingCurve
    from control.lerobot_adapter import LeRobotAdapter

    adapter = LeRobotAdapter(dry_run=True)
    start = adapter.observe()
    goal  = {**start, "elbow_flex": 0.8, "shoulder_lift": 0.4}

    traj = Trajectory(start, goal, duration=3.0, easing=EasingCurve.EASE_IN_OUT)
    for waypoint in traj.steps(hz=50):
        adapter.move_joints(waypoint)
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Iterator


class EasingCurve(str, Enum):
    LINEAR       = "linear"
    EASE_IN_OUT  = "ease_in_out"   # cubic: slow start + slow end
    EASE_OUT     = "ease_out"      # cubic: fast start, soft landing
    EASE_IN      = "ease_in"       # cubic: slow start, fast end


def _apply_easing(t: float, curve: EasingCurve) -> float:
    """Map linear progress t∈[0,1] through the chosen easing curve."""
    t = max(0.0, min(1.0, t))
    if curve is EasingCurve.LINEAR:
        return t
    if curve is EasingCurve.EASE_IN_OUT:
        return t * t * (3.0 - 2.0 * t)
    if curve is EasingCurve.EASE_OUT:
        return 1.0 - (1.0 - t) ** 3
    if curve is EasingCurve.EASE_IN:
        return t ** 3
    return t


@dataclass
class Trajectory:
    """
    A smooth joint-space trajectory from start to goal.

    Parameters
    ----------
    start:             starting joint angles (radians)
    goal:              target joint angles (radians) — may be a subset of start keys
    duration:          total travel time in seconds
    easing:            easing curve applied to the normalised progress t
    speed_multipliers: per-joint speed factor (>1 = reaches target earlier and holds).
                       e.g. {"elbow_flex": 2.5} makes elbow 2.5x faster than the rest.
    """

    start:    dict[str, float]
    goal:     dict[str, float]
    duration: float = 2.0
    easing:   EasingCurve = EasingCurve.EASE_IN_OUT
    speed_multipliers: dict[str, float] = field(default_factory=dict)

    # Filled in post-init from the union of start+goal keys
    _joints: list[str] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        # Only interpolate joints present in goal; start values used as baseline
        self._joints = list(self.goal.keys())
        # Fill missing start values with goal values (no motion for those joints)
        for j in self._joints:
            self.start.setdefault(j, self.goal[j])

    # ------------------------------------------------------------------
    # Core interpolation
    # ------------------------------------------------------------------

    def at(self, t: float) -> dict[str, float]:
        """
        Return interpolated joint angles at normalised progress t ∈ [0, 1].

        Joints with a speed_multiplier > 1 reach their goal earlier and hold there.
        Only joints listed in goal are included in the returned dict.
        """
        result = {}
        for j in self._joints:
            t_j = min(1.0, t * self.speed_multipliers.get(j, 1.0))
            s   = _apply_easing(t_j, self.easing)
            result[j] = self.start[j] + (self.goal[j] - self.start[j]) * s
        return result

    def at_time(self, elapsed: float) -> dict[str, float]:
        """Return interpolated joints at elapsed seconds (clamped to duration)."""
        return self.at(elapsed / self.duration)

    # ------------------------------------------------------------------
    # Step generators
    # ------------------------------------------------------------------

    def steps(self, hz: float = 50) -> Iterator[dict[str, float]]:
        """
        Yield waypoints at the given control rate.

        This is a *timing-free* generator — it yields exactly
        ceil(duration * hz) + 1 waypoints, starting at t=0 and ending at t=1.
        The caller is responsible for sleeping between calls.

        Useful for simulation, testing, and pre-computing waypoint lists.
        """
        n = max(1, round(self.duration * hz))
        for i in range(n + 1):
            yield self.at(i / n)

    def execute(self, adapter, hz: float = 50) -> None:
        """
        Stream waypoints to an adapter in real time.

        Sleeps between waypoints to hit the target control rate.
        The final waypoint is always the exact goal position.

        Args:
            adapter: a LeRobotAdapter (or any object with move_joints)
            hz:      control frequency in Hz
        """
        dt = 1.0 / hz
        n  = max(1, round(self.duration * hz))
        t0 = time.monotonic()

        for i in range(n + 1):
            target_t = i / n
            waypoint  = self.at(target_t)
            adapter.move_joints(waypoint, blocking=False)

            if i < n:
                next_wall = t0 + (i + 1) * dt
                sleep_s   = next_wall - time.monotonic()
                if sleep_s > 0:
                    time.sleep(sleep_s)

        # Guarantee the adapter ends exactly at goal
        adapter.move_joints(self.goal, blocking=True)

    # ------------------------------------------------------------------
    # Convenience
    # ------------------------------------------------------------------

    @staticmethod
    def between(
        start:             dict[str, float],
        goal:              dict[str, float],
        duration:          float = 2.0,
        easing:            EasingCurve = EasingCurve.EASE_IN_OUT,
        speed_multipliers: dict[str, float] | None = None,
    ) -> "Trajectory":
        """Factory shorthand."""
        return Trajectory(dict(start), dict(goal), duration=duration,
                          easing=easing,
                          speed_multipliers=speed_multipliers or {})
