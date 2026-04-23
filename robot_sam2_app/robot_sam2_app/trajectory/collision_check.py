"""PyBullet-based collision pre-check for planned trajectories."""
from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..simulation import PyBulletArmSim

try:
    import pybullet as p
    _PB_AVAILABLE = True
except ImportError:
    _PB_AVAILABLE = False


class CollisionChecker:
    """
    Uses the existing PyBulletArmSim to pre-flight check a trajectory.
    Sets joint states without stepping simulation, then queries contact points.
    """

    def __init__(self, sim: "PyBulletArmSim"):
        self._sim = sim

    def check_trajectory(
        self,
        waypoints: list,  # list of TrajectoryWaypoint (ticks dict + t float)
        sample_every: int = 5,
    ) -> tuple[bool, int | None]:
        """
        Returns (is_safe, first_collision_index) where index refers to the
        sampled waypoint index (multiply by sample_every to get full list index).
        Returns (True, None) if no collision detected or PyBullet unavailable.
        """
        if not _PB_AVAILABLE or self._sim is None or not self._sim.connected:
            return True, None

        robot_id = self._sim.robot_id
        if robot_id is None:
            return True, None

        for i, wp in enumerate(waypoints):
            if i % sample_every != 0:
                continue

            # Temporarily set joint states (no simulation step).
            self._sim.set_visual_from_ticks(wp.ticks)

            # Check for self-collisions.
            contacts = p.getContactPoints(bodyA=robot_id, bodyB=robot_id)
            if contacts and len(contacts) > 0:
                return False, i

            # Check collisions with the world/ground plane if present.
            ground_contacts = p.getContactPoints(bodyA=robot_id)
            for c in (ground_contacts or []):
                # Ignore self-contacts (bodyA == bodyB) already handled above.
                if c[2] != robot_id and c[8] < -0.002:  # penetration depth
                    return False, i

        return True, None
