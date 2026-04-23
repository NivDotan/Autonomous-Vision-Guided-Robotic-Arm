from __future__ import annotations

from dataclasses import dataclass, field

from .config import DEFAULT_TICKS, MOTOR_NAMES


@dataclass
class RobotState:
    curr: dict[str, int] = field(default_factory=lambda: dict(DEFAULT_TICKS))
    target: dict[str, int] = field(default_factory=lambda: dict(DEFAULT_TICKS))
    home: dict[int, int] = field(default_factory=dict)

    tracking_mode: str = "HAND"
    motors_enabled: bool = False
    sim_jog_active: bool = False
    approach_mode: bool = False
    object_reached: bool = False
    is_frozen: bool = False
    gripper_closed: bool = False
    auto_palm: bool = True
    returning_home: bool = False

    # ── Trajectory execution (Tier 1) ────────────────────────────────────────
    trajectory_active: bool = False
    trajectory_waypoints: list = field(default_factory=list)  # list[TrajectoryWaypoint]
    trajectory_index: int = 0

    # ── 3D grasping (Tier 1) ─────────────────────────────────────────────────
    grasp_pose: object = None   # vision.grasp_planner.GraspPose3D | None

    def ticks(self) -> dict[str, int]:
        return {name: int(self.curr[name]) for name in MOTOR_NAMES}

    def set_curr_and_target(self, ticks: dict[str, int]) -> None:
        for name, tick in ticks.items():
            if name in self.curr:
                self.curr[name] = int(tick)
                self.target[name] = int(tick)

