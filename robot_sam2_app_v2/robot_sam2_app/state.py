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

    # ── VL53 sensor approach ─────────────────────────────────────────────────
    vl53_controls_elbow: bool = False
    vl53_dist_mm: int | None = None
    arm_locked: bool = False      # True when VL53 < LOCK_DIST — freeze all except palm
    pre_grasp_palm: bool = False  # True while palm adjusts before gripper closes
    is_centered: bool = False

    place_mode: bool = False       # True while approaching / placing at target location

    # ── Grip retry state machine ──────────────────────────────────────────────
    retreat_mode: bool = False
    pre_approach_ticks: dict = field(default_factory=dict)
    gripper_closed_frames: int = 0
    grip_attempt: int = 0
    current_aim_x: float | None = None
    current_aim_y: float | None = None

    # ── 3D grasping (Tier 1) ─────────────────────────────────────────────────
    grasp_pose: object = None   # vision.grasp_planner.GraspPose3D | None

    def ticks(self) -> dict[str, int]:
        return {name: int(self.curr[name]) for name in MOTOR_NAMES}

    def set_curr_and_target(self, ticks: dict[str, int]) -> None:
        for name, tick in ticks.items():
            if name in self.curr:
                self.curr[name] = int(tick)
                self.target[name] = int(tick)

