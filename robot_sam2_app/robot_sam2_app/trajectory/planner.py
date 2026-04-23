"""
Trajectory planner — bridges C++ kinematics (pykinematics) with the
Python tick-based robot state machine.

Falls back to pure-Python implementations when pykinematics is not built.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np

from ..config import MOTOR_NAMES

# Optional C++ kinematics module.
try:
    import pykinematics as _kin
    _KIN_AVAILABLE = True
except ImportError:
    _kin = None
    _KIN_AVAILABLE = False

# Python fallbacks.
from .velocity_profile import trapezoid_ticks, linear_ticks, ProfileWaypoint
from .spline import cubic_spline_ticks

if TYPE_CHECKING:
    from ..vision.grasp_planner import GraspPose3D


@dataclass
class TrajectoryWaypoint:
    ticks: dict[str, int]
    t: float


class TrajectoryPlanner:
    """
    Generates joint-space and Cartesian trajectories.

    If pykinematics is available, uses the fast C++ solvers for IK and
    trajectory generation, then converts back to tick space.
    Otherwise falls back to pure-Python tick-space interpolation.
    """

    def __init__(self, calib_path: str | None = None):
        if _KIN_AVAILABLE and calib_path:
            try:
                _kin.load_calibration_json(calib_path)
            except Exception as e:
                print(f"[TrajectoryPlanner] Warning: calibration load failed: {e}")

    # ── Public API ────────────────────────────────────────────────────────────

    def plan_joint_space(
        self,
        start_ticks: dict[str, int],
        end_ticks: dict[str, int],
        duration: float,
        dt: float = 0.005,
        profile: str = "trapezoid",   # "trapezoid" | "cubic" | "linear"
    ) -> list[TrajectoryWaypoint]:
        """
        Returns a list of TrajectoryWaypoints from start to end in joint space.

        If pykinematics is available, converts to radians, generates the
        trajectory in rad space, then converts back.  Otherwise works in
        tick space directly.
        """
        if _KIN_AVAILABLE:
            return self._plan_joint_space_cpp(
                start_ticks, end_ticks, duration, dt, profile)
        return self._plan_joint_space_python(
            start_ticks, end_ticks, duration, dt, profile)

    def plan_cartesian(
        self,
        start_ticks: dict[str, int],
        target_xyz: tuple[float, float, float],
        dt: float = 0.005,
        v_max_cart: float = 0.10,  # m/s
    ) -> list[TrajectoryWaypoint]:
        """
        Straight-line Cartesian move to target_xyz (robot base frame, metres).
        Requires pykinematics; falls back to joint-space linear if unavailable.
        """
        if not _KIN_AVAILABLE:
            # Fallback: stay in place (can't compute IK without kinematics lib).
            print("[TrajectoryPlanner] pykinematics unavailable — Cartesian planning skipped.")
            return [TrajectoryWaypoint(dict(start_ticks), 0.0)]

        q_start = self._ticks_to_rad_array(start_ticks)
        T_start = _kin.fk_transform(list(q_start))

        # Build target transform: keep current orientation, move to target_xyz.
        T_end = np.array(T_start)
        T_end[0, 3] = target_xyz[0]
        T_end[1, 3] = target_xyz[1]
        T_end[2, 3] = target_xyz[2]

        cpp_wps = _kin.cartesian_linear(T_start, T_end, list(q_start), v_max_cart, dt)
        return self._cpp_waypoints_to_tick_waypoints(cpp_wps)

    def plan_grasp(
        self,
        start_ticks: dict[str, int],
        grasp_pose: "GraspPose3D",
        pre_grasp_offset: float = 0.08,
        dt: float = 0.005,
        v_max_cart: float = 0.08,
    ) -> list[TrajectoryWaypoint]:
        """
        Three-phase trajectory:
          1. Move to pre-grasp pose (grasp_pose.position + pre_grasp_offset along approach axis)
          2. Approach along axis to grasp pose
          3. Return ticks for gripper-close (caller handles the actual close command)
        """
        pos = np.array(grasp_pose.position_base)
        axis = np.array(grasp_pose.approach_axis)
        axis = axis / (np.linalg.norm(axis) + 1e-9)

        pre_grasp_pos = tuple((pos + axis * pre_grasp_offset).tolist())
        grasp_pos     = tuple(pos.tolist())

        phase1 = self.plan_cartesian(start_ticks, pre_grasp_pos, dt, v_max_cart)
        pre_end = phase1[-1].ticks if phase1 else start_ticks
        phase2 = self.plan_cartesian(pre_end, grasp_pos, dt, v_max_cart * 0.5)

        # Concatenate with adjusted timestamps.
        t_offset = phase1[-1].t if phase1 else 0.0
        result = list(phase1)
        for wp in phase2:
            result.append(TrajectoryWaypoint(wp.ticks, wp.t + t_offset))
        return result

    # ── Internal: C++ path ───────────────────────────────────────────────────

    def _plan_joint_space_cpp(
        self,
        start_ticks: dict[str, int],
        end_ticks: dict[str, int],
        duration: float,
        dt: float,
        profile: str,
    ) -> list[TrajectoryWaypoint]:
        q_start = list(self._ticks_to_rad_array(start_ticks))
        q_end   = list(self._ticks_to_rad_array(end_ticks))

        if profile == "cubic":
            cpp_wps = _kin.cubic_spline([q_start, q_end], duration, dt)
        elif profile == "trapezoid":
            cpp_wps = _kin.trapezoid_profile(q_start, q_end, 1.5, 3.0, dt)
        else:
            # Linear — use quintic with zero BCs.
            cpp_wps = _kin.quintic_spline([q_start, q_end], duration, dt)

        return self._cpp_waypoints_to_tick_waypoints(cpp_wps)

    def _cpp_waypoints_to_tick_waypoints(self, cpp_wps) -> list[TrajectoryWaypoint]:
        result = []
        for wp in cpp_wps:
            ticks_arr = _kin.rad_to_ticks_global(list(wp.q))
            ticks_dict = {
                name: int(ticks_arr[i])
                for i, name in enumerate(MOTOR_NAMES)
            }
            result.append(TrajectoryWaypoint(ticks_dict, wp.t))
        return result

    # ── Internal: Python fallback path ───────────────────────────────────────

    def _plan_joint_space_python(
        self,
        start_ticks: dict[str, int],
        end_ticks: dict[str, int],
        duration: float,
        dt: float,
        profile: str,
    ) -> list[TrajectoryWaypoint]:
        if profile == "cubic":
            wps = cubic_spline_ticks([start_ticks, end_ticks], duration, dt,
                                      motor_names=MOTOR_NAMES)
            return [TrajectoryWaypoint(w.ticks, w.t) for w in wps]
        elif profile == "trapezoid":
            wps = trapezoid_ticks(start_ticks, end_ticks, dt=dt,
                                   motor_names=MOTOR_NAMES)
            return [TrajectoryWaypoint(w.ticks, w.t) for w in wps]
        else:
            wps = linear_ticks(start_ticks, end_ticks, duration, dt,
                                motor_names=MOTOR_NAMES)
            return [TrajectoryWaypoint(w.ticks, w.t) for w in wps]

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _ticks_to_rad_array(self, ticks: dict[str, int]) -> np.ndarray:
        """Convert tick dict → numpy array of radians using global calibration."""
        # Calibration constants matching joint_sim_calibration.json defaults.
        offsets = {"base": 2365, "shoulder": 1740, "elbow": 1410,
                   "palm": 3000, "wrist": 3200, "gripper": 3000}
        TPR = 4096.0
        return np.array([
            (ticks.get(n, 2048) - offsets[n]) * (2 * math.pi / TPR)
            for n in MOTOR_NAMES
        ])
