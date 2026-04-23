"""Python-level cubic spline helpers (used when pykinematics is unavailable)."""
from __future__ import annotations
import numpy as np
from dataclasses import dataclass


@dataclass
class SplineWaypoint:
    ticks: dict[str, int]
    t: float


def cubic_spline_ticks(
    q_list: list[dict[str, int]],
    total_time: float,
    dt: float = 0.005,
    motor_names: tuple = ("base", "shoulder", "elbow", "palm", "wrist", "gripper"),
) -> list[SplineWaypoint]:
    """
    Natural cubic spline through a list of tick-space waypoints.
    Mirrors the C++ cubic_spline() signature but works in tick space.
    """
    n = len(q_list)
    if n < 2:
        raise ValueError("cubic_spline_ticks: need at least 2 waypoints")

    h = total_time / (n - 1)  # uniform interval

    # For each joint: compute second derivatives via natural BC.
    M: dict[str, np.ndarray] = {}
    for name in motor_names:
        y = np.array([q[name] for q in q_list], dtype=float)
        M[name] = _natural_cubic_second_deriv(y, h)

    # Sample.
    num_steps = int(np.ceil(total_time / dt)) + 1
    waypoints = []
    for step in range(num_steps):
        t_abs = min(step * dt, total_time)
        seg = min(int(t_abs / h), n - 2)
        t_loc = t_abs - seg * h
        ticks = {}
        for name in motor_names:
            y = [q[name] for q in q_list]
            a = float(y[seg])
            b = (y[seg + 1] - y[seg]) / h - h / 6 * (2 * M[name][seg] + M[name][seg + 1])
            c = M[name][seg] / 2
            d = (M[name][seg + 1] - M[name][seg]) / (6 * h)
            val = a + b * t_loc + c * t_loc ** 2 + d * t_loc ** 3
            ticks[name] = int(round(val))
        waypoints.append(SplineWaypoint(ticks, t_abs))
    return waypoints


def _natural_cubic_second_deriv(y: np.ndarray, h: float) -> np.ndarray:
    """Thomas algorithm for natural cubic spline second derivatives."""
    n = len(y)
    M = np.zeros(n)
    if n <= 2:
        return M

    rhs = 6 / h ** 2 * (y[2:] - 2 * y[1:-1] + y[:-2])
    diag = np.full(n - 2, 4.0)
    off  = np.ones(n - 3)

    # Forward
    for i in range(1, n - 2):
        w = off[i - 1] / diag[i - 1]
        diag[i] -= w * off[i - 1]
        rhs[i]  -= w * rhs[i - 1]

    # Back substitution
    m = np.zeros(n - 2)
    m[-1] = rhs[-1] / diag[-1]
    for i in range(n - 4, -1, -1):
        m[i] = (rhs[i] - off[i] * m[i + 1]) / diag[i]
    M[1:-1] = m
    return M
