"""Velocity profile helpers (Python-level, used when C++ lib is unavailable)."""
from __future__ import annotations
import numpy as np
from dataclasses import dataclass


@dataclass
class ProfileWaypoint:
    ticks: dict[str, int]
    t: float


def trapezoid_ticks(
    start_ticks: dict[str, int],
    end_ticks: dict[str, int],
    v_max: float = 30.0,   # ticks/s
    a_max: float = 60.0,   # ticks/s²
    dt: float = 0.005,
    motor_names: tuple = ("base", "shoulder", "elbow", "palm", "wrist", "gripper"),
) -> list[ProfileWaypoint]:
    """
    Time-synchronised trapezoidal profile entirely in tick space.
    Falls back gracefully when pykinematics is not built.
    """
    dists = {n: abs(end_ticks[n] - start_ticks[n]) for n in motor_names}
    signs = {n: (1 if end_ticks[n] >= start_ticks[n] else -1) for n in motor_names}

    # Compute per-joint durations.
    def joint_time(dist: float) -> float:
        t_ramp = v_max / a_max
        d_ramp = 0.5 * a_max * t_ramp ** 2
        if dist <= 2 * d_ramp:
            return 2 * (dist / a_max) ** 0.5
        return 2 * t_ramp + (dist - 2 * d_ramp) / v_max

    T = max((joint_time(d) for d in dists.values()), default=dt)
    if T < dt:
        return [ProfileWaypoint(dict(end_ticks), 0.0)]

    # Scale v_max per joint to synchronise.
    v = {}
    a = {}
    t_ramp = {}
    d_ramp_j = {}
    for n in motor_names:
        dist = dists[n]
        if dist < 1e-6:
            v[n] = a[n] = t_ramp[n] = d_ramp_j[n] = 0.0
            continue
        disc = a_max * (a_max * T ** 2 - 4 * dist)
        if disc < 0:
            a[n] = 4 * dist / T ** 2
            v[n] = a[n] * T / 2
        else:
            a[n] = a_max
            v[n] = (a_max * T - disc ** 0.5) / 2
        t_ramp[n] = v[n] / a[n]
        d_ramp_j[n] = 0.5 * a[n] * t_ramp[n] ** 2

    num_steps = int(np.ceil(T / dt)) + 1
    waypoints = []
    for step in range(num_steps):
        t = min(step * dt, T)
        ticks = {}
        for n in motor_names:
            dist = dists[n]
            if dist < 1e-6:
                ticks[n] = start_ticks[n]
                continue
            tr = t_ramp[n]
            dr = d_ramp_j[n]
            if t <= tr:
                pos = 0.5 * a[n] * t ** 2
            elif t <= T - tr:
                pos = dr + v[n] * (t - tr)
            else:
                t2 = T - t
                pos = dist - 0.5 * a[n] * t2 ** 2
            ticks[n] = start_ticks[n] + signs[n] * int(min(pos, dist) + 0.5)
        waypoints.append(ProfileWaypoint(ticks, t))
    return waypoints


def linear_ticks(
    start_ticks: dict[str, int],
    end_ticks: dict[str, int],
    duration: float,
    dt: float = 0.005,
    motor_names: tuple = ("base", "shoulder", "elbow", "palm", "wrist", "gripper"),
) -> list[ProfileWaypoint]:
    """Simple linear interpolation in tick space — absolute minimal fallback."""
    num_steps = max(2, int(np.ceil(duration / dt)) + 1)
    waypoints = []
    for step in range(num_steps):
        tau = min(step * dt / duration, 1.0)
        ticks = {
            n: int(start_ticks[n] + tau * (end_ticks[n] - start_ticks[n]) + 0.5)
            for n in motor_names
        }
        waypoints.append(ProfileWaypoint(ticks, step * dt))
    return waypoints
