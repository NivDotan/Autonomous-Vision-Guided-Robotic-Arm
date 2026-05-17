from __future__ import annotations

import time

from .config import MOTOR_IDS, MOTOR_NAMES

# ── Timing ────────────────────────────────────────────────────────────────────
TOTAL_TIME = 5.0   # seconds for the full move — increase for more grace
STEPS      = 120   # interpolation resolution

# ── Per-motor internal servo speed (Goal_Velocity register) ───────────────────
# Low value = Feetech controller moves slowly and continuously (full torque).
# High value = motor sprints to each step (can sag on gravity-loaded joints).
# Restore to 0 (max) after the move so normal commands aren't throttled.
_GOAL_VELOCITY = {
    "base":     100,
    "shoulder": 800,
    "elbow":    800,
    "palm":     800,
    "wrist":    150,
    "gripper":  200,
}

# ── Per-motor interpolation speed multiplier ──────────────────────────────────
# > 1.0 → motor finishes early and holds; < 1.0 → motor finishes late.
_SPEED_MULT = {
    "base":     0.70,
    "shoulder": 1.50,
    "elbow":    1.50,
    "palm":     1.80,
    "wrist":    1.00,
    "gripper":  1.20,
}

# ── Per-motor easing ──────────────────────────────────────────────────────────
# None = use global ease_in_out.
# "ease_out" = fast approach, decelerates into target (no slam at the end).
_MOTOR_EASING = {
    "base":     None,
    "shoulder": None,
    "elbow":    "ease_out",   # holds whole forearm — fast but gentle landing
    "palm":     "ease_out",
    "wrist":    None,
    "gripper":  None,
}


def _ease_in_out(t: float) -> float:
    return t * t * (3.0 - 2.0 * t)


def _ease_out(t: float) -> float:
    return 1.0 - (1.0 - t) ** 3


_EASING_FN = {
    "ease_in_out": _ease_in_out,
    "ease_out":    _ease_out,
}


def go_home(hardware, home_pos: dict[int, int]) -> None:
    if not hardware.connected:
        return

    current = hardware.read_ticks()
    if current is None:
        print("go_home: could not read motor positions.")
        return

    home_named = {
        name: home_pos.get(motor_id, 2048)
        for name, motor_id in zip(MOTOR_NAMES, MOTOR_IDS)
    }

    # Set per-motor servo speed before moving
    for name in MOTOR_NAMES:
        try:
            hardware.write_goal_velocity(name, _GOAL_VELOCITY[name])
        except Exception:
            pass  # hardware layer may not support this — skip silently

    step_delay = TOTAL_TIME / STEPS
    print(f"go_home: moving to home in {STEPS} steps over {TOTAL_TIME}s...")

    for i in range(1, STEPS + 1):
        raw_t = i / STEPS
        target = {}

        for name in MOTOR_NAMES:
            motor_t  = min(raw_t * _SPEED_MULT[name], 1.0)
            easing   = _MOTOR_EASING[name] or "ease_in_out"
            eased_t  = _EASING_FN[easing](motor_t)

            start = current[name]
            end   = home_named[name]
            target[name] = int(start + (end - start) * eased_t)

        hardware.write_ticks(target)
        time.sleep(step_delay)

    # Final write: guarantee exact home position
    hardware.write_ticks(home_named)

    # Restore full speed
    for name in MOTOR_NAMES:
        try:
            hardware.write_goal_velocity(name, 0)
        except Exception:
            pass

    print("go_home: arrived at home.")
