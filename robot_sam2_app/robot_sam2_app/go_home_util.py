from __future__ import annotations

import time

from .config import MOTOR_IDS, MOTOR_NAMES


def go_home(hardware, home_pos: dict[int, int], steps: int = 40) -> None:
    """Smoothly move all motors from current position to home_pos.

    Mirrors the logic in lerobot/tests/go_home.py but works with either
    FeetechHardware or DaemonHardware via their shared read_ticks/write_ticks
    interface.

    Args:
        hardware: FeetechHardware or DaemonHardware instance (must be connected).
        home_pos: dict mapping motor_id (int) -> tick (int), e.g. {1:2365, ...}
        steps: number of interpolation steps (higher = slower/smoother).
    """
    if not hardware.connected:
        return

    # Read actual current positions.
    current = hardware.read_ticks()
    if current is None:
        print("go_home: could not read motor positions.")
        return

    # Build home dict keyed by motor name.
    home_named = {
        name: home_pos.get(motor_id, 2048)
        for name, motor_id in zip(MOTOR_NAMES, MOTOR_IDS)
    }

    print(f"go_home: moving to home in {steps} steps...")

    for i in range(steps + 1):
        progress = i / steps  # 0.0 → 1.0

        target = {}
        for name in MOTOR_NAMES:
            start = current[name]
            end   = home_named[name]
            target[name] = int(start + (end - start) * progress)

        hardware.write_ticks(target)
        time.sleep(0.02)

    print("go_home: arrived at home.")
