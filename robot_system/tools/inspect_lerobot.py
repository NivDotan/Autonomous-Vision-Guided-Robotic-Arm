"""
Step 1 — Inspect LeRobot installation and robot API.

Usage:
    python tools/inspect_lerobot.py            # just inspect package
    python tools/inspect_lerobot.py --connect  # also connect to robot
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path


def find_package() -> Path:
    import lerobot
    pkg = Path(lerobot.__file__).resolve().parent
    print(f"\n=== LeRobot package path ===")
    print(pkg)
    return pkg


def search_relevant_files(pkg: Path) -> None:
    keywords = [
        "so101", "so_101", "follower", "leader",
        "calibration", "send_action", "get_observation",
    ]
    print(f"\n=== Searching for relevant files ===")
    found: dict[str, list[Path]] = {kw: [] for kw in keywords}

    for py in pkg.rglob("*.py"):
        name = py.name.lower()
        text = py.read_text(encoding="utf-8", errors="ignore").lower()
        for kw in keywords:
            if kw in name or kw in text:
                found[kw].append(py.relative_to(pkg))

    for kw, paths in found.items():
        if paths:
            print(f"\n  [{kw}]")
            for p in sorted(set(paths))[:8]:   # cap at 8 per keyword
                print(f"    {p}")


def connect_and_inspect(args) -> None:
    print(f"\n=== Connecting to robot ===")
    try:
        from lerobot.robots.so101_follower.so101_follower import SO101Follower
        from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
    except ImportError:
        print("ERROR: SO101Follower not found in this lerobot installation.")
        sys.exit(1)

    port = getattr(args, "port", "COM4")
    rid  = getattr(args, "id",   "my_awesome_follower_arm")
    cfg  = SO101FollowerConfig(port=port, cameras={}, id=rid)
    robot = SO101Follower(cfg)
    print(f"  port={port}  id={rid}")

    robot.connect(calibrate=False)
    print("Connected.")

    print(f"\n=== Observation (joint names + current values in degrees) ===")
    obs = robot.get_observation()
    for k, v in obs.items():
        print(f"  {k}: {v:.2f} deg")

    print(f"\n=== Joint name summary ===")
    print("  Raw keys  :", list(obs.keys()))
    print("  Base names:", [k.replace('.pos', '') for k in obs.keys()])

    robot.disconnect()
    print("\nDisconnected.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Inspect LeRobot installation")
    parser.add_argument("--connect", action="store_true",
                        help="Connect to robot and inspect observation/action structure")
    parser.add_argument("--port", default="COM4", help="Serial port for robot (default COM4)")
    parser.add_argument("--id",   default="my_awesome_follower_arm",
                        help="Robot calibration id (default my_awesome_follower_arm)")
    args = parser.parse_args()

    pkg = find_package()
    search_relevant_files(pkg)

    if args.connect:
        connect_and_inspect(args)
    else:
        print("\n(Pass --connect to also connect to the robot and print observation structure)")


if __name__ == "__main__":
    main()
