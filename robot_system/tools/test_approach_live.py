"""
Live approach + grip test — no camera, no SAM2 required.

What it does:
  1. Connects to the robot (COM4) and VL53 sensor (COM3).
  2. Prints current joint angles and VL53 reading.
  3. Waits for you to press ENTER, then starts approaching.
  4. Drives elbow_flex forward while VL53 > grip_dist_mm.
  5. Closes gripper when 3 stable readings <= grip_dist_mm.
  6. Waits, then opens gripper and returns to start position.

Run from robot_system/:
    python tools/test_approach_live.py
    python tools/test_approach_live.py --grip-dist 80 --robot-port COM4 --port COM3
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from control.lerobot_adapter import make_live_adapter
from control.distance_servo import DistanceServo, DistanceConfig, SerialDistanceSensor


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--robot-port", default="COM4")
    p.add_argument("--port",       default="COM3", help="VL53 serial port")
    p.add_argument("--grip-dist",  type=int,   default=75,  help="Grip distance in mm")
    p.add_argument("--stable-win", type=int,   default=15,  help="Stability window mm")
    p.add_argument("--max-jump",   type=int,   default=30,  help="Max jump between readings mm")
    p.add_argument("--confirm-n",  type=int,   default=3,   help="Consecutive stable readings to grip")
    p.add_argument("--kp",         type=float, default=0.0015, help="Elbow P-gain (rad/mm)")
    p.add_argument("--max-elbow",  type=float, default=120.0,  help="Max elbow_flex in degrees")
    p.add_argument("--hold-s",     type=float, default=1.5,    help="Seconds to hold grip")
    args = p.parse_args()

    print("Connecting to robot...")
    adapter = make_live_adapter(port=args.robot_port)
    start_joints = adapter.observe()

    print("\nCurrent joint angles:")
    for k, v in start_joints.items():
        print(f"  {k}: {math.degrees(v):.1f} deg")

    print(f"\nConnecting to VL53 on {args.port}...")
    sensor = SerialDistanceSensor(args.port)
    time.sleep(0.5)   # let buffer fill
    dist = sensor.read()
    print(f"VL53 reading: {dist} mm")

    elbow_limit = (
        start_joints.get("elbow_flex", 0.0),
        math.radians(args.max_elbow),
    )
    print(f"\nElbow travel: {math.degrees(elbow_limit[0]):.1f} -> {args.max_elbow:.1f} deg")
    print(f"Grip distance: {args.grip_dist} mm")
    print(f"\nPosition the arm so it can reach the object, then press ENTER to start approach...")
    try:
        input()
    except KeyboardInterrupt:
        print("Aborted.")
        _cleanup(adapter, sensor, start_joints)
        return

    cfg = DistanceConfig(
        grip_dist_mm  = args.grip_dist,
        stable_window = args.stable_win,
        max_jump      = args.max_jump,
        grip_confirm_n= args.confirm_n,
        Kp_elbow      = args.kp,
        elbow_limit   = elbow_limit,
    )
    servo = DistanceServo(sensor, cfg)

    print("\nApproaching... (Ctrl+C to abort)\n")
    try:
        while True:
            state = servo.step(adapter)
            elbow_deg = math.degrees(adapter.observe().get("elbow_flex", 0.0))
            dist_str  = f"{state.dist_mm} mm" if state.dist_mm is not None else "no data"
            print(
                f"  dist={dist_str:8s}  "
                f"stable={state.is_stable}  close={state.is_close}  "
                f"confirm={state.confirm_count}/{args.confirm_n}  "
                f"elbow={elbow_deg:.1f} deg",
                end="\r",
            )

            if state.should_grip:
                print()
                print("\nStable and close — closing gripper!")
                adapter.close_gripper()
                time.sleep(args.hold_s)

                print("Opening gripper and returning to start...")
                adapter.open_gripper()
                time.sleep(0.5)
                adapter.move_joints(start_joints)
                time.sleep(1.0)
                print("Done.")
                break

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\nAborted — returning to start position...")
        _cleanup(adapter, sensor, start_joints)
        return

    _cleanup(adapter, sensor, start_joints)


def _cleanup(adapter, sensor, start_joints):
    try:
        adapter.open_gripper()
        time.sleep(0.3)
        adapter.move_joints(start_joints)
        time.sleep(0.5)
        adapter.robot.disconnect()
    except Exception as e:
        print(f"Cleanup error: {e}")
    sensor.close()


if __name__ == "__main__":
    main()
