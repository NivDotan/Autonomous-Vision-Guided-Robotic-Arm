"""
Step 15 — CLI entrypoints for the robot_system pipeline.

Commands
--------
handoff      Run the handoff-grasp task (object handed to wrist camera).
table-pick   Run the table-pick task (overhead camera + IK pre-grasp).
calibrate    Collect camera-robot calibration points interactively.
workspace    Re-generate the workspace plot (outputs/workspace.png).
inspect      Inspect a LeRobot dataset or live robot observation.

Global flags
------------
--dry-run    Use simulated sensors/robot (default when no robot connected).
--port COM3  Serial port for VL53 distance sensor.
--cam 0      Camera index for wrist camera.
--out DIR    Output directory for logs and debug frames (default: outputs/).

Examples
--------
python main.py handoff --dry-run
python main.py table-pick --dry-run
python main.py calibrate --out outputs/calibration.json
python main.py workspace --steps 25
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np


# ------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------

def _blank_frame(h=480, w=640):
    return np.zeros((h, w, 3), dtype=np.uint8)


def _make_dummy_calibration():
    """A fitted 4-point calibration on a 640x480 frame for dry-run."""
    from calibration.base_camera_to_robot import CameraRobotCalibration
    cal = CameraRobotCalibration(table_z=0.02)
    cal.add_point((100, 100), (0.35,  0.15))
    cal.add_point((540, 100), (0.35, -0.15))
    cal.add_point((100, 380), (0.20,  0.15))
    cal.add_point((540, 380), (0.20, -0.15))
    cal.fit()
    return cal


# ------------------------------------------------------------------
# Sub-commands
# ------------------------------------------------------------------

def cmd_handoff(args: argparse.Namespace) -> int:
    from perception.sam2_adapter import Sam2Adapter
    from control.lerobot_adapter import LeRobotAdapter
    from control.visual_servo import VisualServo
    from control.distance_servo import DistanceServo, MockDistanceSensor
    from tasks.handoff_grasp import HandoffGrasp, GraspConfig
    from utils.logger import StructuredLogger, DebugImageWriter

    out_dir = Path(args.out)
    log     = StructuredLogger("handoff", log_file=out_dir / "handoff.log")
    writer  = DebugImageWriter(out_dir / "debug_frames", enabled=args.debug_frames)

    log.info("INIT", f"dry_run={args.dry_run}  port={args.port}")

    sam = Sam2Adapter(dry_run=args.dry_run)
    if args.dry_run:
        adapter = LeRobotAdapter(dry_run=True)
        sensor  = MockDistanceSensor([300, 250, 180, 120, 80, 70, 68, 65, 65, 65])
    else:
        from control.lerobot_adapter import make_live_adapter
        from control.distance_servo import SerialDistanceSensor
        adapter = make_live_adapter(port=args.robot_port)
        sensor  = SerialDistanceSensor(args.port)   # COM3 = VL53

    vs    = VisualServo(sam, adapter)
    ds    = DistanceServo(sensor)
    grasp = HandoffGrasp(vs, ds, adapter, config=GraspConfig())

    frame = _blank_frame()
    log.info("INIT", "Starting handoff grasp (click centre of frame)")
    grasp.start(frame, click_xy=(320, 240))

    step = 0
    while True:
        result = grasp.step(frame)
        log.info(result.phase.name, result.message)
        writer.annotate_and_save(frame, [str(result)], tag=result.phase.name, step=step)
        step += 1

        if result.done:
            status = "SUCCESS" if result.success else "FAILED"
            log.info("DONE", status)
            return 0 if result.success else 1

        if not args.dry_run:
            time.sleep(0.05)

    log.close()


def cmd_table_pick(args: argparse.Namespace) -> int:
    from perception.base_camera_detector import BaseDetector
    from perception.sam2_adapter import Sam2Adapter
    from control.lerobot_adapter import LeRobotAdapter
    from control.distance_servo import DistanceServo, MockDistanceSensor
    from tasks.table_pick import TablePick, TablePickConfig
    from utils.logger import StructuredLogger, DebugImageWriter

    out_dir = Path(args.out)
    log     = StructuredLogger("table_pick", log_file=out_dir / "table_pick.log")
    writer  = DebugImageWriter(out_dir / "debug_frames", enabled=args.debug_frames)

    cal = _make_dummy_calibration() if args.dry_run else _load_calibration(args)

    if args.dry_run:
        adapter = LeRobotAdapter(dry_run=True)
        sensor  = MockDistanceSensor([300, 200, 100, 70, 65, 65, 65])
    else:
        from control.lerobot_adapter import make_live_adapter
        from control.distance_servo import SerialDistanceSensor
        adapter = make_live_adapter(port=args.robot_port)
        sensor  = SerialDistanceSensor(args.port)

    task = TablePick(
        detector        = BaseDetector(dry_run=args.dry_run),
        calibration     = cal,
        sam             = Sam2Adapter(dry_run=args.dry_run),
        adapter         = adapter,
        distance_sensor = sensor,
        config          = TablePickConfig(grip_hold_s=0.0 if args.dry_run else 0.4),
    )

    base_frame  = _blank_frame()
    wrist_frame = _blank_frame()

    log.info("INIT", "Starting table-pick")
    task.start(base_frame)

    step = 0
    while True:
        result = task.step(wrist_frame)
        log.info(result.phase.name, result.message)
        writer.annotate_and_save(wrist_frame, [str(result)], tag=result.phase.name, step=step)
        step += 1

        if result.done:
            status = "SUCCESS" if result.success else "FAILED"
            log.info("DONE", status)
            log.close()
            return 0 if result.success else 1

        if not args.dry_run:
            time.sleep(0.05)


def cmd_calibrate(args: argparse.Namespace) -> int:
    from calibration.base_camera_to_robot import CameraRobotCalibration

    print("Camera-robot calibration")
    print("For each point: click the object in the camera, then enter robot (x y) in metres.")
    print("Type 'done' when finished (need at least 4 points).\n")

    cal = CameraRobotCalibration(table_z=args.table_z)

    while True:
        try:
            raw = input("Pixel u v (or 'done'): ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if raw.lower() == "done":
            break
        try:
            u, v = map(float, raw.split())
        except ValueError:
            print("  Enter two numbers separated by space.")
            continue
        try:
            raw_r = input("Robot x y (metres): ").strip()
            x, y  = map(float, raw_r.split())
        except ValueError:
            print("  Enter two numbers separated by space.")
            continue
        cal.add_point((u, v), (x, y))
        print(f"  Added point {cal.n_points}.\n")

    if cal.n_points < 4:
        print(f"Need at least 4 points, got {cal.n_points}. Aborting.")
        return 1

    rms = cal.fit()
    print(f"\nFit complete.  RMS reprojection error: {rms*1000:.2f} mm")
    out = Path(args.cal_out)
    out.parent.mkdir(parents=True, exist_ok=True)
    cal.save(out)
    print(f"Saved -> {out}")
    return 0


def cmd_workspace(args: argparse.Namespace) -> int:
    """Re-run the workspace plotter (same as tools/plot_workspace.py)."""
    import runpy
    sys.argv = ["plot_workspace.py", "--steps", str(args.steps)]
    script = Path(__file__).parent / "tools" / "plot_workspace.py"
    runpy.run_path(str(script), run_name="__main__")
    return 0


def cmd_inspect(args: argparse.Namespace) -> int:
    """Run the LeRobot inspector (same as tools/inspect_lerobot.py)."""
    import runpy
    sys.argv = ["inspect_lerobot.py"]
    if args.connect:
        sys.argv.append("--connect")
    script = Path(__file__).parent / "tools" / "inspect_lerobot.py"
    runpy.run_path(str(script), run_name="__main__")
    return 0


# ------------------------------------------------------------------
# Calibration loader (live mode)
# ------------------------------------------------------------------

def _load_calibration(args):
    from calibration.base_camera_to_robot import CameraRobotCalibration, CalibrationError
    path = getattr(args, "calibration", None) or "outputs/calibration.json"
    try:
        cal = CameraRobotCalibration.load(path)
        if not cal.is_fitted():
            cal.fit()
        return cal
    except Exception as e:
        print(f"ERROR: could not load calibration from {path}: {e}")
        sys.exit(1)


# ------------------------------------------------------------------
# Argument parser
# ------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="robot_system",
        description="SO-101 robotic arm pipeline",
    )

    # Shared hardware flags — available on every subcommand
    hw = argparse.ArgumentParser(add_help=False)
    hw.add_argument("--dry-run",      action="store_true", default=False,
                    help="Use simulated sensors and robot (no hardware needed)")
    hw.add_argument("--port",         default="COM3",
                    help="Serial port for VL53 sensor (default: COM3)")
    hw.add_argument("--robot-port",   default="COM4",
                    help="Serial port for robot arm (default: COM4)")
    hw.add_argument("--cam",          type=int, default=0,
                    help="Camera index (default: 0)")
    hw.add_argument("--out",          default="outputs",
                    help="Output directory for logs/frames (default: outputs/)")
    hw.add_argument("--debug-frames", action="store_true", default=False,
                    help="Save annotated debug frames to --out/debug_frames/")

    sub = p.add_subparsers(dest="command", required=True)

    # handoff
    sub.add_parser("handoff", parents=[hw], help="Run handoff-grasp task")

    # table-pick
    tp = sub.add_parser("table-pick", parents=[hw], help="Run table-pick task")
    tp.add_argument("--calibration", default="outputs/calibration.json",
                    help="Path to camera-robot calibration JSON")

    # calibrate
    cal = sub.add_parser("calibrate", parents=[hw], help="Collect calibration correspondences")
    cal.add_argument("--table-z", type=float, default=0.02,
                     help="Table surface height in robot frame (metres)")
    cal.add_argument("--cal-out", default="outputs/calibration.json",
                     dest="cal_out", help="Where to save calibration JSON")

    # workspace
    ws = sub.add_parser("workspace", parents=[hw], help="Re-generate workspace plot")
    ws.add_argument("--steps", type=int, default=20)

    # inspect
    ins = sub.add_parser("inspect", parents=[hw], help="Inspect LeRobot dataset or robot")
    ins.add_argument("--connect", action="store_true",
                     help="Connect to real robot for live observation")

    return p


def main() -> int:
    parser = build_parser()
    args   = parser.parse_args()

    Path(args.out).mkdir(parents=True, exist_ok=True)

    dispatch = {
        "handoff":    cmd_handoff,
        "table-pick": cmd_table_pick,
        "calibrate":  cmd_calibrate,
        "workspace":  cmd_workspace,
        "inspect":    cmd_inspect,
    }

    fn = dispatch.get(args.command)
    if fn is None:
        parser.print_help()
        return 1
    return fn(args)


if __name__ == "__main__":
    sys.exit(main())
