"""
Tests for TablePick + StructuredLogger + DebugImageWriter + main CLI — Steps 13-15.

Run:
    python tests/test_table_pick.py
or:
    python -m pytest tests/test_table_pick.py -v
"""

from __future__ import annotations

import io
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from perception.base_camera_detector import BaseDetector
from perception.sam2_adapter import Sam2Adapter
from calibration.base_camera_to_robot import CameraRobotCalibration
from control.lerobot_adapter import LeRobotAdapter
from control.distance_servo import DistanceServo, DistanceConfig, MockDistanceSensor
from tasks.table_pick import TablePick, TablePickConfig, PickPhase
from utils.logger import StructuredLogger, DebugImageWriter

H, W = 480, 640


def _blank():
    return np.zeros((H, W, 3), dtype=np.uint8)


def _fitted_cal():
    cal = CameraRobotCalibration(table_z=0.02)
    cal.add_point((100, 100), (0.35,  0.15))
    cal.add_point((540, 100), (0.35, -0.15))
    cal.add_point((100, 380), (0.20,  0.15))
    cal.add_point((540, 380), (0.20, -0.15))
    cal.fit()
    return cal


def _make_task(readings=None):
    if readings is None:
        readings = [65] * 30
    sensor = MockDistanceSensor(readings)
    d_cfg  = DistanceConfig(grip_dist_mm=75, stable_n=3, grip_confirm_n=3)
    task = TablePick(
        detector        = BaseDetector(dry_run=True),
        calibration     = _fitted_cal(),
        sam             = Sam2Adapter(dry_run=True),
        adapter         = LeRobotAdapter(dry_run=True),
        distance_sensor = sensor,
        d_config        = d_cfg,
        config          = TablePickConfig(grip_hold_s=0.0, return_home_after=False),
    )
    return task


# ================================================================
# TablePick
# ================================================================

def test_idle_before_start():
    task = _make_task()
    assert task.phase == PickPhase.IDLE
    print("PASS  test_idle_before_start")


def test_start_triggers_detection():
    task = _make_task()
    task.start(_blank())
    # After start() + successful detection -> should be MOVING_TO_PREGRASP
    assert task.phase in (PickPhase.MOVING_TO_PREGRASP, PickPhase.DETECTING), task.phase
    print(f"PASS  test_start_triggers_detection  (phase={task.phase.name})")


def test_happy_path_reaches_done():
    """With stable close readings, task should reach DONE."""
    task = _make_task(readings=[65] * 50)
    task.start(_blank())
    for _ in range(100):
        r = task.step(_blank())
        if r.done:
            break
    assert r.done
    assert r.success
    assert r.phase == PickPhase.DONE
    print(f"PASS  test_happy_path_reaches_done  ({r.elapsed_s:.2f}s)")


def test_result_has_target():
    """A successful pick result should include the robot-frame target."""
    task = _make_task(readings=[65] * 50)
    task.start(_blank())
    results = []
    for _ in range(100):
        r = task.step(_blank())
        results.append(r)
        if r.done:
            break
    done = results[-1]
    assert done.target_robot is not None
    x, y, z = done.target_robot
    assert 0.20 <= x <= 0.40
    print(f"PASS  test_result_has_target  target=({x:.3f},{y:.3f},{z:.3f})")


def test_far_sensor_does_not_grip():
    """Object always far -> should NOT reach DONE within reasonable steps."""
    task = _make_task(readings=[500] * 200)
    task.start(_blank())
    for _ in range(30):
        r = task.step(_blank())
        if r.done and r.success:
            assert False, "Should not succeed when object is far"
    print("PASS  test_far_sensor_does_not_grip")


def test_gripper_closed_after_done():
    import math
    task = _make_task(readings=[65] * 50)
    adapter = task.adapter
    task.start(_blank())
    for _ in range(100):
        r = task.step(_blank())
        if r.done:
            break
    gripper = adapter.observe()["gripper"]
    assert gripper > 0, f"gripper={math.degrees(gripper):.1f} deg"
    print(f"PASS  test_gripper_closed_after_done  (gripper={math.degrees(gripper):.1f} deg)")


# ================================================================
# StructuredLogger
# ================================================================

def test_logger_writes_to_stdout(capsys=None):
    log = StructuredLogger(name="test", verbose=True)
    log.info("PHASE", "hello world")
    log.close()
    print("PASS  test_logger_writes_to_stdout")


def test_logger_writes_to_file():
    with tempfile.NamedTemporaryFile(suffix=".log", delete=False, mode="w") as f:
        path = f.name
    log = StructuredLogger(name="test", log_file=path)
    log.info("PHASE", "test message")
    log.warn("PHASE", "a warning")
    log.close()
    content = Path(path).read_text()
    assert "test message" in content
    assert "a warning"    in content
    print("PASS  test_logger_writes_to_file")


def test_logger_context_manager():
    with tempfile.NamedTemporaryFile(suffix=".log", delete=False, mode="w") as f:
        path = f.name
    with StructuredLogger(name="test", log_file=path) as log:
        log.error("INIT", "an error")
    content = Path(path).read_text()
    assert "an error" in content
    print("PASS  test_logger_context_manager")


# ================================================================
# DebugImageWriter
# ================================================================

def test_writer_disabled_returns_none():
    writer = DebugImageWriter(enabled=False)
    result = writer.save(_blank(), tag="test")
    assert result is None
    print("PASS  test_writer_disabled_returns_none")


def test_writer_saves_file():
    with tempfile.TemporaryDirectory() as d:
        writer = DebugImageWriter(out_dir=d, enabled=True)
        path   = writer.save(_blank(), tag="test", step=0)
        assert path is not None
        assert path.exists(), f"file not found: {path}"
    print("PASS  test_writer_saves_file")


def test_writer_auto_increments_step():
    with tempfile.TemporaryDirectory() as d:
        writer = DebugImageWriter(out_dir=d, enabled=True)
        p0 = writer.save(_blank(), tag="a")
        p1 = writer.save(_blank(), tag="b")
        assert p0 != p1
        assert "000000" in p0.name
        assert "000001" in p1.name
    print("PASS  test_writer_auto_increments_step")


def test_annotate_and_save_runs_without_crash():
    with tempfile.TemporaryDirectory() as d:
        writer = DebugImageWriter(out_dir=d, enabled=True)
        path   = writer.annotate_and_save(_blank(), ["line1", "line2"], tag="ann")
        assert path is not None
    print("PASS  test_annotate_and_save_runs_without_crash")


# ================================================================
# main.py CLI (dry-run)
# ================================================================

def test_cli_handoff_dry_run():
    """python main.py handoff --dry-run should exit 0."""
    import subprocess
    main_py = Path(__file__).resolve().parents[1] / "main.py"
    result  = subprocess.run(
        [sys.executable, str(main_py), "--dry-run", "handoff"],
        capture_output=True, text=True, timeout=30,
    )
    assert result.returncode == 0, f"stderr:\n{result.stderr}\nstdout:\n{result.stdout}"
    print("PASS  test_cli_handoff_dry_run")


def test_cli_table_pick_dry_run():
    """python main.py table-pick --dry-run should exit 0."""
    import subprocess
    main_py = Path(__file__).resolve().parents[1] / "main.py"
    result  = subprocess.run(
        [sys.executable, str(main_py), "--dry-run", "table-pick"],
        capture_output=True, text=True, timeout=30,
    )
    assert result.returncode == 0, f"stderr:\n{result.stderr}\nstdout:\n{result.stdout}"
    print("PASS  test_cli_table_pick_dry_run")


def test_cli_workspace():
    """python main.py workspace --steps 5 should produce workspace.png."""
    import subprocess
    main_py  = Path(__file__).resolve().parents[1] / "main.py"
    out_file = Path(__file__).resolve().parents[1] / "outputs" / "workspace.png"
    result   = subprocess.run(
        [sys.executable, str(main_py), "workspace", "--steps", "5"],
        capture_output=True, text=True, timeout=60,
    )
    assert result.returncode == 0, result.stderr
    assert out_file.exists()
    print("PASS  test_cli_workspace")


if __name__ == "__main__":
    test_idle_before_start()
    test_start_triggers_detection()
    test_happy_path_reaches_done()
    test_result_has_target()
    test_far_sensor_does_not_grip()
    test_gripper_closed_after_done()
    test_logger_writes_to_stdout()
    test_logger_writes_to_file()
    test_logger_context_manager()
    test_writer_disabled_returns_none()
    test_writer_saves_file()
    test_writer_auto_increments_step()
    test_annotate_and_save_runs_without_crash()
    test_cli_handoff_dry_run()
    test_cli_table_pick_dry_run()
    test_cli_workspace()
    print("\nAll Steps 13-15 tests passed.")
