"""
Automated pick-and-place benchmark.

Runs N grasp trials, records success/fail, and generates a summary report.
Requires the robot to be set up with a known set of objects and ArUco markers
for ground-truth detection.

Usage:
    python benchmarks/pick_place_test.py --trials 20 --output results/
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np


def detect_aruco_pose(frame: np.ndarray, marker_id: int = 0) -> np.ndarray | None:
    """
    Detect an ArUco marker and return its 4x4 pose matrix (camera frame).
    Returns None if the marker is not visible.
    """
    try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        params     = cv2.aruco.DetectorParameters()
        detector   = cv2.aruco.ArucoDetector(aruco_dict, params)
        corners, ids, _ = detector.detectMarkers(frame)
        if ids is None:
            return None
        for i, mid in enumerate(ids.flatten()):
            if mid == marker_id:
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    [corners[i]], 0.05,  # 5cm marker side length
                    np.eye(3), np.zeros(5))
                T = np.eye(4)
                T[:3, :3], _ = cv2.Rodrigues(rvec[0])
                T[:3, 3]     = tvec[0]
                return T
    except Exception:
        pass
    return None


def run_trial(robot_app, target_marker_id: int = 0) -> dict:
    """
    Execute one grasp trial and return a result dict.
    """
    start_time = time.time()
    result = {
        "trial_id":    run_trial._counter,
        "timestamp":   start_time,
        "success":     False,
        "duration_s":  0.0,
        "error":       None,
    }
    run_trial._counter += 1

    try:
        # 1. Detect object via ArUco ground truth.
        frame = robot_app.last_frame_bgr
        if frame is None:
            result["error"] = "No camera frame"
            return result

        T_obj = detect_aruco_pose(frame, marker_id=target_marker_id)
        if T_obj is None:
            result["error"] = f"ArUco marker {target_marker_id} not visible"
            return result

        # 2. Plan grasp.
        from robot_sam2_app.vision.grasp_planner import GraspPose3D
        pos = tuple(T_obj[:3, 3].tolist())
        # Approach from above.
        grasp_pose = GraspPose3D(pos, (0.0, 0.0, -1.0), quality=1.0)
        waypoints = robot_app.traj_planner.plan_grasp(
            robot_app.state.ticks(), grasp_pose)

        # 3. Execute trajectory.
        robot_app.state.trajectory_waypoints = waypoints
        robot_app.state.trajectory_index     = 0
        robot_app.state.trajectory_active    = True

        # Wait for completion.
        timeout = time.time() + 15.0
        while robot_app.state.trajectory_active and time.time() < timeout:
            time.sleep(0.05)

        # 4. Close gripper.
        from robot_sam2_app.config import GRIPPER_CLOSE
        robot_app.state.target["gripper"] = GRIPPER_CLOSE
        time.sleep(1.5)

        # 5. Check success via load sensor.
        if robot_app.hardware.gripper_load_detected():
            result["success"] = True

    except Exception as e:
        result["error"] = str(e)

    result["duration_s"] = time.time() - start_time
    return result


run_trial._counter = 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--output", type=str, default="results")
    parser.add_argument("--marker-id", type=int, default=0)
    args = parser.parse_args()

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Running {args.trials} pick-and-place trials...")

    # Import and start robot app in non-blocking mode (no GUI).
    import sys
    sys.path.insert(0, str(Path(__file__).parent.parent / "robot_sam2_app"))
    from robot_sam2_app.app import RobotApp

    app = RobotApp()
    # Start app in background thread (headless).
    import threading
    t = threading.Thread(target=app.run, daemon=True)
    t.start()
    time.sleep(3.0)  # Let app initialise.

    results = []
    for i in range(args.trials):
        print(f"  Trial {i+1}/{args.trials}...", end=" ", flush=True)
        res = run_trial(app, target_marker_id=args.marker_id)
        results.append(res)
        status = "SUCCESS" if res["success"] else f"FAIL ({res.get('error', '')})"
        print(status)
        time.sleep(2.0)  # Reset between trials.

    # Save results.
    out_path = output_dir / f"pick_place_{int(time.time())}.json"
    with open(out_path, "w") as f:
        json.dump(results, f, indent=2)

    n_success = sum(1 for r in results if r["success"])
    print(f"\nResults: {n_success}/{args.trials} successful ({100*n_success/args.trials:.1f}%)")
    print(f"Saved to {out_path}")


if __name__ == "__main__":
    main()
