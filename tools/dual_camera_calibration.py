"""
Dual-camera calibration / diagnostics for SO101 arm.

Setup:
  - BASE camera  : fixed, looks at the workspace from the arm's base
  - GRIPPER camera: mounted in the gripper, moves with the end-effector

What this script does:
  1. Auto-detects which camera indices are available.
  2. Shows both feeds side-by-side in real time.
  3. Detects an ArUco marker (default: 6x6, ID 0) in both cameras.
     - Base cam  → world-frame pose of the marker.
     - Gripper cam → marker pose relative to the gripper.
  4. [c] Collect a calibration sample (you move the arm to a new pose, press c).
  5. [s] Solve & save hand-eye calibration (camera→base transform) once you have
         ≥5 samples collected.
  6. [q] Quit.

Print a 10×10 cm ArUco marker (6x6_50, ID 0) and hold it in the workspace.
You can generate one at:
  https://chev.me/arucogen/  (Dictionary=6x6_50, Marker ID=0, Size=100mm)
Or generate programmatically with this script's --gen-marker flag.

Usage:
  python dual_camera_calibration.py
  python dual_camera_calibration.py --base 0 --gripper 1
  python dual_camera_calibration.py --gen-marker        # save marker image & exit
  python dual_camera_calibration.py --calibrate-intrinsics  # run checkerboard intrinsics first
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# ── ArUco setup ───────────────────────────────────────────────────────────────

ARUCO_DICT   = cv2.aruco.DICT_6X6_50
MARKER_ID    = 0
MARKER_SIZE_M = 0.10          # physical marker side length in metres

CALIB_OUT = Path("hand_eye_calibration.json")

# ── Camera intrinsics (approximate — replace with your actual calibration) ────
# These are reasonable defaults for a 640×480 USB webcam.
# Run --calibrate-intrinsics to get accurate values.
_DEFAULT_K = np.array([
    [600.0,   0.0, 320.0],
    [  0.0, 600.0, 240.0],
    [  0.0,   0.0,   1.0],
], dtype=np.float64)
_DEFAULT_DIST = np.zeros((5, 1), dtype=np.float64)


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def probe_cameras(max_index: int = 6) -> list[int]:
    """Return list of working camera indices."""
    found = []
    for i in range(max_index):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if cap.isOpened():
            ok, _ = cap.read()
            if ok:
                found.append(i)
        cap.release()
    return found


def open_camera(index: int, width: int = 640, height: int = 480) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def rvec_tvec_to_matrix(rvec, tvec) -> np.ndarray:
    """Convert OpenCV rvec/tvec to 4×4 homogeneous transform."""
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3,  3] = tvec.flatten()
    return T


def matrix_to_rvec_tvec(T: np.ndarray):
    R = T[:3, :3]
    t = T[:3, 3]
    rvec, _ = cv2.Rodrigues(R)
    return rvec, t.reshape(3, 1)


def draw_axes(frame, K, dist, rvec, tvec, length: float = 0.05):
    cv2.drawFrameAxes(frame, K, dist, rvec, tvec, length)


def detect_marker(frame_bgr, aruco_dict, aruco_params, K, dist):
    """
    Returns (corners, rvec, tvec) or (None, None, None) if not found.
    corners: (4,2) array of pixel coords.
    """
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    if ids is None:
        return None, None, None

    # Find our target marker ID
    for i, mid in enumerate(ids.flatten()):
        if mid == MARKER_ID:
            c = corners[i]
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [c], MARKER_SIZE_M, K, dist
            )
            return c[0], rvecs[0][0], tvecs[0][0]
    return None, None, None


def annotate_pose(frame, corners, rvec, tvec, K, dist, label: str):
    cv2.aruco.drawDetectedMarkers(frame, [corners.reshape(1, 4, 2)])
    draw_axes(frame, K, dist, rvec, tvec)
    dist_cm = np.linalg.norm(tvec) * 100
    angle_deg = np.degrees(np.linalg.norm(rvec))
    cv2.putText(frame, f"{label}  d={dist_cm:.1f}cm  rot={angle_deg:.1f}deg",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 80), 2)


def status_bar(frame, text: str, color=(200, 200, 200)):
    h = frame.shape[0]
    cv2.putText(frame, text, (8, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)


# ─────────────────────────────────────────────────────────────────────────────
# Intrinsics calibration (checkerboard)
# ─────────────────────────────────────────────────────────────────────────────

def calibrate_intrinsics(cap: cv2.VideoCapture, cam_name: str,
                          board=(9, 6), square_m=0.025,
                          n_samples=20) -> tuple[np.ndarray, np.ndarray]:
    """Capture checkerboard frames, return (K, dist)."""
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    objp = np.zeros((board[0] * board[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board[0], 0:board[1]].T.reshape(-1, 2)
    objp *= square_m

    obj_pts, img_pts = [], []
    print(f"\n[{cam_name}] Hold a {board[0]}×{board[1]} checkerboard (square={square_m*100:.1f}cm).")
    print("Press [Space] to capture a sample, [q] to finish early.")

    while len(obj_pts) < n_samples:
        ok, frame = cap.read()
        if not ok:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, board)
        display = frame.copy()
        if found:
            cv2.drawChessboardCorners(display, board, corners, found)
            cv2.putText(display, f"Samples: {len(obj_pts)}/{n_samples}  [Space]=capture",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(display, "No board found", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.imshow(f"Intrinsics — {cam_name}", display)
        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        if key == ord(' ') and found:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            obj_pts.append(objp)
            img_pts.append(corners2)
            print(f"  Captured sample {len(obj_pts)}")

    cv2.destroyAllWindows()
    if len(obj_pts) < 5:
        print("Too few samples — using default intrinsics.")
        return _DEFAULT_K.copy(), _DEFAULT_DIST.copy()

    h, w = frame.shape[:2]
    _, K, dist, _, _ = cv2.calibrateCamera(obj_pts, img_pts, (w, h), None, None)
    print(f"[{cam_name}] Calibrated: fx={K[0,0]:.1f} fy={K[1,1]:.1f} "
          f"cx={K[0,2]:.1f} cy={K[1,2]:.1f}")
    return K, dist


# ─────────────────────────────────────────────────────────────────────────────
# Hand-eye calibration
# ─────────────────────────────────────────────────────────────────────────────

def solve_hand_eye(samples: list[dict]) -> np.ndarray | None:
    """
    samples: list of {"T_base_gripper": 4×4, "T_base_cam": 4×4}
      T_base_gripper — robot FK (gripper pose in base frame)
      T_base_cam     — marker pose seen by BASE camera  (= target in base frame)

    We want X such that:
      T_base_cam = T_base_gripper @ X
    i.e. X = inv(T_base_gripper) @ T_base_cam

    In hand-eye parlance (eye-on-hand):  AX = XB
      A = T_gripper_i^{-1} @ T_gripper_j   (relative gripper motion)
      B = T_cam_i^{-1} @ T_cam_j            (relative cam motion, seen by GRIPPER cam)

    Here we use the simpler direct method: accumulate X = inv(T_bg) @ T_bc
    and average the rotation (via quaternion mean) and translation.
    """
    if len(samples) < 2:
        return None

    Rs, ts = [], []
    for s in samples:
        Tbg_inv = np.linalg.inv(s["T_base_gripper"])
        X = Tbg_inv @ s["T_base_marker_base_cam"]
        Rs.append(X[:3, :3])
        ts.append(X[:3,  3])

    # Simple mean of translations
    t_mean = np.mean(ts, axis=0)

    # Geodesic mean of rotations via SVD
    R_sum = np.zeros((3, 3))
    for R in Rs:
        R_sum += R
    U, _, Vt = np.linalg.svd(R_sum)
    R_mean = U @ Vt
    if np.linalg.det(R_mean) < 0:
        U[:, -1] *= -1
        R_mean = U @ Vt

    X_mean = np.eye(4)
    X_mean[:3, :3] = R_mean
    X_mean[:3,  3] = t_mean
    return X_mean


# ─────────────────────────────────────────────────────────────────────────────
# Main loop
# ─────────────────────────────────────────────────────────────────────────────

def run(base_idx: int, gripper_idx: int,
        K_base=None, dist_base=None,
        K_grip=None, dist_grip=None):

    if K_base  is None: K_base  = _DEFAULT_K.copy()
    if dist_base is None: dist_base = _DEFAULT_DIST.copy()
    if K_grip  is None: K_grip  = _DEFAULT_K.copy()
    if dist_grip is None: dist_grip = _DEFAULT_DIST.copy()

    aruco_dict   = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    aruco_params = cv2.aruco.DetectorParameters()

    cap_base   = open_camera(base_idx)
    cap_gripper = open_camera(gripper_idx)

    if not cap_base.isOpened():
        sys.exit(f"Cannot open base camera {base_idx}")
    if not cap_gripper.isOpened():
        sys.exit(f"Cannot open gripper camera {gripper_idx}")

    print(f"\nBase camera   → index {base_idx}")
    print(f"Gripper camera → index {gripper_idx}")
    print("Keys:  [c] collect sample   [s] solve & save   [q] quit\n")

    samples: list[dict] = []

    # Dummy gripper pose (identity) — replace with real FK if you wire in hardware.
    # The script still works for visual calibration without arm feedback.
    T_base_gripper = np.eye(4)

    cv2.namedWindow("Dual Camera — SO101", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Dual Camera — SO101", 1280, 520)

    fps_timer = time.time()
    frame_count = 0
    fps = 0.0

    while True:
        ok_b, frame_base   = cap_base.read()
        ok_g, frame_grip   = cap_gripper.read()

        if not ok_b:
            frame_base = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(frame_base, "BASE CAM ERROR", (150, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2)
        if not ok_g:
            frame_grip = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(frame_grip, "GRIPPER CAM ERROR", (120, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2)

        # ── Detect in BASE camera ─────────────────────────────────────────────
        T_marker_from_base = None
        c_b, rv_b, tv_b = detect_marker(frame_base, aruco_dict, aruco_params,
                                         K_base, dist_base)
        if c_b is not None:
            annotate_pose(frame_base, c_b, rv_b, tv_b, K_base, dist_base, "BASE")
            T_marker_from_base = rvec_tvec_to_matrix(rv_b, tv_b)

        # ── Detect in GRIPPER camera ──────────────────────────────────────────
        T_marker_from_grip = None
        c_g, rv_g, tv_g = detect_marker(frame_grip, aruco_dict, aruco_params,
                                         K_grip, dist_grip)
        if c_g is not None:
            annotate_pose(frame_grip, c_g, rv_g, tv_g, K_grip, dist_grip, "GRIPPER")
            T_marker_from_grip = rvec_tvec_to_matrix(rv_g, tv_g)

        # ── Overlay: relative distance between the two views ──────────────────
        if T_marker_from_base is not None and T_marker_from_grip is not None:
            d_base = np.linalg.norm(tv_b) * 100
            d_grip = np.linalg.norm(tv_g) * 100
            info = f"Marker: base={d_base:.1f}cm  gripper={d_grip:.1f}cm"
            cv2.putText(frame_base, info, (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        # ── Sample counter ────────────────────────────────────────────────────
        frame_count += 1
        if frame_count % 30 == 0:
            fps = 30 / (time.time() - fps_timer + 1e-9)
            fps_timer = time.time()

        status_bar(frame_base, f"BASE  cam={base_idx}  samples={len(samples)}  fps={fps:.0f}")
        status_bar(frame_grip, f"GRIPPER cam={gripper_idx}  [c]=sample  [s]=solve  [q]=quit")

        # ── Labels ────────────────────────────────────────────────────────────
        for f, lbl in [(frame_base, "BASE"), (frame_grip, "GRIPPER")]:
            cv2.putText(f, lbl, (f.shape[1] - 100, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        combined = np.hstack([frame_base, frame_grip])
        cv2.imshow("Dual Camera — SO101", combined)

        # ── Key handling ──────────────────────────────────────────────────────
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        elif key == ord('c'):
            if T_marker_from_base is None:
                print("  [c] No marker visible in BASE camera — skipped.")
            else:
                samples.append({
                    "T_base_gripper": T_base_gripper.tolist(),
                    "T_base_marker_base_cam": T_marker_from_base.tolist(),
                    "T_marker_from_grip": (T_marker_from_grip.tolist()
                                           if T_marker_from_grip is not None else None),
                })
                print(f"  [c] Collected sample {len(samples)}.  "
                      f"marker @ base-cam d={np.linalg.norm(tv_b)*100:.1f}cm")

        elif key == ord('s'):
            if len(samples) < 2:
                print("  [s] Need at least 2 samples — collect more with [c].")
            else:
                X = solve_hand_eye(samples)
                if X is not None:
                    # Pretty-print
                    rvec_x, tvec_x = matrix_to_rvec_tvec(X)
                    angle = np.degrees(np.linalg.norm(rvec_x))
                    dist_x = np.linalg.norm(tvec_x) * 100
                    print(f"\n  Hand-eye result ({len(samples)} samples):")
                    print(f"    Translation : {tvec_x.flatten()*100} cm  (|t|={dist_x:.1f} cm)")
                    print(f"    Rotation    : axis-angle ={rvec_x.flatten()}  ({angle:.1f}°)")
                    print(f"    Matrix X:\n{X}")

                    out = {
                        "n_samples": len(samples),
                        "X_gripper_to_base_cam": X.tolist(),
                        "translation_cm": (tvec_x.flatten() * 100).tolist(),
                        "rotation_deg": angle,
                    }
                    CALIB_OUT.write_text(json.dumps(out, indent=2))
                    print(f"  Saved → {CALIB_OUT.resolve()}")

    cap_base.release()
    cap_gripper.release()
    cv2.destroyAllWindows()


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    global MARKER_SIZE_M
    parser = argparse.ArgumentParser(description="SO101 dual-camera calibration tool")
    parser.add_argument("--base",    type=int, default=None,
                        help="Camera index for base camera (auto-detect if omitted)")
    parser.add_argument("--gripper", type=int, default=None,
                        help="Camera index for gripper camera (auto-detect if omitted)")
    parser.add_argument("--gen-marker", action="store_true",
                        help="Generate ArUco marker image and exit")
    parser.add_argument("--calibrate-intrinsics", action="store_true",
                        help="Run checkerboard intrinsics calibration first")
    parser.add_argument("--marker-size", type=float, default=MARKER_SIZE_M,
                        help=f"Physical marker side length in metres (default {MARKER_SIZE_M})")
    args = parser.parse_args()
    MARKER_SIZE_M = args.marker_size

    # ── Generate marker ───────────────────────────────────────────────────────
    if args.gen_marker:
        d = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        img = cv2.aruco.generateImageMarker(d, MARKER_ID, 600)
        out = Path("aruco_marker_6x6_id0.png")
        cv2.imwrite(str(out), img)
        print(f"Marker saved → {out.resolve()}")
        print(f"Print at 100mm × 100mm for --marker-size 0.10")
        return

    # ── Auto-detect cameras ───────────────────────────────────────────────────
    found = probe_cameras()
    if not found:
        sys.exit("No cameras detected.")
    print(f"Detected cameras: {found}")

    if len(found) == 1:
        print("Only one camera found — showing single feed (base only).")
        base_idx    = found[0]
        gripper_idx = found[0]   # same — will show duplicate
    else:
        base_idx    = args.base    if args.base    is not None else found[0]
        gripper_idx = args.gripper if args.gripper is not None else found[1]
        print(f"Using  base={base_idx}  gripper={gripper_idx}")
        print("Override with: --base N --gripper M")

    K_base = K_grip = _DEFAULT_K.copy()
    dist_base = dist_grip = _DEFAULT_DIST.copy()

    # ── Optional intrinsics calibration ───────────────────────────────────────
    if args.calibrate_intrinsics:
        print("\n=== Intrinsics calibration — BASE camera ===")
        cap_b = open_camera(base_idx)
        K_base, dist_base = calibrate_intrinsics(cap_b, "BASE")
        cap_b.release()

        print("\n=== Intrinsics calibration — GRIPPER camera ===")
        cap_g = open_camera(gripper_idx)
        K_grip, dist_grip = calibrate_intrinsics(cap_g, "GRIPPER")
        cap_g.release()

        np.save("K_base.npy", K_base);    np.save("dist_base.npy", dist_base)
        np.save("K_gripper.npy", K_grip); np.save("dist_gripper.npy", dist_grip)
        print("Intrinsics saved (K_base.npy, K_gripper.npy, dist_*.npy)")
    else:
        # Load previously saved intrinsics if available
        for fname, var_name in [("K_base.npy",    "K_base"),
                                 ("dist_base.npy", "dist_base"),
                                 ("K_gripper.npy", "K_grip"),
                                 ("dist_gripper.npy", "dist_grip")]:
            p = Path(fname)
            if p.exists():
                val = np.load(str(p))
                locals()[var_name]  # just to confirm name is valid
                if "K_base" in fname:    K_base    = val
                elif "dist_base" in fname: dist_base = val
                elif "K_gripper" in fname: K_grip   = val
                elif "dist_gripper" in fname: dist_grip = val
                print(f"Loaded {fname}")

    run(base_idx, gripper_idx,
        K_base=K_base, dist_base=dist_base,
        K_grip=K_grip, dist_grip=dist_grip)


if __name__ == "__main__":
    main()
