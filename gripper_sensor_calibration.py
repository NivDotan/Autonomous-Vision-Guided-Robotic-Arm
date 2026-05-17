"""
Gripper sensor calibration — SO101 arm.

Connects simultaneously to:
  1. Gripper camera    — USB camera (OpenCV)
  2. Feetech servo bus — motor positions via lerobot (COM4 by default)
  3. VL53L1X on ESP32  — distance over USB serial

Live display shows all three data streams in one window.

Calibration modes (keys):
  [i]  Intrinsics  — checkerboard, saves K_gripper.npy / dist_gripper.npy
  [t]  ToF align   — compare camera-estimated depth vs VL53L1X reading, saves tof_calibration.json
  [p]  Pose sample — collect (motor ticks, ArUco pose, ToF) sample for gripper-cam extrinsics
  [s]  Solve & save — compute camera-to-wrist transform from collected pose samples
  [q]  Quit

Dependencies (all already in the project):
  pip install opencv-contrib-python pyserial
  lerobot must be installed for motor bus (already used by the project)
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import serial
import serial.tools.list_ports

# ── Constants — edit to match your setup ──────────────────────────────────────

MOTOR_PORT     = "COM4"          # Feetech servo bus port
MOTOR_IDS      = (1, 2, 3, 4, 5, 6)
MOTOR_NAMES    = ("base", "shoulder", "elbow", "palm", "wrist", "gripper")

ESP32_BAUD     = 115200          # baud rate for ESP32 serial
CAMERA_INDEX   = 1               # gripper camera index (try 0, 1, 2 …)

ARUCO_DICT     = cv2.aruco.DICT_6X6_50
MARKER_ID      = 0
MARKER_SIZE_M  = 0.10            # printed marker physical size in metres

CALIB_OUT      = Path("gripper_calib.json")

# ── Default intrinsics (replaced once you run [i]) ────────────────────────────
_K0    = np.array([[600., 0., 320.], [0., 600., 240.], [0., 0., 1.]], np.float64)
_DIST0 = np.zeros((5, 1), np.float64)


# ─────────────────────────────────────────────────────────────────────────────
# ESP32 / VL53L1X reader  (runs in a background thread)
# ─────────────────────────────────────────────────────────────────────────────

class ToFReader:
    """
    Reads VL53L1X distance from ESP32 over USB serial.

    Expected serial output (any of these formats):
        125
        Distance: 125 mm
        {"dist":125}
        DIST:125
    Value is stored in self.distance_mm (float, or None if no reading yet).
    """

    def __init__(self, port: Optional[str], baud: int = ESP32_BAUD):
        self.port        = port
        self.baud        = baud
        self.distance_mm: Optional[float] = None
        self.connected   = False
        self._ser: Optional[serial.Serial] = None
        self._stop       = threading.Event()
        self._thread     = threading.Thread(target=self._run, daemon=True)

    def start(self) -> bool:
        if self.port is None:
            self.port = _auto_detect_esp32()
        if self.port is None:
            print("[ToF] No ESP32 port found — distance sensor disabled.")
            return False
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=1)
            self.connected = True
            self._thread.start()
            print(f"[ToF] Connected to {self.port} @ {self.baud} baud")
            return True
        except serial.SerialException as e:
            print(f"[ToF] Cannot open {self.port}: {e}")
            return False

    def stop(self):
        self._stop.set()
        if self._ser and self._ser.is_open:
            self._ser.close()

    def _run(self):
        while not self._stop.is_set():
            try:
                line = self._ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                self.distance_mm = _parse_distance(line)
            except Exception:
                pass

    def _parse_distance(self, line: str) -> Optional[float]:
        return _parse_distance(line)


def _parse_distance(line: str) -> Optional[float]:
    """Extract a millimetre value from various ESP32 output formats."""
    import re
    # {"dist": 125}  or  {"distance": 125}
    m = re.search(r'"dist(?:ance)?"\s*:\s*([0-9]+(?:\.[0-9]+)?)', line)
    if m:
        return float(m.group(1))
    # Distance: 125 mm  /  DIST:125  /  Dist=125
    m = re.search(r'[Dd]ist(?:ance)?\s*[:=]\s*([0-9]+(?:\.[0-9]+)?)', line)
    if m:
        return float(m.group(1))
    # bare number
    m = re.fullmatch(r'\s*([0-9]+(?:\.[0-9]+)?)\s*', line)
    if m:
        return float(m.group(1))
    return None


def _auto_detect_esp32() -> Optional[str]:
    """Return the first serial port that looks like an ESP32 / CH340 / CP210x."""
    keywords = ["CP210", "CH340", "CH341", "FTDI", "USB Serial", "ESP32", "Silicon Labs"]
    ports = list(serial.tools.list_ports.comports())
    print(f"[ToF] Available ports: {[p.device for p in ports]}")
    for p in ports:
        desc = (p.description or "") + (p.manufacturer or "")
        if any(k.lower() in desc.lower() for k in keywords):
            print(f"[ToF] Auto-selected {p.device}  ({p.description})")
            return p.device
    if ports:
        print(f"[ToF] Falling back to first port: {ports[0].device}")
        return ports[0].device
    return None


# ─────────────────────────────────────────────────────────────────────────────
# Motor bus reader  (thin wrapper, no lerobot import at module level)
# ─────────────────────────────────────────────────────────────────────────────

class MotorReader:
    def __init__(self, port: str = MOTOR_PORT):
        self.port      = port
        self.bus       = None
        self.connected = False
        self._ticks: dict[str, int] = {}

    def connect(self) -> bool:
        try:
            from lerobot.motors.feetech import FeetechMotorsBus

            class _M:
                def __init__(self, mid): self.id = mid; self.model = "sts3215"

            motors = {f"motor_{i}": _M(i) for i in MOTOR_IDS}
            self.bus = FeetechMotorsBus(port=self.port, motors=motors)
            self.bus.connect()
            self.connected = True
            print(f"[Motors] Connected on {self.port}")
            return True
        except Exception as e:
            print(f"[Motors] Unavailable: {e}")
            return False

    def read(self) -> dict[str, int]:
        if self.bus is None:
            return {}
        try:
            self._ticks = {
                name: int(self.bus.read("Present_Position", f"motor_{mid}", normalize=False))
                for name, mid in zip(MOTOR_NAMES, MOTOR_IDS)
            }
        except Exception:
            pass
        return dict(self._ticks)

    def disconnect(self):
        if self.bus is not None:
            try:
                self.bus.disconnect()
            except Exception:
                pass
            self.bus = None


# ─────────────────────────────────────────────────────────────────────────────
# Intrinsics calibration
# ─────────────────────────────────────────────────────────────────────────────

def run_intrinsics_calibration(cap: cv2.VideoCapture,
                                board=(9, 6), square_m=0.025,
                                n_samples=25) -> tuple[np.ndarray, np.ndarray]:
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    objp = np.zeros((board[0] * board[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board[0], 0:board[1]].T.reshape(-1, 2) * square_m

    obj_pts, img_pts = [], []
    print(f"\n[Intrinsics] Show a {board[0]}×{board[1]} checkerboard (square={square_m*100:.1f}cm).")
    print("  [Space] capture sample   [q] finish early\n")

    frame = None
    while len(obj_pts) < n_samples:
        ok, frame = cap.read()
        if not ok:
            continue
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, board)
        disp  = frame.copy()
        if found:
            cv2.drawChessboardCorners(disp, board, corners, True)
            cv2.putText(disp, f"Samples {len(obj_pts)}/{n_samples}  [Space]=capture",
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 60), 2)
        else:
            cv2.putText(disp, "No checkerboard visible", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 60, 255), 2)
        cv2.imshow("Intrinsics calibration", disp)
        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        if key == ord(' ') and found:
            c2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            obj_pts.append(objp)
            img_pts.append(c2)
            print(f"  Sample {len(obj_pts)} captured")

    cv2.destroyWindow("Intrinsics calibration")
    if len(obj_pts) < 5:
        print("  Too few samples — using default intrinsics.")
        return _K0.copy(), _DIST0.copy()

    h, w = frame.shape[:2]
    rms, K, dist, _, _ = cv2.calibrateCamera(obj_pts, img_pts, (w, h), None, None)
    print(f"\n  RMS reprojection error: {rms:.4f} px")
    print(f"  fx={K[0,0]:.1f}  fy={K[1,1]:.1f}  cx={K[0,2]:.1f}  cy={K[1,2]:.1f}")
    np.save("K_gripper.npy",    K)
    np.save("dist_gripper.npy", dist)
    print("  Saved K_gripper.npy  dist_gripper.npy")
    return K, dist


# ─────────────────────────────────────────────────────────────────────────────
# ArUco helpers
# ─────────────────────────────────────────────────────────────────────────────

def detect_marker(frame, aruco_dict, aruco_params, K, dist):
    """Return (corners_4x2, rvec, tvec) or (None, None, None)."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    if ids is None:
        return None, None, None
    for i, mid in enumerate(ids.flatten()):
        if mid == MARKER_ID:
            c = corners[i]
            rv, tv, _ = cv2.aruco.estimatePoseSingleMarkers([c], MARKER_SIZE_M, K, dist)
            return c[0], rv[0][0], tv[0][0]
    return None, None, None


def rvec_tvec_to_mat(rv, tv) -> np.ndarray:
    R, _ = cv2.Rodrigues(rv)
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = tv.flatten()
    return T


# ─────────────────────────────────────────────────────────────────────────────
# ToF vs camera depth alignment
# ─────────────────────────────────────────────────────────────────────────────

def run_tof_alignment(cap, tof: ToFReader, K, dist,
                       aruco_dict, aruco_params, n_samples=20):
    """
    Move the marker to different distances (10–50 cm range).
    Records (camera_depth_mm, tof_mm) pairs and fits a linear correction:
        tof_corrected = scale * tof_raw + offset
    """
    cam_depths, tof_depths = [], []
    print(f"\n[ToF align] Move ArUco marker toward/away from gripper camera.")
    print("  [Space] capture sample   [q] finish\n")

    while len(cam_depths) < n_samples:
        ok, frame = cap.read()
        if not ok:
            continue
        c, rv, tv = detect_marker(frame, aruco_dict, aruco_params, K, dist)
        disp = frame.copy()
        cam_d = None
        if c is not None:
            cam_d = np.linalg.norm(tv) * 1000   # metres → mm
            cv2.aruco.drawDetectedMarkers(disp, [c.reshape(1, 4, 2)])
            cv2.drawFrameAxes(disp, K, dist, rv, tv, 0.03)
            cv2.putText(disp, f"Cam depth: {cam_d:.0f} mm", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 60), 2)

        tof_d = tof.distance_mm
        tof_str = f"{tof_d:.0f} mm" if tof_d is not None else "---"
        cv2.putText(disp, f"ToF: {tof_str}", (10, 58),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 200, 0), 2)
        cv2.putText(disp, f"Samples: {len(cam_depths)}/{n_samples}  [Space]=capture  [q]=done",
                    (10, disp.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.imshow("ToF alignment", disp)
        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        if key == ord(' ') and cam_d is not None and tof_d is not None:
            cam_depths.append(cam_d)
            tof_depths.append(tof_d)
            print(f"  Sample {len(cam_depths)}: cam={cam_d:.0f} mm  tof={tof_d:.0f} mm  "
                  f"diff={cam_d - tof_d:+.0f} mm")

    cv2.destroyWindow("ToF alignment")
    if len(cam_depths) < 3:
        print("  Too few samples — skipping fit.")
        return {"scale": 1.0, "offset_mm": 0.0}

    x = np.array(tof_depths)
    y = np.array(cam_depths)
    # Least-squares linear fit:  y = scale*x + offset
    A = np.vstack([x, np.ones_like(x)]).T
    scale, offset = np.linalg.lstsq(A, y, rcond=None)[0]

    residuals = y - (scale * x + offset)
    rmse = np.sqrt(np.mean(residuals ** 2))
    print(f"\n  ToF correction:  depth_corrected = {scale:.4f} * tof + {offset:.2f} mm")
    print(f"  RMSE: {rmse:.2f} mm over {len(cam_depths)} samples")

    result = {"scale": float(scale), "offset_mm": float(offset), "rmse_mm": float(rmse),
              "n_samples": len(cam_depths)}
    return result


# ─────────────────────────────────────────────────────────────────────────────
# Pose sample collection & hand-eye solve
# ─────────────────────────────────────────────────────────────────────────────

def solve_camera_to_wrist(samples: list[dict]) -> Optional[np.ndarray]:
    """
    Each sample: {"ticks": {...}, "T_cam_marker": 4×4}
    We want the fixed transform T_cam_wrist (camera mounted on wrist link).
    Since the marker is fixed in the world, T_world_marker is constant.
    We accumulate T_cam_marker across poses and average.
    This gives T_cam_marker — which is the camera→marker transform, proxy
    for the camera extrinsics relative to the wrist.
    """
    if len(samples) < 2:
        return None
    mats = [np.array(s["T_cam_marker"]) for s in samples]
    # Average translations
    t_mean = np.mean([m[:3, 3] for m in mats], axis=0)
    # Average rotations via SVD
    R_sum = sum(m[:3, :3] for m in mats)
    U, _, Vt = np.linalg.svd(R_sum)
    R_mean = U @ Vt
    if np.linalg.det(R_mean) < 0:
        U[:, -1] *= -1
        R_mean = U @ Vt
    T = np.eye(4)
    T[:3, :3] = R_mean
    T[:3,  3] = t_mean
    return T


# ─────────────────────────────────────────────────────────────────────────────
# HUD drawing
# ─────────────────────────────────────────────────────────────────────────────

def draw_hud(frame, ticks: dict, tof_mm: Optional[float],
             cam_depth_mm: Optional[float], n_samples: int, status: str):
    h, w = frame.shape[:2]
    overlay = frame.copy()

    # Semi-transparent panel on the right
    panel_w = 220
    cv2.rectangle(overlay, (w - panel_w, 0), (w, h), (20, 20, 20), -1)
    alpha = 0.6
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)

    x0 = w - panel_w + 8
    y  = 22
    dy = 22

    def put(text, color=(220, 220, 220), scale=0.48):
        nonlocal y
        cv2.putText(frame, text, (x0, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1)
        y += dy

    put("── MOTORS ──", (100, 200, 255), 0.5)
    if ticks:
        for name in MOTOR_NAMES:
            v = ticks.get(name, "?")
            put(f"  {name:<9} {v}", (180, 255, 180))
    else:
        put("  not connected", (80, 80, 200))

    put("")
    put("── VL53L1X ──", (100, 200, 255), 0.5)
    if tof_mm is not None:
        color = (0, 255, 120) if tof_mm < 400 else (0, 200, 255)
        put(f"  ToF: {tof_mm:.0f} mm", color)
    else:
        put("  no reading", (80, 80, 200))

    put("")
    put("── CAMERA ──", (100, 200, 255), 0.5)
    if cam_depth_mm is not None:
        put(f"  ArUco: {cam_depth_mm:.0f} mm", (0, 255, 120))
        if tof_mm is not None:
            diff = cam_depth_mm - tof_mm
            col = (0, 255, 120) if abs(diff) < 10 else (0, 160, 255)
            put(f"  diff:  {diff:+.0f} mm", col)
    else:
        put("  marker not seen", (80, 80, 200))

    put("")
    put(f"Samples: {n_samples}", (200, 200, 100))
    put("")
    put("── KEYS ──", (160, 160, 160), 0.44)
    for k, lbl in [("[i]", "intrinsics"), ("[t]", "tof align"),
                   ("[p]", "pose sample"), ("[s]", "solve+save"), ("[q]", "quit")]:
        put(f"  {k} {lbl}", (160, 160, 160), 0.42)

    # Status bar at bottom
    cv2.rectangle(frame, (0, h - 26), (w - panel_w, h), (30, 30, 30), -1)
    cv2.putText(frame, status, (8, h - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (200, 200, 200), 1)


# ─────────────────────────────────────────────────────────────────────────────
# Main loop
# ─────────────────────────────────────────────────────────────────────────────

def run(camera_idx: int, motor_port: str, esp32_port: Optional[str],
        K: np.ndarray, dist: np.ndarray):

    aruco_dict   = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    aruco_params = cv2.aruco.DetectorParameters()

    cap = cv2.VideoCapture(camera_idx, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {camera_idx}")
    print(f"[Camera] Opened index {camera_idx}")

    motors = MotorReader(motor_port)
    motors.connect()

    tof = ToFReader(esp32_port)
    tof.start()

    # Load saved intrinsics if available
    for fname, attr in [("K_gripper.npy", "K"), ("dist_gripper.npy", "dist")]:
        p = Path(fname)
        if p.exists():
            val = np.load(str(p))
            if attr == "K":    K    = val
            else:              dist = val
            print(f"[Camera] Loaded {fname}")

    tof_calib  = {"scale": 1.0, "offset_mm": 0.0}
    pose_samples: list[dict] = []
    status = "Ready — press [i] to calibrate intrinsics first"

    cv2.namedWindow("Gripper Sensor Calibration", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Gripper Sensor Calibration", 860, 500)

    ticks: dict[str, int] = {}
    ticks_timer = 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            frame = np.zeros((480, 640, 3), np.uint8)
            cv2.putText(frame, "CAMERA ERROR", (180, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        # Read motors at ~10 Hz (not every frame)
        now = time.time()
        if now - ticks_timer > 0.1:
            ticks = motors.read()
            ticks_timer = now

        # ArUco detection
        c, rv, tv = detect_marker(frame, aruco_dict, aruco_params, K, dist)
        cam_depth_mm = None
        if c is not None:
            cam_depth_mm = np.linalg.norm(tv) * 1000
            cv2.aruco.drawDetectedMarkers(frame, [c.reshape(1, 4, 2)])
            cv2.drawFrameAxes(frame, K, dist, rv, tv, MARKER_SIZE_M * 0.5)
            cv2.putText(frame, f"ArUco ID={MARKER_ID}  d={cam_depth_mm:.0f}mm",
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 80), 2)

        # Apply ToF correction to raw reading
        tof_raw = tof.distance_mm
        tof_corrected = None
        if tof_raw is not None:
            tof_corrected = tof_calib["scale"] * tof_raw + tof_calib["offset_mm"]

        draw_hud(frame, ticks, tof_corrected, cam_depth_mm, len(pose_samples), status)
        cv2.imshow("Gripper Sensor Calibration", frame)

        key = cv2.waitKey(1) & 0xFF

        # ── quit ─────────────────────────────────────────────────────────────
        if key == ord('q'):
            break

        # ── intrinsics calibration ────────────────────────────────────────────
        elif key == ord('i'):
            status = "Running intrinsics calibration…"
            K, dist = run_intrinsics_calibration(cap)
            status = f"Intrinsics done — RMS shown in console"

        # ── ToF alignment ─────────────────────────────────────────────────────
        elif key == ord('t'):
            if not tof.connected:
                status = "ToF not connected — check ESP32 port"
            else:
                status = "Running ToF alignment…"
                tof_calib = run_tof_alignment(cap, tof, K, dist,
                                              aruco_dict, aruco_params)
                status = (f"ToF aligned: scale={tof_calib['scale']:.3f} "
                          f"offset={tof_calib['offset_mm']:+.1f}mm")

        # ── collect pose sample ───────────────────────────────────────────────
        elif key == ord('p'):
            if c is None:
                status = "No marker visible — cannot collect pose sample"
            else:
                T_cam_marker = rvec_tvec_to_mat(rv, tv)
                pose_samples.append({
                    "ticks": dict(ticks),
                    "T_cam_marker": T_cam_marker.tolist(),
                    "tof_mm": float(tof_corrected) if tof_corrected is not None else None,
                    "cam_depth_mm": float(cam_depth_mm),
                })
                status = f"Pose sample {len(pose_samples)} collected  (cam={cam_depth_mm:.0f}mm)"
                print(f"[Pose] Sample {len(pose_samples)}: "
                      f"cam={cam_depth_mm:.0f}mm  "
                      f"tof={tof_corrected:.0f}mm  " if tof_corrected else "tof=---  "
                      f"ticks={ticks}")

        # ── solve & save ──────────────────────────────────────────────────────
        elif key == ord('s'):
            if len(pose_samples) < 2:
                status = "Need ≥2 pose samples first — press [p] at different arm positions"
            else:
                T_cam_wrist = solve_camera_to_wrist(pose_samples)
                if T_cam_wrist is not None:
                    rv_out, _ = cv2.Rodrigues(T_cam_wrist[:3, :3])
                    angle_deg = float(np.degrees(np.linalg.norm(rv_out)))
                    t_cm      = T_cam_wrist[:3, 3] * 100

                    result = {
                        "camera_index": camera_idx,
                        "n_pose_samples": len(pose_samples),
                        "T_cam_to_wrist": T_cam_wrist.tolist(),
                        "translation_cm": t_cm.tolist(),
                        "rotation_deg": angle_deg,
                        "tof_calibration": tof_calib,
                        "K": K.tolist(),
                        "dist": dist.tolist(),
                    }
                    CALIB_OUT.write_text(json.dumps(result, indent=2))
                    print(f"\n[Save] Calibration → {CALIB_OUT.resolve()}")
                    print(f"  Camera→wrist translation: {t_cm} cm")
                    print(f"  Camera→wrist rotation:    {angle_deg:.1f}°")
                    status = f"Saved to {CALIB_OUT}  ({len(pose_samples)} samples)"

    cap.release()
    motors.disconnect()
    tof.stop()
    cv2.destroyAllWindows()


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    global MARKER_SIZE_M, ESP32_BAUD
    parser = argparse.ArgumentParser(
        description="Gripper camera + Feetech motors + VL53L1X calibration tool")
    parser.add_argument("--camera",       type=int,   default=CAMERA_INDEX,
                        help=f"Camera index (default {CAMERA_INDEX})")
    parser.add_argument("--motor-port",   type=str,   default=MOTOR_PORT,
                        help=f"Feetech servo bus port (default {MOTOR_PORT})")
    parser.add_argument("--esp32-port",   type=str,   default=None,
                        help="ESP32 serial port (auto-detect if omitted)")
    parser.add_argument("--esp32-baud",   type=int,   default=ESP32_BAUD,
                        help=f"ESP32 baud rate (default {ESP32_BAUD})")
    parser.add_argument("--marker-size",  type=float, default=MARKER_SIZE_M,
                        help=f"ArUco marker physical size in metres (default {MARKER_SIZE_M})")
    parser.add_argument("--gen-marker",   action="store_true",
                        help="Generate ArUco marker PNG and exit")
    args = parser.parse_args()
    MARKER_SIZE_M = args.marker_size
    ESP32_BAUD    = args.esp32_baud

    if args.gen_marker:
        d   = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        img = cv2.aruco.generateImageMarker(d, MARKER_ID, 600)
        out = Path("aruco_marker_6x6_id0.png")
        cv2.imwrite(str(out), img)
        print(f"Marker saved → {out.resolve()}  (print at {MARKER_SIZE_M*100:.0f}cm × {MARKER_SIZE_M*100:.0f}cm)")
        return

    # Try loading saved intrinsics
    K, dist = _K0.copy(), _DIST0.copy()

    run(camera_idx=args.camera,
        motor_port=args.motor_port,
        esp32_port=args.esp32_port,
        K=K, dist=dist)


if __name__ == "__main__":
    main()
