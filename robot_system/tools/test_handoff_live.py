"""
Live handoff test — uses the original robot_sam2_app hardware + go_home,
wrist camera with SAM2 box tracking, VL53 distance approach.

Flow:
  1. Go to home position exactly as the original app does (same speeds/easing).
  2. Open wrist camera.
  3. Click object → SAM2 draws a bounding box, tracks it frame-to-frame.
  4. Visual servo centres the box.
  5. VL53 drives elbow approach until stable at grip distance.
  6. Gripper closes.
  7. Hold, open, return home.

Controls:
  Click   — select object
  R       — reset tracking
  Q / Esc — quit

Run from robot_project/:
    python robot_system/tools/test_handoff_live.py
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# ── path setup ────────────────────────────────────────────────────────────────
ROOT = Path(__file__).resolve().parents[2]          # robot_project/
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "robot_system"))

# Original hardware layer
from robot_sam2_app.robot_sam2_app.hardware import FeetechHardware
from robot_sam2_app.robot_sam2_app.go_home_util import go_home
from robot_sam2_app.robot_sam2_app import config as cfg

# robot_system perception
from perception.sam2_adapter import Sam2Adapter

# ── SAM2 ──────────────────────────────────────────────────────────────────────
SAM2_CHECKPOINT = r"E:/sam2.1_hiera_tiny.pt"
SAM2_MODEL_CFG  = "configs/sam2.1/sam2.1_hiera_t.yaml"


def load_sam2(checkpoint: str, model_cfg: str) -> Sam2Adapter:
    print(f"Loading SAM2 from {checkpoint} ...")
    try:
        import torch
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor
        device = "cuda" if torch.cuda.is_available() else "cpu"
        model  = build_sam2(model_cfg, checkpoint, device=device)
        pred   = SAM2ImagePredictor(model)
        print(f"  SAM2 ready on {device}.")
        return Sam2Adapter(predictor=pred, dry_run=False)
    except KeyboardInterrupt:
        raise   # don't suppress Ctrl+C — let it bubble up to finally
    except Exception as e:
        print(f"  WARNING: SAM2 failed ({e}) — using synthetic box.")
        return Sam2Adapter(dry_run=True, dry_run_radius=70)


# ── VL53 serial reader ────────────────────────────────────────────────────────

class VL53:
    def __init__(self, port: str, baud: int = 115200):
        import serial, threading
        self._val: int | None = None
        self._lock = threading.Lock()
        self._ser  = serial.Serial(port, baud, timeout=1.0)
        self._t    = threading.Thread(target=self._loop, daemon=True)
        self._t.start()

    def _loop(self):
        while True:
            try:
                line = self._ser.readline().decode(errors="ignore").strip().lower()
                if "distance:" not in line:
                    continue
                val = int(line.replace("distance:", "").replace("mm", "").strip())
                if val > 0:
                    with self._lock:
                        self._val = val
            except Exception:
                pass

    def read(self) -> int | None:
        with self._lock:
            return self._val

    def close(self):
        try:
            self._ser.close()
        except Exception:
            pass


# ── Draw helpers ──────────────────────────────────────────────────────────────
FONT = cv2.FONT_HERSHEY_SIMPLEX


def draw_box(frame, box, color=(0, 255, 0), thickness=2):
    x0, y0, x1, y1 = (int(v) for v in box)
    cv2.rectangle(frame, (x0, y0), (x1, y1), color, thickness)
    cx, cy = (x0 + x1) // 2, (y0 + y1) // 2
    cv2.drawMarker(frame, (cx, cy), color, cv2.MARKER_CROSS, 14, 2)


def draw_hud(frame, lines, dist_mm=None, confirm=0, confirm_n=3):
    for i, line in enumerate(lines):
        cv2.putText(frame, line, (10, 26 + i * 26),
                    FONT, 0.65, (0, 255, 0), 1, cv2.LINE_AA)
    if dist_mm is not None:
        bx, by, bw, bh = 10, frame.shape[0] - 45, 180, 16
        cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (50, 50, 50), -1)
        filled = int(bw * min(confirm, confirm_n) / confirm_n)
        cv2.rectangle(frame, (bx, by), (bx + filled, by + bh), (0, 210, 0), -1)
        cv2.putText(frame, f"{dist_mm} mm  [{confirm}/{confirm_n}]",
                    (bx, by - 5), FONT, 0.5, (200, 200, 200), 1)


# ── Motor control helpers (using original hardware) ───────────────────────────

MOTOR_NAMES = cfg.MOTOR_NAMES   # ("base","shoulder","elbow","palm","wrist","gripper")
MOTOR_IDS   = cfg.MOTOR_IDS     # (1,2,3,4,5,6)


def _read_named(hw) -> dict[str, int]:
    raw = hw.read_ticks()
    return raw if raw else {}


def move_elbow(hw, current_ticks: dict[str, int], delta_ticks: int,
               el_min: int, el_max: int) -> int:
    """Move elbow by delta_ticks, clamped. Returns new tick value."""
    new = int(current_ticks.get("elbow", 2048) + delta_ticks)
    new = max(el_min, min(el_max, new))
    hw.write_ticks({**current_ticks, "elbow": new})
    return new


def move_pan_tilt(hw, current_ticks: dict[str, int],
                  d_base: int, d_shoulder: int,
                  base_min: int, base_max: int,
                  sh_min: int, sh_max: int) -> dict[str, int]:
    new_base = max(base_min, min(base_max,
                   current_ticks.get("base", 2048) + d_base))
    new_sh   = max(sh_min,   min(sh_max,
                   current_ticks.get("shoulder", 2048) + d_shoulder))
    updated  = {**current_ticks, "base": new_base, "shoulder": new_sh}
    hw.write_ticks(updated)
    return updated


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port",       default="COM3",  help="VL53 serial port")
    p.add_argument("--home",       default="StartHelloPos_handoff.json",
                                   help="Home position JSON filename (in robot_project/)")
    p.add_argument("--cam",        type=int, default=1)
    p.add_argument("--grip-dist",  type=int, default=75)
    p.add_argument("--stable-win", type=int, default=15)
    p.add_argument("--max-jump",   type=int, default=30)
    p.add_argument("--confirm-n",  type=int, default=3)
    p.add_argument("--hold-s",     type=float, default=1.5)
    p.add_argument("--el-min",     type=int, default=700,  help="Elbow min ticks")
    p.add_argument("--el-max",     type=int, default=2400, help="Elbow max ticks")
    p.add_argument("--kp-pan",     type=float, default=120.0, help="Base ticks per unit x-error")
    p.add_argument("--kp-tilt",    type=float, default=120.0, help="Shoulder ticks per unit y-error")
    p.add_argument("--kp-elbow",   type=float, default=0.8,   help="Elbow ticks per mm distance error")
    p.add_argument("--dead",       type=float, default=0.10,  help="Centering dead zone (norm)")
    p.add_argument("--flip-pan",   action="store_true", help="Flip pan direction")
    p.add_argument("--flip-tilt",  action="store_true", help="Flip tilt direction")
    p.add_argument("--centre-hold", type=int,  default=8,    help="Frames centred before approach starts")
    p.add_argument("--no-sam2",    action="store_true")
    p.add_argument("--checkpoint", default=SAM2_CHECKPOINT)
    p.add_argument("--sam-cfg",    default=SAM2_MODEL_CFG)
    args = p.parse_args()

    # ── 1. Load home position ─────────────────────────────────────
    home_path = ROOT / args.home
    if not home_path.exists():
        home_path = ROOT / "StartHelloPos.json"
    home_pos = {int(k): v for k, v in json.loads(home_path.read_text()).items()}
    print(f"Home position: {home_pos}")

    # Declare resources before try so finally can always clean them up
    hw   = None
    vl53 = None
    cap  = None

    try:
        # ── 2. Connect hardware ───────────────────────────────────
        print("Connecting to robot...")
        hw = FeetechHardware()
        hw.connect()
        if not hw.connected:
            print("ERROR: could not connect to robot.")
            return

        # ── 3. Go to home — exactly as original app does ─────────
        print("Going to home position...")
        go_home(hw, home_pos)

        # Open gripper and build the held_ticks dict from home pos
        held_ticks = _read_named(hw)
        held_ticks["gripper"] = cfg.GRIPPER_OPEN
        hw.write_ticks(held_ticks)
        time.sleep(0.3)

        # ── 4. VL53 sensor ────────────────────────────────────────
        print(f"Connecting to VL53 on {args.port}...")
        vl53 = VL53(args.port)
        time.sleep(0.6)
        print(f"  First reading: {vl53.read()} mm")

        # ── 5. SAM2 ──────────────────────────────────────────────
        if args.no_sam2:
            sam = Sam2Adapter(dry_run=True, dry_run_radius=70)
        else:
            sam = load_sam2(args.checkpoint, args.sam_cfg)

        # ── 6. Camera ────────────────────────────────────────────
        print(f"Opening camera {args.cam}...")
        cap = cv2.VideoCapture(args.cam, cv2.CAP_DSHOW)
        if not cap.isOpened():
            print(f"ERROR: camera {args.cam} not found. Try --cam 0")
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # ── State ─────────────────────────────────────────────────
        click_xy    = None
        tracking    = False
        current_box = None   # (x0, y0, x1, y1) — updated by SAM2 each frame
        BOX_ALPHA   = 0.55   # EMA smoothing
        phase       = "WAITING"

        # Stability buffer for VL53
        from collections import deque
        buf: deque[int] = deque(maxlen=10)
        confirm       = 0
        centre_frames = 0
        settle_frames = 0   # frames to skip after each motor move (let camera settle)

        def on_mouse(event, x, y, flags, param):
            nonlocal click_xy, tracking, current_box, phase, buf, confirm, \
                     centre_frames, settle_frames
            if event == cv2.EVENT_LBUTTONDOWN:
                click_xy      = (x, y)
                tracking      = False
                current_box   = None
                phase         = "CENTERING"
                buf.clear()
                confirm       = 0
                centre_frames = 0
                settle_frames = 0

        cv2.namedWindow("Handoff", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("Handoff", on_mouse)
        print("\nCamera ready. Click on the object to track.")
        print("R = reset | Q/Esc = quit\n")

        dist_mm   = None
        nx = ny   = 0.0

        while True:
            ret, raw = cap.read()
            if not ret:
                print("Camera read failed.")
                break

            frame_rgb = cv2.cvtColor(raw, cv2.COLOR_BGR2RGB)
            display   = raw.copy()

            # Hold ALL motors at their current commanded positions every frame
            hw.write_ticks(held_ticks)

            # ── SAM2 tracking — fresh predict each correction cycle ──
            if click_xy and not tracking:
                sam.set_image(frame_rgb)
                init = sam.predict(click_xy)
                bb   = init.features.bbox
                if bb.area > 0:
                    cx_px = float(init.features.centroid_px[0])
                    cy_px = float(init.features.centroid_px[1])
                    current_box = (float(bb.x_min), float(bb.y_min),
                                   float(bb.x_max), float(bb.y_max))
                    click_xy = (cx_px, cy_px)   # update click to centroid for next predict
                    tracking = True
                    print(f"Tracking started: centroid=({cx_px:.0f},{cy_px:.0f})")

            if tracking and settle_frames == 0:
                # Run SAM2 at last known centroid — self-updates each cycle
                sam.set_image(frame_rgb)
                result = sam.predict(click_xy)
                bb     = result.features.bbox
                if bb.area > 0:
                    cx_px = float(result.features.centroid_px[0])
                    cy_px = float(result.features.centroid_px[1])
                    current_box = (float(bb.x_min), float(bb.y_min),
                                   float(bb.x_max), float(bb.y_max))
                    click_xy = (cx_px, cy_px)   # update for next frame

            if tracking and current_box is not None:
                # Normalised centroid error from frame centre
                H, W = display.shape[:2]
                cx_px = (current_box[0] + current_box[2]) / 2.0
                cy_px = (current_box[1] + current_box[3]) / 2.0
                nx = (cx_px - W / 2.0) / (W / 2.0)
                ny = (cy_px - H / 2.0) / (H / 2.0)

                draw_box(display, current_box)

                # ── Centering servo ───────────────────────────────
                ticks = _read_named(hw)
                dead  = args.dead
                is_centred = abs(nx) < dead and abs(ny) < dead

                if phase == "CENTERING":
                    if settle_frames > 0:
                        settle_frames -= 1
                    elif abs(nx) > dead:
                        ex      = nx - math.copysign(dead, nx)
                        pan_dir = -1 if args.flip_pan else 1
                        d_base  = int(max(-8, min(8, pan_dir * args.kp_pan * ex)))
                        held_ticks["base"] = max(1000, min(3100, held_ticks["base"] + d_base))
                        print(f"  err=({nx:+.2f},{ny:+.2f})  d_base={d_base:+d}")
                        settle_frames = 4
                        centre_frames = 0
                    else:
                        centre_frames += 1
                        if centre_frames >= args.centre_hold:
                            phase = "APPROACHING"
                            centre_frames = 0
                            buf.clear()
                            confirm = 0
                            print("Centred — approaching...")

                # ── Approach via VL53 ─────────────────────────────
                if phase == "APPROACHING":
                    dist_mm = vl53.read()
                    if dist_mm is not None:
                        buf.append(dist_mm)
                        print(f"  VL53: {dist_mm} mm")

                    last3   = list(buf)[-3:] if len(buf) >= 3 else []
                    stable  = (len(last3) == 3
                               and max(last3) - min(last3) <= args.stable_win)
                    no_jump = all(abs(b - a) <= args.max_jump
                                  for a, b in zip(list(buf), list(buf)[1:]))
                    is_close = stable and no_jump and sum(last3) / 3 <= args.grip_dist

                    if is_close:
                        confirm += 1
                    else:
                        confirm = 0

                    if confirm >= args.confirm_n:
                        print(f"\nStable at {dist_mm} mm — closing gripper!")
                        held_ticks["gripper"] = cfg.GRIPPER_CLOSE
                        hw.write_ticks(held_ticks)
                        time.sleep(args.hold_s)
                        print("Opening gripper, returning home...")
                        held_ticks["gripper"] = cfg.GRIPPER_OPEN
                        hw.write_ticks(held_ticks)
                        time.sleep(0.4)
                        go_home(hw, home_pos)
                        held_ticks = _read_named(hw)
                        held_ticks["gripper"] = cfg.GRIPPER_OPEN
                        tracking = False; click_xy = None
                        current_box = None
                        phase = "WAITING"; buf.clear()
                        confirm = 0; dist_mm = None
                        print("\nReady. Click to track next object.")
                        continue

                    # drive elbow toward object
                    if dist_mm is not None and dist_mm > args.grip_dist:
                        delta = int(args.kp_elbow * (dist_mm - args.grip_dist))
                        held_ticks["elbow"] = max(args.el_min, min(args.el_max, held_ticks["elbow"] + delta))

            # ── HUD ───────────────────────────────────────────────
            if tracking:
                if phase == "CENTERING":
                    status = f"CENTERING  Hold:{centre_frames}/{args.centre_hold}  Err:({nx:+.2f},{ny:+.2f})"
                else:
                    status = f"APPROACHING  VL53:{dist_mm}mm  Confirm:{confirm}/{args.confirm_n}"
            else:
                status = "Click object to track"
            cv2.putText(display, status, (10, 30),
                        FONT, 0.65, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.imshow("Handoff", display)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                tracking      = False
                click_xy      = None
                current_box   = None
                phase         = "WAITING"
                buf.clear()
                confirm       = 0
                centre_frames = 0
                settle_frames = 0
                current_box   = None
                print("Reset.")

    except KeyboardInterrupt:
        pass
    except Exception as e:
        import traceback
        print(f"\nERROR: {e}")
        traceback.print_exc()

    finally:
        # ── Cleanup — always runs no matter how we exit ───────────
        print("\nShutting down...")
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()
        if hw is not None and hw.connected:
            try:
                ticks = _read_named(hw)
                if ticks:
                    ticks["gripper"] = cfg.GRIPPER_OPEN
                    hw.write_ticks(ticks)
                    time.sleep(0.3)
                go_home(hw, home_pos)
            except Exception as e:
                print(f"Cleanup go_home error: {e}")
            hw.disconnect()
        if vl53 is not None:
            vl53.close()


if __name__ == "__main__":
    main()
