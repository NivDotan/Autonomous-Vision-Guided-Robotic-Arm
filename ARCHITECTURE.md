# Architecture — SO-101 Robot Vision & Control System

## Overview

An autonomous pick-and-place system built around an SO-101 6-DOF robotic arm.
The camera sees the scene, SAM2 segments the target object, IBVS drives the arm
toward it, and a VL53L1X distance sensor gates the final grip.

```
┌─────────────────────────────────────────────────────────┐
│                        Hardware                          │
│  Base cam (idx 0) ──► Python App ◄── Main cam (idx 1)   │
│  wide view, drives base  │       approach / SAM2 track  │
│                          │                              │
│                    VL53L1X (ESP32/COM3)                  │
│                          │                              │
│                      ZMQ REQ                            │
│                          │                              │
│              motor_daemon.exe (C++, 200 Hz, COM4)        │
│                          │                              │
│       6× Feetech STS3215 (base/shoulder/elbow/           │
│                            palm/wrist/gripper)           │
└─────────────────────────────────────────────────────────┘
```

---

## Processes

### 1. motor_daemon (C++, 200 Hz)

`motor_daemon.exe --port COM4 --zmq-port 5555`

- Owns the serial port exclusively; Python never talks to Feetech directly during normal operation.
- Runs a PID control loop at 200 Hz: reads present positions, steps toward goal positions.
- Exposes a ZMQ REQ/REP socket (msgpack encoded):

| cmd  | name         | payload                      | response                                             |
|------|--------------|------------------------------|------------------------------------------------------|
| 0xFF | STATUS       | —                            | `{status, loop_hz, trajectory_active, current_ticks, target_ticks}` |
| 0x01 | WRITE_TICKS  | `{ticks: [6× int]}`          | `{status: 0}`                                        |
| 0x02 | READ_TICKS   | —                            | `{ticks: [6× int]}`                                  |
| 0x03 | GRIPPER_LOAD | —                            | `{status, load: int16, current: int16, detected: bool}` |

> `detected` uses a hardcoded C++ threshold (150). Python reads `current` (Present_Current register 69) for stability-based grip detection. `load` is kept as a fallback if `current` is unavailable.

### 2. Python App V2 (`robot_sam2_app_v2`)

Single-process event loop at ~25 fps driven by `cv2.waitKey(5)`.

Opens two windows:
- **Robot Brain** — main camera (index 1), full approach pipeline
- **Base Camera** — overview camera (index 0), SAM2 tracks object, drives base motor left/right to center it before approach

Both share one SAM2Segmenter instance (each `segment_bbox` call is stateless).

---

## Module Map

```
app.py              RobotApp — main loop, key handling, state machine coordination
config.py           All tunable constants (thresholds, gains, positions, ports)
state.py            RobotState dataclass — single mutable shared object
control.py          MotionController — IBVS math, vision-to-joint targets
hardware.py         FeetechHardware / DaemonHardware / make_hardware()
tracking.py         ObjectTracker — SAM2 init, OpenCV CSRT frame tracking
vl53_sensor.py      VL53Sensor — threaded serial reader, stability check
go_home_util.py     go_home() — blocking 5 s interpolated move with easing
simulation.py       PyBulletArmSim — 3D visual sim, slider jog
data_logger.py      CSV logger (motor ticks, tracking, VL53 each frame)
vision/
  sam2_segmenter.py     SAM2.1 segmentation → bounding box
  vqa_detector.py       Florence-2 VQA → bbox from natural-language description
  rfdetr_selector.py    (legacy, not used — kept for reference)
  depth_perception.py   Intel RealSense D4xx aligned RGB+depth frames
  grasp_planner.py      3D grasp pose from depth mask
  scene_3d.py           Camera→base coordinate transform (hand-eye calib)
```

---

## Base Camera Pipeline

```
Base cam (index 0) frame
    │
    ▼
base_tracker.process()  ← SAM2 init on click, CSRT per frame
    │  → base_tracking: center_x, success
    ▼
_update_base_camera()
    │  err_x = (frame_w/2 − center_x) / frame_w
    │  if |err_x| > BASE_CAM_DEADBAND_X (0.08):
    │      state.target["base"] += err_x × BASE_CAM_K_BASE (140)
    ▼
"Base Camera" window with overlay:
    │  grey vertical line = frame center
    │  yellow vertical line = object center
    │  err % printed top-left
```

**B** key toggles base cam motor control. Active only when:
- `base_cam_active = True`
- `motors_enabled = True`
- `approach_mode = False` (IBVS takes over when approach starts — pressing A auto-disables B)

**GPU pause during approach**: when `approach_mode` or `retreat_mode` is True, `base_tracker.process()` is skipped entirely — no SAM2 re-init, no CSRT update. The Base Camera window still shows the live feed but the tracking overlay freezes. Resumes automatically when approach ends.

---

## Control Loop (per frame)

```
cap.read() / realsense.read_aligned()
    │
    ├─ _update_vision()
    │      ├─ _handle_grip_state()  ← state machine: gripping / retry / retreat
    │      ├─ tracker.process()     ← CSRT update, draw bbox
    │      └─ controller.update_from_object()  ← IBVS → state.target[]
    │
    ├─ _update_sim()               ← PyBullet visual sync
    │
    ├─ _check_vl53()               ← arm_locked / pre_grasp_palm / grip trigger
    │
    ├─ _update_retreat() OR _update_motion()
    │      └─ _step_proportional() ← move curr toward target by SPEED_LIMIT ticks/frame
    │                                 write_ticks() → ZMQ → daemon → motors
    │
    └─ draw_overlay() + cv2.imshow()
```

---

## Vision Pipeline

```
[Option A] User click on "Robot Brain" window
    │
    ├─[Option B] Press T → type description → Florence-2 VQA
    │                (REFERRING_EXPRESSION_COMPREHENSION)
    │                → (x0, y0, x1, y1) bbox
    │
    ▼
tracker.request_click(x,y)  OR  tracker.request_bbox(xyxy)
    │
    ▼
SAM2Segmenter.segment_bbox()   (on first eligible frame, GPU)
    │  → refines bbox using SAM2.1 hiera-tiny, returns clean mask bbox
    ▼
cv2.TrackerCSRT                 (every frame, CPU)
    │  → tracks bbox, returns center_x/y, area, width/height
    ▼
TrackingResult
    │
    ▼
MotionController.update_from_object()
    │  IBVS:
    │    err_x  = (frame_w × aim_x - center_x) / frame_w  → d_base
    │    err_y  = (frame_h × aim_y - center_y) / frame_h  → d_shoulder
    │    err_area = (vl53_dist - grip_dist) / max_dist     → d_elbow  (VL53 preferred)
    │           or  (threshold - pixel_area) / threshold   → d_elbow  (fallback)
    ▼
state.target[] updated
```

### Florence-2 VQA detector

| Property | Value |
|----------|-------|
| Model | `microsoft/Florence-2-base` (configurable via `VQA_MODEL`) |
| VRAM | ~0.8 GB (float16), lazy-loaded on first T press |
| Task | `REFERRING_EXPRESSION_COMPREHENSION` |
| Input | frame + natural-language description ("the red cup on the left") |
| Output | single `(x0, y0, x1, y1)` bbox, handed to `tracker.request_bbox()` |
| Speed | ~0.2–0.5 s per query (one-shot, not per-frame) |

Press **U** to re-run with the last query. Swap `VQA_MODEL` to `microsoft/Florence-2-large` in config.py for better accuracy at ~1.5 GB VRAM cost.

---

## Approach State Machine

```
HAND mode (default)
    │  press M
    ▼
OBJECT mode — click on object → SAM2 initializes CSRT tracker
    │  press A
    ▼
APPROACH mode — IBVS active, arm drives toward object
    │
    │  VL53 < VL53_LOCK_DIST_MM (130 mm)
    ▼
arm_locked = True — camera ignored, VL53 drives elbow + shoulder
    │
    │  VL53 stable ≤ VL53_GRIP_DIST_MM (110 mm) for 3 consecutive readings
    ▼
pre_grasp_palm — palm moves VL53_PREGRASP_PALM_DELTA ticks, everything else frozen
    │  palm arrived
    ▼
GRIPPING — gripper closes, count frames
    │
    ├─ current stable (spread ≤ CURRENT_STABLE_WINDOW=20, last 5 readings)
    │   AND min(last 5) > CURRENT_GRIP_THRESHOLD=50 after GRIP_LOAD_MIN_FRAMES (10 frames)
    │   → OBJECT CAUGHT → go home (arm returns with gripper closed)
    │
    └─ no grip detected after GRIP_CHECK_FRAMES (25 frames) → MISS
           │
           ├─ grip_attempt < MAX_GRIP_RETRIES (3):
           │     open gripper → retreat to pre_approach_ticks → re-approach → re-grip
           │     (up to 4 total attempts: 1 initial + 3 retries)
           │
           └─ grip_attempt >= MAX_GRIP_RETRIES:
                 → give up → go_home()
```

---

## Gripper Catch Detection

Grip detection uses `Present_Current` (Feetech register 69, 2-byte signed int16) read by the daemon at 200 Hz and exposed via cmd 0x03.

`DaemonHardware.gripper_load_detected()` maintains a rolling deque of the last 10 current readings.
Returns True when **all** of these hold for the last `CURRENT_STABLE_COUNT` (5) readings:
1. Spread (max − min) ≤ `CURRENT_STABLE_WINDOW` (20) — stable, not still closing
2. min of last 5 > `CURRENT_GRIP_THRESHOLD` (50) — above idle/no-grip baseline

Typical current values: idle = 0–6, approaching but empty = 12–20, gripping object = 87–102.

The buffer is reset whenever the gripper opens (start of each retreat / retry) to prevent contamination from the previous grip attempt.

Fallback: if `current` is unavailable, raw `load > GRIP_LOAD_THRESHOLD (100)` is used instead.

---

## CSV Data Logger

`DataLogger` writes one row every 0.5 s to `logs/robot_log_TIMESTAMP.csv`.
Columns: `timestamp`, `time_ms`, 6× `cur_*`, 6× `tgt_*`, tracking fields, vision fields, `vl53_dist_mm`, `gripper_load`, `gripper_current`, all state flags, `grip_attempt`, `gripper_closed_frames`, `free_mode`.

Started automatically at `setup()`. Toggle with **L** key.

---

## VL53 Stability Gate

`VL53Sensor` runs a background thread reading serial lines like `"Distance: 243 mm"`.
Keeps a rolling deque of the last 10 readings.

`is_stable_and_close(threshold, stable_window, max_jump)` returns True when:
1. Last 3 readings span ≤ `VL53_STABLE_WINDOW_MM` (15 mm) — stable
2. No consecutive pair in the buffer differs by > `VL53_MAX_JUMP_MM` (30 mm) — no occlusion spike
3. Average of last 3 ≤ `VL53_GRIP_DIST_MM` (110 mm) — close enough

---

## Motor Convention

| Motor | Name     | ID | Range (ticks) | Direction constant |
|-------|----------|----|---------------|--------------------|
| 1     | base     | 1  | 1000–3000     | —                  |
| 2     | shoulder | 2  | 1000–3000     | `SHOULDER_DIR = -1` |
| 3     | elbow    | 3  | 400–3000      | `ELBOW_DIR = -1`   |
| 4     | palm     | 4  | 1000–3500     | `PALM_DIR = -1`    |
| 5     | wrist    | 5  | —             | —                  |
| 6     | gripper  | 6  | 1500–3000     | 3000=open, 1500=close |

All motors are Feetech STS3215, ticks 0–4096 (center = 2048).
SHOULDER_DIR / ELBOW_DIR / PALM_DIR flip the error sign in IBVS so the arm moves toward the target.

---

## Key Files (not in v2 app)

| File | Purpose |
|------|---------|
| `motor_daemon/src/motor_daemon.cpp` | C++ daemon source |
| `motor_daemon.exe` | Built daemon binary (run from project root) |
| `StartHelloPos_handoff.json` | Home position (motor_id → ticks) loaded at startup |
| `joint_sim_calibration.json` | Tick↔angle calibration for PyBullet sim |
| `lerobot/tests/go_home.py` | End-of-session reset (direct Feetech, bypasses daemon) |
| `start_robot_v2.bat` | Launch script |
| `rf-detr-nano.pth` | RF-DETR model weights |
| `E:/sam2.1_hiera_tiny.pt` | SAM2 checkpoint (on E: drive) |

---

## Python Environment

```
E:\MiniForge\envs\lerobot\python.exe
```

Key packages: `torch`, `sam2`, `rfdetr`, `opencv-python`, `mediapipe`, `pyzmq`, `msgpack`, `pyserial`, `pybullet`
