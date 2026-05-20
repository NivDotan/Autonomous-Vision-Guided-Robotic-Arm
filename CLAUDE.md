# CLAUDE.md — Robot Project

## Rules for Claude (read every session)

- **Always read `ARCHITECTURE.md`** at the start of any session that touches control flow, hardware, the state machine, vision pipeline, or config constants. It is the authoritative system reference.
- **Update both `CLAUDE.md` and `ARCHITECTURE.md`** at the end of any session where we added a feature, changed config constants, modified the state machine, or changed how hardware is used. Do not wait for the user to ask — do it as part of finishing the work.
- The active codebase is `robot_sam2_app_v2/`. Never edit `robot_sam2_app/` (v1, legacy).

---

## How to run

```bat
start_robot_v2.bat
```

This launches two windows:
1. **Motor Daemon** — `motor_daemon.exe --port COM4 --zmq-port 5555` (C++ 200 Hz control loop)
2. **Python App V2** — `E:\MiniForge\envs\lerobot\python.exe -m robot_sam2_app.main` from `robot_sam2_app_v2\`

Wait ~2 s for the daemon to bind before the Python app starts (bat handles this).

### End-of-session reset

```bat
python -u "c:\Users\niv\robot_project\lerobot\tests\go_home.py"
```

This moves only motors 2, 3, 4 (shoulder, elbow, palm) to their home positions using direct Feetech access — bypasses the daemon entirely.

---

## Active codebase: robot_sam2_app_v2

**Never edit `robot_sam2_app/`** — that is v1 (legacy). All work goes into:

```
robot_sam2_app_v2/robot_sam2_app/
    app.py          ← main orchestration loop
    config.py       ← ALL tunable constants (change here, not in logic)
    state.py        ← RobotState dataclass (single source of truth)
    control.py      ← MotionController (vision-to-joint math)
    hardware.py     ← FeetechHardware + DaemonHardware + make_hardware()
    tracking.py     ← ObjectTracker (SAM2 init + CSRT frame tracking)
    vl53_sensor.py  ← VL53Sensor (threaded serial reader + stability check)
    go_home_util.py ← go_home() blocking interpolated move
    simulation.py   ← PyBullet sim (visual only)
    data_logger.py  ← CSV logger
    vision/
        sam2_segmenter.py    ← SAM2 segmentation wrapper
        rfdetr_selector.py   ← RF-DETR object detection (auto-aim)
        depth_perception.py  ← RealSense D4xx wrapper
        grasp_planner.py     ← 3D grasp pose planning
        scene_3d.py          ← camera→base coordinate transform
```

---

## Hardware

| Device | Port | Notes |
|--------|------|-------|
| SO-101 arm | COM4 | 6× Feetech STS3215, motor IDs 1–6 |
| ESP32 + VL53L1X | COM3 | Distance sensor, sends "Distance: NNN mm" lines at 115200 baud |
| Webcam | index 1 | Change `CAMERA_INDEX` in config.py if needed |
| RealSense | USB | Optional; set `REALSENSE_ENABLED = False` to skip |

Motor layout: base=1, shoulder=2, elbow=3, palm=4, wrist=5, gripper=6

**`USE_MOTOR_DAEMON = True`** — always on. Python talks to the C++ daemon over ZMQ (localhost:5555), not directly to Feetech.

---

## Key config constants (config.py)

```python
# Grip detection — current-based (primary)
CURRENT_GRIP_THRESHOLD  = 50   # min motor current to consider gripper loaded (idle=0-6, grip=87-102)
CURRENT_STABLE_WINDOW   = 20   # max spread (max-min) across last N current readings
CURRENT_STABLE_COUNT    = 5    # N consecutive readings that must all be stable + above threshold

# Grip detection — load-based (fallback if current unavailable)
GRIP_LOAD_THRESHOLD     = 100  # raw load units; daemon returns int16 load field

# VL53 gating
VL53_LOCK_DIST_MM        = 130  # below this: arm_locked=True, VL53 drives joints only
VL53_GRIP_DIST_MM        = 110  # stable close trigger for pre-grasp + grip
VL53_PREGRASP_PALM_DELTA = -50  # palm ticks before gripper closes

# Grip timing
GRIP_CHECK_FRAMES    = 25   # frames after close before declaring miss (~1 sec at 25fps)
GRIP_LOAD_MIN_FRAMES = 10   # frames to skip load check (avoid motor-torque spike)

# Retry
MAX_GRIP_RETRIES     = 3    # full retreat+reapproach attempts before giving up (4 total tries)
RETREAT_TOLERANCE    = 30   # ticks — close enough to pre-approach to resume approach

APPROACH_AIM_X/Y            # where in the 4×4 grid the arm aims during approach
```

---

## Architecture decisions

- **DaemonHardware, not FeetechHardware** — The C++ daemon owns the serial port at 200 Hz. Python sends goal ticks via ZMQ; daemon applies PID and writes to motors. cmd 0x03 returns `{"load": int16, "current": int16, "detected": bool}`. Grip detection uses `current` (stable window + threshold), not `detected` (hardcoded C++ threshold). `load` is fallback only.

- **State machine lives in RobotState** — never duplicate flags elsewhere. Check `state.arm_locked`, `state.pre_grasp_palm`, `state.retreat_mode` etc. before adding logic.

- **IBVS in control.py** — err_x→base, err_y→shoulder, err_area→elbow. `centering_factor` gates elbow in normal mode but is bypassed in `arm_locked` mode (VL53 drives directly). Do not add a centering gate in the locked path.

- **Proportional stepper** — `_step_proportional()` moves each joint `SPEED_LIMIT=25` ticks/frame toward target. Direct hardware writes (jog, go_home) must call `state.set_curr_and_target()` after completing, otherwise the stepper overwrites the hardware position on the next frame.

- **Pre-grasp palm is sequential** — palm moves first (`pre_grasp_palm=True`), gripper closes only after palm arrives. If palm is already at its limit, skip straight to grip.

- **Grip retry — retreat and reapproach** — miss → open gripper → reset current buffer → `retreat_mode=True` → step back to `pre_approach_ticks` → when arrived: `approach_mode=True` → re-approach from same start. Up to `MAX_GRIP_RETRIES=3` retreats (4 total attempts). After all retries: `_go_home()`. No local micro-adjustments (shoulder/elbow nudges) — those were removed.

- **Arrow key jog works before S** — `_jog_direct()` writes directly to hardware and syncs `state.curr + state.target` so the proportional stepper doesn't override it next frame. Requires daemon to be connected.

- **Current buffer contamination** — `reset_gripper_current_buffer()` must be called whenever the gripper opens (start of each retreat). Otherwise old high-current readings from a successful grip carry into the next attempt and cause false detection on attempt 2+.

---

## Controls (runtime)

| Key | Action |
|-----|--------|
| S | Enable/disable motors |
| M | Toggle HAND / OBJECT mode |
| A | Toggle approach mode (must have object tracked) — also disables base cam motor control |
| Space | Manual gripper open/close |
| R | Reset to home (smooth 5 s move) |
| Arrow keys | Jog base (←→) / shoulder (↑↓) — works before S |
| B | Toggle base camera motor control (drives base motor from base camera tracking) |
| F | Free-arm mode (low resistance, motors off) |
| Z / X | Palm up / down (manual) |
| C | Auto-palm on |
| U | Auto-detect cup with RF-DETR |
| T | Type target class for RF-DETR |
| J | Toggle PyBullet sim jog |
| L | Start/stop data logging |
| Q | Go home and quit |

---

## Base camera workflow

1. App opens two windows: **Robot Brain** (main camera, index 1) and **Base Camera** (index 0).
2. Click on the object in the **Base Camera** window → SAM2 initializes tracking there.
3. Press **B** → base motor follows the object horizontally to center it in frame.
4. Once the object is visible in the main camera, press **M** then click in **Robot Brain** to track, then **A** to approach.
   - Pressing **A** automatically disables **B** so IBVS takes over the base motor.
5. `BASE_CAM_K_BASE = 140` controls how fast the base rotates; `BASE_CAM_DEADBAND_X = 0.08` is the dead zone.
6. Set `BASE_CAM_ENABLED = False` in config.py to skip the second camera entirely.
7. During `approach_mode` or `retreat_mode`, base camera SAM2/CSRT is paused automatically (GPU freed for main camera). Window still shows live feed with frozen overlay.

---

## Common pitfalls

- **Wrong bat file** — `start_robot.bat` runs v1. Always use `start_robot_v2.bat`.
- **Daemon not running** — Python app will print "DaemonHardware unavailable" and continue in no-hardware mode. Start the daemon first.
- **go_home.py bypasses daemon** — it connects directly to COM4 via Feetech, so the daemon must be stopped first (or just use it after the app exits).
- **Arrow keys need raw key value** — OpenCV on Windows masks arrow keys to 0 with `& 0xFF`. The app captures `raw_key = cv2.waitKey(5)` and checks Windows-specific values (2424832, 2555904, 2490368, 2621440).
