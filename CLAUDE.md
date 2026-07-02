# CLAUDE.md — SO-101 Robot Project

## Rules (read every session)

- **Always read `ARCHITECTURE.md`** at the start of any session that touches control flow, hardware, the state machine, vision pipeline, or config constants.
- **Update both `CLAUDE.md` and `ARCHITECTURE.md`** at the end of any session that added a feature, changed config constants, changed the state machine, or changed hardware usage.
- The active Python app is `robot_sam2_app_v2/`. The active ROS2 stack is `ros2/`.

## How to run — Linux (Ubuntu 24.04 + ROS2 Jazzy)

```bash
# All three at once:
./start_robot_linux.sh

# Or manually, in three terminals:
# 1) motor daemon (replaces motor_daemon.exe)
python3 motor_daemon_py.py --port /dev/ttyACM0 --zmq-port 5555 --pub-port 5556
# 2) vision + state-machine app
cd robot_sam2_app_v2 && python3 -m robot_sam2_app.main
# 3) ROS2 driver + RViz (live arm visualization)
unset LD_LIBRARY_PATH && source /opt/ros/jazzy/setup.bash
cd ~/so101_ws && source install/setup.bash
ros2 launch so101_bringup hardware.launch.py dry_run:=false backend:=daemon
```

## How to run — Windows

```bat
start_robot_v2.bat
```

## Hardware

| Device | Linux | Windows | Notes |
|--------|-------|---------|-------|
| SO-101 arm | `/dev/ttyACM0` | `COM4` | 6× Feetech STS3215, IDs 1–6 |
| ESP32 + VL53L1X | `/dev/ttyUSB0` | `COM3` | `"Distance: NNN mm"` @ 115200 |
| Main camera (gripper) | by v4l2 name `MAIN_CAMERA_NAME` (Arducam) | index `1` | resolved by `find_camera_device()` |
| Base camera (wide) | by v4l2 name `BASE_CAMERA_NAME` (LifeCam) | index `0` | resolved by `find_camera_device()` |

Motor layout: base=1, shoulder=2, elbow=3, palm=4, wrist=5, gripper=6.

On Linux, cameras are resolved by stable v4l2 product name (USB indices reshuffle
across reboots). Edit `MAIN_CAMERA_NAME` / `BASE_CAMERA_NAME` in `config.py` if you
swap cameras. VL53 port is auto-detected by `_find_vl53_port()`.

## Layout

```
ros2/                  ROS2 Jazzy stack (description, bringup, interfaces,
                       driver, perception, calibration, task_planner, gazebo)
robot_sam2_app_v2/     vision + state-machine app
robot_system/          kinematics / perception / calibration library
motor_daemon/          C++ 200 Hz daemon (Windows)
motor_daemon_py.py     Python daemon (Linux) — ZMQ REQ/REP :5555 + PUB :5556
kinematics/            C++ FK/IK/Jacobian (pykinematics)
tools/                 calibration & test helper scripts
docs/                  extended guides
```

## Architecture notes

- **motor_daemon_py.py** mirrors `motor_daemon.exe`: REQ/REP on :5555 (the app sends goal ticks / reads state / gripper load) plus a PUB broadcast on :5556. The ROS2 `so101_driver` subscribes to :5556 for read-only state, so RViz shows the live arm while the app drives it — no socket conflict.
- **ros2/ wraps, never rewrites** the app modules. `so101_driver` converts ticks↔radians and maps project joint names (`base, shoulder, elbow, palm, wrist, gripper`) to official SO-101 names (`shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper`).
- On Linux, lerobot is not installed; `so101_driver` uses `feetech_direct.py` (pure pyserial). The app uses `DaemonHardware` (ZMQ).
- **Daemon torque**: `motor_daemon_py.py` enables torque on connect, **reads it back to verify**, and **re-asserts on the first WRITE**. Without torque, goal writes are silently ignored. It also exits loudly if `:5555` is already bound (stale daemon).
- **Cameras**: opened via `cv2.CAP_V4L2` backend + **MJPG** + 640×480. Two USB cams on one controller exceed USB2 bandwidth in raw YUYV (`VIDIOC_QBUF: Bad file descriptor`); MJPG + the explicit V4L2 backend fixes it.

## Linux gotchas (hard-won — read before debugging "arm won't move")

1. **VL53 port scan must never touch the arm.** `_find_vl53_port()` scans **only `/dev/ttyUSB*`** and skips `PORT`. Opening the arm's `/dev/ttyACM0` (held by the daemon at 1 Mbaud) at 115200 reconfigures the shared tty's baud → the daemon's commands stop reaching the servos and **the arm silently freezes while RViz still follows the commanded ticks** (RViz mirrors commanded ticks, not encoder reads).
2. **"RViz reaches home but the real arm doesn't move"** ⇒ baud/port corruption (gotcha #1) or torque off — not a kinematics bug.
3. **Stale daemon**: the launcher `pkill`s old `motor_daemon_py.py` / `robot_sam2_app.main` first. A leftover daemon holds `:5555` and the app talks to the wrong (no-torque) one.
4. **ROS2 + snap**: always `unset LD_LIBRARY_PATH && source /opt/ros/jazzy/setup.bash` before `ros2 …`, or rviz2/jsp-gui crash with a snap `libpthread` symbol error.
5. **mediapipe ≥ 0.10.35** removed `mp.solutions` → HAND mode (M) is disabled on Linux; everything else works.
6. To confirm hardware independently of the daemon/app: `python3 tools/move_test.py` (raw serial, reads real encoders).

## Controls

| Key | Action |
|-----|--------|
| T | type a description → Grounding DINO finds it → approach |
| A | toggle approach | Space | gripper open/close |
| R | reset to home | S | enable/disable motors |
| M | HAND/OBJECT mode | B | base-camera motor control |
| Arrows | jog base (←→) / shoulder (↑↓) | Q | go home and quit |
