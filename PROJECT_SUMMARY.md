# Project Summary — Autonomous Vision-Guided Robotic Arm

## What This Is

A 6-DOF robotic arm system running on a Windows PC with an NVIDIA RTX 2060 SUPER. The arm uses Feetech STS3215 smart servos (6 joints: base, shoulder, elbow, palm, wrist, gripper) connected via USB-UART on COM4 at 1 Mbit/s. The software stack is Python + C++, built by the owner from scratch on top of LeRobot's motor bus driver.

GitHub: https://github.com/NivDotan/Autonomous-Vision-Guided-Robotic-Arm

---

## Hardware

- **Arm**: 6-axis, Feetech STS3215 servos, IDs 1–6, 4096 ticks/rev
- **PC**: Windows 10, NVIDIA RTX 2060 SUPER, Python 3.10 (MiniForge at E:\MiniForge\envs\lerobot)
- **Camera**: USB webcam (OpenCV index 0)
- **No RealSense yet** — code for 3D grasping is written but hardware not yet purchased

---

## Software Architecture

```
robot_sam2_app/          ← main Python app
kinematics/              ← C++ FK/IK/Jacobian/trajectory (built → pykinematics.pyd)
motor_daemon/            ← C++ 200Hz control daemon (built → motor_daemon.exe)
tracking_cpp/            ← C++ CSRT tracker (code only, not yet built)
dashboard/               ← FastAPI + Three.js web UI (not yet running)
ros2_ws/                 ← ROS2 package (code only, ROS2 not installed)
benchmarks/              ← latency, tracking accuracy, pick-place tests
```

### How to Run

**Terminal 1 (motor daemon):**
```cmd
cd C:\Users\niv\robot_project
motor_daemon.exe --port COM4 --zmq-port 5555
```

**Terminal 2 (app):**
```cmd
cd C:\Users\niv\robot_project\robot_sam2_app
E:\MiniForge\envs\lerobot\python.exe -m robot_sam2_app.main
```

**Key config file**: `robot_sam2_app/robot_sam2_app/config.py`
- `USE_MOTOR_DAEMON = True` — routes motor commands through C++ daemon
- `SAM2_CHECKPOINT = r"E:/sam2.1_hiera_tiny.pt"` — SAM2 weights location
- `PORT = "COM4"` — serial port
- `REALSENSE_ENABLED = True` (but no hardware yet, gracefully disabled)

---

## What Is Actually Running Right Now

| Feature | Status |
|---------|--------|
| SAM2 segmentation + CSRT object tracking | ✅ Running |
| RF-DETR auto object detection | ✅ Running |
| PyBullet simulation window | ✅ Running |
| C++ motor daemon (200Hz, ZeroMQ) | ✅ Running — motors move |
| C++ kinematics (`pykinematics.pyd`) | ✅ Built and importable |
| Python trajectory planner | ✅ Running (uses pykinematics) |
| PyBullet collision pre-check | ✅ Running |
| go_home on Q key | ✅ Running — smoothly returns arm to home |
| MediaPipe hand tracking | ⚠️ Broken — protobuf conflict in conda env |
| 3D grasping (RealSense) | ❌ No hardware |
| Web dashboard | ❌ Not started |
| ROS2 | ❌ Not installed |
| C++ CSRT tracker | ❌ Not built |

---

## Key Files

| File | Role |
|------|------|
| `robot_sam2_app/robot_sam2_app/app.py` | Main orchestration loop, all key handlers |
| `robot_sam2_app/robot_sam2_app/config.py` | All tunable constants and feature flags |
| `robot_sam2_app/robot_sam2_app/hardware.py` | `FeetechHardware` + `DaemonHardware` (ZeroMQ) + `make_hardware()` |
| `robot_sam2_app/robot_sam2_app/state.py` | `RobotState` dataclass — all runtime state |
| `robot_sam2_app/robot_sam2_app/go_home_util.py` | Linear interpolation return-to-home |
| `robot_sam2_app/robot_sam2_app/trajectory/planner.py` | Joint-space, Cartesian, grasp trajectories |
| `robot_sam2_app/robot_sam2_app/vision/grasp_planner.py` | RealSense mask → PCA → 6-DOF grasp pose |
| `robot_sam2_app/robot_sam2_app/filters/kalman_tracker.py` | 2D EKF tracking filter (written, not wired in) |
| `motor_daemon/src/motor_daemon.cpp` | 200Hz C++ loop, ZeroMQ REP server |
| `motor_daemon/src/serial_comm.cpp` | Feetech STS3215 serial protocol (individual reads + SYNC_WRITE) |
| `kinematics/src/dh_kinematics.cpp` | Modified DH FK chain (7 transforms) |
| `kinematics/src/ik_solver.cpp` | Analytical IK + DLS fallback |

---

## DH Parameters (Modified Craig Convention)

| Joint | Name | a (m) | α (rad) | d (m) | tick_offset |
|-------|------|--------|---------|-------|------------|
| 1 | base | 0 | 0 | 0.08 | 2365 |
| 2 | shoulder | 0 | π/2 | 0.48 | 1740 |
| 3 | elbow | 0.3206 | 0 | 0 | 1410 |
| 4 | palm | 0.2613 | 0 | 0 | 3000 |
| 5 | wrist | 0.20 | π/2 | 0 | 3200 |
| 6 | gripper | 0.09 | -π/2 | 0 | 3000 |
| EE | tip | 0.17 | 0 | 0 | fixed |

Tick→rad: `q = (tick - offset) * (2π / 4096)`

---

## Motor Daemon Protocol

ZeroMQ REQ/REP, msgpack-encoded dicts, `raw=False` decoding on Python side.

| cmd | Name | Payload |
|-----|------|---------|
| 0x01 | WRITE_TICKS | `{"cmd":1, "ticks":[t0..t5]}` |
| 0x02 | READ_TICKS | `{"cmd":2}` → `{"status":0, "ticks":[...]}` |
| 0x03 | GRIPPER_LOAD | `{"cmd":3}` → `{"status":0, "detected":bool}` |
| 0x04 | SET_PID | `{"cmd":4, "joint":i, "kp":f, "ki":f, "kd":f, "i_max":f}` |
| 0x05 | SET_TRAJECTORY | `{"cmd":5, "waypoints":[{"t":ms,"ticks":[...]},...]}` |
| 0xFF | STATUS | `{"cmd":255}` → `{"status":0, "loop_hz":f, ...}` |

---

## App Controls

| Key | Action |
|-----|--------|
| S | Enable/disable motors |
| M | Hand ↔ Object tracking mode |
| Click | Select object to track |
| A | Toggle approach mode |
| Space | Open/close gripper |
| U | RF-DETR auto-detect cup |
| T | RF-DETR typed target |
| G | 3D grasp (needs RealSense) |
| J | PyBullet sim jog |
| R | Sync sim sliders |
| Z/X | Manual palm tilt |
| C | Auto palm |
| Q | Return to home + quit |

---

## Known Issues / Pending

- MediaPipe hand tracking crashes due to protobuf version conflict (`pip install protobuf==3.20.3` partially fixes it but may break other things)
- `USE_MOTOR_DAEMON = False` (lerobot FeetechHardware directly) currently does not move motors — only daemon mode works
- Kalman/UKF filters are written but not wired into the tracking pipeline
- `pykinematics` built but trajectory planner needs verification that it's actually calling C++ vs Python fallback
- RealSense, ROS2, web dashboard all written but not activated

---

## Build Commands (Windows, VS2022, MiniForge Python 3.10)

```cmd
cd C:\Users\niv\robot_project
E:\MiniForge\envs\lerobot\Scripts\cmake.exe -B build -G "Visual Studio 17 2022" -A x64 ^
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 ^
  -DPYTHON_EXECUTABLE=E:/MiniForge/envs/lerobot/python.exe ^
  -DPYTHON_INCLUDE_DIR=E:/MiniForge/envs/lerobot/include ^
  -DPYTHON_LIBRARY=E:/MiniForge/envs/lerobot/libs/python310.lib

E:\MiniForge\envs\lerobot\Scripts\cmake.exe --build build --config Release --target pykinematics
E:\MiniForge\envs\lerobot\Scripts\cmake.exe --build build --config Release --target motor_daemon
copy build\_deps\zeromq-build\bin\Release\libzmq-v143-mt-4_3_5.dll .
```
