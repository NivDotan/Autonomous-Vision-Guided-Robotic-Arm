# Autonomous Vision-Guided Robotic Arm

A 6-DOF robotic arm system with real-time vision tracking, C++ kinematics, trajectory planning, and autonomous 3D grasping. Built on top of [LeRobot](https://github.com/huggingface/lerobot) hardware drivers with a fully custom software stack.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        robot_sam2_app                           │
│                                                                 │
│  Camera ──► SAM2 Segmentation ──► CSRT Tracker                 │
│         ──► RF-DETR Detection ──► Auto Target                  │
│         ──► MediaPipe Hands  ──► Hand Teleoperation            │
│                                                                 │
│  Tracker ──► MotionController ──► TrajectoryPlanner            │
│                                    │                            │
│                           pykinematics (C++)                    │
│                           FK / IK / Jacobian                    │
│                                    │                            │
│                           CollisionChecker (PyBullet)           │
│                                    │                            │
│                              DaemonHardware                     │
└──────────────────────────────────────┬──────────────────────────┘
                                       │ ZeroMQ + MessagePack
                             ┌─────────▼──────────┐
                             │   motor_daemon.exe  │
                             │   200Hz C++ loop    │
                             │   PID per joint     │
                             └─────────┬───────────┘
                                       │ RS-485 @ 1Mbit/s
                              Feetech STS3215 × 6
```

---

## Features

### C++ Kinematics Library (`kinematics/`)
- **Modified DH forward kinematics** — 7-transform chain including fixed EE offset
- **Analytical inverse kinematics** — spherical wrist decomposition, sub-millisecond solve
- **Damped Least Squares IK fallback** — numerical solver for singularities
- **Geometric Jacobian** — 6×6 matrix for velocity kinematics
- **Trajectory generation** — trapezoidal velocity profiles, natural cubic splines, Cartesian-linear interpolation
- **Python bindings** via pybind11 — `import pykinematics`

### C++ Motor Daemon (`motor_daemon/`)
- **200Hz real-time control loop** in a dedicated C++ thread
- **Per-joint PID** with anti-windup and EMA derivative filter
- **ZeroMQ REQ/REP + MessagePack** IPC — drop-in for Python hardware driver
- **Direct Feetech STS3215 serial protocol** — no Python GIL overhead
- **Trajectory execution** — accepts waypoint lists and interpolates independently

### Vision Pipeline (`robot_sam2_app/`)
- **SAM2** segmentation on click → **CSRT** bounding-box tracking at camera framerate
- **RF-DETR** object detection — press `U` for cup, `T` for any class
- **MediaPipe Hands** — hand-gesture teleoperation mode
- **3D grasp planning** (RealSense D435) — SAM2 mask + depth → point cloud → PCA → 6-DOF grasp pose
- **Kalman filter** (2D EKF) and **UKF** (3D) for state estimation

### Trajectory & Motion
- **Python TrajectoryPlanner** — joint-space, Cartesian, and 3-phase grasp trajectories
- **PyBullet collision pre-check** — validates trajectory before execution
- **Smooth return-to-home** on quit — linear interpolation from actual motor positions

### Additional Systems
- **Web dashboard** (`dashboard/`) — FastAPI + WebSocket + Three.js real-time arm visualization
- **ROS2 package** (`ros2_ws/`) — vision, kinematics, hardware, sim nodes; `MoveToTarget` action server
- **Benchmarks** (`benchmarks/`) — latency profiler, tracking accuracy vs ArUco, pick-and-place success rate
- **Grasp quality** — epsilon metric (wrench space), auto dataset recording

---

## Hardware

| Component | Detail |
|-----------|--------|
| Arm | 6-axis manipulator, Feetech STS3215 servos (IDs 1–6) |
| Communication | USB-UART → COM4, half-duplex RS-485 @ 1 Mbit/s |
| Camera | USB webcam (OpenCV index 0) |
| Depth camera | Intel RealSense D435i (optional, for 3D grasping) |
| GPU | NVIDIA GPU recommended for SAM2 (`cuda` auto-detected) |

---

## Quick Start

### Prerequisites
```bash
pip install pybullet msgpack pyzmq
pip install git+https://github.com/facebookresearch/sam2.git
pip install rfdetr mediapipe
```

### Build C++ libraries (Windows, VS2022)
```cmd
cmake -B build -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=Release ^
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 ^
  -DPYTHON_EXECUTABLE=<path_to_python.exe>
cmake --build build --config Release --target pykinematics
cmake --build build --config Release --target motor_daemon
```

### Run (two terminals)

**Terminal 1 — motor daemon:**
```cmd
motor_daemon.exe --port COM4 --zmq-port 5555
```

**Terminal 2 — app:**
```cmd
cd robot_sam2_app
python -m robot_sam2_app.main
```

### Controls

| Key | Action |
|-----|--------|
| `S` | Enable / disable motors |
| `M` | Switch Hand ↔ Object tracking mode |
| Click | Select object to track (Object mode) |
| `A` | Toggle approach — arm moves toward object |
| `Space` | Open / close gripper |
| `U` | RF-DETR auto-detect cup |
| `T` | RF-DETR typed target class |
| `G` | Execute 3D grasp (requires RealSense) |
| `J` | PyBullet sim jog mode |
| `R` | Sync sim sliders to current position |
| `Z` / `X` | Manual palm tilt |
| `C` | Auto palm tracking |
| `Q` | Return to home and quit |

### Configuration (`robot_sam2_app/robot_sam2_app/config.py`)

```python
USE_MOTOR_DAEMON  = True   # route through C++ 200Hz daemon
REALSENSE_ENABLED = True   # enable RealSense D435 depth
MOCK_REALSENSE    = False  # use synthetic depth for testing
DASHBOARD_ENABLED = False  # start FastAPI web dashboard
SAM2_CHECKPOINT   = "path/to/sam2.1_hiera_tiny.pt"
PORT              = "COM4"
```

---

## Project Structure

```
robot_project/
├── CMakeLists.txt                    # Super-build
├── kinematics/                       # C++ FK/IK/Jacobian/trajectory
│   ├── include/kinematics.hpp
│   ├── src/
│   ├── bindings/py_kinematics.cpp    # pybind11 → pykinematics
│   └── tests/test_kinematics.cpp
├── motor_daemon/                     # C++ 200Hz control daemon
│   ├── include/
│   └── src/
├── tracking_cpp/                     # C++ CSRT + optical flow
├── robot_sam2_app/
│   └── robot_sam2_app/
│       ├── app.py                    # Main orchestration
│       ├── config.py
│       ├── hardware.py               # FeetechHardware + DaemonHardware
│       ├── trajectory/               # Planner, spline, collision check
│       ├── vision/                   # SAM2, RF-DETR, depth, grasp planner
│       ├── filters/                  # Kalman, UKF
│       └── grasp/                    # Quality metric, dataset recorder
├── dashboard/                        # FastAPI + Three.js web UI
├── ros2_ws/                          # ROS2 Humble package
├── benchmarks/                       # Latency, tracking accuracy, pick-place
└── PROJECT_GUIDE.md                  # Full setup and hardware guide
```

---

## CV Highlights

- Implemented analytical 6-DOF IK in C++ using Modified DH parameters — sub-millisecond via spherical wrist decomposition with DLS fallback
- C++ motor control daemon at 200Hz with per-joint PID (anti-windup, EMA derivative filter) over ZeroMQ/MessagePack, 10× lower latency than Python loop
- RealSense D435 + SAM2 segmentation → PCA on masked point cloud → 6-DOF grasp pose, enabling autonomous pick-and-place without hand-coded positions
- Time-synchronized trapezoidal trajectory planning with PyBullet collision pre-check before execution
- Full ROS2 system with MoveToTarget action server enabling autonomous grasping from natural-language object names via RF-DETR
