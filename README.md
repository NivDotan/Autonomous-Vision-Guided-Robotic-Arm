# Autonomous Vision-Guided Robotic Arm (SO-101)

An autonomous pick-and-place system for the 6-DOF **SO-101** arm. A camera sees the
scene, **SAM2** segments the target object, image-based visual servoing (IBVS)
drives the arm toward it, and a **VL53L1X** time-of-flight sensor gates the final
grasp. Natural-language targets ("the red cup on the left") are located with
**Grounding DINO**.

The project ships two cooperating layers:

- **`robot_sam2_app_v2/`** — the real-time perception + control application (the "brain").
- **`ros2/`** — a clean **ROS2 Jazzy** architecture (8 packages) that wraps the system
  as standard ROS2 nodes for visualization (RViz), simulation (Gazebo), and modular
  reuse.

Both run against a single hardware daemon, so RViz can mirror the live arm while the
application drives it.

---

## Highlights

- **Zero-shot target selection** — type an object description; Grounding DINO returns a
  bounding box, SAM2 refines it, and OpenCV CSRT tracks it at frame rate.
- **Image-based visual servoing** — per-joint error terms drive base/shoulder/elbow
  directly from image features; no hand-coded waypoints.
- **Sensor-gated grasping** — VL53L1X distance gating plus motor-current stability
  detection confirm a successful grip, with automatic retry on a miss.
- **ROS2 Jazzy stack** — `robot_state_publisher` + RViz show the official SO-101 URDF
  tracking the real arm in real time; Gazebo Harmonic world for simulation.
- **Cross-platform** — runs on Linux (Ubuntu 24.04, primary) and Windows.

---

## System architecture

```
                 ┌──────────────────────────────────────────────┐
   Base camera ─►│              robot_sam2_app_v2               │◄─ Gripper camera
   (wide view)   │                                              │   (approach / grasp)
                 │  Grounding DINO ─► SAM2 ─► CSRT tracker       │
                 │        │                                      │
                 │        ▼                                      │
                 │   MotionController (IBVS)  ◄── VL53L1X ToF     │
                 │        │                                      │
                 └────────┼──────────────────────────────────────┘
                          │ ZeroMQ + MessagePack
              ┌───────────▼─────────────┐        ┌──────────────────────────┐
              │  motor daemon            │  PUB   │  ros2/ (so101_driver)     │
              │  REQ/REP :5555           │───────►│  /joint_states ─► RViz    │
              │  PUB     :5556           │ :5556  │  TF, perception, planner  │
              └───────────┬─────────────┘        └──────────────────────────┘
                          │ RS-485 @ 1 Mbit/s
                   6× Feetech STS3215  (base, shoulder, elbow, palm, wrist, gripper)
```

The daemon owns the serial bus exclusively. The application sends goal positions over
REQ/REP; the ROS2 driver subscribes to the PUB broadcast for read-only state — so both
can run simultaneously without contention.

---

## Repository layout

```
ros2/                  ROS2 Jazzy stack — 8 packages (see ros2/README.md)
robot_sam2_app_v2/     vision + state-machine application
robot_system/          kinematics / perception / calibration library
motor_daemon_py.py     Linux motor daemon (ZeroMQ REQ/REP :5555 + PUB :5556)
motor_daemon/          C++ 200 Hz motor daemon (Windows)
kinematics/            C++ FK/IK/Jacobian with pybind11 bindings (optional acceleration)
tools/                 calibration and diagnostic helper scripts
docs/                  extended guides (PROJECT_GUIDE, PROJECT_SUMMARY)
ARCHITECTURE.md        deep-dive: control loop, state machine, vision pipeline
start_robot_linux.sh   one-command launcher (daemon + app + RViz)
```

### ROS2 packages (`ros2/`)

| Package | Role |
|---------|------|
| `so101_description` | URDF/xacro + RViz config |
| `so101_bringup` | launch files for every phase |
| `so101_interfaces` | custom messages and services |
| `so101_driver` | servo driver node (ZeroMQ daemon / direct pyserial), tick↔radian conversion, joint-name remap |
| `so101_perception` | base/gripper cameras, VL53 range, Grounding DINO, SAM2 + CSRT |
| `so101_calibration` | pixel → robot-frame homography service |
| `so101_task_planner` | pick-and-place state machine |
| `so101_gazebo` | Gazebo Harmonic world |

---

## Hardware

| Device | Linux | Windows | Notes |
|--------|-------|---------|-------|
| SO-101 arm | `/dev/ttyACM0` | `COM4` | 6× Feetech STS3215, IDs 1–6 |
| ESP32 + VL53L1X | `/dev/ttyUSB0` | `COM3` | emits `"Distance: NNN mm"` @ 115200 |
| Main camera | by v4l2 name | index 1 | gripper / approach camera |
| Base camera | by v4l2 name | index 0 | wide overview camera |
| GPU | — | — | NVIDIA GPU recommended for SAM2 / Grounding DINO |

Cameras are resolved by stable v4l2 product name (configurable in `config.py`), since
Linux USB indices change across reboots.

---

## Getting started (Linux — Ubuntu 24.04 + ROS2 Jazzy)

### 1. System dependencies

```bash
sudo apt install python3-zmq python3-msgpack v4l-utils \
                 ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2
```

### 2. Python dependencies

```bash
pip install -r requirements.txt --break-system-packages
pip install git+https://github.com/facebookresearch/sam2.git --break-system-packages
# SAM2 checkpoint:
wget -P ~/ https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_tiny.pt
```

Set `SAM2_CHECKPOINT` in `robot_sam2_app_v2/robot_sam2_app/config.py` to the downloaded path.

### 3. Build the ROS2 workspace

The official SO-101 description (URDF + meshes) comes from
[`legalaspro/so101-ros-physical-ai`](https://github.com/legalaspro/so101-ros-physical-ai),
cloned alongside this repo in the colcon workspace:

```bash
cd ~/so101_ws/src
for p in so101_description so101_bringup so101_driver so101_interfaces \
         so101_perception so101_calibration so101_task_planner so101_gazebo; do
  ln -sfn Autonomous-Vision-Guided-Robotic-Arm/ros2/$p $p
done
git clone https://github.com/legalaspro/so101-ros-physical-ai so101_physical_ai
touch so101_physical_ai/so101_bringup/COLCON_IGNORE   # use our bringup
cd ~/so101_ws && colcon build --symlink-install
```

### 4. Run

```bash
# One command — daemon + application + RViz:
./start_robot_linux.sh
```

Or manually in three terminals:

```bash
# 1) motor daemon
python3 motor_daemon_py.py --port /dev/ttyACM0 --zmq-port 5555 --pub-port 5556
# 2) vision + control application
cd robot_sam2_app_v2 && python3 -m robot_sam2_app.main
# 3) ROS2 driver + RViz (live arm visualization)
unset LD_LIBRARY_PATH && source /opt/ros/jazzy/setup.bash
cd ~/so101_ws && source install/setup.bash
ros2 launch so101_bringup hardware.launch.py dry_run:=false backend:=daemon
```

---

## Usage

In the application window:

| Key | Action |
|-----|--------|
| `T` | type a target description → locate → approach |
| `A` | toggle approach |
| `Space` | open / close gripper |
| `S` | enable / disable motors |
| `M` | HAND / OBJECT mode |
| `B` | base-camera motor control |
| Arrows | jog base (←→) / shoulder (↑↓) |
| `R` | reset to home |
| `Q` | return home and quit |

ROS2-only operation (autonomous task via services):

```bash
ros2 launch so101_bringup full_stack.launch.py dry_run:=false backend:=daemon
ros2 service call /start_task so101_interfaces/srv/StartTask \
    '{pick_query: "the red cup", place_query: "the open box"}'
```

---

## Configuration

Key constants live in `robot_sam2_app_v2/robot_sam2_app/config.py`:

```python
PORT              = "/dev/ttyACM0"   # arm serial port
MAIN_CAMERA_NAME  = "Arducam"        # gripper camera (matched by v4l2 name)
BASE_CAMERA_NAME  = "LifeCam"        # wide camera
SAM2_CHECKPOINT   = "~/sam2.1_hiera_tiny.pt"
USE_MOTOR_DAEMON  = True
REALSENSE_ENABLED = False
```

See [`ARCHITECTURE.md`](ARCHITECTURE.md) for the control loop, approach state machine,
grip detection, and vision pipeline in detail.

---

## Windows

```bat
start_robot_v2.bat
```

Launches `motor_daemon.exe --port COM4 --zmq-port 5555` then the application. The C++
daemon (`motor_daemon/`) and optional C++ kinematics (`kinematics/`) are built with
CMake; see `docs/PROJECT_GUIDE.md`.

---

## License

Released under the [MIT License](LICENSE).
