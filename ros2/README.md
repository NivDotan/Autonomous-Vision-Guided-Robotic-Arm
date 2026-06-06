# SO-101 ROS2 Jazzy stack

A clean ROS2 Jazzy (Ubuntu 24.04) architecture for the SO-101 arm, migrated
from the working Windows app `robot_sam2_app_v2`. Existing modules are **wrapped**
as ROS2 nodes rather than rewritten — the Python app remains the source of the
core logic until each phase is fully replaced.

The migration is staged in 7 phases (RViz → driver → cameras/sensors → AI
perception → calibration → task planner → Gazebo). **Only Phase 1 is built so
far.**

## Workspace layout

These packages live inside the existing repo and are discovered by `colcon`
when the workspace is built from `~/so101_ws`:

```
~/so101_ws/
└── src/
    └── Autonomous-Vision-Guided-Robotic-Arm/
        ├── robot_sam2_app_v2/     # existing working app (unchanged)
        ├── robot_system/          # existing kinematics/perception lib (unchanged)
        └── ros2/                  # ← ROS2 packages (this dir)
            ├── so101_description/ # URDF/xacro, meshes, RViz config        [Phase 1]
            ├── so101_bringup/     # launch files + shared config            [Phase 1]
            ├── so101_interfaces/  # custom msgs/srvs                        [Phase 4+]
            ├── so101_driver/      # servo driver node                       [Phase 2]
            ├── so101_perception/  # camera + sensor + AI nodes              [Phase 3/4]
            ├── so101_calibration/ # pixel -> robot service                  [Phase 5]
            ├── so101_task_planner/# pick/place state machine                [Phase 6]
            └── so101_gazebo/      # Gazebo demo                             [Phase 7]
```

## Prerequisites

- Ubuntu 24.04 + ROS2 Jazzy (`source /opt/ros/jazzy/setup.bash`)
- For Phase 1, install the runtime tools (the launch file needs all of these):
  ```bash
  sudo apt install ros-jazzy-robot-state-publisher \
                   ros-jazzy-joint-state-publisher-gui \
                   ros-jazzy-rviz2 ros-jazzy-xacro
  ```

## Workspace discovery (one-time)

The repo root has a `CMakeLists.txt` (the C++ super-build), so `colcon` treats
the whole `Autonomous-Vision-Guided-Robotic-Arm/` directory as a single plain
cmake package and **does not recurse** into `ros2/`. To expose these packages to
a normal `colcon build` from `~/so101_ws`, symlink them into `src/` once:

```bash
cd ~/so101_ws/src
ln -sfn Autonomous-Vision-Guided-Robotic-Arm/ros2/so101_description so101_description
ln -sfn Autonomous-Vision-Guided-Robotic-Arm/ros2/so101_bringup     so101_bringup
```

(The files stay in the repo and remain version-controlled; only the symlinks
live in `src/`.) Alternatively, skip the symlinks and build with an explicit
base path: `colcon build --base-paths src/Autonomous-Vision-Guided-Robotic-Arm/ros2`.

---

## Phase 1: RViz visualization

Visualize the SO-101 in RViz and drive its joints with sliders. No hardware.

### Build

```bash
cd ~/so101_ws
colcon build --packages-select so101_description so101_bringup
source install/setup.bash
```

### Run

```bash
ros2 launch so101_bringup display.launch.py
```

This starts `robot_state_publisher`, `joint_state_publisher_gui`, and `rviz2`
with the default config. You should see the arm rendered with `base_link` as the
fixed frame; moving the gui sliders moves each joint live.

Options:

```bash
# Hide the slider gui (publish nothing; joints stay at zero)
ros2 launch so101_bringup display.launch.py gui:=false

# Use the official SO-ARM101 model instead of the placeholder
# (requires fetching assets first — see so101_description/urdf/FETCH_OFFICIAL_URDF.md)
ros2 launch so101_bringup display.launch.py use_placeholder:=false
```

### What's in the model

The default model is a **minimal placeholder** (boxes/cylinders, no meshes) so
RViz works with zero downloads. Its 6 revolute joints are named to match the
project's `MOTOR_NAMES` — `base, shoulder, elbow, palm, wrist, gripper` — so a
future driver's `/joint_states` map directly onto it. Link lengths come from
`robot_system/kinematics/geometry.py`; joint limits are derived from the tick
ranges in `robot_sam2_app_v2/.../config.py`.

> The placeholder is for visualization only — not a kinematic calibration.
> See `so101_description/urdf/FETCH_OFFICIAL_URDF.md` to swap in the real
> SO-ARM101 URDF + meshes, including the joint-name mapping table.

### Verify

```bash
# URDF parses cleanly
xacro $(ros2 pkg prefix so101_description)/share/so101_description/urdf/so101.urdf.xacro | check_urdf /dev/stdin

# TF tree is connected (writes frames.pdf)
ros2 run tf2_tools view_frames

# 6 named joints present
ros2 topic echo /joint_states --once
```

---

## Phase 2: Real servo driver

Wraps `robot_sam2_app_v2/.../hardware.py`'s `make_hardware()`. Defaults to
`dry_run=true` — no hardware connected, commands are accepted but dropped.

```bash
colcon build --symlink-install --packages-select so101_driver
source install/setup.bash
ros2 launch so101_bringup hardware.launch.py            # dry_run (default)
ros2 launch so101_bringup hardware.launch.py dry_run:=false backend:=feetech serial_port:=/dev/ttyACM0
```

Topics: `/joint_states` (pub), `/joint_command` (sub), `/emergency_stop` (sub),
`/gripper_state` (pub).
Services: `/emergency_stop_srv`.

**Daemon backend**: needs `motor_daemon` compiled for Linux.
`TODO(hardware): build motor_daemon/src/motor_daemon.cpp on Ubuntu.`

---

## Phase 3: Cameras and distance sensor

```bash
colcon build --symlink-install --packages-select so101_perception
ros2 launch so101_bringup sensors.launch.py dry_run_sensor:=true
```

Nodes: `base_camera_node` → `/base_camera/image_raw`, `gripper_camera_node` →
`/gripper_camera/image_raw`, `distance_sensor_node` → `/range`.

Set `dry_run_sensor:=false` and `sensor_port:=/dev/ttyACM1` with ESP32 connected.
`TODO(hardware): confirm /dev/ttyACM* path with `ls /dev/ttyACM*``.

---

## Phase 4: AI perception (Grounding DINO + SAM2)

Requires Phase 3. Needs `transformers` (for Grounding DINO) and `sam2` pip packages.

```bash
ros2 launch so101_bringup full_perception.launch.py sam2_checkpoint:=/path/to/sam2.1_hiera_tiny.pt
```

Services: `/detect_object` (Grounding DINO, service-based), `/initialize_tracking` (SAM2+CSRT).
Topics: `/detected_objects`, `/target_pixel`, `/drop_zone_pixel`.
Service: `/set_drop_zone` (manual pixel or AI query).

`TODO(hardware): set sam2_checkpoint param to your actual .pt file path.`

---

## Phase 5: Pixel-to-robot calibration

Wraps `robot_system/calibration/base_camera_to_robot.py` (homography DLT).

```bash
colcon build --symlink-install --packages-select so101_calibration
ros2 launch so101_bringup calibration.launch.py calibration_file:=/path/to/cal.json
```

Service: `/pixel_to_robot_xy`.
Start uncalibrated (no JSON), collect points, call `CameraRobotCalibration.save()`,
then restart with `calibration_file` pointing to the saved JSON.

---

## Phase 6: Autonomous task planner

Full pick-and-place state machine. Requires all previous phases.

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch so101_bringup autonomous.launch.py \
    dry_run:=false backend:=feetech serial_port:=/dev/ttyACM0 \
    sam2_checkpoint:=/path/to/sam2.1_hiera_tiny.pt \
    calibration_file:=/path/to/calibration.json

# In another terminal — start a task:
ros2 service call /start_task so101_interfaces/srv/StartTask \
    '{pick_query: "the red cup", place_query: "the open box on the right"}'

# Abort at any time:
ros2 service call /abort_task std_srvs/srv/Trigger '{}'
```

States: `IDLE → SCAN_OBJECTS → SELECT_OBJECT → PLAN_PICK → MOVE_TO_PREGRASP →
ALIGN_WITH_GRIPPER_CAM → GRASP → VERIFY_GRASP → MOVE_TO_DROP → PLACE →
VERIFY_PLACE → RETRY_OR_DONE`.

Monitor: `ros2 topic echo /task_state`.

---

## Phase 7: Gazebo simulation

Secondary demo layer. Requires:
```bash
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
```

```bash
colcon build --symlink-install --packages-select so101_gazebo
source install/setup.bash
ros2 launch so101_bringup gazebo_sim.launch.py
```

Opens Gazebo Harmonic with a table + placeholder objects.
`TODO(gazebo): spawn SO-101 SDF model once official meshes are available.`

---

## Package summary

| Package | Phase | Type | Key nodes |
|---------|-------|------|-----------|
| `so101_description` | 1 | ament_cmake | (URDF/xacro assets) |
| `so101_bringup` | 1–7 | ament_cmake | (launch orchestration) |
| `so101_interfaces` | 2+ | ament_cmake | (msgs + srvs) |
| `so101_driver` | 2 | ament_python | `driver_node` |
| `so101_perception` | 3–4 | ament_python | `base_camera_node`, `gripper_camera_node`, `distance_sensor_node`, `object_detection_node`, `sam_node`, `drop_zone_detector_node` |
| `so101_calibration` | 5 | ament_python | `calibration_node` |
| `so101_task_planner` | 6 | ament_python | `task_planner_node` |
| `so101_gazebo` | 7 | ament_cmake | (world + launch) |
