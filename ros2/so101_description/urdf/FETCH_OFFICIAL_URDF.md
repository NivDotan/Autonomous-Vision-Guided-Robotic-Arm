# Using the official SO-101 URDF + meshes

Two maintained sources exist. Use whichever you prefer.

---

## Option A — legalaspro/so101-ros-physical-ai (recommended)

A complete, maintained ROS2 Jazzy stack for the SO-101 with Xacro, STL meshes,
ros2_control, and MoveIt2. Its `so101_description` package is the most complete
available as of mid-2026.

```bash
cd ~/so101_ws/src
git clone https://github.com/legalaspro/so101-ros-physical-ai so101_physical_ai
```

> **Name conflict**: that repo also has a package called `so101_description`.
> Our package in `ros2/so101_description/` uses the same name.
> **Resolution**: remove our `so101_description` symlink and let colcon use theirs:
> ```bash
> rm ~/so101_ws/src/so101_description          # remove our symlink
> colcon build --packages-skip so101_description --symlink-install  # skip ours
> # theirs is auto-discovered via so101_physical_ai/so101_description
> ```
> Alternatively, rename ours by editing `ros2/so101_description/package.xml`
> `<name>` tag to `so101_description_placeholder` and updating the symlink name.

### After cloning — build and launch

```bash
cd ~/so101_ws
colcon build --symlink-install
source install/setup.bash

# Phase 1 with real meshes:
ros2 launch so101_bringup display.launch.py use_placeholder:=false

# Also set use_official_names:=true in driver so /joint_states names match the URDF:
ros2 launch so101_bringup hardware.launch.py use_official_names:=true
```

### Joint name mapping

The official package uses different joint names from the v2 app's MOTOR_NAMES.
Motor IDs are identical. The driver auto-remaps in both directions.

| Project (MOTOR_NAMES) | Motor ID | Official SO-101 joint |
|-----------------------|----------|-----------------------|
| `base`                | 1        | `shoulder_pan`        |
| `shoulder`            | 2        | `shoulder_lift`       |
| `elbow`               | 3        | `elbow_flex`          |
| `palm`                | 4        | `wrist_flex`          |
| `wrist`               | 5        | `wrist_roll`          |
| `gripper`             | 6        | `gripper`             |

The driver's `use_official_names` parameter controls which names appear on
`/joint_states`. `/joint_command` accepts both naming conventions always.

---

## Option B — TheRobotStudio/SO-ARM100 (URDF only, no ROS2 package)

The canonical source for the physical URDF and STL meshes without any ROS2 stack.

```bash
# Sparse clone — only the SO-101 simulation files
git clone --filter=blob:none --sparse \
    https://github.com/TheRobotStudio/SO-ARM100 /tmp/SO-ARM100
cd /tmp/SO-ARM100
git sparse-checkout set Simulation/SO101 STL/SO101
```

Then copy into our package:

```bash
PKG=~/so101_ws/src/Autonomous-Vision-Guided-Robotic-Arm/ros2/so101_description

# URDF → rename as the xacro wrapper expects
cp /tmp/SO-ARM100/Simulation/SO101/so101_new_calib.urdf \
   $PKG/urdf/so101_official.urdf.xacro

# Meshes (the URDF references these as relative paths; fix to package://)
mkdir -p $PKG/meshes/so101
cp /tmp/SO-ARM100/STL/SO101/Individual/*.stl $PKG/meshes/so101/
```

Fix the mesh paths in `so101_official.urdf.xacro`: replace any relative `../STL/...`
or absolute paths with `package://so101_description/meshes/so101/<file>.stl`.

Build and launch:
```bash
colcon build --symlink-install --packages-select so101_description so101_bringup
source install/setup.bash
ros2 launch so101_bringup display.launch.py use_placeholder:=false
```

> **Note**: The TheRobotStudio URDF uses the same joint names as the official ones
> listed in the mapping table above (`shoulder_pan`, `shoulder_lift`, etc.).

---

## Checking which URDF is loaded

```bash
# After launching, inspect the robot_description parameter:
ros2 param get /robot_state_publisher robot_description | grep "joint name"

# Or view the TF tree:
ros2 run tf2_tools view_frames   # generates frames.pdf
```
