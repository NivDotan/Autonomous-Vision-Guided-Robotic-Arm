# RoboNine SO-ARM101 Gazebo Sim — Handoff Notes

Context for continuing this work in a new chat. This describes a **Gazebo Harmonic
simulation** of the **RoboNine parallel-gripper SO-ARM101** on a table, with a
gripper camera, set up on Ubuntu 24.04 + ROS2 Jazzy.

## Goal
Simulate the RoboNine gripper variant of the SO-101 (the user physically swapped to
the RoboNine 3D-printed parallel gripper, which has a camera), so they can run their
zero-shot vision algo (Grounding DINO / SAM2) on the simulated gripper camera, and
manually drive the arm + gripper.

## What was set up
- Cloned RoboNine's package into the workspace and **use it directly** (it ships a
  ready ROS2 description + Gazebo + RViz):
  - Source: https://github.com/roboninecom/SO-ARM100-101-Parallel-Gripper
    (the `simulation/so_arm_101_description` package).
  - Installed at: `~/so101_ws/src/so_arm_101_description/`
  - Build: `cd ~/so101_ws && colcon build --packages-select so_arm_101_description`
- Joint names (RoboNine, NOT the official SO-101 names):
  - Arm (5 revolute): `base_link_to_link1, link1_to_link2, link2_to_link3,
    link3_to_link4, link4_to_link5`
  - Gripper: `right_clamp` (prismatic, 0.0=closed … 0.037=open),
    `left_clamp` (prismatic, -0.037=open … 0.0=closed).

## Patches we had to apply to the RoboNine package (all in `so_arm_101_description/`)
RoboNine's package does **not** run as-is on Jazzy. Changes made:

1. **Mimic-joint crash (fatal).** `left_clamp` originally was a `<mimic>` of
   `right_clamp` but the ros2_control block gave it a `<command_interface>`. Jazzy
   `gz_ros2_control` aborts:
   *"Activated mimic joints cannot have command interfaces."*
   - Current fix: removed the URDF `<mimic>` tag and made `left_clamp` explicitly
     commandable, so Gazebo and the jog GUI can expose both jaws.
2. **Invisible arm (meshes).** Gazebo couldn't resolve `package://`/`model://` mesh
   URIs → arm spawned but rendered invisible.
   - `launch/sim.launch.py`: set `os.environ['GZ_SIM_RESOURCE_PATH']` to the package
     share parent (`…/install/so_arm_101_description/share`).
3. **Table scene.** Added `worlds/table_world.sdf` (ground, sun, **table** top z=0.75,
   **red cylinder** `pick_object`, **green** `drop_zone`, and the **gz Sensors system
   plugin** so cameras render). `sim.launch.py` now loads this world instead of
   `empty.sdf`.
4. **Gripper camera.** Added to `urdf/so_101.urdf.xacro`: a `gripper_camera_link` on
   `link5_1` + a `<gazebo><sensor type="camera">` publishing **`/gripper_camera/image_raw`**,
   plus a `ros_gz_bridge` node in `sim.launch.py`. (Camera **rpy is still a guess** —
   needs tuning; see Known issues.)
5. **Joint effort too low.** All arm joints had `effort="1.5"`. The elbow
   (`link2_to_link3`) carries the forearm+gripper and **sagged to its 0.0 lower limit,
   never moving**. Raised all `effort="1.5"` → `effort="50.0"` in `so_101.urdf.xacro`.
6. **Gripper in the jog GUI.** `rqt_joint_trajectory_controller` only shows
   *JointTrajectoryControllers*. The gripper was a separate
   `position_controllers/JointGroupPositionController`, so it didn't appear. Merged
   both `right_clamp` and `left_clamp` into `arm_controller`'s joints and removed the
   separate `gripper_controller`. Now `arm_controller` drives all 7 joints.

## Required apt packages (Jazzy)
```bash
sudo apt install ros-jazzy-gz-ros2-control ros-jazzy-position-controllers \
  ros-jazzy-joint-trajectory-controller ros-jazzy-joint-state-broadcaster \
  ros-jazzy-ros2controlcli ros-jazzy-rqt-joint-trajectory-controller \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image
```

## How to RUN (must use a NATIVE GNOME terminal — Ctrl+Alt+T — NOT VS Code's)
The VS Code/snap terminal makes the Gazebo GUI crash with
`gz sim gui: symbol lookup error … /snap/core20/… libpthread.so.0`, which kills the
whole sim (server + controllers). A native terminal avoids it. Always:
```bash
unset LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash
source ~/so101_ws/install/setup.bash
```
Then:
```bash
# Gazebo (arm on table + cylinder + drop zone + gripper camera)
ros2 launch so_arm_101_description sim.launch.py
# RViz only
ros2 launch so_arm_101_description display.launch.py
```
Verify controllers:
```bash
ros2 control list_controllers     # arm_controller + joint_state_broadcaster = active
```

## How to CONTROL
- **Manual sliders:** `ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller`
  → select `controller_manager` → `arm_controller` → red button → 7 sliders
  (5 arm + `right_clamp` + `left_clamp`). If "no plugin matching": run
  `rqt --force-discover` once.
- **Command line (arm + gripper, gripper = 6th/7th values):**
  ```bash
  ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
   '{joint_names: [base_link_to_link1,link1_to_link2,link2_to_link3,link3_to_link4,link4_to_link5,right_clamp,left_clamp],
     points: [{positions: [0.0,0.0,1.5,0.0,0.0,0.037,-0.037], time_from_start: {sec: 2}}]}'
  ```
  - `link2_to_link3` range is **0.0 → 3.316** (positive only).
  - `right_clamp`: 0.0 = closed, 0.037 = open.
  - `left_clamp`: 0.0 = closed, -0.037 = open.
- **Camera (for the vision algo):** topic **`/gripper_camera/image_raw`**.
  - View: `ros2 run rqt_image_view rqt_image_view` → pick the topic.
  - Run detector: `python3 ~/so101_ws/src/Autonomous-Vision-Guided-Robotic-Arm/tools/detect_on_topic.py --topic /gripper_camera/image_raw --query "red cylinder"`

## Known issues / TODO
1. **Both gripper jaws are now exposed in the jog GUI.** `left_clamp` is no longer a
   URDF mimic joint; it has its own command interface and is part of `arm_controller`.
   The GUI should show 7 sliders total: 5 arm joints, `right_clamp`, and `left_clamp`.
   Open the gripper by commanding `right_clamp` positive and `left_clamp` negative
   (`right_clamp ~= 0.037`, `left_clamp ~= -0.037`). Close with both at `0.0`.
2. **Gripper camera angle is a guess** — `gripper_camera_joint` rpy in
   `urdf/so_101.urdf.xacro` may not frame the gripper/objects. View
   `/gripper_camera/image_raw` and tune the rpy, then rebuild + relaunch.
3. **Robot placement / world anchoring.** The model has a `world` link + fixed
   `world_to_base` joint, so Gazebo **ignores the spawn `-x/-y/-z/-Y`** — the robot
   pose must be set in `world_to_base`'s `<origin>` in `so_101.urdf.xacro` (currently
   `0 0 0`, i.e. on the ground, not on the table). Earlier attempts to place it on the
   table / rotate it 90° were rolled back at the user's request.
4. **Always rebuild after editing the URDF/yaml:** `colcon build --packages-select
   so_arm_101_description` (ament_python copies share files on build), then relaunch.

## File map (everything is under `~/so101_ws/src/so_arm_101_description/`)
- `urdf/so_101.urdf.xacro` — links/joints, world_to_base, gripper camera, efforts.
- `urdf/so_101.ros2_control.xacro` — ros2_control + gz plugin; both clamps commandable.
- `config/controllers.yaml` — arm_controller (7 joints incl. both clamps), jsb.
- `worlds/table_world.sdf` — table + cylinder + drop zone + sensors plugin.
- `launch/sim.launch.py` — Gazebo + spawn + controllers + camera bridge + resource path.
- `launch/display.launch.py` — RViz.
