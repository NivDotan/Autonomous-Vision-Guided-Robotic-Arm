#!/usr/bin/env bash
# One command for the RoboNine SO-ARM101 sim:
#   1) launch Gazebo with the gripper camera
#   2) wait for ros2_control + camera
#   3) move the robot to robonine_gazebo_start_pos.json
#   4) open the jog GUI and camera viewer
#
# Run this from a native GNOME terminal, not the VS Code/snap terminal:
#   ./start_robonine_sim_all.sh

set -eo pipefail

REPO="$(cd "$(dirname "$0")" && pwd)"
WS="${HOME}/so101_ws"
ROS_SETUP="unset LD_LIBRARY_PATH; source /opt/ros/jazzy/setup.bash; source '${WS}/install/setup.bash'"

unset LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"

echo "[cleanup] stopping old Gazebo/rqt instances..."
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "ruby.*gz" 2>/dev/null || true
pkill -9 -f "rqt_joint_trajectory_controller" 2>/dev/null || true
pkill -9 -f "rqt_image_view" 2>/dev/null || true
bash -lc "${ROS_SETUP}; ros2 daemon stop" >/dev/null 2>&1 || true
sleep 1

echo "[1/4] launching RoboNine Gazebo sim..."
gnome-terminal --title="RoboNine Gazebo Sim" -- bash -lc "
  ${ROS_SETUP}
  ros2 launch so_arm_101_description sim.launch.py
  echo
  echo '--- sim exited ---'
  read -r -p 'Press Enter to close...'
" &

echo "[2/4] waiting for /controller_manager..."
until timeout 5 ros2 control list_controllers 2>/dev/null | grep -q "arm_controller"; do
  sleep 1
done

echo "[2/4] waiting for arm_controller to become active..."
until timeout 5 ros2 control list_controllers 2>/dev/null | grep -q "arm_controller.*active"; do
  sleep 1
done

echo "[2/4] waiting for /gripper_camera/image_raw..."
until timeout 5 ros2 topic list 2>/dev/null | grep -q "^/gripper_camera/image_raw$"; do
  sleep 1
done

echo "[3/4] moving robot to robonine_gazebo_start_pos.json..."
cd "${REPO}"
python3 tools/robonine_move_start.py --pose robonine_gazebo_start_pos.json --time 8.0

echo "[4/4] opening jog GUI and camera viewer..."
gnome-terminal --title="Jog GUI" -- bash -lc "
  ${ROS_SETUP}
  ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
  echo
  echo '--- jog GUI exited ---'
  read -r -p 'Press Enter to close...'
" &

gnome-terminal --title="Gripper Camera" -- bash -lc "
  ${ROS_SETUP}
  ros2 run rqt_image_view rqt_image_view /gripper_camera/image_raw
  echo
  echo '--- camera viewer exited ---'
  read -r -p 'Press Enter to close...'
" &

echo
echo "Ready."
echo "In the jog GUI, select /controller_manager -> arm_controller."
echo "You should see 7 sliders."
