#!/usr/bin/env bash
# Show the saved Gazebo start pose in RViz using the RoboNine SO-ARM101 mesh URDF.

set -eo pipefail

REPO="$(cd "$(dirname "$0")" && pwd)"
WS="${HOME}/so101_ws"
POSE_FILE="${1:-${REPO}/robonine_gazebo_start_pos.json}"
ROS_SETUP="source /opt/ros/jazzy/setup.bash; source '${WS}/install/setup.bash'"

echo "[cleanup] stopping old RViz start-pose viewers..."
pkill -9 -f "publish_start_pose_joint_states.py" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
pkill -9 -f "joint_state_publisher_gui" 2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
sleep 1

echo "[1/2] opening RViz display..."
gnome-terminal --title="RViz Start Pose" -- bash -lc "
  ${ROS_SETUP}
  cd '${REPO}'
  ros2 launch '${REPO}/ros2/so101_bringup/launch/robonine_display.launch.py'
  echo
  echo '--- RViz display exited ---'
  read -r -p 'Press Enter to close...'
" &

echo "[2/2] publishing ${POSE_FILE} as /joint_states..."
gnome-terminal --title="Start Pose Joint States" -- bash -lc "
  ${ROS_SETUP}
  cd '${REPO}'
  python3 tools/publish_start_pose_joint_states.py --model robonine --pose '${POSE_FILE}'
  echo
  echo '--- joint-state publisher exited ---'
  read -r -p 'Press Enter to close...'
" &

echo "RViz should show the pose from: ${POSE_FILE}"
