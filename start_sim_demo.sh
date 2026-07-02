#!/usr/bin/env bash
# start_sim_demo.sh — one command to launch the full Gazebo demo:
#   1) Gazebo (arm on the table + objects + scene camera)
#   2) Grounding DINO detector running on the scene camera
#
# Usage:  ./start_sim_demo.sh  ["query"]
#   e.g.  ./start_sim_demo.sh "red cylinder"

REPO="$(cd "$(dirname "$0")" && pwd)"
WS="$HOME/so101_ws"
QUERY="${1:-red cylinder}"

# kill any previous sim
pkill -9 -f "gz sim"      2>/dev/null
pkill -9 -f "ruby.*gz"    2>/dev/null
pkill -9 -f detect_on_topic 2>/dev/null
sleep 1

ROS_SETUP="unset LD_LIBRARY_PATH; source /opt/ros/jazzy/setup.bash; source '$WS/install/setup.bash'"

echo "Launching Gazebo sim..."
gnome-terminal --title="Gazebo Sim" -- bash -c "
    $ROS_SETUP
    ros2 launch so101_gazebo gazebo.launch.py
    echo '--- gazebo exited ---'; read -p 'Enter to close'" &

echo "Waiting 12s for Gazebo + camera to come up..."
sleep 12

echo "Launching Grounding DINO detector on the scene camera (query: '$QUERY')..."
gnome-terminal --title="Detector (scene cam)" -- bash -c "
    $ROS_SETUP
    cd '$REPO'
    python3 tools/detect_on_topic.py --topic /gripper_camera/image_raw --query '$QUERY'
    echo '--- detector exited ---'; read -p 'Enter to close'" &

echo ""
echo "Done. Two windows: Gazebo + the detector view."
echo "In the detector window: T = new query, +/- = threshold, Q = quit."
echo "Move the arm any time:  python3 ros2/so101_gazebo/scripts/move_arm.py --pose ready"
