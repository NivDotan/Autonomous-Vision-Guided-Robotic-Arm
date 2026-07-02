#!/usr/bin/env bash
# start_sim_search.sh — ONE command: launch Gazebo, wait for the camera, then
# run the autonomous arm-sweep search for an object.
#
# Usage:  ./start_sim_search.sh ["query"]
#   e.g.  ./start_sim_search.sh "red cylinder"

REPO="$(cd "$(dirname "$0")" && pwd)"
WS="$HOME/so101_ws"
QUERY="${1:-red cylinder}"
ROS_SETUP="unset LD_LIBRARY_PATH; source /opt/ros/jazzy/setup.bash; source '$WS/install/setup.bash'"

# NOTE: patterns must NOT match this launcher's own name (start_sim_search.sh),
# or pkill kills itself → "Killed". Match the .py scripts specifically.
pkill -9 -f "gz sim" 2>/dev/null; pkill -9 -f "ruby.*gz" 2>/dev/null
pkill -9 -f "sim_search.py" 2>/dev/null; pkill -9 -f "detect_on_topic.py" 2>/dev/null
sleep 1

echo "[1/2] Launching Gazebo..."
gnome-terminal --title="Gazebo Sim" -- bash -c "
    $ROS_SETUP
    ros2 launch so101_gazebo gazebo.launch.py
    echo '--- gazebo exited ---'; read -p 'Enter to close'" &

echo "[2/2] Search window will wait for the camera, then sweep + detect..."
gnome-terminal --title="Sim Search" -- bash -c "
    $ROS_SETUP
    cd '$REPO'
    echo 'Waiting for /gripper_camera/image_raw to come up...'
    until ros2 topic list 2>/dev/null | grep -q /gripper_camera/image_raw; do sleep 1; done
    # wait for an actual frame (camera rendering ready)
    timeout 30 ros2 topic echo --once /gripper_camera/image_raw >/dev/null 2>&1
    sleep 2
    echo 'Camera up — starting search.'
    python3 ros2/so101_gazebo/scripts/sim_search.py --query '$QUERY'
    echo '--- search exited ---'; read -p 'Enter to close'" &

echo ""
echo "Launched. Gazebo + the search will start automatically once the camera is live."
echo "Query: '$QUERY'   (change: ./start_sim_search.sh \"green square\")"
