#!/usr/bin/env bash
# One command for RoboNine Gazebo + the ROS2 autonomous algorithm.
#
# Usage:
#   ./start_robonine_sim_algo.sh
#   ./start_robonine_sim_algo.sh "red cylinder" "green square"
#
# The first argument is the pick query. If omitted, the script starts the stack
# and prints the /start_task command instead of auto-starting the task.

set -eo pipefail

REPO="$(cd "$(dirname "$0")" && pwd)"
WS="${HOME}/so101_ws"
PICK_QUERY="${1:-}"
PLACE_QUERY="${2:-green square}"
ROS_SETUP="unset LD_LIBRARY_PATH; source /opt/ros/jazzy/setup.bash; source '${WS}/install/setup.bash'"

unset LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"

echo "[cleanup] stopping old Gazebo/rqt/algo instances..."
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "ruby.*gz" 2>/dev/null || true
pkill -9 -f "rqt_joint_trajectory_controller" 2>/dev/null || true
pkill -9 -f "rqt_image_view" 2>/dev/null || true
pkill -9 -f "robonine_sim_algo_bridge.py" 2>/dev/null || true
pkill -9 -f "object_detection_node" 2>/dev/null || true
pkill -9 -f "sam_node" 2>/dev/null || true
pkill -9 -f "drop_zone_detector_node" 2>/dev/null || true
pkill -9 -f "task_planner_node" 2>/dev/null || true
bash -lc "${ROS_SETUP}; ros2 daemon stop" >/dev/null 2>&1 || true
sleep 1

echo "[1/6] launching RoboNine Gazebo sim..."
gnome-terminal --title="RoboNine Gazebo Sim" -- bash -lc "
  ${ROS_SETUP}
  ros2 launch so_arm_101_description sim.launch.py
  echo
  echo '--- sim exited ---'
  read -r -p 'Press Enter to close...'
" &

echo "[2/6] waiting for controller and simulated cameras..."
until timeout 5 ros2 control list_controllers 2>/dev/null | grep -q "arm_controller.*active"; do
  sleep 1
done
until timeout 5 ros2 topic list 2>/dev/null | grep -q "^/gripper_camera/image_raw$"; do
  sleep 1
done
DETECTION_TOPIC="/gripper_camera/image_raw"
for _ in {1..5}; do
  if timeout 5 ros2 topic list 2>/dev/null | grep -q "^/scene_camera/image_raw$"; then
    DETECTION_TOPIC="/scene_camera/image_raw"
    break
  fi
  sleep 1
done
echo "[2/6] detection camera: ${DETECTION_TOPIC}"

echo "[3/6] moving robot to robonine_gazebo_start_pos.json..."
cd "${REPO}"
python3 tools/robonine_move_start.py --pose robonine_gazebo_start_pos.json --time 8.0

echo "[4/6] starting sim algorithm bridge..."
gnome-terminal --title="Sim Algo Bridge" -- bash -lc "
  ${ROS_SETUP}
  cd '${REPO}'
  python3 tools/robonine_sim_algo_bridge.py --pose robonine_gazebo_start_pos.json
  echo
  echo '--- sim algo bridge exited ---'
  read -r -p 'Press Enter to close...'
" &

echo "[5/6] starting perception + task planner..."
gnome-terminal --title="SO101 Algorithm Stack" -- bash -lc "
  ${ROS_SETUP}
  cd '${REPO}'
  ros2 run so101_perception object_detection_node --ros-args \
    -r /base_camera/image_raw:=${DETECTION_TOPIC} &
  ros2 run so101_perception sam_node &
  ros2 run so101_perception drop_zone_detector_node --ros-args \
    -r /base_camera/image_raw:=${DETECTION_TOPIC} &
  ros2 run so101_task_planner task_planner_node --ros-args -p dry_run:=true &
  wait
  echo
  echo '--- algorithm stack exited ---'
  read -r -p 'Press Enter to close...'
" &

echo "[6/6] opening jog GUI and camera viewer..."
gnome-terminal --title="Jog GUI" -- bash -lc "
  ${ROS_SETUP}
  ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
  echo
  echo '--- jog GUI exited ---'
  read -r -p 'Press Enter to close...'
" &

gnome-terminal --title="Detection Camera" -- bash -lc "
  ${ROS_SETUP}
  ros2 run rqt_image_view rqt_image_view ${DETECTION_TOPIC}
  echo
  echo '--- detection camera viewer exited ---'
  read -r -p 'Press Enter to close...'
" &

echo
echo "Ready."
echo "Algorithm bridge: /joint_command ticks -> /arm_controller/joint_trajectory."
echo "Detection uses ${DETECTION_TOPIC}. Tracking uses /gripper_camera/image_raw."

if [[ -n "${PICK_QUERY}" ]]; then
  echo "Waiting for planner services..."
  until timeout 5 ros2 service list 2>/dev/null | grep -q "^/set_drop_zone$"; do
    sleep 1
  done
  until timeout 5 ros2 service list 2>/dev/null | grep -q "^/start_task$"; do
    sleep 1
  done
  sleep 2
  echo "Setting drop zone from ${DETECTION_TOPIC}: '${PLACE_QUERY}'"
  DROP_OUTPUT="$(ros2 service call /set_drop_zone so101_interfaces/srv/SetDropZone \
    "{mode: 'ai', query: '${PLACE_QUERY}', pixel_x: 0.0, pixel_y: 0.0}" || true)"
  echo "${DROP_OUTPUT}"
  if ! grep -q "success=True" <<<"${DROP_OUTPUT}"; then
    echo "AI drop-zone not found; using manual image-center drop zone."
    ros2 service call /set_drop_zone so101_interfaces/srv/SetDropZone \
      "{mode: 'manual', query: '', pixel_x: 320.0, pixel_y: 240.0}"
  fi
  echo "Starting task: pick='${PICK_QUERY}' place='${PLACE_QUERY}'"
  ros2 service call /start_task so101_interfaces/srv/StartTask \
    "{pick_query: '${PICK_QUERY}', place_query: '${PLACE_QUERY}'}"
else
  echo
  echo "Set the drop zone with:"
  echo "ros2 service call /set_drop_zone so101_interfaces/srv/SetDropZone \"{mode: 'ai', query: 'green square', pixel_x: 0.0, pixel_y: 0.0}\""
  echo
  echo "Start a task with:"
  echo "ros2 service call /start_task so101_interfaces/srv/StartTask \"{pick_query: 'red cylinder', place_query: 'green square'}\""
fi
