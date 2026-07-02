#!/usr/bin/env bash
# Start one autonomous RoboNine sim task against the already-running sim stack.
#
# Usage:
#   ./run_robonine_task.sh "red cylinder" "green square"

set -eo pipefail

PICK_QUERY="${1:-red cylinder}"
PLACE_QUERY="${2:-green square}"
WS="${HOME}/so101_ws"

unset LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"

echo "Waiting for planner services..."
until timeout 5 ros2 service list 2>/dev/null | grep -q "^/set_drop_zone$"; do
  sleep 1
done
until timeout 5 ros2 service list 2>/dev/null | grep -q "^/start_task$"; do
  sleep 1
done

echo "Setting drop zone: ${PLACE_QUERY}"
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
