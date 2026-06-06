#!/usr/bin/env bash
# start_robot_linux.sh — Linux equivalent of start_robot_v2.bat
# Opens 3 terminals: motor daemon + v2 app + ROS2/RViz
#
# Usage:  chmod +x start_robot_linux.sh && ./start_robot_linux.sh

REPO="$(cd "$(dirname "$0")" && pwd)"
WS="$HOME/so101_ws"
ARM_PORT="${ARM_PORT:-/dev/ttyACM0}"

# ── permission check ──────────────────────────────────────────────────────────
if [ ! -e "$ARM_PORT" ]; then
    echo "ERROR: $ARM_PORT not found. Is the arm plugged in?"
    exit 1
fi
if ! [ -w "$ARM_PORT" ]; then
    sudo chmod a+rw "$ARM_PORT"
fi

echo "Launching SO-101 robot system..."

# ── Terminal 1: Motor Daemon ──────────────────────────────────────────────────
gnome-terminal --title="1 · Motor Daemon" -- bash -c "
    echo '=== Motor Daemon (replaces motor_daemon.exe) ==='
    python3 '$REPO/motor_daemon_py.py' \
        --port '$ARM_PORT' \
        --zmq-port 5555 \
        --pub-port 5556
    echo '--- daemon exited ---'; read -p 'Press Enter'"  &

echo "Waiting 2s for daemon to bind..."
sleep 2

# ── Terminal 2: v2 Python App (full vision pipeline) ─────────────────────────
gnome-terminal --title="2 · Robot App V2" -- bash -c "
    echo '=== Robot App V2 (vision + state machine) ==='
    cd '$REPO/robot_sam2_app_v2'
    python3 -m robot_sam2_app.main
    echo '--- app exited ---'; read -p 'Press Enter'"  &

sleep 1

# ── Terminal 3: ROS2 + RViz (live arm position) ───────────────────────────────
gnome-terminal --title="3 · ROS2 / RViz" -- bash -c "
    echo '=== ROS2 driver + RViz (live arm position) ==='
    unset LD_LIBRARY_PATH
    source /opt/ros/jazzy/setup.bash
    cd '$WS' && source install/setup.bash
    ros2 launch so101_bringup hardware.launch.py \
        dry_run:=false \
        backend:=daemon
    echo '--- ROS2 exited ---'; read -p 'Press Enter'"  &

echo ""
echo "All 3 terminals launched."
echo ""
echo "Controls (in the Robot Brain window):"
echo "  T     - type object description → Grounding DINO finds it → arm approaches"
echo "  A     - start approach"
echo "  Space - open/close gripper"
echo "  R     - reset to home"
echo "  Q     - quit"
