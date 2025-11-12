#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REAL_ROBOT_SCRIPT="$HOME/autonomy_stack_go2/system_real_robot.sh"
REAL_ROBOT_PID=""

cleanup() {
  if [[ -n "$REAL_ROBOT_PID" ]] && ps -p "$REAL_ROBOT_PID" > /dev/null 2>&1; then
    kill "$REAL_ROBOT_PID" 2>/dev/null
    wait "$REAL_ROBOT_PID" 2>/dev/null
  fi
}

trap cleanup EXIT INT TERM

if [[ -x "$REAL_ROBOT_SCRIPT" ]]; then
  echo "Launching real robot system from $REAL_ROBOT_SCRIPT"
  bash "$REAL_ROBOT_SCRIPT" &
  REAL_ROBOT_PID=$!
else
  echo "Warning: $REAL_ROBOT_SCRIPT not found or not executable; skipping real robot launch."
fi

cd "$SCRIPT_DIR"
source install/setup.bash
ros2 launch base_link_waypoint_publisher base_link_waypoint.launch.py "$@"
