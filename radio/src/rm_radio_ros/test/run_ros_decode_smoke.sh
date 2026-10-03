#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/../../.."
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

mkdir -p /tmp/ros_log /tmp/ros_home
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_log}"
export ROS_HOME="${ROS_HOME:-/tmp/ros_home}"

log_file=/tmp/rm_gfsk_launch.log
rm -f "$log_file"

timeout 20s ros2 launch rm_radio_ros gfsk.launch.py dry_run:=true >"$log_file" 2>&1 &
launch_pid=$!

cleanup() {
  kill "$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT

sleep 4
"${PYTHON:-/usr/bin/python3}" src/rm_radio_ros/test/ros_decode_smoke.py
cat "$log_file"
