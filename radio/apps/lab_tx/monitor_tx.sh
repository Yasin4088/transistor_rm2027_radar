#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/env.sh"

echo "Expected TX status topics:"
echo "  /rm_ganraoyuan_node/status"
echo "  /rm_iq_replay_tx_node/status"
echo

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 not found after sourcing env." >&2
  exit 2
fi

echo "Current ROS topics:"
ros2 topic list | sort
