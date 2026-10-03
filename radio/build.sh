#!/usr/bin/env bash
set -Eeuo pipefail

RADIO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$RADIO_ROOT"

# ROS2 Humble is built for system Python 3.10. Do not activate the vision venv.
set +u
source /opt/ros/humble/setup.bash
set -u
export PYTHONNOUSERSITE=1
colcon build --symlink-install
