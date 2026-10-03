#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"

if [[ -z "${RM_ALGO_DIR:-}" || -z "${RM_ALGO_PYTHON:-}" ]]; then
  echo "algorithm disabled: RM_ALGO_DIR or RM_ALGO_PYTHON is empty" >&2
  exit 0
fi
if [[ ! -x "$RM_ALGO_PYTHON" ]]; then
  echo "algorithm python is not executable: $RM_ALGO_PYTHON" >&2
  exit 1
fi
cd "$RM_ALGO_DIR"
export RM_ALGO_DISABLE_REFEREE_SERIAL=1
exec "$RM_ALGO_PYTHON" "$RM_RADIO_WS/src/rm_radio_ros/rm_radio_ros/app_support/run_algorithm.py"
