#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VISION_PYTHON="/home/elysia/robomaster/shark-radar-system/shark-radar-vision/.venv/bin/python"
RADIO_ARGS=()
RUN_VISION=true

while (($#)); do
  case "$1" in
    --dry-run|--no-panel|--no-browser|--headless|--native-gui)
      RADIO_ARGS+=("$1")
      shift
      ;;
    --radio-only)
      RUN_VISION=false
      shift
      ;;
    --help|-h)
      cat <<'EOF'
Usage: scripts/start_integrated_radar.sh [--dry-run] [--no-panel] [--no-browser]
                                         [--headless|--native-gui] [--radio-only]

Starts the in-repository ROS2/GNU Radio backend and the Python 3.12 vision
process as one supervised application. --dry-run avoids real SDR and referee
serial I/O. LabTX is deliberately separate and is never started here.
EOF
      exit 0
      ;;
    *)
      echo "unknown option: $1" >&2
      exit 2
      ;;
  esac
done

if [[ ! -x "$VISION_PYTHON" ]]; then
  echo "vision Python not found: $VISION_PYTHON" >&2
  exit 2
fi
if [[ ! -f "$PROJECT_ROOT/radio/install/setup.bash" ]]; then
  echo "radio workspace is not built; run ./radio/build.sh first" >&2
  exit 2
fi

radio_pid=""
vision_pid=""
cleanup() {
  trap - EXIT INT TERM HUP
  local pid
  for pid in "$vision_pid" "$radio_pid"; do
    if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done
  wait "$vision_pid" 2>/dev/null || true
  wait "$radio_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

"$PROJECT_ROOT/radio/apps/match_rx/start.sh" "${RADIO_ARGS[@]}" &
radio_pid=$!

# Give ROS discovery and the UDP sidecar a brief deterministic startup window.
for _ in $(seq 1 50); do
  kill -0 "$radio_pid" 2>/dev/null || {
    wait "$radio_pid"
    exit $?
  }
  if ss -lun 2>/dev/null | grep -q ':37602 '; then
    break
  fi
  sleep 0.1
done

if [[ "$RUN_VISION" == "true" ]]; then
  (
    cd "$PROJECT_ROOT"
    exec "$VISION_PYTHON" main.py
  ) &
  vision_pid=$!
fi

while true; do
  if ! kill -0 "$radio_pid" 2>/dev/null; then
    wait "$radio_pid"
    exit $?
  fi
  if [[ "$RUN_VISION" == "true" ]] && ! kill -0 "$vision_pid" 2>/dev/null; then
    wait "$vision_pid"
    exit $?
  fi
  sleep 1
done
