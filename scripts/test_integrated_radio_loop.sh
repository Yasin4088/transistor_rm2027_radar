#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VISION_PYTHON="${VISION_PYTHON:-$PROJECT_ROOT/.venv/bin/python}"
SHARED_VISION_PYTHON="/home/elysia/robomaster/shark-radar-system/shark-radar-vision/.venv/bin/python"
radio_pid=""
MATCH_RX_ARGS=(--dry-run --no-panel --no-browser)

while (($#)); do
  case "$1" in
    --allow-gnuradio-3101)
      export RM_RADIO_ALLOW_GNURADIO_3101=true
      MATCH_RX_ARGS+=(--allow-gnuradio-3101)
      shift
      ;;
    --help|-h)
      cat <<'EOF'
Usage: scripts/test_integrated_radio_loop.sh [--allow-gnuradio-3101]

Runs dry-run integrated radio loopback verification.
--allow-gnuradio-3101 enables compatibility mode for GNU Radio 3.10.1.
EOF
      exit 0
      ;;
    *)
      echo "unknown option: $1" >&2
      exit 2
      ;;
  esac
done

cleanup() {
  trap - EXIT INT TERM HUP
  if [[ "$radio_pid" =~ ^[0-9]+$ ]] && kill -0 "$radio_pid" 2>/dev/null; then
    kill -TERM "$radio_pid" 2>/dev/null || true
    wait "$radio_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

if [[ ! -x "$VISION_PYTHON" && -x "$SHARED_VISION_PYTHON" ]]; then
  VISION_PYTHON="$SHARED_VISION_PYTHON"
  echo "[radio-loop-test] using existing shared vision environment: $VISION_PYTHON" >&2
fi
if [[ ! -x "$VISION_PYTHON" ]]; then
  echo "vision Python not found: $VISION_PYTHON" >&2
  echo "create $PROJECT_ROOT/.venv or set VISION_PYTHON explicitly" >&2
  exit 2
fi
if [[ ! -f "$PROJECT_ROOT/radio/install/setup.bash" ]]; then
  "$PROJECT_ROOT/radio/build.sh"
fi

"$PROJECT_ROOT/radio/apps/match_rx/start.sh" "${MATCH_RX_ARGS[@]}" &
radio_pid=$!
for _ in $(seq 1 100); do
  kill -0 "$radio_pid" 2>/dev/null || {
    wait "$radio_pid"
    exit $?
  }
  if ss -lun 2>/dev/null | grep -q ':37602 '; then
    break
  fi
  sleep 0.1
done

cd "$PROJECT_ROOT"
PYTHONPATH=src "$VISION_PYTHON" scripts/verify_radio_bridge.py
