#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_ROOT="${HARDWARE_LOOP_LOG_DIR:-/tmp/rm_radio_hardware_loop_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$RUN_ROOT"

export RM_RADIO_SIDE="${RM_RADIO_SIDE:-red}"
export RX1_URI="${RX1_URI:-ip:192.168.1.10}"
export RX2_URI="${RX2_URI:-ip:192.168.2.1}"
export BROADCAST_TX_URI="${BROADCAST_TX_URI:-ip:192.168.3.1}"
export RX_RF_PORT="${RX_RF_PORT:-A_BALANCED}"
export REFEREE_PORT="${REFEREE_PORT:-/dev/transistor-referee}"

# shellcheck disable=SC1091
source "$PROJECT_ROOT/radio/apps/lab_tx/env.sh"
# shellcheck disable=SC1091
source "$PROJECT_ROOT/radio/apps/common/lab_tx_safety.sh"

real_referee=false
case "${1:-}" in
  "") ;;
  --real-referee) real_referee=true ;;
  --help|-h)
    cat <<'EOF'
Usage: scripts/test_hardware_radio_loop.sh [--real-referee]

Default: real three-SDR RF receive/decode, but 0x0305 ends at referee dry-run.
--real-referee additionally writes 0x0305 to the configured referee serial port.

Required for either mode:
  RM_RADIO_LAB_TX_ANTENNA_CONFIRM=true
  RM_RADIO_CABLED_LOOP_CONFIRM=true
  RM_RADIO_EXTERNAL_ATTENUATION_DB=<total external attenuation, at least 40>

The RF path must be Nano-B TX -> 50-ohm coax -> attenuator(s) -> PZSDR RX1.
Remove all antennas from the participating TX/RX ports before running.
EOF
    exit 0
    ;;
  *) echo "unknown argument: $1" >&2; exit 2 ;;
esac

require_lab_tx_confirmation
require_cabled_loop_attenuation

if [[ "$RX1_URI" == "$RX2_URI" || "$RX1_URI" == "$BROADCAST_TX_URI" || "$RX2_URI" == "$BROADCAST_TX_URI" ]]; then
  echo "RX1_URI, RX2_URI, and BROADCAST_TX_URI must identify three distinct SDRs" >&2
  exit 2
fi
if ! command -v timeout >/dev/null 2>&1 || ! command -v iio_info >/dev/null 2>&1; then
  echo "hardware loop requires timeout and iio_info" >&2
  exit 2
fi

if [[ "$real_referee" == "true" ]]; then
  if [[ "${RM_RADIO_REAL_REFEREE_CONFIRM:-false}" != "true" ]]; then
    echo "--real-referee requires RM_RADIO_REAL_REFEREE_CONFIRM=true" >&2
    exit 2
  fi
  if [[ ! -c "$REFEREE_PORT" || ! -r "$REFEREE_PORT" || ! -w "$REFEREE_PORT" ]]; then
    echo "referee serial device is not a readable/writable character device: $REFEREE_PORT" >&2
    exit 2
  fi
  referee_dry_run=false
  verifier_args=(--expect-written)
else
  referee_dry_run=true
  verifier_args=()
fi

echo "[hardware-loop] probing three distinct IIO devices"
for role_uri in \
  "PZSDR-RX1=$RX1_URI" \
  "Nano-A-RX2=$RX2_URI" \
  "Nano-B-TX=$BROADCAST_TX_URI"; do
  role="${role_uri%%=*}"
  uri="${role_uri#*=}"
  if ! timeout --foreground --signal=TERM --kill-after=1s 5s \
    iio_info -u "$uri" >"$RUN_ROOT/probe_${role}.log" 2>&1; then
    echo "[hardware-loop] $role unreachable at $uri; see $RUN_ROOT/probe_${role}.log" >&2
    exit 3
  fi
  echo "[hardware-loop] $role reachable at $uri"
done

rx_pid=""
tx_pid=""
cleanup_done=false
cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM HUP
  if [[ "$cleanup_done" == "true" ]]; then
    exit "$exit_code"
  fi
  cleanup_done=true
  set +e
  echo "[hardware-loop] cleanup: stop TX and verify hard mute before RX shutdown"
  LAB_TX_TERM_GRACE_SECONDS=20 lab_tx_stop_pids "$tx_pid"
  lab_tx_cleanup_residual_tx \
    "rm_broadcast_tx_node" "start_broadcast_tx.sh" "flowgraph_name:=ganraoyuan"
  residual_rc=$?
  lab_tx_hard_mute_one_and_verify "$BROADCAST_TX_URI" \
    >>"$RUN_ROOT/final_tx_mute.log" 2>&1
  mute_rc=$?
  lab_tx_stop_pids "$rx_pid"
  if [[ "$exit_code" -eq 0 && ( "$residual_rc" -ne 0 || "$mute_rc" -ne 0 ) ]]; then
    exit_code=4
  fi
  echo "[hardware-loop] logs=$RUN_ROOT"
  exit "$exit_code"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

if [[ ! -f "$PROJECT_ROOT/radio/install/setup.bash" ]]; then
  "$PROJECT_ROOT/radio/build.sh"
fi

echo "[hardware-loop] starting live RX/fusion with referee_dry_run=$referee_dry_run"
LOG_DIR="$RUN_ROOT/match_rx" \
RM_RADIO_DRY_RUN=false \
REFEREE_DRY_RUN="$referee_dry_run" \
RM_RADIO_RECORDING_ENABLED=false \
RUN_RECORDER=false \
RUN_AUTO_PASSWORD=false \
RADAR_AUTO_DOUBLE_ENABLED=false \
AUTO_SEND_INVINCIBLE_TARGETS=false \
RUN_VISION_BRIDGE=false \
MATCH_WRAPPER_PID_FILE="$RUN_ROOT/match_rx.pid" \
MATCH_WRAPPER_LOCK_FILE="$RUN_ROOT/match_rx.lock" \
"$PROJECT_ROOT/radio/apps/match_rx/start.sh" \
  --no-panel --no-browser --headless >"$RUN_ROOT/match_rx_wrapper.log" 2>&1 &
rx_pid=$!

sleep 2
if ! lab_tx_pid_is_running "$rx_pid"; then
  wait "$rx_pid"
  exit $?
fi

echo "[hardware-loop] starting attenuated Nano-B broadcast TX"
LOG_DIR="$RUN_ROOT/broadcast_tx" \
"$PROJECT_ROOT/radio/apps/lab_tx/start_broadcast_tx.sh" \
  >"$RUN_ROOT/broadcast_tx_wrapper.log" 2>&1 &
tx_pid=$!

sleep 2
if ! lab_tx_pid_is_running "$tx_pid"; then
  wait "$tx_pid"
  exit $?
fi

echo "[hardware-loop] waiting for physical 0x0A01 -> fusion -> 0x0305 evidence"
python3 "$PROJECT_ROOT/scripts/verify_hardware_radio_ros.py" \
  --timeout-sec "${HARDWARE_LOOP_TIMEOUT_SEC:-45}" \
  "${verifier_args[@]}" | tee "$RUN_ROOT/verification.json"
