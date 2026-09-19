#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/env.sh"
# shellcheck disable=SC1091
source "$RM_RADIO_WS/apps/common/lab_tx_safety.sh"

if [[ -z "$INTERFERENCE_TX_URI" ]]; then
  echo "干扰波 TX 当前未连接，不能启动双路 TX" >&2
  exit 2
fi

require_lab_tx_confirmation

runtime_audit_args=()
if [[ "${RM_RADIO_REQUIRE_LATEST_RUNTIME:-false}" == "true" ]]; then
  runtime_audit_args+=(--require-latest)
fi
python3 "$RM_RADIO_WS/apps/common/check_gnuradio_runtime.py" "${runtime_audit_args[@]}"

rm_radio_guard_residual "lab-tx-dual" \
  "rm_broadcast_tx_node" "rm_interference_tx_node" "start_dual_tx.sh" \
  "flowgraph_name:=ganraoyuan"

rm_radio_ensure_dir "$LOG_DIR"

side="${RM_RADIO_SIDE,,}"
case "$side" in
  red) broadcast_freq=433200000 ;;
  blue) broadcast_freq=433920000 ;;
  *) echo "RM_RADIO_SIDE must be red or blue; got $RM_RADIO_SIDE" >&2; exit 2 ;;
esac
read -r interference_freq interference_bw < <(tx_frequency_for_side_level "$side" "$INTERFERENCE_LEVEL")

if [[ "$BROADCAST_TX_URI" == "$INTERFERENCE_TX_URI" ]]; then
  echo "BROADCAST_TX_URI and INTERFERENCE_TX_URI must be different for dual TX" >&2
  exit 2
fi

TX_RESIDUAL_PATTERNS=(
  "rm_broadcast_tx_node"
  "rm_interference_tx_node"
  "start_dual_tx.sh"
  "flowgraph_name:=ganraoyuan"
)
broadcast_pid=""
interference_pid=""
dual_tx_cleanup_done=false

dual_tx_cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM HUP
  if [[ "$dual_tx_cleanup_done" == "true" ]]; then
    exit "$exit_code"
  fi
  dual_tx_cleanup_done=true
  set +e
  {
    echo "[dual-tx] cleanup start exit_code=$exit_code"
    lab_tx_stop_pids "$broadcast_pid" "$interference_pid"
    lab_tx_cleanup_residual_tx "${TX_RESIDUAL_PATTERNS[@]}"
  } >>"$LOG_DIR/tx_safety.log" 2>&1
  local process_rc=$?
  # Allow the IIO backend to release buffers before taking direct hardware
  # control, then require both maximum attenuation and TX-LO powerdown.
  sleep 0.4
  lab_tx_hard_mute_and_verify "$BROADCAST_TX_URI" "$INTERFERENCE_TX_URI" >>"$LOG_DIR/tx_safety.log" 2>&1
  local mute_rc=$?
  lab_tx_verify_no_residual_tx "${TX_RESIDUAL_PATTERNS[@]}" >>"$LOG_DIR/tx_safety.log" 2>&1
  local residual_rc=$?
  echo "[dual-tx] cleanup end process_rc=$process_rc mute_rc=$mute_rc residual_rc=$residual_rc" >>"$LOG_DIR/tx_safety.log"
  if [[ "$exit_code" -eq 0 && ( "$process_rc" -ne 0 || "$mute_rc" -ne 0 || "$residual_rc" -ne 0 ) ]]; then
    exit_code=4
  fi
  exit "$exit_code"
}

trap dual_tx_cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP
rm -f "$LOG_DIR/dual_tx.pids"

echo "[dual-tx] lab-only dual TX startup"
echo "[dual-tx] side=$side level=$INTERFERENCE_LEVEL"
echo "[dual-tx] broadcast TX=$BROADCAST_TX_URI freq=$broadcast_freq attenuation=$BROADCAST_TX_ATTENUATION_DB dB rf_port=$BROADCAST_TX_RF_PORT"
echo "[dual-tx] interference TX=$INTERFERENCE_TX_URI freq=$interference_freq bw=$interference_bw attenuation=$INTERFERENCE_TX_ATTEND_GR dB rf_port=$INTERFERENCE_TX_RF_PORT"
echo "[dual-tx] logs=$LOG_DIR"

if [[ "$RM_RADIO_IIO_PREINIT" == "true" ]]; then
  # shellcheck disable=SC1091
  source "$RM_RADIO_WS/apps/common/iio_init.sh"
  tx_preinit_mode="$(ad9361_tx_preinit_mode "$RM_RADIO_SAMPLE_RATE")"
  init_ad9361_tx "$BROADCAST_TX_URI" "$broadcast_freq" 540000 "$BROADCAST_TX_ATTENUATION_DB" "$BROADCAST_TX_RF_PORT" "$RM_RADIO_SAMPLE_RATE" "$tx_preinit_mode" 2>>"$LOG_DIR/broadcast_tx_iio_init.log"
  init_ad9361_tx "$INTERFERENCE_TX_URI" "$interference_freq" "$interference_bw" "$INTERFERENCE_TX_ATTEND_GR" "$INTERFERENCE_TX_RF_PORT" "$RM_RADIO_SAMPLE_RATE" "$tx_preinit_mode" 2>>"$LOG_DIR/interference_tx_iio_init.log"
fi

build_broadcast_setters_json() {
  python3 - "$side" "$BROADCAST_TX_ATTENUATION_DB" "$RM_RADIO_SAMPLE_RATE" <<'PY'
import json
import sys
from rm_radio_ros.core.radio_config import BROADCAST_FREQUENCIES
from rm_radio_ros.core.virtual_link_test import _default_broadcast_config
from rm_radio_ros.core.rm_protocol import ACCESS_CODES
from apps.lab_tx.radio_tx_dashboard import build_tx_setters

side = sys.argv[1]
attenuation = float(sys.argv[2])
sample_rate = int(sys.argv[3])
state = {
    "side": side,
    "wave": "broadcast",
    "level": 1,
    "password": "R1L001",
    "tx": {"attenuation_db": attenuation, "sample_rate": sample_rate},
    "broadcast": _default_broadcast_config(),
}
setters = build_tx_setters(state)
setters["center_f"] = BROADCAST_FREQUENCIES[side]
setters["BW_ganrao"] = 540000
setters["access"] = list(ACCESS_CODES["broadcast"])
setters["attend_gr"] = attenuation
print(json.dumps(setters, separators=(",", ":")))
PY
}

broadcast_setters_json="${BROADCAST_SETTERS_JSON:-$(build_broadcast_setters_json)}"
interference_setters_json="${INTERFERENCE_SETTERS_JSON:-$(build_interference_setters_json)}"
flowgraph_dir="$RM_RADIO_WS/src/rm_radio_ros/flowgraphs/ganraoyuan"
# Direct rclpy invocations need string parameters represented as YAML scalars.
# Keep the environment exports below only as a compatibility fallback; these
# explicit ROS parameters are the authoritative inputs to the wrapper.
ros_broadcast_uri="$(python3 -c 'import json,sys; print(json.dumps(sys.argv[1]))' "$BROADCAST_TX_URI")"
ros_interference_uri="$(python3 -c 'import json,sys; print(json.dumps(sys.argv[1]))' "$INTERFERENCE_TX_URI")"
ros_broadcast_setters="$(python3 -c 'import json,sys; print(json.dumps(sys.argv[1]))' "$broadcast_setters_json")"
ros_interference_setters="$(python3 -c 'import json,sys; print(json.dumps(sys.argv[1]))' "$interference_setters_json")"
ros_sample_rate="$(python3 -c 'import sys; print(float(sys.argv[1]))' "$RM_RADIO_SAMPLE_RATE")"

(
  set -eo pipefail
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  cd "$RM_RADIO_WS"
  export PYTHONPATH="$RM_RADIO_WS/src/rm_radio_ros:${PYTHONPATH:-}"
  export RM_RADIO_RX_ONLY=0
  export RM_RADIO_LAB_TX_ANTENNA_CONFIRM=true
  export RM_RADIO_TX_PREFILL_PACKETS="${RM_RADIO_TX_PREFILL_PACKETS:-0}"
  export RM_RADIO_INTERFERENCE_TX_URI="$BROADCAST_TX_URI"
  export INTERFERENCE_TX_URI="$BROADCAST_TX_URI"
  export RM_RADIO_SETTERS_JSON="$broadcast_setters_json"
  exec python3 -c 'from rm_radio_ros.nodes.ganraoyuan_node import main; main()' \
    --ros-args \
    -r __node:=rm_broadcast_tx_node \
    -p flowgraph_name:=ganraoyuan \
    -p flowgraph_dir:="$flowgraph_dir" \
    -p rx_only:=false \
    -p qt_platform:=offscreen \
    -p disable_gui_sinks:=true \
    -p interference_tx_uri:="$ros_broadcast_uri" \
    -p sample_rate:="$ros_sample_rate" \
    -p setters_json:="$ros_broadcast_setters" \
    -p radio_side:="$side" \
    -p interference_level:="$INTERFERENCE_LEVEL"
) >"$LOG_DIR/broadcast_tx.log" 2>&1 &
broadcast_pid=$!

(
  set -eo pipefail
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  cd "$RM_RADIO_WS"
  export PYTHONPATH="$RM_RADIO_WS/src/rm_radio_ros:${PYTHONPATH:-}"
  export RM_RADIO_RX_ONLY=0
  export RM_RADIO_LAB_TX_ANTENNA_CONFIRM=true
  export RM_RADIO_TX_PREFILL_PACKETS="${RM_RADIO_TX_PREFILL_PACKETS:-0}"
  export RM_RADIO_INTERFERENCE_TX_URI="$INTERFERENCE_TX_URI"
  export INTERFERENCE_TX_URI="$INTERFERENCE_TX_URI"
  export RM_RADIO_SETTERS_JSON="$interference_setters_json"
  exec python3 -c 'from rm_radio_ros.nodes.ganraoyuan_node import main; main()' \
    --ros-args \
    -r __node:=rm_interference_tx_node \
    -p flowgraph_name:=ganraoyuan \
    -p flowgraph_dir:="$flowgraph_dir" \
    -p rx_only:=false \
    -p qt_platform:=offscreen \
    -p disable_gui_sinks:=true \
    -p interference_tx_uri:="$ros_interference_uri" \
    -p sample_rate:="$ros_sample_rate" \
    -p setters_json:="$ros_interference_setters" \
    -p radio_side:="$side" \
    -p interference_level:="$INTERFERENCE_LEVEL"
) >"$LOG_DIR/interference_tx.log" 2>&1 &
interference_pid=$!

cat >"$LOG_DIR/dual_tx.pids" <<EOF
broadcast_tx_pid=$broadcast_pid
interference_tx_pid=$interference_pid
EOF

# A PID can be allocated even when the child fails during Python/GNU Radio
# construction.  Publish the PID file for controllers, then require a short
# stable startup window before declaring the pair ready.
sleep "${RM_RADIO_TX_STARTUP_VERIFY_DELAY_SEC:-0.5}"
if ! lab_tx_verify_dual_running "$$" "$LOG_DIR/dual_tx.pids"; then
  echo "[dual-tx] one or both TX children failed during startup; see $LOG_DIR" >&2
  exit 3
fi

echo "[dual-tx] broadcast PID=$broadcast_pid log=$LOG_DIR/broadcast_tx.log"
echo "[dual-tx] interference PID=$interference_pid log=$LOG_DIR/interference_tx.log"
echo "[dual-tx] stop with: kill $broadcast_pid $interference_pid"

set +e
wait -n "$broadcast_pid" "$interference_pid"
first_exit_rc=$?
set -e
if [[ "$first_exit_rc" -eq 0 ]]; then
  first_exit_rc=3
fi
echo "[dual-tx] a TX child exited unexpectedly (rc=$first_exit_rc); stopping both paths" >&2
exit "$first_exit_rc"
