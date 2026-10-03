#!/usr/bin/env bash
set -euo pipefail

RADIO_TX_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$RADIO_TX_SCRIPT_DIR/env.sh"
# shellcheck disable=SC1091
source "$RM_RADIO_WS/apps/common/lab_tx_safety.sh"
# shellcheck disable=SC1091
source "$RM_RADIO_WS/apps/common/browser_tabs.sh"

export RM_RADIO_DASHBOARD_HOST="${RM_RADIO_DASHBOARD_HOST:-$(yaml_get app.host 127.0.0.1)}"
export RM_RADIO_DASHBOARD_PORT="${RM_RADIO_DASHBOARD_PORT:-${RADIO_TX_DASHBOARD_PORT:-$(yaml_get app.port 8780)}}"
export RADIO_TX_DASHBOARD_HOST="$RM_RADIO_DASHBOARD_HOST"
export RADIO_TX_DASHBOARD_PORT="$RM_RADIO_DASHBOARD_PORT"

LAB_TX_AUTO_OPEN_BROWSER="$(yaml_get app.auto_open_browser true)"
dashboard_host="$RM_RADIO_DASHBOARD_HOST"
dashboard_port="$RM_RADIO_DASHBOARD_PORT"
while (($#)); do
  case "$1" in
    --no-browser)
      LAB_TX_AUTO_OPEN_BROWSER=false
      shift
      ;;
    --host)
      [[ $# -ge 2 ]] || { echo "--host requires a value" >&2; exit 2; }
      dashboard_host="$2"
      shift 2
      ;;
    --host=*)
      dashboard_host="${1#*=}"
      shift
      ;;
    --port)
      [[ $# -ge 2 ]] || { echo "--port requires a value" >&2; exit 2; }
      dashboard_port="$2"
      shift 2
      ;;
    --port=*)
      dashboard_port="${1#*=}"
      shift
      ;;
    --help|-h)
      echo "Usage: apps/lab_tx/start.sh [--host HOST] [--port PORT] [--no-browser]"
      exit 0
      ;;
    *)
      echo "unknown LabTX dashboard option: $1" >&2
      exit 2
      ;;
  esac
done

LAB_DASHBOARD_PID_FILE="${LAB_DASHBOARD_PID_FILE:-/tmp/rm_radio_lab_tx_dashboard.pid}"
LAB_DASHBOARD_LOCK_FILE="${LAB_DASHBOARD_LOCK_FILE:-/tmp/rm_radio_lab_tx_dashboard.lock}"

unregister_lab_dashboard_wrapper() {
  if [[ -f "$LAB_DASHBOARD_PID_FILE" ]] && [[ "$(tr -d '[:space:]' 2>/dev/null <"$LAB_DASHBOARD_PID_FILE")" == "$$" ]]; then
    rm -f "$LAB_DASHBOARD_PID_FILE"
  fi
}

replace_lab_dashboard_wrapper() {
  command -v flock >/dev/null 2>&1 || { echo "lab-tx-dashboard requires flock" >&2; return 127; }
  local lock_fd old_pid old_cmd
  exec {lock_fd}>"$LAB_DASHBOARD_LOCK_FILE"
  flock -x "$lock_fd"
  old_pid="$(tr -d '[:space:]' 2>/dev/null <"$LAB_DASHBOARD_PID_FILE" || true)"
  if [[ "$old_pid" =~ ^[0-9]+$ && "$old_pid" != "$$" ]] && kill -0 "$old_pid" 2>/dev/null; then
    old_cmd="$(ps -o cmd= -p "$old_pid" 2>/dev/null || true)"
    if [[ "$old_cmd" == *"lab_tx/start_dashboard.sh"* ]]; then
      echo "[lab-tx-dashboard] replacing previous wrapper PID=$old_pid"
      kill -TERM "$old_pid" 2>/dev/null || true
      for _ in $(seq 1 120); do
        kill -0 "$old_pid" 2>/dev/null || break
        sleep 0.1
      done
      if kill -0 "$old_pid" 2>/dev/null; then
        kill -KILL "$old_pid" 2>/dev/null || true
        sleep 0.2
      fi
    fi
  fi
  printf '%s\n' "$$" >"$LAB_DASHBOARD_PID_FILE"
  flock -u "$lock_fd"
  exec {lock_fd}>&-
}

trap unregister_lab_dashboard_wrapper EXIT
trap 'unregister_lab_dashboard_wrapper; exit 130' INT
trap 'unregister_lab_dashboard_wrapper; exit 143' TERM HUP
replace_lab_dashboard_wrapper

TX_DASHBOARD_RESIDUAL_PATTERNS=(
  "radio_tx_dashboard.py"
  "start_dual_tx.sh"
  "rm_broadcast_tx_node"
  "rm_interference_tx_node"
  "ganraoyuan_node"
  "flowgraph_name:=ganraoyuan"
  "iq_replay_tx_node"
  "iio_writedev"
  "rm_radio_cw_test"
)

# A saved TX may be physically unplugged while the other one is being worked
# on.  Mute every currently reachable configured device, while retaining the
# disconnected device's stable URI in the panel for the next reconnect.
lab_tx_hard_mute_reachable_configured() {
  local uri
  local -a reachable=()
  local -A seen=()
  for uri in "$@"; do
    [[ -n "$uri" && -z "${seen[$uri]:-}" ]] || continue
    seen[$uri]=1
    if timeout 3s iio_info -u "$uri" >/dev/null 2>&1; then
      reachable+=("$uri")
    else
      echo "[lab-tx-dashboard] TX 当前不可达，保留配置但跳过硬件静音：$uri" >&2
    fi
  done
  if ((${#reachable[@]})); then
    lab_tx_hard_mute_and_verify "${reachable[@]}"
  fi
}

# Starting the TX panel means replacing the complete previous LabTX runtime.
export RM_RADIO_KILL_RESIDUAL=true
rm_radio_guard_residual "lab-tx-dashboard" "${TX_DASHBOARD_RESIDUAL_PATTERNS[@]}"

# A stale process may have enabled either LO before it died. Never expose the
# new panel until both TX devices are confirmed at maximum attenuation/powerdown.
lab_tx_hard_mute_reachable_configured "$BROADCAST_TX_URI" "$INTERFERENCE_TX_URI"

cd "$RM_RADIO_WS"
export PYTHONPATH="$RM_RADIO_WS/src/rm_radio_ros:${PYTHONPATH:-}"

dashboard_pid=""
browser_opener_pid=""

cleanup_dashboard() {
  local exit_code=$?
  trap - EXIT INT TERM HUP
  set +e
  if [[ "$browser_opener_pid" =~ ^[0-9]+$ ]] && kill -0 "$browser_opener_pid" >/dev/null 2>&1; then
    # Only stop the readiness waiter. The normal Chrome process is detached
    # and deliberately remains outside the dashboard lifecycle.
    kill -TERM "$browser_opener_pid" >/dev/null 2>&1 || true
    wait "$browser_opener_pid" 2>/dev/null || true
  fi
  if [[ "$dashboard_pid" =~ ^[0-9]+$ ]] && kill -0 "$dashboard_pid" >/dev/null 2>&1; then
    kill -TERM -- "-$dashboard_pid" >/dev/null 2>&1 || kill -TERM "$dashboard_pid" >/dev/null 2>&1 || true
    lab_tx_stop_pids "$dashboard_pid"
  fi
  lab_tx_cleanup_residual_tx "${TX_DASHBOARD_RESIDUAL_PATTERNS[@]}"
  sleep 0.4
  lab_tx_hard_mute_reachable_configured "$BROADCAST_TX_URI" "$INTERFERENCE_TX_URI"
  unregister_lab_dashboard_wrapper
  exit "$exit_code"
}

trap cleanup_dashboard EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

setsid python3 "$RADIO_TX_SCRIPT_DIR/radio_tx_dashboard.py" \
  --host "$dashboard_host" \
  --port "$dashboard_port" &
dashboard_pid=$!
if [[ "${LAB_TX_AUTO_OPEN_BROWSER,,}" =~ ^(1|true|yes|on)$ ]]; then
  browser_host="$dashboard_host"
  [[ "$browser_host" == "0.0.0.0" || "$browser_host" == "::" ]] && browser_host="127.0.0.1"
  browser_log_dir="${RM_RADIO_LOG_ROOT:-/tmp/rm_radio_runtime_logs}"
  mkdir -p "$browser_log_dir"
  (
    rm_radio_wait_and_open_browser_tab "http://$browser_host:$dashboard_port/"
  ) >"$browser_log_dir/lab_tx_browser.log" 2>&1 &
  browser_opener_pid=$!
fi
wait "$dashboard_pid"
