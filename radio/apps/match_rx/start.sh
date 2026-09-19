#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"
# shellcheck disable=SC1091
source "$RM_RADIO_WS/apps/common/browser_tabs.sh"

for arg in "$@"; do
  case "$arg" in
    --dry-run) export RM_RADIO_DRY_RUN=true REFEREE_DRY_RUN=true ;;
    --no-panel) export RUN_PANEL=false ;;
    --panel-only) export RUN_RX=false RUN_REFEREE=false RUN_PANEL=true RUN_AUTO_PASSWORD=false RUN_RADAR_INTEGRATION=false RUN_RECORDER=false RUN_VISION_BRIDGE=false ;;
    --rx-only) export RUN_RX=true RUN_REFEREE=false RUN_PANEL=false RUN_AUTO_PASSWORD=false RUN_RADAR_INTEGRATION=false RUN_RECORDER=false RUN_VISION_BRIDGE=false ;;
    --native-gui) export RM_RADIO_NATIVE_GUI=true ;;
    --no-native-gui|--headless) export RM_RADIO_NATIVE_GUI=false ;;
    --no-browser) export RM_RADIO_AUTO_OPEN_BROWSER=false ;;
    --help|-h)
      cat <<'EOF'
Usage: apps/match_rx/start.sh [--dry-run] [--no-panel] [--panel-only] [--rx-only]
                              [--native-gui|--no-native-gui] [--no-browser]

Starts the match RX application: dual ANTSDR RX, referee serial bridge, and
the match web panel. Configuration comes from apps/match_rx/config.yaml and may
be overridden with environment variables.

By default the launcher opens the Web spectrum/waterfall panel in a normal
Chrome tab. RX1 uses the continuous Gaussian + M&M decoder, while RX2 uses
the legacy streaming decoder plus its IQ-assisted path. --native-gui is an
optional GNU Radio diagnostic mode, while --no-browser starts the web service
without opening a browser.
When no graphical desktop is reachable the launcher falls back to headless
web spectrum mode instead of aborting the RX application.
EOF
      exit 0
      ;;
  esac
done

RUN_RX="${RUN_RX:-true}"
RUN_REFEREE="${RUN_REFEREE:-true}"
RUN_PANEL="${RUN_PANEL:-true}"
RUN_AUTO_PASSWORD="${RUN_AUTO_PASSWORD:-true}"
RUN_RADAR_INTEGRATION="${RUN_RADAR_INTEGRATION:-true}"
RUN_RECORDER="${RUN_RECORDER:-$RM_RADIO_RECORDING_ENABLED}"

declare -a PIDS=()
declare -a NAMES=()
BROWSER_OPENER_PID=""

MATCH_WRAPPER_PID_FILE="${MATCH_WRAPPER_PID_FILE:-/tmp/rm_radio_match_rx.pid}"
MATCH_WRAPPER_LOCK_FILE="${MATCH_WRAPPER_LOCK_FILE:-/tmp/rm_radio_match_rx.lock}"

unregister_match_wrapper() {
  if [[ -f "$MATCH_WRAPPER_PID_FILE" ]] && [[ "$(tr -d '[:space:]' 2>/dev/null <"$MATCH_WRAPPER_PID_FILE")" == "$$" ]]; then
    rm -f "$MATCH_WRAPPER_PID_FILE"
  fi
}

replace_match_wrapper() {
  command -v flock >/dev/null 2>&1 || { echo "match-rx requires flock" >&2; return 127; }
  local lock_fd old_pid old_cmd
  exec {lock_fd}>"$MATCH_WRAPPER_LOCK_FILE"
  flock -x "$lock_fd"
  old_pid="$(tr -d '[:space:]' 2>/dev/null <"$MATCH_WRAPPER_PID_FILE" || true)"
  if [[ "$old_pid" =~ ^[0-9]+$ && "$old_pid" != "$$" ]] && kill -0 "$old_pid" 2>/dev/null; then
    old_cmd="$(ps -o cmd= -p "$old_pid" 2>/dev/null || true)"
    if [[ "$old_cmd" == *"match_rx/start.sh"* ]]; then
      echo "[match-rx] replacing previous wrapper PID=$old_pid"
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
  printf '%s\n' "$$" >"$MATCH_WRAPPER_PID_FILE"
  flock -u "$lock_fd"
  exec {lock_fd}>&-
}

trap unregister_match_wrapper EXIT
trap 'unregister_match_wrapper; exit 130' INT
trap 'unregister_match_wrapper; exit 143' TERM HUP
replace_match_wrapper

runtime_audit_args=()
if [[ "${RM_RADIO_REQUIRE_LATEST_RUNTIME:-false}" == "true" ]]; then
  runtime_audit_args+=(--require-latest)
fi
python3 "$RM_RADIO_WS/apps/common/check_gnuradio_runtime.py" "${runtime_audit_args[@]}"

# 每次启动都替换旧的 RX 套件；BASHPID/祖先进程过滤保证不会误杀本次启动器。
MATCH_RESIDUAL_PATTERNS=(
  "rx2.launch.py"
  "gfsk.launch.py"
  "referee_serial.launch.py"
  "rm_gfsk_node"
  "rm_gfsk_interference_node"
  "rm_referee_serial_node"
  "rm_match_dashboard"
  "rm_match_recorder"
  "rm_auto_password_node"
  "rm_radar_integration"
  "rm_vision_udp_bridge"
  "optional_algorithm.sh"
  "iio_readdev -u"
  "rm_radio_runtime_logs/match_rx_"
)
export RM_RADIO_KILL_RESIDUAL=true
rm_radio_guard_residual "match-rx" "${MATCH_RESIDUAL_PATTERNS[@]}"

RUN_ID="match_rx_$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${LOG_DIR:-$RM_RADIO_LOG_ROOT/$RUN_ID}"
mkdir -p "$LOG_DIR"

is_true() {
  case "${1,,}" in
    1|true|yes|on) return 0 ;;
    *) return 1 ;;
  esac
}

probe_rx_device() {
  local uri="$1"
  local timeout_sec="${RM_RADIO_RX_PROBE_TIMEOUT_SEC:-3}"
  if ! command -v timeout >/dev/null 2>&1; then
    echo "GNU timeout is unavailable; skip blocking RX probe for $uri" >&2
    return 1
  fi
  timeout --foreground --signal=TERM --kill-after=1s "${timeout_sec}s" \
    iio_info -u "$uri" >/dev/null
}

probe_graphical_session() {
  [[ -n "${DISPLAY:-}" ]] || return 1
  if command -v xdpyinfo >/dev/null 2>&1; then
    xdpyinfo >/dev/null 2>&1
    return $?
  fi
  if command -v xset >/dev/null 2>&1; then
    xset q >/dev/null 2>&1
    return $?
  fi
  local display_number="${DISPLAY#*:}"
  display_number="${display_number%%.*}"
  [[ -S "/tmp/.X11-unix/X${display_number}" ]]
}

prepare_graphical_session() {
  if probe_graphical_session; then
    return 0
  fi

  local original_display="${DISPLAY:-}"
  local original_xauthority="${XAUTHORITY:-}"
  local socket display auth
  for socket in /tmp/.X11-unix/X*; do
    [[ -S "$socket" ]] || continue
    display=":${socket##*X}"
    for auth in \
      "${XAUTHORITY:-}" \
      "/run/user/$(id -u)/gdm/Xauthority" \
      "${HOME:-}/.Xauthority" \
      ""; do
      [[ -n "$auth" && ! -r "$auth" ]] && continue
      export DISPLAY="$display"
      if [[ -n "$auth" ]]; then
        export XAUTHORITY="$auth"
      else
        unset XAUTHORITY
      fi
      if probe_graphical_session; then
        echo "[match-rx] graphical session DISPLAY=$DISPLAY XAUTHORITY=${XAUTHORITY:-<default>}"
        return 0
      fi
    done
  done

  if [[ -n "$original_display" ]]; then export DISPLAY="$original_display"; else unset DISPLAY; fi
  if [[ -n "$original_xauthority" ]]; then export XAUTHORITY="$original_xauthority"; else unset XAUTHORITY; fi
  return 1
}

GRAPHICAL_SESSION_READY=false
if is_true "$RM_RADIO_NATIVE_GUI" || { is_true "$RUN_PANEL" && is_true "$RM_RADIO_AUTO_OPEN_BROWSER"; }; then
  if prepare_graphical_session; then
    GRAPHICAL_SESSION_READY=true
  fi
fi

NATIVE_GUI_ACTIVE=false
if is_true "$RUN_RX" && is_true "$RM_RADIO_NATIVE_GUI" && [[ "$GRAPHICAL_SESSION_READY" == "true" ]]; then
  NATIVE_GUI_ACTIVE=true
  DISABLE_GUI_SINKS=false
  RADIO_QT_PLATFORM="${RM_RADIO_QT_PLATFORM:-xcb}"
  [[ "$RADIO_QT_PLATFORM" == "offscreen" ]] && RADIO_QT_PLATFORM=xcb
  # RX1 cannot be opened by iio_readdev and the GNU Radio IIO source at the
  # same time. Native mode keeps one streaming source and feeds both decoding
  # and Qt sinks from it.
  export BROADCAST_IIO_CAPTURE_DECODER_ENABLED=false
  export BROADCAST_IIO_CAPTURE_DECODER_ONLY=false
  echo "[match-rx] GNU Radio native spectrum enabled (RX1 + RX2 Qt windows)"
else
  DISABLE_GUI_SINKS=true
  RADIO_QT_PLATFORM=offscreen
  if is_true "$RUN_RX" && is_true "$RM_RADIO_NATIVE_GUI"; then
    echo "[match-rx][WARN] no reachable graphical session; using headless web spectrum" >&2
  fi
fi

start_service() {
  local name="$1"
  shift
  local log_file="$LOG_DIR/${name}.log"
  local service_pid
  echo "[match-rx] starting $name -> $log_file"
  (
    set -Eeuo pipefail
    cd "$RM_RADIO_WS"
    # Every child owns a process group, so Ctrl+C/TERM can stop the complete
    # ros2 launch tree instead of leaving grandchildren behind.
    exec setsid "$@"
  ) >"$log_file" 2>&1 &
  service_pid=$!
  PIDS+=("$service_pid")
  NAMES+=("$name")
  # The Dashboard starts after these services.  Tell it which process groups
  # are already owned by this wrapper so its optional standalone controls do
  # not create a second RX/referee stack.
  case "$name" in
    rx) export RM_RADIO_SUPERVISED_RX_PID="$service_pid" ;;
    referee) export RM_RADIO_SUPERVISED_REFEREE_PID="$service_pid" ;;
  esac
}

open_panel_when_ready() {
  local url="http://$RM_RADIO_APP_HOST:$RM_RADIO_APP_PORT/"
  if ! is_true "$RM_RADIO_AUTO_OPEN_BROWSER"; then
    return 0
  fi
  if [[ "$GRAPHICAL_SESSION_READY" != "true" ]]; then
    echo "[match-rx][WARN] web panel is running at $url, but no desktop browser session is reachable" >&2
    return 0
  fi
  (
    set +e
    rm_radio_wait_and_open_browser_tab "$url"
  ) >"$LOG_DIR/browser.log" 2>&1 &
  BROWSER_OPENER_PID=$!
  echo "[match-rx] browser auto-open scheduled -> $LOG_DIR/browser.log"
}

stop_all() {
  trap - EXIT INT TERM HUP
  local pid
  if [[ "$BROWSER_OPENER_PID" =~ ^[0-9]+$ ]] && {
    kill -0 "$BROWSER_OPENER_PID" >/dev/null 2>&1;
  }; then
    # This PID is only the short readiness waiter. Chrome is launched in its
    # own normal session and is never owned or terminated by this launcher.
    kill -TERM "$BROWSER_OPENER_PID" >/dev/null 2>&1 || true
    for _ in $(seq 1 20); do
      if ! kill -0 "$BROWSER_OPENER_PID" >/dev/null 2>&1; then
        break
      fi
      sleep 0.1
    done
    kill -KILL "$BROWSER_OPENER_PID" >/dev/null 2>&1 || true
    wait "$BROWSER_OPENER_PID" 2>/dev/null || true
  fi
  for pid in "${PIDS[@]:-}"; do
    if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" >/dev/null 2>&1; then
      kill -TERM -- "-$pid" >/dev/null 2>&1 || kill -TERM "$pid" >/dev/null 2>&1 || true
    fi
  done
  local deadline=$((SECONDS + 5))
  local running
  while ((SECONDS < deadline)); do
    running=false
    for pid in "${PIDS[@]:-}"; do
      if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" >/dev/null 2>&1; then
        running=true
        break
      fi
    done
    [[ "$running" == "false" ]] && break
    sleep 0.1
  done
  for pid in "${PIDS[@]:-}"; do
    if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" >/dev/null 2>&1; then
      kill -KILL -- "-$pid" >/dev/null 2>&1 || kill -KILL "$pid" >/dev/null 2>&1 || true
    fi
    [[ "$pid" =~ ^[0-9]+$ ]] && wait "$pid" 2>/dev/null || true
  done
  # A child can fork between the first PID scan and signal delivery. Re-scan
  # the complete RX scope once more before declaring shutdown complete.
  rm_radio_guard_residual "match-rx-cleanup" "${MATCH_RESIDUAL_PATTERNS[@]}" || true
  unregister_match_wrapper
}

trap stop_all EXIT
trap 'stop_all; exit 130' INT
trap 'stop_all; exit 143' TERM HUP

if is_true "$RUN_RX" && [[ "${BROADCAST_RX_GAIN_MODE,,}" != "manual" ]]; then
  echo "[match-rx] RX1 信息波接收机必须使用 manual；当前 BROADCAST_RX_GAIN_MODE=$BROADCAST_RX_GAIN_MODE" >&2
  exit 2
fi

if is_true "$RUN_RX" && ! python3 -c '
import math, sys
try:
    gain, limit = map(float, sys.argv[1:3])
except (TypeError, ValueError):
    raise SystemExit(1)
raise SystemExit(0 if math.isfinite(gain) and math.isfinite(limit) and -1.0 <= gain <= limit <= 73.0 else 1)
' "$BROADCAST_RX_GAIN" "$BROADCAST_RX_GAIN_MAX"; then
  echo "[match-rx] RX1 增益必须满足 -1 <= BROADCAST_RX_GAIN <= BROADCAST_RX_GAIN_MAX <= 73 dB；当前 gain=$BROADCAST_RX_GAIN max=$BROADCAST_RX_GAIN_MAX" >&2
  exit 2
fi

broadcast_freq=433200000
rx1_setters="{\"cen_f\":$broadcast_freq,\"center_F\":$broadcast_freq,\"BW\":540000,\"LowPass\":$BROADCAST_RX_LOW_PASS_HZ,\"bw_re\":540000,\"sen_re\":0.7814,\"rx_tracking\":true,\"GainMode\":\"$BROADCAST_RX_GAIN_MODE\",\"Gain\":$BROADCAST_RX_GAIN}"
if [[ "${RM_RADIO_SIDE,,}" == "blue" ]]; then
  broadcast_freq=433920000
  rx1_setters="{\"cen_f\":$broadcast_freq,\"center_F\":$broadcast_freq,\"BW\":540000,\"LowPass\":$BROADCAST_RX_LOW_PASS_HZ,\"bw_re\":540000,\"sen_re\":0.7814,\"rx_tracking\":true,\"GainMode\":\"$BROADCAST_RX_GAIN_MODE\",\"Gain\":$BROADCAST_RX_GAIN}"
fi

if [[ "${RM_RADIO_SIDE,,}" == "red" ]]; then
  case "$INTERFERENCE_LEVEL" in
    1) int_freq=432200000; int_bw=940000; int_lp=500000; int_sen=1.4097 ;;
    2) int_freq=432500000; int_bw=860000; int_lp=500000; int_sen=1.28405 ;;
    3) int_freq=432800000; int_bw=250000; int_lp=160000; int_sen=0.32585 ;;
    *) echo "INTERFERENCE_LEVEL must be 1, 2, or 3" >&2; exit 2 ;;
  esac
elif [[ "${RM_RADIO_SIDE,,}" == "blue" ]]; then
  case "$INTERFERENCE_LEVEL" in
    1) int_freq=434920000; int_bw=940000; int_lp=500000; int_sen=1.4097 ;;
    2) int_freq=434620000; int_bw=860000; int_lp=500000; int_sen=1.28405 ;;
    3) int_freq=434320000; int_bw=250000; int_lp=160000; int_sen=0.32585 ;;
    *) echo "INTERFERENCE_LEVEL must be 1, 2, or 3" >&2; exit 2 ;;
  esac
else
  echo "RM_RADIO_SIDE must be red or blue" >&2
  exit 2
fi
if [[ "${INTERFERENCE_FRONTEND_PROFILE,,}" == "broadcast" ]]; then
  # Experimental A/B mode: retain the interference RF frequency and protocol,
  # but use the information receiver's analog bandwidth, filter and sensitivity.
  int_bw=540000
  int_lp="$BROADCAST_RX_LOW_PASS_HZ"
  int_sen=0.7814
fi
rx2_setters="{\"cen_f\":$int_freq,\"center_F\":$int_freq,\"BW\":$int_bw,\"LowPass\":$int_lp,\"bw_re\":$int_bw,\"sncy\":\"sncy_gr\",\"sen_re\":$int_sen,\"rx_tracking\":true,\"GainMode\":\"$INTERFERENCE_RX_GAIN_MODE\",\"Gain\":$INTERFERENCE_RX_GAIN}"

if is_true "$RUN_RX"; then
  # GNU Radio 的 fmcomms2 source 不设置 rf_port_select，E310 会沿用设备残留端口。
  # 433MHz 必须走 5M-3G 路径(B_BALANCED)，因此启动 flowgraph 前先确定性预设两台 RX 端口。
  if ! is_true "$RM_RADIO_DRY_RUN"; then
    # shellcheck disable=SC1091
    source "$RM_RADIO_WS/apps/common/iio_init.sh"
    echo "[match-rx] 预初始化并回读确认 RX1 信息波: $RX1_URI -> ${RX_RF_PORT:-B_BALANCED}, gain_mode=manual gain=${BROADCAST_RX_GAIN}dB"
    if ! probe_rx_device "$RX1_URI" >"$LOG_DIR/rx1_preinit.log" 2>&1; then
      echo "[match-rx][WARN] RX1 信息波未连接或探测超时；跳过预初始化。ROS、裁判系统、视觉通信和面板继续启动，见 $LOG_DIR/rx1_preinit.log" >&2
    elif ! init_ad9361_rx "$RX1_URI" "$broadcast_freq" 540000 manual "$BROADCAST_RX_GAIN" "${RX_RF_PORT:-B_BALANCED}" defer \
      >>"$LOG_DIR/rx1_preinit.log" 2>&1; then
      echo "[match-rx][WARN] RX1 信息波 manual/${BROADCAST_RX_GAIN}dB 硬件回读失败；以设备降级状态继续启动，见 $LOG_DIR/rx1_preinit.log" >&2
    else
      echo "[match-rx] RX1 信息波 manual/${BROADCAST_RX_GAIN}dB 已由硬件回读确认（$LOG_DIR/rx1_preinit.log）"
    fi
    if [[ "${RM_RADIO_RX_SKIP_PREINIT:-false}" != "true" ]]; then
      echo "[match-rx] 预初始化 RX2 端口 $RX2_URI -> ${RX_RF_PORT:-B_BALANCED}"
      if ! probe_rx_device "$RX2_URI" >"$LOG_DIR/rx2_preinit.log" 2>&1; then
        echo "[match-rx][WARN] RX2 干扰波未连接或探测超时；跳过预初始化并继续启动，见 $LOG_DIR/rx2_preinit.log" >&2
      elif ! init_ad9361_rx "$RX2_URI" "$int_freq" "$int_bw" "$INTERFERENCE_RX_GAIN_MODE" "$INTERFERENCE_RX_GAIN" "${RX_RF_PORT:-B_BALANCED}" defer \
        >>"$LOG_DIR/rx2_preinit.log" 2>&1; then
        echo "[match-rx][WARN] RX2 端口预初始化/回读失败；以设备降级状态继续启动，见 $LOG_DIR/rx2_preinit.log" >&2
      fi
    else
      echo "[match-rx] RM_RADIO_RX_SKIP_PREINIT=true：跳过 RX2 预初始化。"
    fi
  fi
  start_service rx ros2 launch rm_radio_ros rx2.launch.py \
    dry_run:="$RM_RADIO_DRY_RUN" \
    radio_side:="$RM_RADIO_SIDE" \
    rx1_uri:="$RX1_URI" \
    rx2_uri:="$RX2_URI" \
    interference_level:="$INTERFERENCE_LEVEL" \
    broadcast_setters_json:="$rx1_setters" \
    interference_setters_json:="$rx2_setters" \
    broadcast_demod_mode:="$BROADCAST_DEMOD_MODE" \
    interference_demod_mode:="$INTERFERENCE_DEMOD_MODE" \
    broadcast_frontend_profile:="$BROADCAST_FRONTEND_PROFILE" \
    interference_frontend_profile:="$INTERFERENCE_FRONTEND_PROFILE" \
    broadcast_input_sps:="$BROADCAST_INPUT_SPS" \
    interference_input_sps:="$INTERFERENCE_INPUT_SPS" \
    broadcast_iq_decoder_enabled:="$BROADCAST_IQ_DECODER_ENABLED" \
    interference_iq_decoder_enabled:="$INTERFERENCE_IQ_DECODER_ENABLED" \
    iio_capture_decoder_enabled:="$BROADCAST_IIO_CAPTURE_DECODER_ENABLED" \
    iio_capture_decoder_only:="$BROADCAST_IIO_CAPTURE_DECODER_ONLY" \
    iio_capture_seconds:="$BROADCAST_IIO_CAPTURE_SECONDS" \
    iio_capture_period_sec:="$BROADCAST_IIO_CAPTURE_PERIOD_SEC" \
    iio_capture_start_delay_sec:="$BROADCAST_IIO_CAPTURE_START_DELAY_SEC" \
    broadcast_iq_decoder_autotune_mode:="$BROADCAST_IQ_DECODER_AUTOTUNE_MODE" \
    broadcast_iq_decoder_low_pass_hz:="$BROADCAST_IQ_DECODER_LOW_PASS_HZ" \
    broadcast_iq_decoder_sps_values:="$BROADCAST_IQ_DECODER_SPS_VALUES" \
    broadcast_iq_decoder_offset_step:="$BROADCAST_IQ_DECODER_OFFSET_STEP" \
    broadcast_iq_decoder_max_payloads:="$BROADCAST_IQ_DECODER_MAX_PAYLOADS" \
    broadcast_iq_decoder_max_frames:="$BROADCAST_IQ_DECODER_MAX_FRAMES" \
    interference_iq_decoder_autotune_mode:="$INTERFERENCE_IQ_DECODER_AUTOTUNE_MODE" \
    interference_iq_decoder_low_pass_hz:="$INTERFERENCE_IQ_DECODER_LOW_PASS_HZ" \
    spectrum_period_sec:="$SPECTRUM_PERIOD_SEC" \
    spectrum_fft_size:="$SPECTRUM_FFT_SIZE" \
    spectrum_bin_count:="$SPECTRUM_BIN_COUNT" \
    recording_enabled:="$RUN_RECORDER" \
    recording_root:="$RM_RADIO_RECORDING_IQ_ROOT" \
    recording_control_topic:="$RM_RADIO_RECORDING_CONTROL_TOPIC" \
    recording_output_sample_rate:="$RM_RADIO_RECORDING_OUTPUT_SAMPLE_RATE" \
    recording_segment_sec:="$RM_RADIO_RECORDING_SEGMENT_SEC" \
    recording_queue_chunks:="$RM_RADIO_RECORDING_QUEUE_CHUNKS" \
    recording_min_free_gb:="$RM_RADIO_RECORDING_IQ_MIN_FREE_GB" \
    recording_iq_compression:="$RM_RADIO_RECORDING_IQ_COMPRESSION" \
    recording_iq_compression_level:="$RM_RADIO_RECORDING_IQ_COMPRESSION_LEVEL" \
    disable_gui_sinks:="$DISABLE_GUI_SINKS" \
    qt_platform:="$RADIO_QT_PLATFORM"
fi

if is_true "$RUN_RECORDER"; then
  start_service recorder ros2 run rm_radio_ros rm_match_recorder --ros-args \
    -p enabled:=true \
    -p auto_start:="$RM_RADIO_RECORDING_AUTO_START" \
    -p record_root:="$RM_RADIO_RECORDING_ROOT" \
    -p iq_record_root:="$RM_RADIO_RECORDING_IQ_ROOT" \
    -p match_run_id:="$RM_RADIO_MATCH_RUN_ID" \
    -p config_path:="$MATCH_RX_CONFIG" \
    -p game_status_topic:="$RM_RADIO_RECORDING_GAME_STATUS_TOPIC" \
    -p control_topic:="$RM_RADIO_RECORDING_CONTROL_TOPIC" \
    -p start_confirm_frames:="$RM_RADIO_RECORDING_START_CONFIRM_FRAMES" \
    -p stop_confirm_frames:="$RM_RADIO_RECORDING_STOP_CONFIRM_FRAMES" \
    -p stop_game_progress:="$RM_RADIO_RECORDING_STOP_GAME_PROGRESS" \
    -p post_roll_sec:="$RM_RADIO_RECORDING_POST_ROLL_SEC" \
    -p max_duration_sec:="$RM_RADIO_RECORDING_MAX_DURATION_SEC" \
    -p finalize_timeout_sec:="$RM_RADIO_RECORDING_FINALIZE_TIMEOUT_SEC" \
    -p min_free_gb:="$RM_RADIO_RECORDING_EVENT_MIN_FREE_GB" \
    -p event_topics:="$RM_RADIO_RECORDING_EVENT_TOPICS" \
    -p event_compression:="$RM_RADIO_RECORDING_EVENT_COMPRESSION" \
    -p event_compression_level:="$RM_RADIO_RECORDING_EVENT_COMPRESSION_LEVEL" \
    -p event_queue_size:="$RM_RADIO_RECORDING_EVENT_QUEUE_SIZE"
fi

if is_true "$RUN_REFEREE"; then
  start_service referee ros2 launch rm_radio_ros referee_serial.launch.py \
    dry_run:="$REFEREE_DRY_RUN" \
    port:="$REFEREE_PORT" \
    baudrate:="$REFEREE_BAUDRATE" \
    sender_id:="$RADAR_SENDER_ID" \
    receiver_id:="$REFEREE_RECEIVER_ID" \
    bridge_topic:="$REFEREE_BRIDGE_TOPIC" \
    radar_cmd_topic:="$RADAR_CMD_TOPIC" \
    require_serial_open_on_start:="$REQUIRE_REFEREE_SERIAL_OPEN_ON_START" \
    frame_timeout_sec:="$REFEREE_FRAME_TIMEOUT_SEC" \
    auto_send_invincible_targets:="$AUTO_SEND_INVINCIBLE_TARGETS" \
    invincible_targets_data_cmd_id:="$INVINCIBLE_TARGETS_DATA_CMD_ID" \
    invincible_targets_send_rate_hz:="$INVINCIBLE_TARGETS_SEND_RATE_HZ" \
    invincible_targets_freshness_sec:="$INVINCIBLE_TARGETS_FRESHNESS_SEC" \
    auto_interference_level:=false \
    auto_rx_interference_level:=true \
    apply_referee_level_only_when_running:=false \
    radio_side:="$RM_RADIO_SIDE" \
    interference_level:="$INTERFERENCE_LEVEL"
fi

if is_true "$RUN_AUTO_PASSWORD"; then
  start_service auto_password ros2 run rm_radio_ros rm_auto_password_node --ros-args \
    -p status_topic:=/rm_referee_serial_node/status \
    -p send_topic:=/rm_referee_serial_node/send_radar_cmd
fi

if is_true "$RUN_RADAR_INTEGRATION"; then
  start_service radar_integration ros2 run rm_radio_ros rm_radar_integration --ros-args \
    -p radio_side:="$RM_RADIO_SIDE" \
    -p auto_sync_side:="$RADAR_AUTO_SYNC_SIDE" \
    -p telemetry_topic:="$RADAR_TELEMETRY_TOPIC" \
    -p send_rate_hz:="$RADAR_INTEGRATION_RATE_HZ" \
    -p state_rate_hz:="$RADAR_STATE_RATE_HZ" \
    -p vision_timeout_sec:="$RADAR_VISION_TIMEOUT_SEC" \
    -p radio_timeout_sec:="$RADAR_RADIO_TIMEOUT_SEC" \
    -p radio_ghost_sec:="$RADAR_RADIO_GHOST_SEC" \
    -p radar_cmd_topic:="$RADAR_CMD_TOPIC" \
    -p auto_double_vulnerability_enabled:="$RADAR_AUTO_DOUBLE_ENABLED" \
    -p auto_double_vulnerability_mask_path:="$RADAR_AUTO_DOUBLE_MASK_PATH" \
    -p auto_double_vulnerability_zone_dwell_sec:="$RADAR_AUTO_DOUBLE_ZONE_DWELL_SEC" \
    -p auto_double_vulnerability_hero_hp_drop_amount:="$RADAR_AUTO_DOUBLE_HERO_HP_DROP_AMOUNT" \
    -p auto_double_vulnerability_hero_hp_window_sec:="$RADAR_AUTO_DOUBLE_HERO_HP_WINDOW_SEC" \
    -p auto_double_vulnerability_endgame_sec:="$RADAR_AUTO_DOUBLE_ENDGAME_SEC" \
    -p auto_double_vulnerability_referee_timeout_sec:="$RADAR_AUTO_DOUBLE_REFEREE_TIMEOUT_SEC"
fi

if is_true "$RUN_VISION_BRIDGE"; then
  start_service vision_bridge ros2 run rm_radio_ros rm_vision_udp_bridge --ros-args \
    -p listen_host:="$VISION_BRIDGE_LISTEN_HOST" \
    -p listen_port:="$VISION_BRIDGE_LISTEN_PORT" \
    -p vision_host:="$VISION_BRIDGE_TARGET_HOST" \
    -p vision_port:="$VISION_BRIDGE_TARGET_PORT" \
    -p telemetry_topic:="$RADAR_TELEMETRY_TOPIC" \
    -p radar_cmd_topic:="$RADAR_CMD_TOPIC"
fi

if is_true "$RUN_PANEL"; then
  export RM_RADIO_MATCH_WEB_ROOT="$MATCH_RX_DIR/web"
  export RM_RADIO_RUNTIME_LOG_DIR="$LOG_DIR"
  start_service panel ros2 run rm_radio_ros rm_match_dashboard --host "$RM_RADIO_APP_HOST" --port "$RM_RADIO_APP_PORT"
  echo "[match-rx] panel http://$RM_RADIO_APP_HOST:$RM_RADIO_APP_PORT/"
  open_panel_when_ready
fi

if is_true "$RUN_ALGORITHM"; then
  start_service algorithm "$SCRIPT_DIR/optional_algorithm.sh"
fi

echo "[match-rx] logs: $LOG_DIR"
while true; do
  for i in "${!PIDS[@]}"; do
    pid="${PIDS[$i]}"
    name="${NAMES[$i]}"
    [[ "$pid" == "0" ]] && continue
    if ! kill -0 "$pid" >/dev/null 2>&1; then
      status=0
      wait "$pid" || status=$?
      if [[ "$name" == "referee" ]] && is_true "$REQUIRE_REFEREE_SERIAL_OPEN_ON_START"; then
        echo "[match-rx][ERROR] referee fail-fast exited with $status. See $LOG_DIR/${name}.log" >&2
        exit "${status:-1}"
      elif [[ "$name" == "referee" || "$name" == "algorithm" ]]; then
        echo "[match-rx][WARN] $name exited with $status; other modules continue. See $LOG_DIR/${name}.log"
        PIDS[$i]=0
      elif [[ "$name" == "recorder" ]]; then
        echo "[match-rx][WARN] $name exited with $status; other modules continue. See $LOG_DIR/${name}.log"
        PIDS[$i]=0
      else
        echo "[match-rx][ERROR] $name exited with $status. See $LOG_DIR/${name}.log" >&2
        exit "$status"
      fi
    fi
  done
  sleep 1
done
