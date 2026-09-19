#!/usr/bin/env bash
set -euo pipefail

MATCH_RX_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export MATCH_RX_DIR
export RM_RADIO_WS="${RM_RADIO_WS:-$(cd "$MATCH_RX_DIR/../.." && pwd)}"
export MATCH_RX_CONFIG="${MATCH_RX_CONFIG:-$MATCH_RX_DIR/config.yaml}"

# Hidden local.env files made deployments silently diverge. A caller may still
# opt into an explicit, visible one-shot override file when diagnosing hardware.
if [[ -n "${MATCH_RX_ENV_FILE:-}" ]]; then
  # shellcheck disable=SC1090
  source "$MATCH_RX_ENV_FILE"
fi

yaml_get() {
  python3 "$RM_RADIO_WS/apps/common/read_yaml.py" "$MATCH_RX_CONFIG" "$1" "${2:-}"
}

export RM_RADIO_APP_HOST="${RM_RADIO_APP_HOST:-$(yaml_get app.host 127.0.0.1)}"
export RM_RADIO_APP_PORT="${RM_RADIO_APP_PORT:-$(yaml_get app.port 8765)}"
export RM_RADIO_LOG_ROOT="${RM_RADIO_LOG_ROOT:-$(yaml_get app.log_root '~/.local/state/shark-radio/runtime-logs')}"
RM_RADIO_LOG_ROOT="${RM_RADIO_LOG_ROOT/#\~\//$HOME/}"
export RM_RADIO_LOG_ROOT
export RM_RADIO_DRY_RUN="${RM_RADIO_DRY_RUN:-$(yaml_get app.dry_run false)}"
export RM_RADIO_AUTO_OPEN_BROWSER="${RM_RADIO_AUTO_OPEN_BROWSER:-$(yaml_get app.auto_open_browser true)}"

export RM_RADIO_SIDE="${RM_RADIO_SIDE:-$(yaml_get radio.side red)}"
export INTERFERENCE_LEVEL="${INTERFERENCE_LEVEL:-$(yaml_get radio.interference_level 1)}"
export RM_RADIO_INTELLIGENT_MODE="${RM_RADIO_INTELLIGENT_MODE:-$(yaml_get radio.intelligent_mode true)}"
export RX1_URI="${RX1_URI:-$(yaml_get radio.rx1_uri ip:192.168.9.110)}"
export RX2_URI="${RX2_URI:-$(yaml_get radio.rx2_uri ip:192.168.9.111)}"
export RM_RADIO_SAMPLE_RATE="${RM_RADIO_SAMPLE_RATE:-$(yaml_get radio.sample_rate 2000000)}"
export RX_RF_PORT="${RX_RF_PORT:-$(yaml_get radio.rx_rf_port B_BALANCED)}"
export BROADCAST_RX_GAIN_MODE="${BROADCAST_RX_GAIN_MODE:-$(yaml_get radio.broadcast_gain_mode manual)}"
export BROADCAST_RX_GAIN="${BROADCAST_RX_GAIN:-$(yaml_get radio.broadcast_gain 20)}"
export BROADCAST_RX_GAIN_MAX="${BROADCAST_RX_GAIN_MAX:-$(yaml_get radio.broadcast_gain_max 73)}"
export BROADCAST_RX_LOW_PASS_HZ="${BROADCAST_RX_LOW_PASS_HZ:-$(yaml_get radio.broadcast_low_pass_hz 260000)}"
export INTERFERENCE_RX_GAIN_MODE="${INTERFERENCE_RX_GAIN_MODE:-$(yaml_get radio.interference_gain_mode fast_attack)}"
export INTERFERENCE_RX_GAIN="${INTERFERENCE_RX_GAIN:-$(yaml_get radio.interference_gain 20)}"
export RM_RADIO_START_AFTER_MATCH_RUNNING="${RM_RADIO_START_AFTER_MATCH_RUNNING:-$(yaml_get radio.start_after_match_running false)}"

export BROADCAST_DEMOD_MODE="${BROADCAST_DEMOD_MODE:-$(yaml_get decoder.broadcast.demod_mode mm)}"
export INTERFERENCE_DEMOD_MODE="${INTERFERENCE_DEMOD_MODE:-$(yaml_get decoder.interference.demod_mode legacy)}"
export BROADCAST_FRONTEND_PROFILE="${BROADCAST_FRONTEND_PROFILE:-$(yaml_get decoder.broadcast.frontend_profile broadcast)}"
export INTERFERENCE_FRONTEND_PROFILE="${INTERFERENCE_FRONTEND_PROFILE:-$(yaml_get decoder.interference.frontend_profile interference)}"
export BROADCAST_INPUT_SPS="${BROADCAST_INPUT_SPS:-$(yaml_get decoder.broadcast.input_sps 94)}"
export INTERFERENCE_INPUT_SPS="${INTERFERENCE_INPUT_SPS:-$(yaml_get decoder.interference.input_sps 94)}"
export BROADCAST_IQ_DECODER_ENABLED="${BROADCAST_IQ_DECODER_ENABLED:-$(yaml_get decoder.broadcast.iq_snapshot_enabled false)}"
export INTERFERENCE_IQ_DECODER_ENABLED="${INTERFERENCE_IQ_DECODER_ENABLED:-$(yaml_get decoder.interference.iq_snapshot_enabled true)}"
export BROADCAST_IIO_CAPTURE_DECODER_ENABLED="${BROADCAST_IIO_CAPTURE_DECODER_ENABLED:-$(yaml_get decoder.broadcast.iio_capture_enabled false)}"
export BROADCAST_IIO_CAPTURE_DECODER_ONLY="${BROADCAST_IIO_CAPTURE_DECODER_ONLY:-$(yaml_get decoder.broadcast.iio_capture_only false)}"
export BROADCAST_IIO_CAPTURE_SECONDS="${BROADCAST_IIO_CAPTURE_SECONDS:-$(yaml_get decoder.broadcast.capture_seconds 0.6)}"
export BROADCAST_IIO_CAPTURE_PERIOD_SEC="${BROADCAST_IIO_CAPTURE_PERIOD_SEC:-$(yaml_get decoder.broadcast.capture_period_sec 1.0)}"
export BROADCAST_IIO_CAPTURE_START_DELAY_SEC="${BROADCAST_IIO_CAPTURE_START_DELAY_SEC:-$(yaml_get decoder.broadcast.capture_start_delay_sec 1.0)}"
export BROADCAST_IQ_DECODER_AUTOTUNE_MODE="${BROADCAST_IQ_DECODER_AUTOTUNE_MODE:-$(yaml_get decoder.broadcast.iq_autotune_mode off)}"
export BROADCAST_IQ_DECODER_LOW_PASS_HZ="${BROADCAST_IQ_DECODER_LOW_PASS_HZ:-$(yaml_get decoder.broadcast.iq_low_pass_hz 260000)}"
export BROADCAST_IQ_DECODER_SPS_VALUES="${BROADCAST_IQ_DECODER_SPS_VALUES:-$(yaml_get decoder.broadcast.iq_sps_values '92,92.5,93,93.5,94,94.5,95')}"
export BROADCAST_IQ_DECODER_OFFSET_STEP="${BROADCAST_IQ_DECODER_OFFSET_STEP:-$(yaml_get decoder.broadcast.iq_offset_step 8)}"
export BROADCAST_IQ_DECODER_MAX_PAYLOADS="${BROADCAST_IQ_DECODER_MAX_PAYLOADS:-$(yaml_get decoder.broadcast.iq_max_payloads 256)}"
export BROADCAST_IQ_DECODER_MAX_FRAMES="${BROADCAST_IQ_DECODER_MAX_FRAMES:-$(yaml_get decoder.broadcast.iq_max_frames 64)}"
export INTERFERENCE_IQ_DECODER_AUTOTUNE_MODE="${INTERFERENCE_IQ_DECODER_AUTOTUNE_MODE:-$(yaml_get decoder.interference.iq_autotune_mode quick)}"
export INTERFERENCE_IQ_DECODER_LOW_PASS_HZ="${INTERFERENCE_IQ_DECODER_LOW_PASS_HZ:-$(yaml_get decoder.interference.iq_low_pass_hz 0)}"
export SPECTRUM_PERIOD_SEC="${SPECTRUM_PERIOD_SEC:-$(yaml_get spectrum.period_sec 0.04)}"
export SPECTRUM_FFT_SIZE="${SPECTRUM_FFT_SIZE:-$(yaml_get spectrum.fft_size 1024)}"
export SPECTRUM_BIN_COUNT="${SPECTRUM_BIN_COUNT:-$(yaml_get spectrum.bin_count 256)}"
export RM_RADIO_NATIVE_GUI="${RM_RADIO_NATIVE_GUI:-$(yaml_get spectrum.native_gui false)}"
export RM_RADIO_NATIVE_SPECTRUM_ONLY="${RM_RADIO_NATIVE_SPECTRUM_ONLY:-$(yaml_get spectrum.native_spectrum_only true)}"
export RM_RADIO_QT_PLATFORM="${RM_RADIO_QT_PLATFORM:-$(yaml_get spectrum.qt_platform offscreen)}"

export REFEREE_PORT="${REFEREE_PORT:-$(yaml_get referee.port /dev/ttyUSB0)}"
export REFEREE_BAUDRATE="${REFEREE_BAUDRATE:-$(yaml_get referee.baudrate 115200)}"
default_radar_sender_id=9
if [[ "${RM_RADIO_SIDE,,}" == "blue" ]]; then
  default_radar_sender_id=109
fi
export RADAR_SENDER_ID="${RADAR_SENDER_ID:-$(yaml_get referee.sender_id "$default_radar_sender_id")}"
export REFEREE_RECEIVER_ID="${REFEREE_RECEIVER_ID:-$(yaml_get referee.receiver_id 32896)}"
export REFEREE_BRIDGE_TOPIC="${REFEREE_BRIDGE_TOPIC:-$(yaml_get referee.bridge_topic /rm_gfsk_node/referee_bridge,/rm_gfsk_interference_node/referee_bridge)}"
export RADAR_CMD_TOPIC="${RADAR_CMD_TOPIC:-$(yaml_get referee.radar_cmd_topic /rm_radar_algorithm/radar_cmd)}"
export REFEREE_DRY_RUN="${REFEREE_DRY_RUN:-$(yaml_get referee.dry_run "$RM_RADIO_DRY_RUN")}"
export REQUIRE_REFEREE_SERIAL_OPEN_ON_START="${REQUIRE_REFEREE_SERIAL_OPEN_ON_START:-$(yaml_get referee.require_serial_open_on_start false)}"
export REFEREE_FRAME_TIMEOUT_SEC="${REFEREE_FRAME_TIMEOUT_SEC:-$(yaml_get referee.frame_timeout_sec 2.0)}"
export AUTO_SEND_INVINCIBLE_TARGETS="${AUTO_SEND_INVINCIBLE_TARGETS:-$(yaml_get referee.auto_send_invincible_targets true)}"
export INVINCIBLE_TARGETS_DATA_CMD_ID="${INVINCIBLE_TARGETS_DATA_CMD_ID:-$(yaml_get referee.invincible_targets_data_cmd_id 564)}"
export INVINCIBLE_TARGETS_SEND_RATE_HZ="${INVINCIBLE_TARGETS_SEND_RATE_HZ:-$(yaml_get referee.invincible_targets_send_rate_hz 3.0)}"
export INVINCIBLE_TARGETS_FRESHNESS_SEC="${INVINCIBLE_TARGETS_FRESHNESS_SEC:-$(yaml_get referee.invincible_targets_freshness_sec 1.0)}"

export RM_RADIO_RECORDING_ENABLED="${RM_RADIO_RECORDING_ENABLED:-$(yaml_get recording.enabled true)}"
export RM_RADIO_RECORDING_AUTO_START="${RM_RADIO_RECORDING_AUTO_START:-$(yaml_get recording.auto_start true)}"
export RM_RADIO_RECORDING_ROOT="${RM_RADIO_RECORDING_ROOT:-$(yaml_get recording.root '~/.local/share/shark-radio/match-records')}"
RM_RADIO_RECORDING_ROOT="${RM_RADIO_RECORDING_ROOT/#\~\//$HOME/}"
export RM_RADIO_RECORDING_ROOT
iq_root_overridden=false
if [[ -n "${RM_RADIO_RECORDING_IQ_ROOT+x}" ]]; then
  iq_root_overridden=true
fi
export RM_RADIO_RECORDING_IQ_ROOT="${RM_RADIO_RECORDING_IQ_ROOT:-$(yaml_get recording.iq.root /media/shark/Resources/SHARK-radio-iq)}"
export RM_RADIO_RECORDING_IQ_MOUNT_POINT="${RM_RADIO_RECORDING_IQ_MOUNT_POINT:-$(yaml_get recording.iq.mount_point '')}"
export RM_RADIO_RECORDING_IQ_FALLBACK_ROOT="${RM_RADIO_RECORDING_IQ_FALLBACK_ROOT:-$(yaml_get recording.iq.fallback_root "$RM_RADIO_RECORDING_ROOT")}"
RM_RADIO_RECORDING_IQ_ROOT="${RM_RADIO_RECORDING_IQ_ROOT/#\~\//$HOME/}"
RM_RADIO_RECORDING_IQ_FALLBACK_ROOT="${RM_RADIO_RECORDING_IQ_FALLBACK_ROOT/#\~\//$HOME/}"
export RM_RADIO_RECORDING_IQ_ROOT RM_RADIO_RECORDING_IQ_FALLBACK_ROOT
if [[ "$iq_root_overridden" == "false" && -n "$RM_RADIO_RECORDING_IQ_MOUNT_POINT" ]]; then
  iq_mount_options=""
  if command -v findmnt >/dev/null 2>&1; then
    iq_mount_options="$(findmnt -n -o OPTIONS --target "$RM_RADIO_RECORDING_IQ_MOUNT_POINT" 2>/dev/null || true)"
  fi
  if ! command -v mountpoint >/dev/null 2>&1 \
      || ! mountpoint -q "$RM_RADIO_RECORDING_IQ_MOUNT_POINT" \
      || [[ ",$iq_mount_options," != *,rw,* ]]; then
    echo "[match-rx][WARN] IQ 外接盘未以可写方式挂载：$RM_RADIO_RECORDING_IQ_MOUNT_POINT（options=${iq_mount_options:-unknown}）；大尺寸 IQ 本次回退到本地 $RM_RADIO_RECORDING_IQ_FALLBACK_ROOT，小型事件日志始终写入 $RM_RADIO_RECORDING_ROOT。" >&2
    export RM_RADIO_RECORDING_IQ_ROOT="$RM_RADIO_RECORDING_IQ_FALLBACK_ROOT"
  fi
fi
export RM_RADIO_MATCH_RUN_ID="${RM_RADIO_MATCH_RUN_ID:-${SHARK_MATCH_RUN_ID:-radio_$(date +%Y%m%d_%H%M%S)_$$}}"
export RM_RADIO_RECORDING_CONTROL_TOPIC="${RM_RADIO_RECORDING_CONTROL_TOPIC:-$(yaml_get recording.control_topic /rm_match_recorder/control)}"
export RM_RADIO_RECORDING_GAME_STATUS_TOPIC="${RM_RADIO_RECORDING_GAME_STATUS_TOPIC:-$(yaml_get recording.game_status_topic /rm_referee_serial_node/game_status)}"
export RM_RADIO_RECORDING_START_CONFIRM_FRAMES="${RM_RADIO_RECORDING_START_CONFIRM_FRAMES:-$(yaml_get recording.start_confirm_frames 2)}"
export RM_RADIO_RECORDING_STOP_CONFIRM_FRAMES="${RM_RADIO_RECORDING_STOP_CONFIRM_FRAMES:-$(yaml_get recording.stop_confirm_frames 2)}"
export RM_RADIO_RECORDING_STOP_GAME_PROGRESS="${RM_RADIO_RECORDING_STOP_GAME_PROGRESS:-$(yaml_get recording.stop_game_progress 5)}"
export RM_RADIO_RECORDING_POST_ROLL_SEC="${RM_RADIO_RECORDING_POST_ROLL_SEC:-$(yaml_get recording.post_roll_sec 10.0)}"
export RM_RADIO_RECORDING_MAX_DURATION_SEC="${RM_RADIO_RECORDING_MAX_DURATION_SEC:-$(yaml_get recording.max_duration_sec 900.0)}"
export RM_RADIO_RECORDING_FINALIZE_TIMEOUT_SEC="${RM_RADIO_RECORDING_FINALIZE_TIMEOUT_SEC:-$(yaml_get recording.finalize_timeout_sec 5.0)}"
export RM_RADIO_RECORDING_EVENT_MIN_FREE_GB="${RM_RADIO_RECORDING_EVENT_MIN_FREE_GB:-$(yaml_get recording.events.min_free_gb 0.1)}"
export RM_RADIO_RECORDING_IQ_MIN_FREE_GB="${RM_RADIO_RECORDING_IQ_MIN_FREE_GB:-$(yaml_get recording.iq.min_free_gb 15.0)}"
export RM_RADIO_RECORDING_OUTPUT_SAMPLE_RATE="${RM_RADIO_RECORDING_OUTPUT_SAMPLE_RATE:-$(yaml_get recording.iq.output_sample_rate 1000000)}"
export RM_RADIO_RECORDING_SEGMENT_SEC="${RM_RADIO_RECORDING_SEGMENT_SEC:-$(yaml_get recording.iq.segment_sec 30.0)}"
export RM_RADIO_RECORDING_QUEUE_CHUNKS="${RM_RADIO_RECORDING_QUEUE_CHUNKS:-$(yaml_get recording.iq.queue_chunks 64)}"
export RM_RADIO_RECORDING_IQ_COMPRESSION="${RM_RADIO_RECORDING_IQ_COMPRESSION:-$(yaml_get recording.iq.compression zlib)}"
export RM_RADIO_RECORDING_IQ_COMPRESSION_LEVEL="${RM_RADIO_RECORDING_IQ_COMPRESSION_LEVEL:-$(yaml_get recording.iq.compression_level 1)}"
export RM_RADIO_RECORDING_EVENT_COMPRESSION="${RM_RADIO_RECORDING_EVENT_COMPRESSION:-$(yaml_get recording.events.compression zstd)}"
export RM_RADIO_RECORDING_EVENT_COMPRESSION_LEVEL="${RM_RADIO_RECORDING_EVENT_COMPRESSION_LEVEL:-$(yaml_get recording.events.compression_level 1)}"
export RM_RADIO_RECORDING_EVENT_QUEUE_SIZE="${RM_RADIO_RECORDING_EVENT_QUEUE_SIZE:-$(yaml_get recording.events.queue_size 4096)}"
export RM_RADIO_RECORDING_EVENT_TOPICS="${RM_RADIO_RECORDING_EVENT_TOPICS:-$(yaml_get recording.events.topics '')}"

export RUN_RADAR_INTEGRATION="${RUN_RADAR_INTEGRATION:-$(yaml_get radar_integration.enabled true)}"
export RADAR_AUTO_SYNC_SIDE="${RADAR_AUTO_SYNC_SIDE:-$(yaml_get radar_integration.auto_sync_side true)}"
export RADAR_INTEGRATION_RATE_HZ="${RADAR_INTEGRATION_RATE_HZ:-$(yaml_get radar_integration.send_rate_hz 4.8)}"
export RADAR_STATE_RATE_HZ="${RADAR_STATE_RATE_HZ:-$(yaml_get radar_integration.state_rate_hz 20.0)}"
export RADAR_VISION_TIMEOUT_SEC="${RADAR_VISION_TIMEOUT_SEC:-$(yaml_get radar_integration.vision_timeout_sec 0.5)}"
export RADAR_RADIO_TIMEOUT_SEC="${RADAR_RADIO_TIMEOUT_SEC:-$(yaml_get radar_integration.radio_timeout_sec 1.0)}"
export RADAR_RADIO_GHOST_SEC="${RADAR_RADIO_GHOST_SEC:-$(yaml_get radar_integration.radio_ghost_sec 2.0)}"
export RADAR_TELEMETRY_TOPIC="${RADAR_TELEMETRY_TOPIC:-$(yaml_get radar_integration.telemetry_topic /rm_radar_algorithm/telemetry)}"
export RADAR_AUTO_DOUBLE_ENABLED="${RADAR_AUTO_DOUBLE_ENABLED:-$(yaml_get radar_integration.auto_double_vulnerability.enabled true)}"
export RADAR_AUTO_DOUBLE_MASK_PATH="${RADAR_AUTO_DOUBLE_MASK_PATH:-$(yaml_get radar_integration.auto_double_vulnerability.mask_path bundled)}"
export RADAR_AUTO_DOUBLE_ZONE_DWELL_SEC="${RADAR_AUTO_DOUBLE_ZONE_DWELL_SEC:-$(yaml_get radar_integration.auto_double_vulnerability.zone_dwell_sec 3.0)}"
export RADAR_AUTO_DOUBLE_HERO_HP_DROP_AMOUNT="${RADAR_AUTO_DOUBLE_HERO_HP_DROP_AMOUNT:-$(yaml_get radar_integration.auto_double_vulnerability.hero_hp_drop_amount 100)}"
export RADAR_AUTO_DOUBLE_HERO_HP_WINDOW_SEC="${RADAR_AUTO_DOUBLE_HERO_HP_WINDOW_SEC:-$(yaml_get radar_integration.auto_double_vulnerability.hero_hp_window_sec 3.0)}"
export RADAR_AUTO_DOUBLE_ENDGAME_SEC="${RADAR_AUTO_DOUBLE_ENDGAME_SEC:-$(yaml_get radar_integration.auto_double_vulnerability.endgame_sec 30.0)}"
export RADAR_AUTO_DOUBLE_REFEREE_TIMEOUT_SEC="${RADAR_AUTO_DOUBLE_REFEREE_TIMEOUT_SEC:-$(yaml_get radar_integration.auto_double_vulnerability.referee_timeout_sec 2.5)}"

export RUN_VISION_BRIDGE="${RUN_VISION_BRIDGE:-$(yaml_get vision_bridge.enabled true)}"
export VISION_BRIDGE_LISTEN_HOST="${VISION_BRIDGE_LISTEN_HOST:-$(yaml_get vision_bridge.listen_host 127.0.0.1)}"
export VISION_BRIDGE_LISTEN_PORT="${VISION_BRIDGE_LISTEN_PORT:-$(yaml_get vision_bridge.listen_port 37602)}"
export VISION_BRIDGE_TARGET_HOST="${VISION_BRIDGE_TARGET_HOST:-$(yaml_get vision_bridge.vision_host 127.0.0.1)}"
export VISION_BRIDGE_TARGET_PORT="${VISION_BRIDGE_TARGET_PORT:-$(yaml_get vision_bridge.vision_port 37601)}"

export RUN_ALGORITHM="${RUN_ALGORITHM:-$(yaml_get algorithm.enabled false)}"
export ALLOW_ALGORITHM_FAILURE="${ALLOW_ALGORITHM_FAILURE:-$(yaml_get algorithm.allow_failure true)}"
export RM_ALGO_DIR="${RM_ALGO_DIR:-$(yaml_get algorithm.dir "")}"
export RM_ALGO_PYTHON="${RM_ALGO_PYTHON:-$(yaml_get algorithm.python "")}"

export ROS_LOG_DIR="${ROS_LOG_DIR:-$RM_RADIO_LOG_ROOT/ros}"
export ROS_HOME="${ROS_HOME:-/tmp/ros_home}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/matplotlib}"
export XDG_CACHE_HOME="${XDG_CACHE_HOME:-/tmp}"
mkdir -p "$ROS_LOG_DIR" "$ROS_HOME" "$MPLCONFIGDIR" "$XDG_CACHE_HOME" "$RM_RADIO_LOG_ROOT"

set +u
source /opt/ros/humble/setup.bash
if [[ -f "$RM_RADIO_WS/install/setup.bash" ]]; then
  source "$RM_RADIO_WS/install/setup.bash"
fi
set -u
export PYTHONPATH="$RM_RADIO_WS/src/rm_radio_ros:$RM_RADIO_WS:${PYTHONPATH:-}"

# 列出与本仓库相关、仍在运行的电台节点/进程 PID（排除自身与 grep）。
rm_radio_residual_pids() {
  # Command substitution runs this function in a Bash subshell where $$ still
  # names the parent shell. BASHPID is the actual process visible to ps.
  local self="${BASHPID:-$$}"
  local ancestors=" $self "
  local ancestor="$self"
  local next_ancestor
  while [[ "$ancestor" =~ ^[0-9]+$ && "$ancestor" -gt 1 ]]; do
    next_ancestor="$(ps -o ppid= -p "$ancestor" 2>/dev/null | tr -d '[:space:]')"
    [[ "$next_ancestor" =~ ^[0-9]+$ && "$next_ancestor" -gt 0 ]] || break
    [[ "$ancestors" == *" $next_ancestor "* ]] && break
    ancestors+="$next_ancestor "
    ancestor="$next_ancestor"
  done
  local needles
  printf -v needles '%s\n' "$@"
  ps -eo pid=,cmd= 2>/dev/null | NEEDLES="$needles" awk -v ancestors="$ancestors" '
    BEGIN {
      n = split(ENVIRON["NEEDLES"], pats, "\n")
    }
    {
      pid = $1
      $1 = ""
      cmd = $0
      if (index(ancestors, " " pid " ") > 0) next
      if (cmd ~ /awk|[ \/]grep /) next
      for (i = 1; i <= n; i++) {
        if (pats[i] != "" && index(cmd, pats[i]) > 0) { print pid; break }
      }
    }
  '
}

# 启动前检查残留：发现同类进程则打印并（默认）拒绝启动。
# RM_RADIO_KILL_RESIDUAL=true 时先清理再继续；RM_RADIO_ALLOW_RESIDUAL=true 时忽略。
rm_radio_guard_residual() {
  local app="$1"; shift
  local pids
  pids="$(rm_radio_residual_pids "$@")"
  [[ -z "$pids" ]] && return 0

  echo "[$app] 检测到残留的电台进程：" >&2
  ps -o pid=,etime=,cmd= -p $(echo "$pids" | tr '\n' ' ') 2>/dev/null | sed 's/^/  /' >&2 || true

  if [[ "${RM_RADIO_ALLOW_RESIDUAL:-false}" == "true" ]]; then
    echo "[$app] RM_RADIO_ALLOW_RESIDUAL=true，忽略残留继续启动。" >&2
    return 0
  fi
  if [[ "${RM_RADIO_KILL_RESIDUAL:-false}" == "true" ]]; then
    echo "[$app] RM_RADIO_KILL_RESIDUAL=true，正在清理残留进程..." >&2
    # Signal individual PIDs only. A wrapper shell and its ROS children may
    # share the caller's process group, so killing the PGID can kill this guard.
    local pid
    for pid in $pids; do kill -TERM "$pid" 2>/dev/null || true; done
    sleep 2
    local left
    left="$(rm_radio_residual_pids "$@")"
    if [[ -n "$left" ]]; then
      for pid in $left; do kill -KILL "$pid" 2>/dev/null || true; done
      sleep 1
    fi
    if [[ -n "$(rm_radio_residual_pids "$@")" ]]; then
      echo "[$app] 清理后仍有残留，拒绝启动。" >&2
      return 3
    fi
    echo "[$app] 残留已清理。" >&2
    return 0
  fi

  cat >&2 <<MSG
[$app] 拒绝启动：已有电台进程在运行，重复启动会互抢 SDR/话题并导致状态跳变。
  先停止旧实例，或用以下方式之一重跑：
    RM_RADIO_KILL_RESIDUAL=true  自动清理残留再启动
    RM_RADIO_ALLOW_RESIDUAL=true 明确允许多实例共存（不推荐）
MSG
  return 3
}
