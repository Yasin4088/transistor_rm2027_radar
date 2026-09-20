#!/usr/bin/env bash
set -euo pipefail

RADIO_TX_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export RADIO_TX_DIR
export LAB_TX_CONFIG="${LAB_TX_CONFIG:-$RADIO_TX_DIR/config.yaml}"

# Hidden local.env files made deployments silently diverge. A caller may still
# opt into an explicit, visible one-shot override file when diagnosing hardware.
if [[ -n "${LAB_TX_ENV_FILE:-}" ]]; then
  # shellcheck disable=SC1090
  source "$LAB_TX_ENV_FILE"
fi

export RM_RADIO_WS="${RM_RADIO_WS:-$(cd "$RADIO_TX_DIR/../.." && pwd)}"

yaml_get() {
  python3 "$RM_RADIO_WS/apps/common/read_yaml.py" "$LAB_TX_CONFIG" "$1" "${2:-}"
}

rm_radio_ensure_dir() {
  local path="$1"
  if [[ -e "$path" && ! -d "$path" ]]; then
    mv -- "$path" "${path}.blocked_$(date +%Y%m%d_%H%M%S)"
  fi
  mkdir -p "$path"
}

# Keep this wrapper outside match topology. These variables are consumed by
# shared scripts and diagnostics, but no match launcher is called from LabTX.
export RM_RADIO_MATCH_MODE=lab_tx
export RM_RADIO_RX_ONLY=false
export RM_RADIO_RX2=false
export RM_RADIO_INTERFERENCE_TX=true
export AUTO_INTERFERENCE_LEVEL=false
export AUTO_RX_INTERFERENCE_LEVEL=false

export RM_RADIO_SIDE="${RM_RADIO_SIDE:-$(yaml_get radio.side red)}"
export INTERFERENCE_LEVEL="${INTERFERENCE_LEVEL:-$(yaml_get radio.interference_level 1)}"
export RM_RADIO_SAMPLE_RATE="${RM_RADIO_SAMPLE_RATE:-$(yaml_get radio.sample_rate 1000000)}"
export RM_RADIO_IIO_PREINIT="${RM_RADIO_IIO_PREINIT:-true}"
export RX_URI="${RX_URI:-$(yaml_get devices.rx1_uri ip:192.168.9.110)}"
export RX1_URI="${RX1_URI:-$(yaml_get devices.rx1_uri ip:192.168.9.110)}"
export RX2_URI="${RX2_URI:-$(yaml_get devices.rx2_uri ip:192.168.9.111)}"
export RX_RF_PORT="${RX_RF_PORT:-$(yaml_get devices.rx_rf_port B_BALANCED)}"
export BROADCAST_RX_GAIN_MODE="${BROADCAST_RX_GAIN_MODE:-$(yaml_get rx.broadcast_gain_mode slow_attack)}"
export BROADCAST_RX_GAIN="${BROADCAST_RX_GAIN:-$(yaml_get rx.broadcast_gain 20)}"
export INTERFERENCE_RX_GAIN_MODE="${INTERFERENCE_RX_GAIN_MODE:-$(yaml_get rx.interference_gain_mode fast_attack)}"
export INTERFERENCE_RX_GAIN="${INTERFERENCE_RX_GAIN:-$(yaml_get rx.interference_gain 20)}"
export BROADCAST_TX_URI="${BROADCAST_TX_URI:-$(yaml_get devices.broadcast_tx_uri ip:192.168.2.1)}"
# GNU Radio 3.10.7 gr-iio raises full-scale I/Q by about 24 dB relative to 3.10.1.
# 61 dB preserves the former bench level obtained at 37 dB until recalibration.
export BROADCAST_TX_ATTENUATION_DB="${BROADCAST_TX_ATTENUATION_DB:-$(yaml_get tx.broadcast_attenuation_db 61)}"
export BROADCAST_TX_RF_PORT="${BROADCAST_TX_RF_PORT:-${TX_RF_PORT:-$(yaml_get devices.tx_rf_port A)}}"
# Empty means the interference SDR is currently disconnected / awaiting identification.
export INTERFERENCE_TX_URI="${INTERFERENCE_TX_URI:-$(yaml_get devices.interference_tx_uri '')}"
export INTERFERENCE_TX_ATTEND_GR="${INTERFERENCE_TX_ATTEND_GR:-$(yaml_get tx.interference_attenuation_db 11)}"
export INTERFERENCE_TX_RF_PORT="${INTERFERENCE_TX_RF_PORT:-${TX_RF_PORT:-$(yaml_get devices.tx_rf_port A)}}"
export INTERFERENCE_PASSWORD="${INTERFERENCE_PASSWORD:-$(yaml_get tx.interference_password R1L001)}"
export RM_RADIO_LAB_TX_ANTENNA_CONFIRM="${RM_RADIO_LAB_TX_ANTENNA_CONFIRM:-$(yaml_get tx.lab_antenna_confirm false)}"
# The generated gr-iio flowgraph was validated with these YAML values. Do not
# let a stale shell override restore experimental buffering values.
export RM_RADIO_TX_BUFFER_SIZE="$(yaml_get tx.buffer_size 1048576)"
export RM_RADIO_TX_PREFILL_PACKETS="$(yaml_get tx.prefill_packets 0)"

export BROADCAST_IIO_CAPTURE_DECODER_ENABLED="${BROADCAST_IIO_CAPTURE_DECODER_ENABLED:-$(yaml_get decoder.broadcast_iio_capture_enabled true)}"
export BROADCAST_IIO_CAPTURE_DECODER_ONLY="${BROADCAST_IIO_CAPTURE_DECODER_ONLY:-$(yaml_get decoder.broadcast_iio_capture_only true)}"
export BROADCAST_IQ_DECODER_AUTOTUNE_MODE="${BROADCAST_IQ_DECODER_AUTOTUNE_MODE:-$(yaml_get decoder.broadcast_iq_autotune_mode off)}"
export BROADCAST_IQ_DECODER_LOW_PASS_HZ="${BROADCAST_IQ_DECODER_LOW_PASS_HZ:-$(yaml_get decoder.broadcast_iq_low_pass_hz 260000)}"
export BROADCAST_IIO_CAPTURE_SECONDS="${BROADCAST_IIO_CAPTURE_SECONDS:-$(yaml_get decoder.broadcast_iio_capture_seconds 0.6)}"
export BROADCAST_IIO_CAPTURE_PERIOD_SEC="${BROADCAST_IIO_CAPTURE_PERIOD_SEC:-$(yaml_get decoder.broadcast_iio_capture_period_sec 1.0)}"
export BROADCAST_IIO_CAPTURE_START_DELAY_SEC="${BROADCAST_IIO_CAPTURE_START_DELAY_SEC:-$(yaml_get decoder.broadcast_iio_capture_start_delay_sec 1.0)}"
export BROADCAST_IQ_DECODER_OFFSET_STEP="${BROADCAST_IQ_DECODER_OFFSET_STEP:-$(yaml_get decoder.broadcast_iq_offset_step 8)}"
export BROADCAST_IQ_DECODER_MAX_PAYLOADS="${BROADCAST_IQ_DECODER_MAX_PAYLOADS:-$(yaml_get decoder.broadcast_iq_max_payloads 256)}"
export BROADCAST_IQ_DECODER_MAX_FRAMES="${BROADCAST_IQ_DECODER_MAX_FRAMES:-$(yaml_get decoder.broadcast_iq_max_frames 64)}"

export IQ_REPLAY_PATHS="${IQ_REPLAY_PATHS:-$(yaml_get iq_replay.paths "")}"
export IQ_REPLAY_TX_TYPE="${IQ_REPLAY_TX_TYPE:-$(yaml_get iq_replay.tx_type broadcast)}"
export IQ_REPLAY_SAMPLE_RATE="${IQ_REPLAY_SAMPLE_RATE:-$(yaml_get iq_replay.sample_rate 2000000)}"
export IQ_REPLAY_REPEAT="${IQ_REPLAY_REPEAT:-$(yaml_get iq_replay.repeat true)}"
export IQ_REPLAY_AMPLITUDE_SCALE="${IQ_REPLAY_AMPLITUDE_SCALE:-$(yaml_get iq_replay.amplitude_scale 1.0)}"
export IQ_REPLAY_TX_ATTENUATION_DB="${IQ_REPLAY_TX_ATTENUATION_DB:-$(yaml_get iq_replay.attenuation_db 60)}"
export IQ_REPLAY_TX_BUFFER_SIZE="${IQ_REPLAY_TX_BUFFER_SIZE:-$(yaml_get iq_replay.buffer_size 1048576)}"
export IQ_REPLAY_CACHE_TO_TMP="${IQ_REPLAY_CACHE_TO_TMP:-$(yaml_get iq_replay.cache_to_tmp true)}"
export IQ_REPLAY_CACHE_DIR="${IQ_REPLAY_CACHE_DIR:-$(yaml_get iq_replay.cache_dir /tmp/rm_radio_iq_replay)}"

export RM_RADIO_LOG_ROOT="${RM_RADIO_LOG_ROOT:-$(yaml_get app.log_root /tmp/rm_radio_runtime_logs)}"
export LOG_DIR="${LOG_DIR:-$RM_RADIO_LOG_ROOT/radio_tx_$(date +%Y%m%d_%H%M%S)}"

set +u
source /opt/ros/humble/setup.bash
if [[ -f "$RM_RADIO_WS/install/setup.bash" ]]; then
  source "$RM_RADIO_WS/install/setup.bash"
fi
set -u
export PYTHONPATH="$RM_RADIO_WS/src/rm_radio_ros:$RM_RADIO_WS:${PYTHONPATH:-}"

case "${RM_RADIO_SIDE,,}" in
  red|blue) ;;
  *)
    echo "RM_RADIO_SIDE must be red or blue; got $RM_RADIO_SIDE" >&2
    exit 2
    ;;
esac

case "$INTERFERENCE_LEVEL" in
  1|2|3) ;;
  *)
    echo "INTERFERENCE_LEVEL must be 1, 2, or 3; got $INTERFERENCE_LEVEL" >&2
    exit 2
    ;;
esac

if [[ ! "$INTERFERENCE_PASSWORD" =~ ^[A-Za-z0-9]{6}$ ]]; then
  echo "INTERFERENCE_PASSWORD must be exactly 6 ASCII letters/digits; got $INTERFERENCE_PASSWORD" >&2
  exit 2
fi

case "${IQ_REPLAY_TX_TYPE,,}" in
  broadcast|interference) ;;
  *)
    echo "IQ_REPLAY_TX_TYPE must be broadcast or interference; got $IQ_REPLAY_TX_TYPE" >&2
    exit 2
    ;;
esac

tx_frequency_for_side_level() {
  local side="${1,,}"
  local level="$2"
  case "$side:$level" in
    red:1) echo "432200000 940000" ;;
    red:2) echo "432500000 860000" ;;
    red:3) echo "432800000 250000" ;;
    blue:1) echo "434920000 940000" ;;
    blue:2) echo "434620000 860000" ;;
    blue:3) echo "434320000 250000" ;;
    *) echo "invalid side/level: $side/$level" >&2; return 2 ;;
  esac
}

build_interference_setters_json() {
  local freq bw
  read -r freq bw < <(tx_frequency_for_side_level "$RM_RADIO_SIDE" "$INTERFERENCE_LEVEL")
  local password_bytes
  password_bytes="$(printf '%s' "$INTERFERENCE_PASSWORD" | od -An -t u1 | tr -s ' ' ' ' | sed 's/^ //;s/ $//;s/ /,/g')"
  printf '{"center_f":%s,"BW_ganrao":%s,"attend_gr":%s,"access":[22,232,211,119,21,28,113,45],"cmd_id":[10,6],"payload_data":[%s],"Period":100}' \
    "$freq" "$bw" "$INTERFERENCE_TX_ATTEND_GR" "$password_bytes"
}

require_lab_tx_confirmation() {
  if [[ "${RM_RADIO_LAB_TX_ANTENNA_CONFIRM:-}" == "true" ]]; then
    return 0
  fi
  cat >&2 <<'MSG'
Refusing to start lab TX.

Set RM_RADIO_LAB_TX_ANTENNA_CONFIRM=true only after checking antenna,
attenuation, SDR URI, and local RF safety. This directory is not for matches.
MSG
  return 2
}

require_cabled_loop_attenuation() {
  local confirm="${RM_RADIO_CABLED_LOOP_CONFIRM:-false}"
  local attenuation="${RM_RADIO_EXTERNAL_ATTENUATION_DB:-}"
  local minimum="40"

  if [[ "$confirm" != "true" ]]; then
    cat >&2 <<'MSG'
Refusing to start a cabled RF loop test.

Remove every antenna and connect TX -> external attenuator(s) -> RX by 50-ohm
coax. Then set RM_RADIO_CABLED_LOOP_CONFIRM=true.
MSG
    return 2
  fi
  if [[ ! "$attenuation" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "RM_RADIO_EXTERNAL_ATTENUATION_DB must be numeric" >&2
    return 2
  fi
  if ! awk -v value="$attenuation" -v minimum="$minimum" \
    'BEGIN { exit !(value >= minimum) }'; then
    echo "Refusing cabled RF test: external attenuation ${attenuation} dB is below required ${minimum} dB" >&2
    return 2
  fi
  echo "[lab-tx] cabled loop confirmed: external attenuation=${attenuation} dB (minimum=${minimum} dB)"
}

# 列出与本仓库相关、仍在运行的电台节点/发射进程 PID（排除自身与 grep）。
# 用法：rm_radio_residual_pids "<关键字1>" "<关键字2>" ...
rm_radio_residual_pids() {
  # Command substitution runs this function in a Bash subshell where $$ still
  # names the parent shell. BASHPID is the actual process visible to ps.
  local self="${BASHPID:-$$}"
  # Exclude the complete caller ancestry, not only PPID.  Test harnesses often
  # wrap this launcher in `timeout` or an extra shell whose command line can
  # contain start_dual_tx.sh; none of those callers may be treated as residue.
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
# 用法：rm_radio_guard_residual "<应用名>" "<关键字1>" "<关键字2>" ...
rm_radio_guard_residual() {
  local app="$1"; shift
  local pids
  pids="$(rm_radio_residual_pids "$@")"
  [[ -z "$pids" ]] && return 0

  echo "[$app] 检测到残留的电台进程：" >&2
  # A matching process can exit between the scan and this diagnostic read.
  # Do not let that harmless race abort the guard before its policy checks.
  ps -o pid=,etime=,cmd= -p $(echo "$pids" | tr '\n' ' ') 2>/dev/null | sed 's/^/  /' >&2 || true

  if [[ "${RM_RADIO_ALLOW_RESIDUAL:-false}" == "true" ]]; then
    echo "[$app] RM_RADIO_ALLOW_RESIDUAL=true，忽略残留继续启动。" >&2
    return 0
  fi
  if [[ "${RM_RADIO_KILL_RESIDUAL:-false}" == "true" ]]; then
    echo "[$app] RM_RADIO_KILL_RESIDUAL=true，正在清理残留进程..." >&2
    # Never signal a process group here.  A background ROS/TX process can share
    # the current non-interactive shell or terminal PGID; kill -- -PGID could
    # terminate this guard and its caller.  Re-scan below catches descendants.
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
