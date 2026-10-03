#!/usr/bin/env bash
set -euo pipefail

RADIO_TX_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# This test must be deterministic; do not let ambient shell overrides silently
# replace the committed gain/capture settings below.
# shellcheck disable=SC1091
source "$RADIO_TX_SCRIPT_DIR/env.sh"
# shellcheck disable=SC1091
source "$RM_RADIO_WS/apps/common/lab_tx_safety.sh"

export RM_RADIO_RX_ONLY=false
export AUTO_INTERFERENCE_LEVEL=false
export AUTO_RX_INTERFERENCE_LEVEL=false
require_lab_tx_confirmation

# Four-SDR lab defaults proven on the current Z103/E310 bench. These are
# receive-side choices only; TX still follows the official RF/frame settings.
export BROADCAST_RX_GAIN_MODE="${BROADCAST_RX_GAIN_MODE:-slow_attack}"
export BROADCAST_RX_GAIN="${BROADCAST_RX_GAIN:-20}"
export INTERFERENCE_RX_GAIN_MODE="${INTERFERENCE_RX_GAIN_MODE:-fast_attack}"
export INTERFERENCE_RX_GAIN="${INTERFERENCE_RX_GAIN:-20}"
# Follow the experimental match baseline unless the caller explicitly selects
# a legacy A/B mode.  The old defaults forced capture-only and silently skipped
# the continuous Gaussian + M&M flowgraph.
export BROADCAST_DEMOD_MODE="${BROADCAST_DEMOD_MODE:-mm}"
export BROADCAST_IQ_DECODER_ENABLED="${BROADCAST_IQ_DECODER_ENABLED:-false}"
export BROADCAST_IIO_CAPTURE_DECODER_ENABLED="${BROADCAST_IIO_CAPTURE_DECODER_ENABLED:-false}"
export BROADCAST_IIO_CAPTURE_DECODER_ONLY="${BROADCAST_IIO_CAPTURE_DECODER_ONLY:-false}"
export BROADCAST_IQ_DECODER_AUTOTUNE_MODE="${BROADCAST_IQ_DECODER_AUTOTUNE_MODE:-off}"
export BROADCAST_IQ_DECODER_LOW_PASS_HZ="${BROADCAST_IQ_DECODER_LOW_PASS_HZ:-260000}"
export BROADCAST_IQ_DECODER_SPS_VALUES="${BROADCAST_IQ_DECODER_SPS_VALUES:-92,92.5,93,93.5,94,94.5,95}"
export BROADCAST_IQ_DECODER_OFFSET_STEP="${BROADCAST_IQ_DECODER_OFFSET_STEP:-8}"
export BROADCAST_IQ_DECODER_MAX_PAYLOADS="${BROADCAST_IQ_DECODER_MAX_PAYLOADS:-256}"
export BROADCAST_IQ_DECODER_MAX_FRAMES="${BROADCAST_IQ_DECODER_MAX_FRAMES:-64}"
export BROADCAST_IIO_CAPTURE_SECONDS="${BROADCAST_IIO_CAPTURE_SECONDS:-0.6}"
export BROADCAST_IIO_CAPTURE_PERIOD_SEC="${BROADCAST_IIO_CAPTURE_PERIOD_SEC:-1.0}"
export BROADCAST_IIO_CAPTURE_START_DELAY_SEC="${BROADCAST_IIO_CAPTURE_START_DELAY_SEC:-1.0}"

TEST_LEVELS="${TEST_LEVELS:-2 3}"
TEST_SECONDS="${TEST_SECONDS:-18}"
RX_WARMUP_SECONDS="${RX_WARMUP_SECONDS:-8}"
POST_TX_SECONDS="${POST_TX_SECONDS:-3}"
TX_STARTUP_VERIFY_SECONDS="${TX_STARTUP_VERIFY_SECONDS:-1}"
TOPIC_TIMEOUT_SECONDS="$((TEST_SECONDS + RX_WARMUP_SECONDS + POST_TX_SECONDS + 12))"

export RM_RADIO_LOG_ROOT="${RM_RADIO_LOG_ROOT:-$RM_RADIO_WS/log}"
LOG_ROOT="${LOG_ROOT:-$RM_RADIO_WS/log/four_sdr_live_parse_$(date +%Y%m%d_%H%M%S)}"
rm_radio_ensure_dir "$LOG_ROOT"

TX_LAUNCHER_PID=""
RX_LAUNCHER_PID=""
TOPIC_PIDS=()
TX_RESIDUAL_PATTERNS=(
  "rm_broadcast_tx_node"
  "rm_interference_tx_node"
  "start_dual_tx.sh"
  "flowgraph_name:=ganraoyuan"
)

stop_pids() {
  lab_tx_stop_pids "$@"
}

stop_dual_tx_for_case() {
  local tx_launcher_pid="$1"
  local tx_dir="$2"
  local pids=()
  [[ -n "$tx_launcher_pid" ]] && pids+=("$tx_launcher_pid")
  if [[ -f "$tx_dir/dual_tx.pids" ]]; then
    while IFS='=' read -r _name pid; do
      [[ "$pid" =~ ^[0-9]+$ ]] && pids+=("$pid")
    done < "$tx_dir/dual_tx.pids"
  fi
  if ((${#pids[@]})); then
    stop_pids "${pids[@]}"
  fi
}

mute_txs() {
  local out_file="$1"
  {
    echo "mute_start=$(date --iso-8601=seconds)"
    lab_tx_hard_mute_and_verify "$BROADCAST_TX_URI" "$INTERFERENCE_TX_URI"
    echo "mute_end=$(date --iso-8601=seconds)"
  } >>"$out_file" 2>&1
}

finalize_tx_safety() {
  local out_file="$1"
  local rc=0
  lab_tx_cleanup_residual_tx "${TX_RESIDUAL_PATTERNS[@]}" >>"$out_file" 2>&1 || rc=1
  sleep 0.4
  mute_txs "$out_file" || rc=1
  lab_tx_verify_no_residual_tx "${TX_RESIDUAL_PATTERNS[@]}" >>"$out_file" 2>&1 || rc=1
  return "$rc"
}

cleanup() {
  set +e
  stop_dual_tx_for_case "$TX_LAUNCHER_PID" "${CURRENT_TX_DIR:-}"
  lab_tx_cleanup_residual_tx "${TX_RESIDUAL_PATTERNS[@]}" >>"${CURRENT_CASE_DIR:-$LOG_ROOT}/tx_safety_on_exit.log" 2>&1
  stop_pids "${TOPIC_PIDS[@]:-}" "$RX_LAUNCHER_PID"
  if [[ -n "${CURRENT_CASE_DIR:-}" ]]; then
    sleep 0.4
    mute_txs "$CURRENT_CASE_DIR/mute_on_exit.log"
  fi
}
trap cleanup EXIT

summarize_case() {
  local case_dir="$1"
  python3 - "$case_dir" <<'PY'
import ast
import json
import pathlib
import re
import sys

root = pathlib.Path(sys.argv[1])

def load_json_lines(path):
    out = []
    if not path.exists():
        return out
    for line in path.read_text(errors="ignore").splitlines():
        line = line.strip()
        if not line or line == "---" or line.startswith("WARNING"):
            continue
        if line.startswith("data:"):
            line = line.split(":", 1)[1].strip()
        if len(line) >= 2 and line[0] == line[-1] == "'":
            line = line[1:-1]
        if not line.startswith("{"):
            continue
        try:
            out.append(json.loads(line))
        except Exception:
            pass
    return out

def stat_value(status, name):
    return int(((status.get("decoder") or {}).get("stats") or {}).get(name, 0) or 0)

def broadcast_tx_payloads():
    path = root / "tx" / "broadcast_tx.log"
    if not path.exists():
        return {}
    match = re.search(
        r"applied flowgraph setter set_command_cycle=(\[.*\])",
        path.read_text(errors="ignore"),
    )
    if match is None:
        return {}
    try:
        cycle = ast.literal_eval(match.group(1))
    except (SyntaxError, ValueError):
        return {}
    expected = {}
    for item in cycle:
        cmd = [int(value) & 0xFF for value in item.get("cmd_id", [])]
        if len(cmd) == 2:
            expected[(cmd[0] << 8) | cmd[1]] = bytes(
                int(value) & 0xFF for value in item.get("payload_data", [])
            )
    return expected

def summarize_role(role):
    statuses = load_json_lines(root / f"{role}_status.log")
    frames = load_json_lines(root / f"{role}_frames.log")
    bridges = load_json_lines(root / f"{role}_bridge.log")
    print(f"{role}_status_count={len(statuses)}")
    print(f"{role}_frame_count={len(frames)}")
    print(f"{role}_bridge_count={len(bridges)}")
    if statuses:
        print(f"{role}_max_air_payloads={max(stat_value(s, 'air_payloads') for s in statuses)}")
        print(f"{role}_max_iq_air_payloads={max(stat_value(s, 'iq_air_payloads') for s in statuses)}")
        print(f"{role}_max_iq_referee_frames={max(stat_value(s, 'iq_referee_frames') for s in statuses)}")
        print(f"{role}_max_iq_decode_batches={max(stat_value(s, 'iq_decode_batches') for s in statuses)}")
        print(f"{role}_max_frames={max(stat_value(s, 'frames') for s in statuses)}")
        tail = statuses[-1]
        decoder = tail.get("decoder") or {}
        print(f"{role}_tail_stats={json.dumps(decoder.get('stats') or {}, ensure_ascii=False)}")
        print(f"{role}_tail_iq_decode={json.dumps(decoder.get('iq_decode_diagnostics') or {}, ensure_ascii=False)}")
    if frames:
        cmds = [str(frame.get("cmd_hex")) for frame in frames]
        uniq = []
        for cmd in cmds:
            if cmd not in uniq:
                uniq.append(cmd)
        print(f"{role}_cmds_unique={','.join(uniq)}")
        print(f"{role}_cmds_head={','.join(cmds[:24])}")
        print(f"{role}_first_frame={json.dumps(frames[0], ensure_ascii=False)}")
        passwords = sorted({
            str(((frame.get("parsed") or {}).get("password") or ""))
            for frame in frames
            if frame.get("cmd_hex") == "0x0A06" and (frame.get("parsed") or {}).get("password")
        })
        if passwords:
            print(f"{role}_passwords={','.join(passwords)}")
    if role == "rx1":
        expected = broadcast_tx_payloads()
        raw_count = sum(not bool(frame.get("reconstructed")) for frame in frames)
        reconstructed_count = len(frames) - raw_count
        matches = 0
        mismatches = 0
        for frame in frames:
            try:
                cmd_id = int(frame.get("cmd_id"))
                data = bytes.fromhex(str(frame.get("data_hex", "")))
            except (TypeError, ValueError):
                mismatches += 1
                continue
            if cmd_id in expected and data == expected[cmd_id]:
                matches += 1
            else:
                mismatches += 1
        print(f"rx1_raw_frame_count={raw_count}")
        print(f"rx1_reconstructed_frame_count={reconstructed_count}")
        print(f"rx1_tx_payload_match_count={matches}")
        print(f"rx1_tx_payload_mismatch_count={mismatches}")
        audit_pass = bool(frames) and bool(expected) and mismatches == 0
        print(f"rx1_tx_payload_audit_pass={str(audit_pass).lower()}")

def pass_fail_summary():
    rx1_frames = load_json_lines(root / "rx1_frames.log")
    rx2_frames = load_json_lines(root / "rx2_frames.log")
    rx1_cmds = {str(frame.get("cmd_hex")) for frame in rx1_frames}
    rx2_cmds = {str(frame.get("cmd_hex")) for frame in rx2_frames}
    expected_rx1 = {"0x0A01", "0x0A02", "0x0A03", "0x0A04", "0x0A05"}
    rx2_passwords = {
        str(((frame.get("parsed") or {}).get("password") or ""))
        for frame in rx2_frames
        if frame.get("cmd_hex") == "0x0A06" and (frame.get("parsed") or {}).get("password")
    }
    rx1_missing = sorted(expected_rx1 - rx1_cmds)
    rx1_pass = not rx1_missing
    expected_payloads = broadcast_tx_payloads()
    rx1_tx_audit_pass = bool(rx1_frames) and bool(expected_payloads)
    for frame in rx1_frames:
        try:
            cmd_id = int(frame.get("cmd_id"))
            data = bytes.fromhex(str(frame.get("data_hex", "")))
        except (TypeError, ValueError):
            rx1_tx_audit_pass = False
            break
        if cmd_id not in expected_payloads or data != expected_payloads[cmd_id]:
            rx1_tx_audit_pass = False
            break
    rx2_pass = "0x0A06" in rx2_cmds and bool(rx2_passwords)
    print(f"rx1_parse_pass={str(rx1_pass).lower()}")
    print(f"rx1_missing_cmds={','.join(rx1_missing)}")
    print(f"rx2_parse_pass={str(rx2_pass).lower()}")
    print(
        f"case_parse_pass="
        f"{str(rx1_pass and rx1_tx_audit_pass and rx2_pass).lower()}"
    )

for role in ("rx1", "rx2"):
    summarize_role(role)
pass_fail_summary()

for log_name in (
    "rx2.launch.log",
    "dual_tx_launcher.log",
    "tx/broadcast_tx.log",
    "tx/interference_tx.log",
    "mute.log",
    "tx_safety_final.log",
    "check_four_sdr.log",
):
    path = root / log_name
    if not path.exists():
        continue
    text = path.read_text(errors="ignore")
    errors = len(re.findall(r"Traceback|Exception|RCLError|Unable to|failed|No such device", text, flags=re.I))
    pure_u = sum(len(match.group(0)) for match in re.finditer(r"(?m)^U+$", text))
    print(f"{log_name.replace('/', '_')}_error_lines={errors}")
    if pure_u:
        print(f"{log_name.replace('/', '_')}_pure_U_count={pure_u}")
PY
}

run_level() {
  local level="$1"
  export INTERFERENCE_LEVEL="$level"
  export RM_RADIO_INTERFERENCE_LEVEL="$level"
  local case_dir="$LOG_ROOT/level${level}"
  local tx_dir="$case_dir/tx"
  CURRENT_CASE_DIR="$case_dir"
  CURRENT_TX_DIR="$tx_dir"
  rm_radio_ensure_dir "$case_dir"
  rm_radio_ensure_dir "$tx_dir"

  {
    echo "LOG_DIR=$case_dir"
    echo "mode=four_sdr_live_parse"
    echo "start=$(date --iso-8601=seconds)"
    echo "side=$RM_RADIO_SIDE level=$INTERFERENCE_LEVEL"
    echo "broadcast_tx=$BROADCAST_TX_URI attenuation=$BROADCAST_TX_ATTENUATION_DB"
    echo "interference_tx=$INTERFERENCE_TX_URI attenuation=$INTERFERENCE_TX_ATTEND_GR"
    echo "rx1=$RX1_URI rx2=$RX2_URI"
    echo "test_seconds=$TEST_SECONDS rx_warmup_seconds=$RX_WARMUP_SECONDS"
    echo "broadcast_rx_gain_mode=$BROADCAST_RX_GAIN_MODE gain=$BROADCAST_RX_GAIN"
    echo "broadcast_demod_mode=$BROADCAST_DEMOD_MODE iq_snapshot_enabled=$BROADCAST_IQ_DECODER_ENABLED"
    echo "broadcast_iio_capture_decoder_enabled=$BROADCAST_IIO_CAPTURE_DECODER_ENABLED only=$BROADCAST_IIO_CAPTURE_DECODER_ONLY"
    echo "broadcast_iq_decoder_autotune_mode=$BROADCAST_IQ_DECODER_AUTOTUNE_MODE low_pass=$BROADCAST_IQ_DECODER_LOW_PASS_HZ"
  } >"$case_dir/summary.txt"

  LOG_DIR="$case_dir/check" bash "$RADIO_TX_SCRIPT_DIR/check_four_sdr.sh" >"$case_dir/check_four_sdr.log" 2>&1
  mute_txs "$case_dir/mute_before.log"

  bash "$RM_RADIO_WS/apps/match_rx/start.sh" --rx-only >"$case_dir/rx2.launch.log" 2>&1 &
  RX_LAUNCHER_PID=$!
  sleep "$RX_WARMUP_SECONDS"
  if ! kill -0 "$RX_LAUNCHER_PID" >/dev/null 2>&1; then
    wait "$RX_LAUNCHER_PID" 2>/dev/null || true
    RX_LAUNCHER_PID=""
    echo "dual RX failed during warmup; see $case_dir/rx2.launch.log" >&2
    sed -n '1,160p' "$case_dir/rx2.launch.log" >&2 || true
    return 3
  fi

  timeout "${TOPIC_TIMEOUT_SECONDS}s" ros2 topic echo /rm_gfsk_node/status --field data --no-daemon >"$case_dir/rx1_status.log" 2>&1 &
  TOPIC_PIDS+=("$!")
  timeout "${TOPIC_TIMEOUT_SECONDS}s" ros2 topic echo /rm_gfsk_node/frames --field data --no-daemon >"$case_dir/rx1_frames.log" 2>&1 &
  TOPIC_PIDS+=("$!")
  timeout "${TOPIC_TIMEOUT_SECONDS}s" ros2 topic echo /rm_gfsk_node/referee_bridge --field data --no-daemon >"$case_dir/rx1_bridge.log" 2>&1 &
  TOPIC_PIDS+=("$!")
  timeout "${TOPIC_TIMEOUT_SECONDS}s" ros2 topic echo /rm_gfsk_interference_node/status --field data --no-daemon >"$case_dir/rx2_status.log" 2>&1 &
  TOPIC_PIDS+=("$!")
  timeout "${TOPIC_TIMEOUT_SECONDS}s" ros2 topic echo /rm_gfsk_interference_node/frames --field data --no-daemon >"$case_dir/rx2_frames.log" 2>&1 &
  TOPIC_PIDS+=("$!")
  timeout "${TOPIC_TIMEOUT_SECONDS}s" ros2 topic echo /rm_gfsk_interference_node/referee_bridge --field data --no-daemon >"$case_dir/rx2_bridge.log" 2>&1 &
  TOPIC_PIDS+=("$!")
  sleep 1

  LOG_DIR="$tx_dir" bash "$RADIO_TX_SCRIPT_DIR/start_dual_tx.sh" >"$case_dir/dual_tx_launcher.log" 2>&1 &
  TX_LAUNCHER_PID=$!
  for _ in $(seq 1 50); do
    [[ -s "$tx_dir/dual_tx.pids" ]] && break
    kill -0 "$TX_LAUNCHER_PID" >/dev/null 2>&1 || break
    sleep 0.1
  done
  if ! lab_tx_verify_dual_running "$TX_LAUNCHER_PID" "$tx_dir/dual_tx.pids"; then
    wait "$TX_LAUNCHER_PID" 2>/dev/null || true
    echo "dual TX failed to start; see $case_dir/dual_tx_launcher.log" >&2
    sed -n '1,120p' "$case_dir/dual_tx_launcher.log" >&2 || true
    return 3
  fi
  sleep "$TX_STARTUP_VERIFY_SECONDS"
  if ! lab_tx_verify_dual_running "$TX_LAUNCHER_PID" "$tx_dir/dual_tx.pids"; then
    echo "dual TX failed during startup warmup; see $case_dir/dual_tx_launcher.log and $tx_dir" >&2
    return 3
  fi
  if ((TEST_SECONDS > TX_STARTUP_VERIFY_SECONDS)); then
    sleep "$((TEST_SECONDS - TX_STARTUP_VERIFY_SECONDS))"
  fi
  if ! lab_tx_verify_dual_running "$TX_LAUNCHER_PID" "$tx_dir/dual_tx.pids"; then
    echo "dual TX exited before the test interval completed; results are invalid" >&2
    return 3
  fi
  stop_dual_tx_for_case "$TX_LAUNCHER_PID" "$tx_dir"
  TX_LAUNCHER_PID=""
  if ! finalize_tx_safety "$case_dir/tx_safety_final.log"; then
    echo "TX safety finalization failed; see $case_dir/tx_safety_final.log" >&2
    return 4
  fi
  sleep "$POST_TX_SECONDS"

  stop_pids "${TOPIC_PIDS[@]}"
  TOPIC_PIDS=()
  stop_pids "$RX_LAUNCHER_PID"
  RX_LAUNCHER_PID=""

  {
    echo "end=$(date --iso-8601=seconds)"
    summarize_case "$case_dir"
  } >>"$case_dir/summary.txt"
  cat "$case_dir/summary.txt"
}

for level in $TEST_LEVELS; do
  run_level "$level"
done

echo "all_logs=$LOG_ROOT"
