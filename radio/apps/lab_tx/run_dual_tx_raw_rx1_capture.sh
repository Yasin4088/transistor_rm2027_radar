#!/usr/bin/env bash
set -euo pipefail

RADIO_TX_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$RADIO_TX_SCRIPT_DIR/env.sh"
# shellcheck disable=SC1091
source "$RM_RADIO_WS/apps/common/lab_tx_safety.sh"

export RM_RADIO_RX_ONLY=false
export AUTO_INTERFERENCE_LEVEL=false
export AUTO_RX_INTERFERENCE_LEVEL=false

require_lab_tx_confirmation

LEVEL="${1:-${INTERFERENCE_LEVEL:-2}}"
export INTERFERENCE_LEVEL="$LEVEL"
export RM_RADIO_INTERFERENCE_LEVEL="$LEVEL"

CAPTURE_SECONDS="${CAPTURE_SECONDS:-2.0}"
TX_WARMUP_SECONDS="${TX_WARMUP_SECONDS:-8}"
RX_GAIN_MODE="${RX_GAIN_MODE:-$BROADCAST_RX_GAIN_MODE}"
RX_GAIN="${RX_GAIN:-$BROADCAST_RX_GAIN}"
RX_BANDWIDTH="${RX_BANDWIDTH:-540000}"
RX_SAMPLE_RATE="${RX_SAMPLE_RATE:-2000000}"
SCAN_LOW_PASS_HZ="${SCAN_LOW_PASS_HZ:-260000}"
RUN_AUTOTUNE="${RUN_AUTOTUNE:-false}"
RUN_OFFLINE_FULL="${RUN_OFFLINE_FULL:-false}"
RUN_BROADCAST_SCAN="${RUN_BROADCAST_SCAN:-true}"
LOG_ROOT="${LOG_ROOT:-$RM_RADIO_WS/log/dual_tx_raw_rx1_capture_$(date +%Y%m%d_%H%M%S)_lvl${LEVEL}}"

rm_radio_ensure_dir "$LOG_ROOT"

TX_LAUNCHER_PID=""
TX_RESIDUAL_PATTERNS=(
  "rm_broadcast_tx_node"
  "rm_interference_tx_node"
  "start_dual_tx.sh"
  "flowgraph_name:=ganraoyuan"
)

stop_pids() {
  lab_tx_stop_pids "$@"
}

stop_dual_tx() {
  local pids=()
  [[ -n "$TX_LAUNCHER_PID" ]] && pids+=("$TX_LAUNCHER_PID")
  if [[ -f "$LOG_ROOT/tx/dual_tx.pids" ]]; then
    while IFS='=' read -r _name pid; do
      [[ "$pid" =~ ^[0-9]+$ ]] && pids+=("$pid")
    done < "$LOG_ROOT/tx/dual_tx.pids"
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
  stop_dual_tx
  lab_tx_cleanup_residual_tx "${TX_RESIDUAL_PATTERNS[@]}" >>"$LOG_ROOT/tx_safety_on_exit.log" 2>&1
  sleep 0.4
  mute_txs "$LOG_ROOT/tx_safety_on_exit.log"
}
trap cleanup EXIT

convert_s16_to_fc32() {
  local input="$1"
  local output="$2"
  python3 - "$input" "$output" "$RX_SAMPLE_RATE" <<'PY'
import json
import sys
import numpy as np

raw = np.fromfile(sys.argv[1], dtype="<i2")
if raw.size % 2:
    raw = raw[:-1]
iq = raw.reshape(-1, 2).astype(np.float32) / 32768.0
out = (iq[:, 0] + 1j * iq[:, 1]).astype(np.complex64)
out.tofile(sys.argv[2])
mag = np.abs(out)
sample_rate = float(sys.argv[3])
print(json.dumps({
    "samples": int(out.size),
    "seconds": float(out.size) / sample_rate if sample_rate > 0 else 0.0,
    "mean_abs": float(np.mean(mag)) if mag.size else 0.0,
    "rms_abs": float(np.sqrt(np.mean(np.square(mag)))) if mag.size else 0.0,
    "max_abs": float(np.max(mag)) if mag.size else 0.0,
}, ensure_ascii=False))
PY
}

summarize() {
  python3 - "$LOG_ROOT" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])

def read_json(path):
    try:
        text = path.read_text(errors="ignore").strip().splitlines()
    except FileNotFoundError:
        return None
    for line in reversed(text):
        line = line.strip()
        if line.startswith("{"):
            try:
                return json.loads(line)
            except Exception:
                pass
    return None

summary = {
    "capture_stats": read_json(root / "capture_stats.json"),
  "broadcast_scan": read_json(root / "broadcast_scan.json"),
  "offline_full": read_json(root / "offline_full.json"),
    "autotune": read_json(root / "autotune.json"),
}
for key in ("broadcast_scan", "offline_full"):
    value = summary.get(key) or {}
    if value:
        # The targeted broadcast scan stores threshold variants and a selected
        # summary one level down.  Report that selected summary to the sweep.
        if key == "broadcast_scan" and isinstance(value.get("best_summary"), dict):
            value = value["best_summary"]
        summary[key] = {
            "best_access": value.get("best_access"),
            "best_access_hamming_distance": value.get("best_access_hamming_distance"),
            "air_payload_count": value.get("air_payload_count"),
            "referee_frame_count": value.get("referee_frame_count"),
            "commands": value.get("commands"),
            "first_referee_frame": value.get("first_referee_frame"),
        }
auto = summary.get("autotune") or {}
if auto:
    summary["autotune"] = {
        "best_candidate": auto.get("best_candidate"),
        "best_score": auto.get("best_score"),
        "best_summary": auto.get("best_summary"),
        "peak_offsets_hz": auto.get("peak_offsets_hz"),
    }
print(json.dumps(summary, ensure_ascii=False, indent=2))
PY
}

side="${RM_RADIO_SIDE,,}"
case "$side" in
  red) broadcast_freq=433200000 ;;
  blue) broadcast_freq=433920000 ;;
  *) echo "RM_RADIO_SIDE must be red or blue; got $RM_RADIO_SIDE" >&2; exit 2 ;;
esac
RX_CENTER_FREQUENCY="${RX_CENTER_FREQUENCY:-$broadcast_freq}"

rm_radio_ensure_dir "$LOG_ROOT/tx"

{
  echo "LOG_ROOT=$LOG_ROOT"
  echo "start=$(date --iso-8601=seconds)"
  echo "side=$RM_RADIO_SIDE level=$INTERFERENCE_LEVEL"
  echo "broadcast_tx=$BROADCAST_TX_URI attenuation=$BROADCAST_TX_ATTENUATION_DB"
  echo "interference_tx=$INTERFERENCE_TX_URI attenuation=$INTERFERENCE_TX_ATTEND_GR"
  echo "rx1=$RX1_URI rx_gain_mode=$RX_GAIN_MODE rx_gain=$RX_GAIN rx_bw=$RX_BANDWIDTH rx_sr=$RX_SAMPLE_RATE"
  echo "rx1_center_frequency=$RX_CENTER_FREQUENCY official_broadcast_frequency=$broadcast_freq"
} >"$LOG_ROOT/summary.txt"

LOG_DIR="$LOG_ROOT/check" bash "$RADIO_TX_SCRIPT_DIR/check_four_sdr.sh" >"$LOG_ROOT/check_four_sdr.log" 2>&1

# shellcheck disable=SC1091
source "$RM_RADIO_WS/apps/common/iio_init.sh"
mute_txs "$LOG_ROOT/mute_before.log"
init_ad9361_rx "$RX1_URI" "$RX_CENTER_FREQUENCY" "$RX_BANDWIDTH" "$RX_GAIN_MODE" "$RX_GAIN" "$RX_RF_PORT" "$RX_SAMPLE_RATE" 2>"$LOG_ROOT/rx_init.log"

LOG_DIR="$LOG_ROOT/tx" bash "$RADIO_TX_SCRIPT_DIR/start_dual_tx.sh" >"$LOG_ROOT/dual_tx_launcher.log" 2>&1 &
TX_LAUNCHER_PID=$!

# Do not record and label pure noise as a radio test when either TX failed to
# start.  Wait briefly for the launcher to publish both child PIDs, then verify
# that all three processes are alive before the RF warmup begins.
for _ in $(seq 1 50); do
  [[ -s "$LOG_ROOT/tx/dual_tx.pids" ]] && break
  kill -0 "$TX_LAUNCHER_PID" >/dev/null 2>&1 || break
  sleep 0.1
done
if ! lab_tx_verify_dual_running "$TX_LAUNCHER_PID" "$LOG_ROOT/tx/dual_tx.pids"; then
  wait "$TX_LAUNCHER_PID" 2>/dev/null || true
  echo "dual TX failed to start; see $LOG_ROOT/dual_tx_launcher.log" >&2
  sed -n '1,120p' "$LOG_ROOT/dual_tx_launcher.log" >&2 || true
  exit 3
fi
sleep "$TX_WARMUP_SECONDS"
if ! lab_tx_verify_dual_running "$TX_LAUNCHER_PID" "$LOG_ROOT/tx/dual_tx.pids"; then
  echo "dual TX failed during warmup; see $LOG_ROOT/dual_tx_launcher.log and $LOG_ROOT/tx" >&2
  exit 3
fi

set +e
timeout "$CAPTURE_SECONDS" iio_readdev -u "$RX1_URI" -b 32768 cf-ad9361-lpc voltage0 voltage1 >"$LOG_ROOT/rx1_s16.bin" 2>"$LOG_ROOT/iio_readdev.err"
read_rc=$?
set -e
echo "iio_readdev_rc=$read_rc" >>"$LOG_ROOT/summary.txt"

stop_dual_tx
TX_LAUNCHER_PID=""
if ! finalize_tx_safety "$LOG_ROOT/tx_safety_final.log"; then
  echo "TX safety finalization failed; see $LOG_ROOT/tx_safety_final.log" >&2
  exit 4
fi

if [[ ! -s "$LOG_ROOT/rx1_s16.bin" ]]; then
  echo "rx1_s16_empty=true" >>"$LOG_ROOT/summary.txt"
  summarize | tee "$LOG_ROOT/result_summary.json"
  exit 1
fi

convert_s16_to_fc32 "$LOG_ROOT/rx1_s16.bin" "$LOG_ROOT/rx1.fc32" >"$LOG_ROOT/capture_stats.json"

export PYTHONPATH="$RM_RADIO_WS/src/rm_radio_ros:${PYTHONPATH:-}"
if [[ "$RUN_BROADCAST_SCAN" == "true" ]]; then
python3 - "$LOG_ROOT/rx1.fc32" "$SCAN_LOW_PASS_HZ" >"$LOG_ROOT/broadcast_scan.json" 2>"$LOG_ROOT/broadcast_scan.err" <<'PY' || true
import json
import sys
from pathlib import Path

from rm_radio_ros.core.offline_iq_scan import scan_iq_file_windows, summarize_scan_result

path = Path(sys.argv[1])
low_pass_hz = float(sys.argv[2])
reports = []
for threshold in (3, 6, 10):
    result = scan_iq_file_windows(
        path,
        window_seconds=1.0,
        step_seconds=0.5,
        sample_rate=2_000_000.0,
        sps_values=(92, 92.5, 93, 93.5, 94, 94.5, 95),
        offset_step=2,
        decode=True,
        decode_threshold=threshold,
        max_decoded_payloads=4096,
        max_decoded_frames=512,
        aggregate_decode=True,
        low_pass_hz=low_pass_hz,
        allowed_access_names=("broadcast",),
    )
    reports.append({
        "threshold": threshold,
        "summary": summarize_scan_result(result, frame_limit=10),
    })
print(json.dumps({
    "file": str(path),
    "low_pass_hz": low_pass_hz,
    "reports": reports,
    "best_summary": max(
        (item["summary"] for item in reports),
        key=lambda summary: (
            int(summary.get("referee_frame_count") or 0),
            int(summary.get("air_payload_count") or 0),
            -int(summary.get("best_access_hamming_distance") or 999),
        ),
    ),
}, ensure_ascii=False))
PY
fi

if [[ "$RUN_OFFLINE_FULL" == "true" ]]; then
python3 -m rm_radio_ros.core.offline_iq_scan \
  --summary \
  --summary-frame-limit 10 \
  --decode-threshold 3 \
  --max-decoded-payloads 2048 \
  --max-decoded-frames 512 \
  "$LOG_ROOT/rx1.fc32" >"$LOG_ROOT/offline_full.json" 2>"$LOG_ROOT/offline_full.err" || true
fi

if [[ "$RUN_AUTOTUNE" == "true" ]]; then
python3 - "$LOG_ROOT/rx1.fc32" "$INTERFERENCE_LEVEL" "$SCAN_LOW_PASS_HZ" >"$LOG_ROOT/autotune.json" 2>"$LOG_ROOT/autotune.err" <<'PY' || true
import json
import sys
from pathlib import Path
from rm_radio_ros.core.rx_autotune import autotune_iq_file

result = autotune_iq_file(
    Path(sys.argv[1]),
    profile="broadcast",
    level=int(sys.argv[2]),
    sample_rate=2_000_000.0,
    base_low_pass_hz=float(sys.argv[3]),
    mode="quick",
    window_seconds=1.0,
    step_seconds=0.5,
    offset_step=2,
    max_frequency_candidates=8,
    max_candidates=24,
)
print(json.dumps(result, ensure_ascii=False))
PY
fi

summarize | tee "$LOG_ROOT/result_summary.json"
echo "end=$(date --iso-8601=seconds)" >>"$LOG_ROOT/summary.txt"
echo "all_logs=$LOG_ROOT"
