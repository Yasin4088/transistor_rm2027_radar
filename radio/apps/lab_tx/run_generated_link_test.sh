#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export RX_URI="${RX_URI:-ip:192.168.1.10}"
export RM_RADIO_SIDE="${RM_RADIO_SIDE:-blue}"
export INTERFERENCE_LEVEL="${INTERFERENCE_LEVEL:-1}"
export TX_RF_PORT="${TX_RF_PORT:-A}"
export RX_RF_PORT="${RX_RF_PORT:-A_BALANCED}"
export BROADCAST_RX_GAIN_MODE="${BROADCAST_RX_GAIN_MODE:-fast_attack}"
export INTERFERENCE_RX_GAIN_MODE="${INTERFERENCE_RX_GAIN_MODE:-fast_attack}"
export BROADCAST_RX_GAIN="${BROADCAST_RX_GAIN:-20}"
export INTERFERENCE_RX_GAIN="${INTERFERENCE_RX_GAIN:-20}"
export RM_RADIO_IIO_PREINIT="${RM_RADIO_IIO_PREINIT:-true}"

# shellcheck disable=SC1091
source "$SCRIPT_DIR/env.sh"

require_lab_tx_confirmation
require_cabled_loop_attenuation

# This test reuses one explicitly selected TX sequentially for its cases.  Do
# not silently claim a generic usb: context, which may be the information SDR.
TX_URI="${GENERATED_TX_URI:-$BROADCAST_TX_URI}"
if [[ -z "$TX_URI" ]]; then
  echo "GENERATED_TX_URI/BROADCAST_TX_URI is empty" >&2
  exit 2
fi
RX_URI="$RX_URI"
SIDE="${RM_RADIO_SIDE,,}"
LEVEL="$INTERFERENCE_LEVEL"
TX_SAMPLE_RATE="${GENERATED_TX_SAMPLE_RATE:-$RM_RADIO_SAMPLE_RATE}"
RX_SAMPLE_RATE="${GENERATED_RX_SAMPLE_RATE:-2000000}"
export RM_RADIO_SAMPLE_RATE="$TX_SAMPLE_RATE"
CASES="${GENERATED_LINK_TEST_CASES:-broadcast interference}"
ATTENUATIONS="${GENERATED_LINK_ATTENUATIONS:-60 50 40 30}"
TX_SECONDS="${GENERATED_LINK_TX_SECONDS:-12}"
RX_WARMUP_SECONDS="${GENERATED_LINK_RX_WARMUP_SECONDS:-4}"
CAPTURE_SECONDS="${GENERATED_LINK_CAPTURE_SECONDS:-3}"
CAPTURE_SETTLE_SECONDS="${GENERATED_LINK_CAPTURE_SETTLE_SECONDS:-4}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-86}"
LOG_DIR="${LOG_DIR:-${RM_RADIO_LOG_ROOT:-/tmp/rm_radio_runtime_logs}/generated_link_test_$(date +%Y%m%d_%H%M%S)}"

rm_radio_ensure_dir "$LOG_DIR"

# shellcheck disable=SC1091
source "$RM_RADIO_WS/apps/common/iio_init.sh"

case "$SIDE" in
  red|blue) ;;
  *) echo "RM_RADIO_SIDE must be red or blue; got $SIDE" >&2; exit 2 ;;
esac

case "$LEVEL" in
  1|2|3) ;;
  *) echo "INTERFERENCE_LEVEL must be 1, 2, or 3; got $LEVEL" >&2; exit 2 ;;
esac

broadcast_frequency() {
  case "$1" in
    red) echo 433200000 ;;
    blue) echo 433920000 ;;
    *) return 2 ;;
  esac
}

case_frequency_bw_lp() {
  local kind="$1"
  if [[ "$kind" == "broadcast" ]]; then
    echo "$(broadcast_frequency "$SIDE") 540000 320000"
    return
  fi
  local freq bw
  read -r freq bw < <(tx_frequency_for_side_level "$SIDE" "$LEVEL")
  if [[ "$LEVEL" == "3" ]]; then
    echo "$freq $bw 160000"
  else
    echo "$freq $bw 500000"
  fi
}

rx_profile_for_case() {
  [[ "$1" == "broadcast" ]] && echo "broadcast" || echo "interference"
}

rx_gain_for_case() {
  [[ "$1" == "broadcast" ]] && echo "$BROADCAST_RX_GAIN" || echo "$INTERFERENCE_RX_GAIN"
}

rx_gain_mode_for_case() {
  [[ "$1" == "broadcast" ]] && echo "$BROADCAST_RX_GAIN_MODE" || echo "$INTERFERENCE_RX_GAIN_MODE"
}

rx_setters_json() {
  local kind="$1"
  local freq="$2"
  local bw="$3"
  local lp="$4"
  python3 - "$kind" "$freq" "$bw" "$lp" "$LEVEL" <<'PY'
import json
import sys

kind, freq, bw, low_pass, level = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5])
if kind == "broadcast":
    sen_re = 1.5628 / 2.0
    sncy = None
else:
    sen_re = {1: 2.8194 / 2.0, 2: 2.5681 / 2.0, 3: 0.6517 / 2.0}[level]
    sncy = "sncy_gr"
out = {
    "rx_profile": kind,
    "cen_f": freq,
    "BW": bw,
    "bw_re": bw,
    "LowPass": low_pass,
    "sen_re": sen_re,
    "GainMode": "fast_attack",
    "Gain": 20,
}
if sncy:
    out["sncy"] = sncy
print(json.dumps(out, separators=(",", ":")))
PY
}

tx_setters_json() {
  local kind="$1"
  local freq="$2"
  local bw="$3"
  local attenuation="$4"
  local password="${INTERFERENCE_PASSWORD:-R1L001}"
  python3 - "$kind" "$freq" "$bw" "$attenuation" "$password" <<'PY'
import json
import sys

kind, freq, bw, attenuation, password = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), float(sys.argv[4]), sys.argv[5]
broadcast_access = [0x2F, 0x6F, 0x4C, 0x74, 0xB9, 0x14, 0x49, 0x2E]
interference_access = [0x16, 0xE8, 0xD3, 0x77, 0x15, 0x1C, 0x71, 0x2D]
if kind == "broadcast":
    cycle = [
        {"cmd_id": [0x0A, 0x01], "payload_data": [0xF2,0x03,0xE4,0x07,0x56,0x04,0x48,0x08,0xBA,0x04,0xAC,0x08,0x1E,0x05,0x10,0x09,0x82,0x05,0x74,0x09,0xE6,0x05,0xD8,0x09]},
        {"cmd_id": [0x0A, 0x02], "payload_data": [0xF4,0x01,0xC2,0x01,0x90,0x01,0x86,0x01,0x00,0x00,0x58,0x02]},
        {"cmd_id": [0x0A, 0x03], "payload_data": [0x2D,0x00,0x78,0x00,0x76,0x00,0x00,0x00,0x2C,0x01]},
        {"cmd_id": [0x0A, 0x04], "payload_data": [0x7B,0x00,0xC8,0x01,0x6D,0x20,0x00,0x00]},
        {"cmd_id": [0x0A, 0x05], "payload_data": list(range(1, 42))},
    ]
    out = {
        "center_f": freq,
        "BW_ganrao": bw,
        "attend_gr": attenuation,
        "access": broadcast_access,
        "command_cycle": cycle,
        "Period": 100.0,
    }
else:
    data = list(password.encode("ascii")[:6])
    if len(data) != 6:
        raise SystemExit("INTERFERENCE_PASSWORD must be 6 ASCII bytes")
    out = {
        "center_f": freq,
        "BW_ganrao": bw,
        "attend_gr": attenuation,
        "access": interference_access,
        "cmd_id": [0x0A, 0x06],
        "payload_data": data,
        "Period": 100.0,
    }
print(json.dumps(out, separators=(",", ":")))
PY
}

write_rx_params() {
  local path="$1"
  local profile="$2"
  local rx_setters="$3"
  cat > "$path" <<YAML
rm_gfsk_node:
  ros__parameters:
    flowgraph_name: "gfsk"
    flowgraph_dir: "$RM_RADIO_WS/src/rm_radio_ros/flowgraphs/gfsk"
    flowgraph_module: "RM"
    flowgraph_class: "RM"
    auto_start: true
    dry_run: false
    qt_platform: "offscreen"
    disable_gui_sinks: true
    rx_only: true
    rx_uri: "$RX_URI"
    radio_side: "$SIDE"
    rx_profile: "$profile"
    interference_level: $LEVEL
    status_period_sec: 1.0
    setters_apply_settle_sec: 0.2
    air_extractor_max_access_hamming: 3
    air_extractor_allow_inverted: true
    demod_bit_diagnostics_every_n_batches: 5
    setters_json: '$rx_setters'
YAML
}

write_tx_params() {
  local path="$1"
  local tx_setters="$2"
  cat > "$path" <<YAML
rm_ganraoyuan_node:
  ros__parameters:
    flowgraph_name: "ganraoyuan"
    flowgraph_dir: "$RM_RADIO_WS/src/rm_radio_ros/flowgraphs/ganraoyuan"
    flowgraph_module: "RM"
    flowgraph_class: "RM"
    auto_start: true
    dry_run: false
    qt_platform: "offscreen"
    disable_gui_sinks: true
    qt_event_period_sec: 0.05
    interference_tx_uri: "$TX_URI"
    radio_side: "$SIDE"
    interference_level: $LEVEL
    status_period_sec: 1.0
    setters_json: '$tx_setters'
YAML
}

stop_pids() {
  set +e
  for pid in "$@"; do
    if [[ -n "${pid:-}" ]]; then
      kill "$pid" >/dev/null 2>&1 || true
    fi
  done
  sleep 0.4
  for pid in "$@"; do
    if [[ -n "${pid:-}" ]]; then
      kill -9 "$pid" >/dev/null 2>&1 || true
    fi
  done
  set -e
}

convert_s16_to_fc32() {
  local input="$1"
  local output="$2"
  python3 - "$input" "$output" <<'PY'
import sys
import numpy as np

raw = np.fromfile(sys.argv[1], dtype="<i2")
if raw.size % 2:
    raw = raw[:-1]
iq = raw.reshape(-1, 2).astype(np.float32) / 32768.0
out = (iq[:, 0] + 1j * iq[:, 1]).astype(np.complex64)
out.tofile(sys.argv[2])
print(f"samples={out.size} mean_abs={float(np.mean(np.abs(out))):.8g} max_abs={float(np.max(np.abs(out))):.8g}")
PY
}

summarize_case() {
  local case_dir="$1"
  python3 - "$case_dir" <<'PY'
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
            line = line.split(":", 1)[1].strip().strip("'")
        if not line.startswith("{"):
            continue
        try:
            out.append(json.loads(line))
        except Exception:
            pass
    return out

statuses = load_json_lines(root / "status.log")
frames = load_json_lines(root / "frames.log")
bridges = load_json_lines(root / "bridge.log")
print(f"status_json_count={len(statuses)}")
print(f"frame_json_count={len(frames)}")
print(f"bridge_json_count={len(bridges)}")
if statuses:
    def stat_value(status, name):
        return int(status.get("decoder", {}).get("stats", {}).get(name, 0) or 0)
    print(f"max_air_payloads={max(stat_value(s, 'air_payloads') for s in statuses)}")
    print(f"max_iq_air_payloads={max(stat_value(s, 'iq_air_payloads') for s in statuses)}")
    print(f"max_iq_decode_batches={max(stat_value(s, 'iq_decode_batches') for s in statuses)}")
    print(f"max_frames={max(stat_value(s, 'frames') for s in statuses)}")
    print(f"max_bridge_outputs={max(stat_value(s, 'bridge_outputs') for s in statuses)}")
    for idx, status in list(enumerate(statuses))[-5:]:
        dec = status.get("decoder", {})
        stats = dec.get("stats", {})
        iq = dec.get("iq_diagnostics", {})
        iq_dec = dec.get("iq_decode_diagnostics", {})
        ext = dec.get("extractor_diagnostics", {})
        print(
            "status_tail",
            idx,
            "air", stats.get("air_payloads"),
            "iq_air", stats.get("iq_air_payloads"),
            "iq_batches", stats.get("iq_decode_batches"),
            "frames", stats.get("frames"),
            "bridge", stats.get("bridge_outputs"),
            "mean_abs", iq.get("mean_abs"),
            "max_abs", iq.get("max_abs"),
            "iq_best", iq_dec.get("best_access_match"),
            "iq_published", iq_dec.get("published_frames"),
            "broadcast_ham", (ext.get("broadcast") or {}).get("best_hamming"),
            "broadcast_inv", (ext.get("broadcast") or {}).get("best_hamming_inverted"),
            "interf_ham", (ext.get("interference") or {}).get("best_hamming"),
            "interf_inv", (ext.get("interference") or {}).get("best_hamming_inverted"),
        )
if frames:
    print("frame_cmds_head=" + ",".join(str(f.get("cmd_hex")) for f in frames[:12]))
    print("first_frame=" + json.dumps(frames[0], ensure_ascii=False))
if bridges:
    print("first_bridge=" + json.dumps(bridges[0], ensure_ascii=False))

for name in ("offline_lp.log", "offline_full.log"):
    path = root / name
    if path.exists():
        text = path.read_text(errors="ignore").strip().splitlines()
        if text:
            print(f"{name}={text[-1]}")
for txlog in sorted(root.glob("tx*.log")):
    text = txlog.read_text(errors="ignore")
    pure_u = sum(len(m.group(0)) for m in re.finditer(r"(?m)^U+$", text))
    errors = len(re.findall(r"Traceback|Exception|RCLError|Unable to", text))
    print(f"{txlog.name}: pure_U_count={pure_u} error_lines={errors}")
PY
}

cat > "$LOG_DIR/summary.txt" <<EOF
LOG_DIR=$LOG_DIR
mode=generated_link_test
tx_uri=$TX_URI
rx_uri=$RX_URI
side=$SIDE
interference_level=$LEVEL
tx_rf_port=$TX_RF_PORT
rx_rf_port=$RX_RF_PORT
tx_sample_rate=$TX_SAMPLE_RATE
rx_sample_rate=$RX_SAMPLE_RATE
cases=$CASES
attenuations=$ATTENUATIONS
start=$(date --iso-8601=seconds)
EOF

export PYTHONPATH="$RM_RADIO_WS/src/rm_radio_ros:${PYTHONPATH:-}"
export ROS_DOMAIN_ID
export QT_QPA_PLATFORM=offscreen
export RM_RADIO_TX_BUFFER_SIZE="${RM_RADIO_TX_BUFFER_SIZE:-1048576}"
export RM_RADIO_TX_PREFILL_PACKETS="${RM_RADIO_TX_PREFILL_PACKETS:-0}"
export RM_RADIO_RX_TRACKING="${RM_RADIO_RX_TRACKING:-true}"

for kind in $CASES; do
  if [[ "$kind" != "broadcast" && "$kind" != "interference" ]]; then
    echo "skip unknown case: $kind" | tee -a "$LOG_DIR/summary.txt"
    continue
  fi
  read -r freq bw lowpass < <(case_frequency_bw_lp "$kind")
  profile="$(rx_profile_for_case "$kind")"
  rx_gain="$(rx_gain_for_case "$kind")"
  rx_gain_mode="$(rx_gain_mode_for_case "$kind")"
  for att in $ATTENUATIONS; do
    case_dir="$LOG_DIR/${kind}_att${att}"
    rm_radio_ensure_dir "$case_dir"
    rx_setters="$(rx_setters_json "$kind" "$freq" "$bw" "$lowpass")"
    tx_setters="$(tx_setters_json "$kind" "$freq" "$bw" "$att")"
    write_rx_params "$case_dir/rx_params.yaml" "$profile" "$rx_setters"
    write_tx_params "$case_dir/tx_params.yaml" "$tx_setters"
    {
      echo "--- $kind att=$att ---"
      echo "freq=$freq bw=$bw lowpass=$lowpass rx_profile=$profile"
      echo "rx_setters=$rx_setters"
      echo "tx_setters=$tx_setters"
    } >> "$LOG_DIR/summary.txt"

    if [[ "$RM_RADIO_IIO_PREINIT" == "true" ]]; then
      init_ad9361_rx "$RX_URI" "$freq" "$bw" "$rx_gain_mode" "$rx_gain" "$RX_RF_PORT" "$RX_SAMPLE_RATE" 2>>"$case_dir/iio_init.log"
      tx_preinit_mode="$(ad9361_tx_preinit_mode "$TX_SAMPLE_RATE")"
      init_ad9361_tx "$TX_URI" "$freq" "$bw" "$att" "$TX_RF_PORT" "$TX_SAMPLE_RATE" "$tx_preinit_mode" 2>>"$case_dir/iio_init.log"
    fi

    python3 -c 'from rm_radio_ros.nodes.gfsk_node import main; main()' --ros-args --params-file "$case_dir/rx_params.yaml" > "$case_dir/rx.log" 2>&1 &
    rx_pid=$!
    sleep "$RX_WARMUP_SECONDS"

    timeout "$((TX_SECONDS + 8))s" ros2 topic echo /rm_gfsk_node/status --field data --no-daemon > "$case_dir/status.log" 2>&1 &
    status_pid=$!
    timeout "$((TX_SECONDS + 8))s" ros2 topic echo /rm_gfsk_node/frames --field data --no-daemon > "$case_dir/frames.log" 2>&1 &
    frames_pid=$!
    timeout "$((TX_SECONDS + 8))s" ros2 topic echo /rm_gfsk_node/referee_bridge --field data --no-daemon > "$case_dir/bridge.log" 2>&1 &
    bridge_pid=$!
    sleep 1

    set +e
    timeout "${TX_SECONDS}s" python3 -c 'from rm_radio_ros.nodes.ganraoyuan_node import main; main()' --ros-args --params-file "$case_dir/tx_params.yaml" > "$case_dir/tx.log" 2>&1
    echo "tx_ros_rc=$?" >> "$case_dir/result.txt"
    set -e
    sleep 2
    stop_pids "$status_pid" "$frames_pid" "$bridge_pid" "$rx_pid"
    sleep 1

    if [[ "$RM_RADIO_IIO_PREINIT" == "true" ]]; then
      init_ad9361_rx "$RX_URI" "$freq" "$bw" "$rx_gain_mode" "$rx_gain" "$RX_RF_PORT" "$RX_SAMPLE_RATE" 2>>"$case_dir/iio_capture_init.log"
      tx_preinit_mode="$(ad9361_tx_preinit_mode "$TX_SAMPLE_RATE")"
      init_ad9361_tx "$TX_URI" "$freq" "$bw" "$att" "$TX_RF_PORT" "$TX_SAMPLE_RATE" "$tx_preinit_mode" 2>>"$case_dir/iio_capture_init.log"
    fi
    set +e
    timeout "$((CAPTURE_SETTLE_SECONDS + CAPTURE_SECONDS + 8))s" python3 -c 'from rm_radio_ros.nodes.ganraoyuan_node import main; main()' --ros-args --params-file "$case_dir/tx_params.yaml" > "$case_dir/tx_capture.log" 2>&1 &
    tx_cap_pid=$!
    sleep "$CAPTURE_SETTLE_SECONDS"
    timeout "${CAPTURE_SECONDS}s" iio_readdev -u "$RX_URI" -b 32768 cf-ad9361-lpc voltage0 voltage1 > "$case_dir/rx_iq_s16.bin" 2> "$case_dir/iio_readdev.err"
    read_rc=$?
    echo "iio_readdev_rc=$read_rc" >> "$case_dir/result.txt"
    stop_pids "$tx_cap_pid"
    set -e

    if [[ -s "$case_dir/rx_iq_s16.bin" ]]; then
      convert_s16_to_fc32 "$case_dir/rx_iq_s16.bin" "$case_dir/rx_capture.fc32" > "$case_dir/capture_stats.txt"
      set +e
      python3 -m rm_radio_ros.core.offline_iq_scan --summary --summary-frame-limit 10 --decode-threshold 3 --max-decoded-payloads 2048 --max-decoded-frames 512 --low-pass-hz "$lowpass" "$case_dir/rx_capture.fc32" > "$case_dir/offline_lp.log" 2>&1
      echo "offline_lp_rc=$?" >> "$case_dir/result.txt"
      python3 -m rm_radio_ros.core.offline_iq_scan --summary --summary-frame-limit 10 --decode-threshold 3 --max-decoded-payloads 2048 --max-decoded-frames 512 "$case_dir/rx_capture.fc32" > "$case_dir/offline_full.log" 2>&1
      echo "offline_full_rc=$?" >> "$case_dir/result.txt"
      set -e
    fi

    summarize_case "$case_dir" >> "$LOG_DIR/summary.txt"
  done
done

echo "end=$(date --iso-8601=seconds)" >> "$LOG_DIR/summary.txt"
cat "$LOG_DIR/summary.txt"
