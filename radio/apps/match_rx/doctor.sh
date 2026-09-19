#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"

check_cmd() {
  if command -v "$1" >/dev/null 2>&1; then
    echo "ok command $1"
  else
    echo "bad command $1 missing"
  fi
}

check_iio() {
  local label="$1"
  local uri="$2"
  if [[ "${RM_RADIO_DRY_RUN,,}" == "true" ]]; then
    echo "warn $label dry_run skip iio check uri=$uri"
    return 0
  fi
  if ! command -v iio_info >/dev/null 2>&1; then
    echo "warn $label iio_info missing uri=$uri"
    return 0
  fi
  if timeout 5s iio_info -u "$uri" >/dev/null 2>&1; then
    echo "ok $label uri=$uri"
  else
    echo "bad $label unreachable uri=$uri"
  fi
}

echo "SharkRadio match RX doctor"
echo "config=$MATCH_RX_CONFIG"
config_rel="${MATCH_RX_CONFIG#$RM_RADIO_WS/}"
if git -C "$RM_RADIO_WS" ls-files --error-unmatch "$config_rel" >/dev/null 2>&1; then
  echo "ok config is tracked by Git"
  if git -C "$RM_RADIO_WS" diff --quiet -- "$config_rel"; then
    echo "ok config matches current commit"
  else
    echo "warn config has uncommitted deployment changes"
  fi
else
  echo "warn config is not tracked by Git; it will not deploy to another computer"
fi
echo "workspace=$RM_RADIO_WS"
echo "side=$RM_RADIO_SIDE level=$INTERFERENCE_LEVEL rx1=$RX1_URI rx2=$RX2_URI referee=$REFEREE_PORT"
check_cmd ros2
check_cmd python3
if [[ "${RM_RADIO_RECORDING_ENABLED,,}" == "true" ]]; then
  if [[ "${RM_RADIO_RECORDING_EVENT_COMPRESSION,,}" == "zstd" ]]; then
    check_cmd zstd
  fi
  echo "recording=events_root=$RM_RADIO_RECORDING_ROOT iq_root=$RM_RADIO_RECORDING_IQ_ROOT output_rate=$RM_RADIO_RECORDING_OUTPUT_SAMPLE_RATE iq_min_free_gb=$RM_RADIO_RECORDING_IQ_MIN_FREE_GB"
  python3 - "$RM_RADIO_SAMPLE_RATE" "$RM_RADIO_RECORDING_OUTPUT_SAMPLE_RATE" <<'PY'
import sys

input_rate = float(sys.argv[1])
output_rate = float(sys.argv[2])
ratio = input_rate / output_rate if output_rate > 0 else 0
if input_rate > 0 and output_rate > 0 and abs(ratio - round(ratio)) <= 1e-6:
    print(f"ok recording integer decimation {input_rate:g}/{output_rate:g}={round(ratio)}")
else:
    print(f"bad recording output rate must divide input rate: {input_rate:g}/{output_rate:g}")
PY
fi
python3 "$RM_RADIO_WS/apps/common/check_gnuradio_runtime.py" || true
check_iio rx1 "$RX1_URI"
check_iio rx2 "$RX2_URI"
if [[ -e "$REFEREE_PORT" ]]; then
  echo "ok referee_port exists $REFEREE_PORT"
else
  echo "warn referee_port missing $REFEREE_PORT"
fi
python3 - <<'PY'
mods = ["rclpy", "serial", "numpy", "scipy"]
for mod in mods:
    try:
        __import__(mod)
        print(f"ok python {mod}")
    except Exception as exc:
        print(f"bad python {mod}: {exc}")
PY
