#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/env.sh"

if [[ -z "$INTERFERENCE_TX_URI" ]]; then
  echo "干扰波 TX 当前未连接，需单独接入并识别后设置 INTERFERENCE_TX_URI" >&2
  exit 2
fi

require_lab_tx_confirmation

runtime_audit_args=()
if [[ "${RM_RADIO_REQUIRE_LATEST_RUNTIME:-false}" == "true" ]]; then
  runtime_audit_args+=(--require-latest)
fi
python3 "$RM_RADIO_WS/apps/common/check_gnuradio_runtime.py" "${runtime_audit_args[@]}"

export INTERFERENCE_SETTERS_JSON="${INTERFERENCE_SETTERS_JSON:-$(build_interference_setters_json)}"

echo "Starting lab interference TX. This is not a match launcher." >&2
echo "TX URI=$INTERFERENCE_TX_URI side=${RM_RADIO_SIDE,,} level=$INTERFERENCE_LEVEL attenuation=$INTERFERENCE_TX_ATTEND_GR dB" >&2
echo "IIO buffer=$RM_RADIO_TX_BUFFER_SIZE" >&2

if [[ "$RM_RADIO_IIO_PREINIT" == "true" ]]; then
  # shellcheck disable=SC1091
  source "$RM_RADIO_WS/apps/common/iio_init.sh"
  read -r tx_freq tx_bw < <(tx_frequency_for_side_level "$RM_RADIO_SIDE" "$INTERFERENCE_LEVEL")
  tx_preinit_mode="$(ad9361_tx_preinit_mode "$RM_RADIO_SAMPLE_RATE")"
  init_ad9361_tx "$INTERFERENCE_TX_URI" "$tx_freq" "$tx_bw" "$INTERFERENCE_TX_ATTEND_GR" "$INTERFERENCE_TX_RF_PORT" "$RM_RADIO_SAMPLE_RATE" "$tx_preinit_mode"
fi

cd "$RM_RADIO_WS"
export RM_RADIO_TX_PREFILL_PACKETS="${RM_RADIO_TX_PREFILL_PACKETS:-0}"
export RM_RADIO_TX_BUFFER_SIZE="${RM_RADIO_TX_BUFFER_SIZE:-1048576}"
exec ros2 launch rm_radio_ros ganraoyuan.launch.py \
  radio_side:="$RM_RADIO_SIDE" \
  interference_level:="$INTERFERENCE_LEVEL" \
  interference_tx_uri:="$INTERFERENCE_TX_URI" \
  sample_rate:="$RM_RADIO_SAMPLE_RATE" \
  setters_json:="$INTERFERENCE_SETTERS_JSON" \
  disable_gui_sinks:=true \
  qt_platform:=offscreen
