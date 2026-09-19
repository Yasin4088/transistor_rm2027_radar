#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/env.sh"

case "${IQ_REPLAY_TX_TYPE,,}" in
  broadcast)
    IQ_REPLAY_TX_URI="$BROADCAST_TX_URI"
    IQ_REPLAY_TX_RF_PORT="$BROADCAST_TX_RF_PORT"
    ;;
  interference)
    IQ_REPLAY_TX_URI="$INTERFERENCE_TX_URI"
    IQ_REPLAY_TX_RF_PORT="$INTERFERENCE_TX_RF_PORT"
    ;;
esac
if [[ -z "$IQ_REPLAY_TX_URI" ]]; then
  echo "${IQ_REPLAY_TX_TYPE} TX 当前未连接或 URI 为空，不能启动 IQ 回放" >&2
  exit 2
fi

require_lab_tx_confirmation

runtime_audit_args=()
if [[ "${RM_RADIO_REQUIRE_LATEST_RUNTIME:-false}" == "true" ]]; then
  runtime_audit_args+=(--require-latest)
fi
python3 "$RM_RADIO_WS/apps/common/check_gnuradio_runtime.py" "${runtime_audit_args[@]}"

if [[ -z "${IQ_REPLAY_PATHS// }" ]]; then
  cat >&2 <<'MSG'
IQ_REPLAY_PATHS is empty.

Set iq_replay.paths in apps/lab_tx/config.yaml or use a one-shot environment override, for example:
  export IQ_REPLAY_PATHS="/path/to/capture.fc32"
MSG
  exit 2
fi

echo "Starting lab IQ replay TX. This is not a match launcher." >&2
echo "TX URI=$IQ_REPLAY_TX_URI paths=$IQ_REPLAY_PATHS attenuation=$IQ_REPLAY_TX_ATTENUATION_DB dB" >&2
echo "IIO buffer=$IQ_REPLAY_TX_BUFFER_SIZE cache_to_tmp=$IQ_REPLAY_CACHE_TO_TMP cache_dir=$IQ_REPLAY_CACHE_DIR" >&2

if [[ "$RM_RADIO_IIO_PREINIT" == "true" ]]; then
  # shellcheck disable=SC1091
  source "$RM_RADIO_WS/apps/common/iio_init.sh"
  tx_freq=433200000
  tx_bw=540000
  if [[ "${IQ_REPLAY_TX_TYPE,,}" == "interference" ]]; then
    read -r tx_freq tx_bw < <(tx_frequency_for_side_level "$RM_RADIO_SIDE" "$INTERFERENCE_LEVEL")
  elif [[ "${RM_RADIO_SIDE,,}" == "blue" ]]; then
    tx_freq=433920000
  fi
  init_ad9361_tx "$IQ_REPLAY_TX_URI" "$tx_freq" "$tx_bw" "$IQ_REPLAY_TX_ATTENUATION_DB" "$IQ_REPLAY_TX_RF_PORT" "$IQ_REPLAY_SAMPLE_RATE"
fi

cd "$RM_RADIO_WS"
exec ros2 run rm_radio_ros rm_iq_replay_tx_node \
  --ros-args \
  -p "tx_uri:=\"$IQ_REPLAY_TX_URI\"" \
  -p iq_paths:="$IQ_REPLAY_PATHS" \
  -p radio_side:="${RM_RADIO_SIDE,,}" \
  -p tx_type:="${IQ_REPLAY_TX_TYPE,,}" \
  -p interference_level:="$INTERFERENCE_LEVEL" \
  -p attenuation_db:="$IQ_REPLAY_TX_ATTENUATION_DB" \
  -p iq_amplitude_scale:="$IQ_REPLAY_AMPLITUDE_SCALE" \
  -p sample_rate:="$IQ_REPLAY_SAMPLE_RATE" \
  -p repeat:="$IQ_REPLAY_REPEAT" \
  -p tx_buffer_size:="$IQ_REPLAY_TX_BUFFER_SIZE" \
  -p cache_iq_to_tmp:="$IQ_REPLAY_CACHE_TO_TMP" \
  -p iq_cache_dir:="$IQ_REPLAY_CACHE_DIR"
