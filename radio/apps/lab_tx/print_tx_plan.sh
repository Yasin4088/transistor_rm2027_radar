#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/env.sh"

freq_bw="$(tx_frequency_for_side_level "$RM_RADIO_SIDE" "$INTERFERENCE_LEVEL")"
freq="${freq_bw%% *}"
bw="${freq_bw##* }"

cat <<EOF
LabTX lab plan
workspace: $RM_RADIO_WS
side: ${RM_RADIO_SIDE,,}
broadcast_tx_uri: $BROADCAST_TX_URI
broadcast_center_hz: $(if [[ "${RM_RADIO_SIDE,,}" == "blue" ]]; then echo 433920000; else echo 433200000; fi)
broadcast_bandwidth_hz: 540000
broadcast_attenuation_db: $BROADCAST_TX_ATTENUATION_DB
interference_level: $INTERFERENCE_LEVEL
interference_tx_uri: ${INTERFERENCE_TX_URI:-<等待连接识别>}
interference_center_hz: $freq
interference_bandwidth_hz: $bw
interference_attenuation_db: $INTERFERENCE_TX_ATTEND_GR
interference_password: $INTERFERENCE_PASSWORD
generated_tx_buffer_size: $RM_RADIO_TX_BUFFER_SIZE
lab_tx_confirmed: ${RM_RADIO_LAB_TX_ANTENNA_CONFIRM:-false}
iq_replay_paths: ${IQ_REPLAY_PATHS:-}
iq_replay_tx_type: ${IQ_REPLAY_TX_TYPE,,}
iq_replay_sample_rate: $IQ_REPLAY_SAMPLE_RATE
iq_replay_repeat: $IQ_REPLAY_REPEAT
iq_replay_amplitude_scale: $IQ_REPLAY_AMPLITUDE_SCALE
iq_replay_attenuation_db: $IQ_REPLAY_TX_ATTENUATION_DB
iq_replay_tx_buffer_size: $IQ_REPLAY_TX_BUFFER_SIZE
iq_replay_cache_to_tmp: $IQ_REPLAY_CACHE_TO_TMP
iq_replay_cache_dir: $IQ_REPLAY_CACHE_DIR
generated_setters_json: $(build_interference_setters_json)
EOF
