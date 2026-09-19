#!/usr/bin/env bash
set -euo pipefail

RADIO_TX_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$RADIO_TX_DIR/env.sh"
# shellcheck disable=SC1091
source "$RM_RADIO_WS/apps/common/iio_init.sh"

RX_SAMPLE_RATE="${RX_SAMPLE_RATE:-2000000}"
RX_RF_PORT="${RX_RF_PORT:-B_BALANCED}"

side="${RM_RADIO_SIDE,,}"
case "$side" in
  red) broadcast_freq=433200000 ;;
  blue) broadcast_freq=433920000 ;;
  *) echo "RM_RADIO_SIDE must be red or blue; got $RM_RADIO_SIDE" >&2; exit 2 ;;
esac
read -r interference_freq interference_bw < <(tx_frequency_for_side_level "$side" "$INTERFERENCE_LEVEL")

read_rx_rate() {
  local uri="$1"
  iio_attr -u "$uri" -i -c ad9361-phy voltage0 sampling_frequency 2>/dev/null || true
}

print_rx_status() {
  local label="$1"
  local uri="$2"
  echo "[$label] uri=$uri"
  echo "  sampling_frequency_available=$(iio_attr -u "$uri" -i -c ad9361-phy voltage0 sampling_frequency_available 2>/dev/null || true)"
  echo "  sampling_frequency=$(read_rx_rate "$uri")"
  echo "  rx_path_rates=$(iio_attr -u "$uri" -d ad9361-phy rx_path_rates 2>/dev/null || true)"
  echo "  filter_fir_config=$(iio_attr -u "$uri" -d ad9361-phy filter_fir_config 2>/dev/null | head -n 1 || true)"
  echo "  rf_bandwidth=$(iio_attr -u "$uri" -i -c ad9361-phy voltage0 rf_bandwidth 2>/dev/null || true)"
  echo "  rf_port_select=$(iio_attr -u "$uri" -i -c ad9361-phy voltage0 rf_port_select 2>/dev/null || true)"
}

warmup_auto_fir() {
  local uri="$1"
  local freq="$2"
  local bw="$3"
  echo "[$uri] 2MS/s not active after normal init; trying GNU Radio Auto FIR warmup" >&2
  timeout 8s python3 - "$uri" "$freq" "$bw" "$RX_SAMPLE_RATE" <<'PY' || true
import sys
from gnuradio import iio

uri, freq, bw, sample_rate = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
source = iio.fmcomms2_source_fc32(uri, [True, True], 32768)
source.set_frequency(freq)
source.set_samplerate(sample_rate)
source.set_filter_params("Auto", "", 0, 0)
print("GNU Radio Auto FIR warmup requested", flush=True)
PY
}

reset_one_rx() {
  local label="$1"
  local uri="$2"
  local freq="$3"
  local bw="$4"
  local gain_mode="$5"
  local gain="$6"

  echo "[$label] reset RX uri=$uri freq=$freq bw=$bw sample_rate=$RX_SAMPLE_RATE rf_port=$RX_RF_PORT"
  init_ad9361_rx "$uri" "$freq" "$bw" "$gain_mode" "$gain" "$RX_RF_PORT" "$RX_SAMPLE_RATE"
  local actual
  actual="$(read_rx_rate "$uri")"
  if [[ "$actual" != "$RX_SAMPLE_RATE" ]]; then
    warmup_auto_fir "$uri" "$freq" "$bw"
    init_ad9361_rx "$uri" "$freq" "$bw" "$gain_mode" "$gain" "$RX_RF_PORT" "$RX_SAMPLE_RATE"
  fi
  print_rx_status "$label" "$uri"
}

reset_one_rx "broadcast_rx" "$RX1_URI" "$broadcast_freq" 540000 "$BROADCAST_RX_GAIN_MODE" "$BROADCAST_RX_GAIN"
reset_one_rx "interference_rx" "$RX2_URI" "$interference_freq" "$interference_bw" "$INTERFERENCE_RX_GAIN_MODE" "$INTERFERENCE_RX_GAIN"
