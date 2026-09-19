#!/usr/bin/env bash
set -euo pipefail

_iio_attr_set() {
  local uri="$1"
  local direction="$2"
  local channel="$3"
  local attr="$4"
  local value="$5"
  local flag=()
  case "$direction" in
    input) flag=(-i) ;;
    output) flag=(-o) ;;
    *) flag=() ;;
  esac
  iio_attr -u "$uri" "${flag[@]}" -c ad9361-phy "$channel" "$attr" "$value" >/dev/null
}

_iio_attr_try_set() {
  local uri="$1"
  local direction="$2"
  local channel="$3"
  local attr="$4"
  local value="$5"
  _iio_attr_set "$uri" "$direction" "$channel" "$attr" "$value" 2>/dev/null || true
}

_iio_attr_get() {
  local uri="$1"
  local direction="$2"
  local channel="$3"
  local attr="$4"
  local flag=()
  case "$direction" in
    input) flag=(-i) ;;
    output) flag=(-o) ;;
    *) flag=() ;;
  esac
  iio_attr -u "$uri" "${flag[@]}" -c ad9361-phy "$channel" "$attr" 2>/dev/null || true
}

_iio_device_attr_set() {
  local uri="$1"
  local attr="$2"
  local value="$3"
  iio_attr -u "$uri" -d ad9361-phy "$attr" "$value" >/dev/null 2>&1 || true
}

_iio_altvoltage_set() {
  local uri="$1"
  local channel="$2"
  local attr="$3"
  local value="$4"
  iio_attr -u "$uri" -c ad9361-phy "$channel" "$attr" "$value" >/dev/null
}

_iio_altvoltage_get() {
  local uri="$1"
  local channel="$2"
  local attr="$3"
  iio_attr -u "$uri" -c ad9361-phy "$channel" "$attr" 2>/dev/null || true
}

# TX safety operations must not wait forever when a network IIO context stops
# responding.  Keep RX behavior unchanged, and wrap every iio_info/iio_attr
# command used by TX init/mute in a short, externally enforced deadline.
_iio_tx_run_with_timeout() {
  local timeout_seconds="${RM_RADIO_IIO_COMMAND_TIMEOUT_SEC:-3}"
  if [[ ! "$timeout_seconds" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
     ! awk -v value="$timeout_seconds" 'BEGIN { exit(value > 0 ? 0 : 1) }'; then
    echo "invalid RM_RADIO_IIO_COMMAND_TIMEOUT_SEC=$timeout_seconds" >&2
    return 2
  fi
  if ! command -v timeout >/dev/null 2>&1; then
    echo "GNU timeout is required for bounded TX IIO safety operations" >&2
    return 127
  fi
  timeout --foreground --signal=TERM --kill-after=1s "${timeout_seconds}s" "$@"
}

_iio_tx_attr_set() {
  local uri="$1"
  local direction="$2"
  local channel="$3"
  local attr="$4"
  local value="$5"
  local flag=()
  case "$direction" in
    input) flag=(-i) ;;
    output) flag=(-o) ;;
    *) flag=() ;;
  esac
  _iio_tx_run_with_timeout iio_attr -u "$uri" "${flag[@]}" -c ad9361-phy "$channel" "$attr" "$value" >/dev/null
}

_iio_tx_attr_try_set() {
  _iio_tx_attr_set "$@" 2>/dev/null || true
}

_iio_tx_attr_get() {
  local uri="$1"
  local direction="$2"
  local channel="$3"
  local attr="$4"
  local flag=()
  case "$direction" in
    input) flag=(-i) ;;
    output) flag=(-o) ;;
    *) flag=() ;;
  esac
  _iio_tx_run_with_timeout iio_attr -u "$uri" "${flag[@]}" -c ad9361-phy "$channel" "$attr" 2>/dev/null || true
}

_iio_tx_device_attr_set() {
  local uri="$1"
  local attr="$2"
  local value="$3"
  _iio_tx_run_with_timeout iio_attr -u "$uri" -d ad9361-phy "$attr" "$value" >/dev/null 2>&1 || true
}

_iio_tx_device_attr_get() {
  local uri="$1"
  local attr="$2"
  _iio_tx_run_with_timeout iio_attr -u "$uri" -d ad9361-phy "$attr" 2>/dev/null || true
}

_iio_tx_altvoltage_set() {
  local uri="$1"
  local channel="$2"
  local attr="$3"
  local value="$4"
  _iio_tx_run_with_timeout iio_attr -u "$uri" -c ad9361-phy "$channel" "$attr" "$value" >/dev/null
}

_iio_tx_altvoltage_get() {
  local uri="$1"
  local channel="$2"
  local attr="$3"
  _iio_tx_run_with_timeout iio_attr -u "$uri" -c ad9361-phy "$channel" "$attr" 2>/dev/null || true
}

_iio_numeric_token() {
  awk 'match($0, /-?[0-9]+([.][0-9]+)?/) { print substr($0, RSTART, RLENGTH); exit }' <<< "${1:-}"
}

_iio_number_close() {
  local actual expected tolerance
  actual="$(_iio_numeric_token "${1:-}")"
  expected="$(_iio_numeric_token "${2:-}")"
  tolerance="${3:-0.01}"
  [[ -n "$actual" && -n "$expected" ]] || return 1
  awk -v actual="$actual" -v expected="$expected" -v tolerance="$tolerance" 'BEGIN {
    delta = actual - expected
    if (delta < 0) delta = -delta
    exit(delta <= tolerance ? 0 : 1)
  }'
}

init_ad9361_rx() {
  local uri="$1"
  local freq_hz="$2"
  local bandwidth_hz="$3"
  local gain_mode="${4:-fast_attack}"
  local gain_db="${5:-20}"
  local rf_port="${6:-B_BALANCED}"
  local sample_rate="${7:-2000000}"
  local configure_sample_rate=true

  # Rates below 2.083333 MSPS require an AD9361 FIR configuration. A caller
  # that starts a GNU Radio fmcomms2 block with Auto filtering can defer the
  # rate here; that block will load the FIR and then set the requested rate.
  if [[ -z "$sample_rate" || "${sample_rate,,}" == "defer" ]]; then
    sample_rate="deferred"
    configure_sample_rate=false
  fi

  if [[ -z "$uri" ]]; then
    echo "init_ad9361_rx: URI is empty" >&2
    return 2
  fi
  iio_info -u "$uri" >/dev/null

  # voltage0 is the physical RX1 path used by the E310.  Configuration of a
  # second channel is best-effort because some firmwares do not expose it, but
  # RX1 itself must be writable and must pass the readback below.
  local write_rc=0
  _iio_device_attr_set "$uri" ensm_mode fdd
  _iio_device_attr_set "$uri" trx_rate_governor nominal
  _iio_altvoltage_set "$uri" altvoltage0 frequency "$freq_hz" || write_rc=1
  if [[ "$configure_sample_rate" == "true" ]]; then
    _iio_attr_set "$uri" input voltage0 sampling_frequency "$sample_rate" || write_rc=1
  fi
  _iio_attr_set "$uri" input voltage0 rf_bandwidth "$bandwidth_hz" || write_rc=1
  _iio_attr_set "$uri" input voltage0 gain_control_mode "$gain_mode" || write_rc=1
  if [[ "$gain_mode" == "manual" ]]; then
    _iio_attr_set "$uri" input voltage0 hardwaregain "$gain_db" || write_rc=1
  fi
  _iio_attr_set "$uri" input voltage0 rf_port_select "$rf_port" || write_rc=1

  # RX2 is optional on some boards.  Mirror RX1 when present without making
  # its absence a startup failure.
  for ch in voltage1; do
    if [[ "$configure_sample_rate" == "true" ]]; then
      _iio_attr_try_set "$uri" input "$ch" sampling_frequency "$sample_rate"
    fi
    _iio_attr_try_set "$uri" input "$ch" rf_bandwidth "$bandwidth_hz"
    _iio_attr_try_set "$uri" input "$ch" gain_control_mode "$gain_mode"
    if [[ "$gain_mode" == "manual" ]]; then
      _iio_attr_try_set "$uri" input "$ch" hardwaregain "$gain_db"
    fi
    _iio_attr_try_set "$uri" input "$ch" rf_port_select "$rf_port"
  done

  local actual_port actual_gain_mode actual_freq actual_bw actual_rate actual_rssi
  actual_port="$(_iio_attr_get "$uri" input voltage0 rf_port_select)"
  actual_gain_mode="$(_iio_attr_get "$uri" input voltage0 gain_control_mode)"
  actual_freq="$(_iio_altvoltage_get "$uri" altvoltage0 frequency)"
  actual_bw="$(_iio_attr_get "$uri" input voltage0 rf_bandwidth)"
  actual_rate="$(_iio_attr_get "$uri" input voltage0 sampling_frequency)"
  actual_rssi="$(_iio_attr_get "$uri" input voltage0 rssi)"
  echo "IIO RX init uri=$uri freq=$actual_freq bw=$actual_bw sample_rate=$actual_rate gain_mode=$actual_gain_mode rf_port=$actual_port rssi=${actual_rssi:-n/a}" >&2

  local verify_rc="$write_rc"
  # AD9361 fractional-N tuning can quantize 433 MHz requests by a few hertz.
  # Keep the check fail-closed while accepting that hardware granularity.
  _iio_number_close "$actual_freq" "$freq_hz" "${RM_RADIO_RX_FREQUENCY_READBACK_TOLERANCE_HZ:-5}" || verify_rc=1
  _iio_number_close "$actual_bw" "$bandwidth_hz" 0.5 || verify_rc=1
  if [[ "$configure_sample_rate" == "true" ]]; then
    _iio_number_close "$actual_rate" "$sample_rate" 0.5 || verify_rc=1
  fi
  [[ "${actual_gain_mode,,}" == "${gain_mode,,}" ]] || verify_rc=1
  [[ "$actual_port" == "$rf_port" ]] || verify_rc=1
  if [[ "$gain_mode" == "manual" ]]; then
    local actual_gain
    actual_gain="$(_iio_attr_get "$uri" input voltage0 hardwaregain)"
    _iio_number_close "$actual_gain" "$gain_db" 0.05 || verify_rc=1
  fi
  if [[ "$verify_rc" -ne 0 ]]; then
    echo "init_ad9361_rx verification failed uri=$uri requested_freq=$freq_hz requested_bw=$bandwidth_hz requested_sample_rate=$sample_rate requested_gain_mode=$gain_mode requested_rf_port=$rf_port actual_freq=${actual_freq:-n/a} actual_bw=${actual_bw:-n/a} actual_sample_rate=${actual_rate:-n/a} actual_gain_mode=${actual_gain_mode:-n/a} actual_rf_port=${actual_port:-n/a} write_rc=$write_rc" >&2
    return 1
  fi
}

ad9361_tx_preinit_mode() {
  local sample_rate="${1:-}"
  local no_fir_min_rate="${RM_RADIO_AD9361_NO_FIR_MIN_SAMPLE_RATE_HZ:-2083333}"

  # AD9361 rates below roughly 2.083333 MSPS require a decimating/interpolating
  # FIR. GNU Radio's fmcomms2 sink can install that FIR in Auto mode, but a raw
  # IIO pre-initializer cannot safely do so without owning the complete filter
  # design. Keep ordinary rates strict and defer only this low-rate case.
  if [[ "$sample_rate" =~ ^[0-9]+([.][0-9]+)?$ ]] && \
      awk -v rate="$sample_rate" -v minimum="$no_fir_min_rate" \
        'BEGIN { exit(rate < minimum ? 0 : 1) }'; then
    printf '%s\n' defer
  else
    printf '%s\n' strict
  fi
}

init_ad9361_tx() {
  local uri="$1"
  local freq_hz="$2"
  local bandwidth_hz="$3"
  local attenuation_db="${4:-60}"
  local rf_port="${5:-A}"
  local sample_rate="${6:-2000000}"
  local sample_rate_mode="${7:-strict}"
  local configure_sample_rate=true

  sample_rate_mode="${sample_rate_mode,,}"
  case "$sample_rate_mode" in
    strict)
      ;;
    defer|deferred)
      sample_rate_mode="defer"
      configure_sample_rate=false
      ;;
    *)
      echo "init_ad9361_tx: invalid sample-rate mode '$sample_rate_mode' (expected strict or defer)" >&2
      return 2
      ;;
  esac

  if [[ -z "$uri" ]]; then
    echo "init_ad9361_tx: URI is empty" >&2
    return 2
  fi
  _iio_tx_run_with_timeout iio_info -u "$uri" >/dev/null

  local write_rc=0
  _iio_tx_device_attr_set "$uri" ensm_mode fdd
  _iio_tx_device_attr_set "$uri" trx_rate_governor nominal
  _iio_tx_altvoltage_set "$uri" altvoltage1 frequency "$freq_hz" || write_rc=1
  _iio_tx_altvoltage_set "$uri" altvoltage1 powerdown 0 || write_rc=1
  # voltage0 is the physical TX1 path used by the Z103.  Some firmwares do not
  # expose voltage1, so require channel 0 and keep channel 1 best-effort.
  if [[ "$configure_sample_rate" == "true" ]]; then
    _iio_tx_attr_set "$uri" output voltage0 sampling_frequency "$sample_rate" || write_rc=1
  fi
  _iio_tx_attr_set "$uri" output voltage0 rf_bandwidth "$bandwidth_hz" || write_rc=1
  _iio_tx_attr_set "$uri" output voltage0 hardwaregain "-$attenuation_db" || write_rc=1
  _iio_tx_attr_set "$uri" output voltage0 rf_port_select "$rf_port" || write_rc=1
  local attr_value
  if [[ "$configure_sample_rate" == "true" ]]; then
    _iio_tx_attr_try_set "$uri" output voltage1 sampling_frequency "$sample_rate"
  fi
  for attr_value in \
    "rf_bandwidth:$bandwidth_hz" \
    "hardwaregain:-$attenuation_db" \
    "rf_port_select:$rf_port"; do
    _iio_tx_attr_try_set "$uri" output voltage1 "${attr_value%%:*}" "${attr_value#*:}"
  done

  local actual_port actual_gain actual_freq actual_bw actual_rate actual_powerdown
  actual_port="$(_iio_tx_attr_get "$uri" output voltage0 rf_port_select)"
  actual_gain="$(_iio_tx_attr_get "$uri" output voltage0 hardwaregain)"
  actual_freq="$(_iio_tx_altvoltage_get "$uri" altvoltage1 frequency)"
  actual_bw="$(_iio_tx_attr_get "$uri" output voltage0 rf_bandwidth)"
  actual_rate="$(_iio_tx_attr_get "$uri" output voltage0 sampling_frequency)"
  actual_powerdown="$(_iio_tx_altvoltage_get "$uri" altvoltage1 powerdown)"
  echo "IIO TX init uri=$uri freq=$actual_freq bw=$actual_bw sample_rate=$actual_rate requested_sample_rate=$sample_rate sample_rate_mode=$sample_rate_mode attenuation=${actual_gain:-n/a} rf_port=$actual_port tx_lo_powerdown=${actual_powerdown:-n/a}" >&2

  local verify_rc="$write_rc"
  # AD9361 fractional-N tuning can quantize UHF LO requests by a few hertz.
  # Keep the check fail-closed while accepting the same bounded error as RX.
  _iio_number_close "$actual_freq" "$freq_hz" "${RM_RADIO_TX_FREQUENCY_READBACK_TOLERANCE_HZ:-5}" || verify_rc=1
  _iio_number_close "$actual_bw" "$bandwidth_hz" 0.5 || verify_rc=1
  # Z103/AD9361 can quantize an integer sampling-rate request by one hertz.
  # Keep verification strict, but accept the observed bounded hardware error.
  if [[ "$configure_sample_rate" == "true" ]]; then
    _iio_number_close "$actual_rate" "$sample_rate" 2 || verify_rc=1
  fi
  _iio_number_close "$actual_gain" "-$attenuation_db" 0.01 || verify_rc=1
  [[ "$actual_port" == "$rf_port" ]] || verify_rc=1
  [[ "${actual_powerdown%% *}" == "0" ]] || verify_rc=1
  if [[ "$verify_rc" -ne 0 ]]; then
    echo "init_ad9361_tx verification failed uri=$uri requested_freq=$freq_hz requested_bw=$bandwidth_hz requested_sample_rate=$sample_rate sample_rate_mode=$sample_rate_mode requested_attenuation=-$attenuation_db requested_rf_port=$rf_port actual_freq=${actual_freq:-n/a} actual_bw=${actual_bw:-n/a} actual_sample_rate=${actual_rate:-n/a} actual_attenuation=${actual_gain:-n/a} actual_rf_port=${actual_port:-n/a} actual_tx_lo_powerdown=${actual_powerdown:-n/a} write_rc=$write_rc" >&2
    # A partial initialization may already have enabled the LO.  Fail safe even
    # when the caller has not installed its own EXIT trap yet.
    mute_ad9361_tx "$uri" 89.75 >/dev/null 2>&1 || true
    return 1
  fi
}

mute_ad9361_tx() {
  local uri="$1"
  local attenuation_db="${2:-89.75}"

  if [[ -z "$uri" ]]; then
    echo "mute_ad9361_tx: URI is empty" >&2
    return 2
  fi
  _iio_tx_run_with_timeout iio_info -u "$uri" >/dev/null

  local write_rc=0
  _iio_tx_attr_set "$uri" output voltage0 hardwaregain "-$attenuation_db" || write_rc=1
  _iio_tx_attr_try_set "$uri" output voltage1 hardwaregain "-$attenuation_db"
  _iio_tx_altvoltage_set "$uri" altvoltage1 powerdown 1 || write_rc=1
  _iio_tx_device_attr_set "$uri" ensm_mode alert

  local actual_gain actual_powerdown actual_mode
  actual_gain="$(_iio_tx_attr_get "$uri" output voltage0 hardwaregain)"
  actual_powerdown="$(_iio_tx_altvoltage_get "$uri" altvoltage1 powerdown)"
  actual_mode="$(_iio_tx_device_attr_get "$uri" ensm_mode)"
  echo "IIO TX mute uri=$uri attenuation=${actual_gain:-n/a} tx_lo_powerdown=${actual_powerdown:-n/a} ensm=${actual_mode:-n/a}" >&2
  _iio_number_close "$actual_gain" "-$attenuation_db" 0.01 || write_rc=1
  [[ "${actual_powerdown%% *}" == "1" ]] || write_rc=1
  if [[ "$write_rc" -ne 0 ]]; then
    echo "IIO TX mute verification failed uri=$uri requested_attenuation=-$attenuation_db" >&2
    return 1
  fi
}
