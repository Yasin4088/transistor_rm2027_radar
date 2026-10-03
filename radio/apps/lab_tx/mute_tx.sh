#!/usr/bin/env bash
# Hard-mute the lab TX SDRs after a bench run: set max TX attenuation and power down the
# TX LO so the SDR radiates nothing. This is a lab-safety helper (RX-only match chain never
# transmits); use it after stopping start_dual_tx.sh / start_interference_tx.sh, especially
# when the TX nodes were killed abruptly (kill -9) instead of stopped cleanly.
#
# Usage: bash apps/lab_tx/mute_tx.sh [tx_uri ...]
#   Defaults to BROADCAST_TX_URI and INTERFERENCE_TX_URI; an empty URI is skipped.
set -uo pipefail

uris=("$@")
if [[ ${#uris[@]} -eq 0 ]]; then
  uris=("${BROADCAST_TX_URI:-ip:192.168.2.1}" "${INTERFERENCE_TX_URI:-}")
fi

if ! command -v iio_attr >/dev/null 2>&1; then
  echo "[mute-tx] iio_attr not found; cannot hardware-mute TX" >&2
  exit 2
fi

rc=0
for uri in "${uris[@]}"; do
  [[ -z "$uri" ]] && continue
  if ! iio_info -u "$uri" >/dev/null 2>&1; then
    echo "[mute-tx] $uri unreachable; skip" >&2
    continue
  fi
  # Max TX attenuation (hardwaregain minimum, -89.75 dB) on the AD9361 TX output channel.
  iio_attr -u "$uri" -o -c ad9361-phy voltage0 hardwaregain -89.75 >/dev/null 2>&1 || rc=1
  # Power down the TX local oscillator so the SDR emits nothing.
  iio_attr -u "$uri" -c ad9361-phy altvoltage1 powerdown 1 >/dev/null 2>&1 || rc=1
  gain=$(iio_attr -u "$uri" -o -c ad9361-phy voltage0 hardwaregain 2>/dev/null | tr -d '\r' | tail -1)
  pd=$(iio_attr -u "$uri" -c ad9361-phy altvoltage1 powerdown 2>/dev/null | tr -d '\r' | tail -1)
  echo "[mute-tx] $uri -> hardwaregain=${gain:-?} TX_LO_powerdown=${pd:-?}"
done
exit $rc
