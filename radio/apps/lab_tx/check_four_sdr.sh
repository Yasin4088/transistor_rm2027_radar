#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/env.sh"

check_uri() {
  local label="$1"
  local uri="$2"
  if [[ -z "$uri" ]]; then
    echo "[$label] URI is empty" >&2
    return 2
  fi
  echo "==== $label $uri"
  if ! timeout 8s iio_info -u "$uri" >"$LOG_DIR/${label}_iio_info.log" 2>&1; then
    sed -n '1,40p' "$LOG_DIR/${label}_iio_info.log" >&2 || true
    echo "[$label] iio_info failed: $uri" >&2
    return 1
  fi
  grep -E 'IIO context created|hw_model:|hw_serial:|uri:' "$LOG_DIR/${label}_iio_info.log" | sed -n '1,24p'
}

print_usb_iio_contexts() {
  local found=0
  echo "[four-sdr] WSL USB 0456:b673 IIO contexts:"
  for dev in /sys/bus/usb/devices/*; do
    [[ -f "$dev/idVendor" && -f "$dev/idProduct" ]] || continue
    [[ "$(cat "$dev/idVendor" 2>/dev/null)" == "0456" ]] || continue
    [[ "$(cat "$dev/idProduct" 2>/dev/null)" == "b673" ]] || continue
    local bus devnum uri serial product log_file
    bus="$(cat "$dev/busnum" 2>/dev/null || true)"
    devnum="$(cat "$dev/devnum" 2>/dev/null || true)"
    [[ -n "$bus" && -n "$devnum" ]] || continue
    uri="usb:${bus}.${devnum}.5"
    serial="$(cat "$dev/serial" 2>/dev/null || true)"
    product="$(cat "$dev/product" 2>/dev/null || true)"
    log_file="$LOG_DIR/usb_${bus}_${devnum}_iio_info.log"
    found=1
    echo "  - $uri serial=${serial:-unknown} product=${product:-unknown}"
    if timeout 4s iio_info -u "$uri" >"$log_file" 2>&1; then
      grep -E 'IIO context created|hw_model:|hw_serial:|uri:' "$log_file" | sed 's/^/      /' | sed -n '1,12p'
    else
      sed -n '1,8p' "$log_file" | sed 's/^/      /'
    fi
  done
  if [[ "$found" -eq 0 ]]; then
    echo "  - none"
  fi
}

rm_radio_ensure_dir "$LOG_DIR"

echo "[four-sdr] side=$RM_RADIO_SIDE level=$INTERFERENCE_LEVEL"
echo "[four-sdr] logs=$LOG_DIR"
echo "[four-sdr] information TX=$BROADCAST_TX_URI"
echo "[four-sdr] interference TX=${INTERFERENCE_TX_URI:-<等待连接识别>}"
echo "[four-sdr] E310 broadcast RX=$RX1_URI"
echo "[four-sdr] E310 interference RX=$RX2_URI"

status=0
check_uri broadcast_tx "$BROADCAST_TX_URI" || status=1
if [[ -n "$INTERFERENCE_TX_URI" ]]; then
  check_uri interference_tx "$INTERFERENCE_TX_URI" || status=1
else
  echo "[interference_tx] 当前未连接，跳过检测"
fi
check_uri broadcast_rx "$RX1_URI" || status=1
check_uri interference_rx "$RX2_URI" || status=1

if [[ -n "$INTERFERENCE_TX_URI" && "$BROADCAST_TX_URI" == "$INTERFERENCE_TX_URI" ]]; then
  echo "broadcast and interference TX URI must be different" >&2
  status=2
fi
if [[ "$RX1_URI" == "$RX2_URI" ]]; then
  echo "RX1 and RX2 URI must be different" >&2
  status=2
fi

if [[ "$status" -ne 0 ]]; then
  print_usb_iio_contexts >&2
  exit "$status"
fi

echo "[four-sdr] all configured SDR URIs are reachable"
