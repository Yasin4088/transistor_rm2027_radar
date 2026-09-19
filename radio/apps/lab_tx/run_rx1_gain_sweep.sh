#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LEVEL="${1:-2}"
STAMP="$(date +%Y%m%d_%H%M%S)"
ROOT="${ROOT:-/tmp/rm_radio_runtime_logs/rx1_gain_sweep_lvl${LEVEL}_${STAMP}}"
mkdir -p "$ROOT"

for mode in fast_attack slow_attack manual; do
  for gain in 20 35 50; do
    case_dir="$ROOT/${mode}_g${gain}"
    echo "CASE mode=$mode gain=$gain log=$case_dir"
    LOG_ROOT="$case_dir" \
      RX_GAIN_MODE="$mode" \
      RX_GAIN="$gain" \
      CAPTURE_SECONDS="${CAPTURE_SECONDS:-1.2}" \
      TX_WARMUP_SECONDS="${TX_WARMUP_SECONDS:-7}" \
      RUN_AUTOTUNE=false \
      RUN_OFFLINE_FULL=false \
      bash "$SCRIPT_DIR/run_dual_tx_raw_rx1_capture.sh" "$LEVEL" || true
  done
done

python3 - "$ROOT" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
print(f"ROOT={root}")
for case_dir in sorted(path for path in root.iterdir() if path.is_dir()):
    result_path = case_dir / "result_summary.json"
    if not result_path.exists():
        print(case_dir.name, "missing_result")
        continue
    data = json.loads(result_path.read_text(errors="ignore"))
    capture = data.get("capture_stats") or {}
    scan = data.get("broadcast_scan") or {}
    best = scan.get("best_summary") or scan
    print(
        case_dir.name,
        "mean_abs=", capture.get("mean_abs"),
        "max_abs=", capture.get("max_abs"),
        "best_ham=", best.get("best_access_hamming_distance"),
        "payloads=", best.get("air_payload_count"),
        "frames=", best.get("referee_frame_count"),
        "cmds=", best.get("commands"),
    )
PY
