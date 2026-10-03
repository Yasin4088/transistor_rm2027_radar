#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# A/B comparisons must not inherit unrelated ambient shell overrides.
# shellcheck disable=SC1091
source "$SCRIPT_DIR/env.sh"

if [[ "${RM_RADIO_LAB_TX_ANTENNA_CONFIRM:-false}" != "true" ]]; then
  echo "Set RM_RADIO_LAB_TX_ANTENNA_CONFIRM=true before running RF A/B tests." >&2
  exit 2
fi

MODES="${DEMOD_AB_MODES:-mm capture_only legacy_stream}"
ATTENUATIONS="${DEMOD_AB_ATTENUATIONS:-30 35 40}"
LEVEL="${DEMOD_AB_LEVEL:-1}"
INTERFERENCE_ATTENUATION="${DEMOD_AB_INTERFERENCE_ATTENUATION:-10}"
TEST_SECONDS="${DEMOD_AB_TEST_SECONDS:-12}"
RX_WARMUP_SECONDS="${DEMOD_AB_RX_WARMUP_SECONDS:-4}"
POST_TX_SECONDS="${DEMOD_AB_POST_TX_SECONDS:-1}"
ROOT="${DEMOD_AB_LOG_ROOT:-$RM_RADIO_WS/log/demod_ab_$(date +%Y%m%d_%H%M%S)}"

mkdir -p "$ROOT"

run_case() {
  local mode="$1"
  local attenuation="$2"
  local demod_mode iq_enabled capture_enabled capture_only
  case "$mode" in
    mm)
      demod_mode=mm
      iq_enabled=false
      capture_enabled=false
      capture_only=false
      ;;
    legacy_stream)
      demod_mode=legacy
      iq_enabled=false
      capture_enabled=false
      capture_only=false
      ;;
    capture_only)
      demod_mode=legacy
      iq_enabled=true
      capture_enabled=true
      capture_only=true
      ;;
    *)
      echo "Unknown DEMOD_AB mode: $mode" >&2
      return 2
      ;;
  esac

  echo "[demod-ab] mode=$mode broadcast_attenuation=$attenuation interference_attenuation=$INTERFERENCE_ATTENUATION level=$LEVEL"
  TEST_LEVELS="$LEVEL" \
  TEST_SECONDS="$TEST_SECONDS" \
  RX_WARMUP_SECONDS="$RX_WARMUP_SECONDS" \
  POST_TX_SECONDS="$POST_TX_SECONDS" \
  BROADCAST_TX_ATTENUATION_DB="$attenuation" \
  INTERFERENCE_TX_ATTEND_GR="$INTERFERENCE_ATTENUATION" \
  BROADCAST_DEMOD_MODE="$demod_mode" \
  BROADCAST_IQ_DECODER_ENABLED="$iq_enabled" \
  BROADCAST_IIO_CAPTURE_DECODER_ENABLED="$capture_enabled" \
  BROADCAST_IIO_CAPTURE_DECODER_ONLY="$capture_only" \
  LOG_ROOT="$ROOT/a${attenuation}_i${INTERFERENCE_ATTENUATION}/${mode}" \
    bash "$SCRIPT_DIR/run_four_sdr_live_test.sh"
}

for attenuation in $ATTENUATIONS; do
  for mode in $MODES; do
    run_case "$mode" "$attenuation"
  done
done

python3 - "$ROOT" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])


def load_json_lines(path):
    output = []
    if not path.exists():
        return output
    for line in path.read_text(errors='ignore').splitlines():
        line = line.strip()
        if line.startswith('data:'):
            line = line.split(':', 1)[1].strip()
        if len(line) >= 2 and line[0] == line[-1] == "'":
            line = line[1:-1]
        if not line.startswith('{'):
            continue
        try:
            output.append(json.loads(line))
        except Exception:
            pass
    return output


print('case\tmode\ttopic_frames\tstatus_frames\tair_payloads\tiq_air_payloads\tcommands')
for case_dir in sorted(root.glob('a*_i*/*/level*')):
    statuses = load_json_lines(case_dir / 'rx1_status.log')
    frames = load_json_lines(case_dir / 'rx1_frames.log')
    tail = statuses[-1] if statuses else {}
    stats = ((tail.get('decoder') or {}).get('stats') or {})
    commands = sorted({str(frame.get('cmd_hex')) for frame in frames})
    print(
        case_dir.parent.parent.name,
        case_dir.parent.name,
        len(frames),
        stats.get('frames', '-'),
        stats.get('air_payloads', '-'),
        stats.get('iq_air_payloads', '-'),
        ','.join(commands),
        sep='\t',
    )

print(f'all_logs={root}')
PY
