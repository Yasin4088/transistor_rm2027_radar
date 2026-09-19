#!/usr/bin/env bash

# Shared process and hardware shutdown helpers for lab TX scripts.
# This file deliberately does not enable/disable shell options; callers own
# their errexit policy so EXIT traps can keep cleanup best-effort.

LAB_TX_SAFETY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

lab_tx_pid_is_running() {
  local pid="${1:-}"
  [[ "$pid" =~ ^[0-9]+$ ]] || return 1
  kill -0 "$pid" >/dev/null 2>&1 || return 1
  local stat
  stat="$(ps -o stat= -p "$pid" 2>/dev/null | awk 'NR == 1 { print $1 }')"
  [[ -n "$stat" && "$stat" != Z* ]]
}

lab_tx_stop_pids() {
  # Match RX performs its own one-second child cleanup, while the dual-TX
  # launcher also waits for children and hard-mutes both SDRs.  Give those
  # traps enough time before escalating to SIGKILL.
  local grace_seconds="${LAB_TX_TERM_GRACE_SECONDS:-8}"
  local -a pids=()
  local pid
  for pid in "$@"; do
    [[ "$pid" =~ ^[0-9]+$ && "$pid" != "$$" ]] || continue
    pids+=("$pid")
  done
  ((${#pids[@]})) || return 0

  for pid in "${pids[@]}"; do
    lab_tx_pid_is_running "$pid" && kill -TERM "$pid" >/dev/null 2>&1 || true
  done

  local deadline=$((SECONDS + grace_seconds))
  local any_running
  while ((SECONDS < deadline)); do
    any_running=false
    for pid in "${pids[@]}"; do
      if lab_tx_pid_is_running "$pid"; then
        any_running=true
        break
      fi
    done
    [[ "$any_running" == "false" ]] && break
    sleep 0.1
  done

  for pid in "${pids[@]}"; do
    lab_tx_pid_is_running "$pid" && kill -KILL "$pid" >/dev/null 2>&1 || true
  done
  for pid in "${pids[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
}

lab_tx_dual_child_pids() {
  local pid_file="$1"
  [[ -s "$pid_file" ]] || return 1
  local name pid
  local broadcast_pid=""
  local interference_pid=""
  while IFS='=' read -r name pid; do
    case "$name" in
      broadcast_tx_pid) broadcast_pid="$pid" ;;
      interference_tx_pid) interference_pid="$pid" ;;
    esac
  done < "$pid_file"
  [[ "$broadcast_pid" =~ ^[0-9]+$ && "$interference_pid" =~ ^[0-9]+$ ]] || return 1
  printf '%s\n%s\n' "$broadcast_pid" "$interference_pid"
}

lab_tx_verify_dual_running() {
  local launcher_pid="$1"
  local pid_file="$2"
  if ! lab_tx_pid_is_running "$launcher_pid"; then
    echo "[tx-safety] dual TX launcher is not running: pid=$launcher_pid" >&2
    return 1
  fi
  local child_pids
  if ! child_pids="$(lab_tx_dual_child_pids "$pid_file")"; then
    echo "[tx-safety] dual TX PID file is missing or invalid: $pid_file" >&2
    return 1
  fi
  local pid
  while IFS= read -r pid; do
    if ! lab_tx_pid_is_running "$pid"; then
      echo "[tx-safety] dual TX child is not running: pid=$pid file=$pid_file" >&2
      return 1
    fi
  done <<< "$child_pids"
}

lab_tx_live_residual_pids() {
  if ! declare -F rm_radio_residual_pids >/dev/null 2>&1; then
    echo "[tx-safety] rm_radio_residual_pids is unavailable" >&2
    return 2
  fi
  local candidates pid
  candidates="$(rm_radio_residual_pids "$@")"
  for pid in $candidates; do
    lab_tx_pid_is_running "$pid" && printf '%s\n' "$pid"
  done
}

lab_tx_verify_no_residual_tx() {
  local pids
  pids="$(lab_tx_live_residual_pids "$@")" || return $?
  [[ -z "$pids" ]] && return 0
  echo "[tx-safety] residual TX processes remain:" >&2
  ps -o pid=,ppid=,pgid=,etime=,cmd= -p $(echo "$pids" | tr '\n' ' ') 2>/dev/null | sed 's/^/  /' >&2 || true
  return 1
}

lab_tx_cleanup_residual_tx() {
  local pids
  pids="$(lab_tx_live_residual_pids "$@")" || return $?
  if [[ -n "$pids" ]]; then
    echo "[tx-safety] stopping residual TX PIDs: $(echo "$pids" | tr '\n' ' ')" >&2
    # Signal individual PIDs.  Never signal a process group here: a lab script
    # and its children can share the invoking terminal's PGID.
    # shellcheck disable=SC2086
    lab_tx_stop_pids $pids
  fi
  lab_tx_verify_no_residual_tx "$@"
}

lab_tx_hard_mute_one_and_verify() {
  local uri="$1"
  [[ -n "$uri" ]] || {
    echo "[tx-safety] refusing to mute an empty TX URI" >&2
    return 1
  }
  # shellcheck disable=SC1091
  source "$LAB_TX_SAFETY_DIR/iio_init.sh"
  if ! mute_ad9361_tx "$uri" 89.75; then
    echo "[tx-safety] hardware mute command failed: $uri" >&2
    return 1
  fi

  local gain powerdown ensm
  gain="$(_iio_tx_attr_get "$uri" output voltage0 hardwaregain | tr -d '\r' | tail -n 1)"
  powerdown="$(_iio_tx_altvoltage_get "$uri" altvoltage1 powerdown | tr -d '\r' | tail -n 1)"
  ensm="$(_iio_tx_device_attr_get "$uri" ensm_mode | tr -d '\r' | tail -n 1)"
  echo "[tx-safety] verify uri=$uri hardwaregain=${gain:-?} tx_lo_powerdown=${powerdown:-?} ensm=${ensm:-?}"
  if [[ ! "$gain" =~ -89\.75(0000)?([[:space:]]*dB)? ]] || [[ "${powerdown%% *}" != "1" ]]; then
    echo "[tx-safety] hard-mute verification failed: $uri" >&2
    return 1
  fi
}

lab_tx_hard_mute_and_verify() {
  local rc=0
  local uri
  local -A seen=()
  for uri in "$@"; do
    [[ -n "$uri" && -z "${seen[$uri]:-}" ]] || continue
    seen[$uri]=1
    lab_tx_hard_mute_one_and_verify "$uri" || rc=1
  done
  return "$rc"
}
