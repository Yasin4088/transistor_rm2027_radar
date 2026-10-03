#!/usr/bin/env bash

# Open the dashboard in the user's normal browser profile. No app mode,
# temporary profile, kiosk flag, or browser lifecycle ownership is used.
rm_radio_open_browser_tab() {
  local url="$1"
  local browser
  for browser in google-chrome google-chrome-stable chromium chromium-browser; do
    if command -v "$browser" >/dev/null 2>&1; then
      setsid "$browser" "$url" </dev/null >/dev/null 2>&1 &
      return 0
    fi
  done
  if command -v xdg-open >/dev/null 2>&1; then
    xdg-open "$url" </dev/null >/dev/null 2>&1 &
    return 0
  fi
  if command -v gio >/dev/null 2>&1; then
    gio open "$url" </dev/null >/dev/null 2>&1 &
    return 0
  fi
  echo "no Chrome, xdg-open, or gio command is available; open $url manually" >&2
  return 127
}

rm_radio_wait_for_http() {
  local url="$1"
  local attempts="${2:-100}"
  local interval_sec="${3:-0.1}"
  local _
  for _ in $(seq 1 "$attempts"); do
    if command -v curl >/dev/null 2>&1 && curl -fsS --max-time 0.5 -o /dev/null "$url"; then
      return 0
    fi
    sleep "$interval_sec"
  done
  echo "web panel did not become ready: $url" >&2
  return 1
}

rm_radio_wait_and_open_browser_tab() {
  local url="$1"
  rm_radio_wait_for_http "$url" "${2:-100}" "${3:-0.1}" || return
  rm_radio_open_browser_tab "$url"
}
