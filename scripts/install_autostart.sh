#!/usr/bin/env bash
set -eu

PROJECT_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
START_SCRIPT="$PROJECT_ROOT/scripts/start_launcher.sh"
TEMPLATE="$PROJECT_ROOT/scripts/transistor-radar-launcher.desktop.in"
AUTOSTART_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/autostart"
TARGET="$AUTOSTART_DIR/transistor-radar-launcher.desktop"
TEMP_FILE=$(mktemp)
trap 'rm -f "$TEMP_FILE"' EXIT

escape_sed_replacement() {
    printf '%s' "$1" | sed 's/[&|]/\\&/g'
}

mkdir -p "$AUTOSTART_DIR"
escaped_root=$(escape_sed_replacement "$PROJECT_ROOT")
escaped_start=$(escape_sed_replacement "$START_SCRIPT")
sed \
    -e "s|@PROJECT_ROOT@|$escaped_root|g" \
    -e "s|@START_SCRIPT@|$escaped_start|g" \
    "$TEMPLATE" > "$TEMP_FILE"
install -m 0644 "$TEMP_FILE" "$TARGET"

echo "已启用登录自启动: $TARGET"
echo "登录桌面后将自动打开 Transistor 雷达比赛启动台。"
