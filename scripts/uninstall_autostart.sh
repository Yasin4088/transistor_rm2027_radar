#!/usr/bin/env bash
set -eu

TARGET="${XDG_CONFIG_HOME:-$HOME/.config}/autostart/shark-radar-launcher.desktop"
if [ -e "$TARGET" ]; then
    rm -f "$TARGET"
    echo "已关闭登录自启动: $TARGET"
else
    echo "登录自启动未安装。"
fi
