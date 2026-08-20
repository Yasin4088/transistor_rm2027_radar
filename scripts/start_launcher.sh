#!/usr/bin/env bash
set -eu

PROJECT_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
PYTHON_BIN="$PROJECT_ROOT/.venv/bin/python"

# radio_ros 是比赛默认裁判通信方式。桌面启动不会读取交互式 shell，
# 因此在启动台及其 QProcess 子进程继承环境前显式加载 ROS2 Humble。
if [ -r /opt/ros/humble/setup.sh ]; then
    # shellcheck disable=SC1091
    # ROS Humble's generated setup scripts read AMENT_TRACE_SETUP_FILES and
    # several other optional variables without nounset-safe expansion.
    # Temporarily relax `set -u` only while importing that external script.
    set +u
    . /opt/ros/humble/setup.sh
    set -u
fi

# Desktop autostart does not source an interactive shell. Discover the common
# Hikrobot MVS install location before Python imports the vendor binding.
if [ -z "${MVCAM_COMMON_RUNENV:-}" ]; then
    for mvs_root in /opt/MVS/lib /opt/MVS /usr/local/MVS/lib; do
        if [ -f "$mvs_root/64/libMvCameraControl.so" ]; then
            export MVCAM_COMMON_RUNENV="$mvs_root"
            break
        fi
    done
fi
if [ -n "${MVCAM_COMMON_RUNENV:-}" ] && [ -d "$MVCAM_COMMON_RUNENV/64" ]; then
    export LD_LIBRARY_PATH="$MVCAM_COMMON_RUNENV/64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

if [ ! -x "$PYTHON_BIN" ]; then
    PYTHON_BIN=$(command -v python3 || true)
fi

if [ -z "$PYTHON_BIN" ]; then
    echo "未找到 Python 3，请先安装 Python 或创建项目 .venv。" >&2
    exit 1
fi

cd "$PROJECT_ROOT"
exec "$PYTHON_BIN" -u launcher.py "$@"
