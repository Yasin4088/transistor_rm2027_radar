from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import signal
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable, Deque, Dict, Optional
from urllib.parse import parse_qs, urlparse

import rclpy
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from ..core.radio_config import (
    BROADCAST_FREQUENCIES,
    BROADCAST_RX_GAIN_DEFAULT,
    BROADCAST_RX_GAIN_MIN,
    BROADCAST_RX_GAIN_MAX,
    broadcast_rx_setters_for_side,
    interference_rx_setters_for_side_level,
    interference_setters_for_side_level,
)


RECEPTION_FRAME_RATE_WARN_THRESHOLD_HZ = 2.0
RX_STARTUP_GRACE_SEC = 5.0
RX_EXPECTED_FRAME_STALE_SEC = 3.0
RX_LOSS_WARN_RATE = 0.05
RX_LOSS_BAD_RATE = 0.20
STATUS_STALE_WARN_SEC = 5.0
STATUS_STALE_BAD_SEC = 10.0
EXPECTED_BROADCAST_COMMANDS = {
    "0x0A01": "坐标",
    "0x0A02": "血量",
    "0x0A03": "发弹量",
    "0x0A04": "经济",
    "0x0A05": "增益状态",
}
EXPECTED_INTERFERENCE_COMMAND = "0x0A06"


def _now() -> float:
    return time.time()


def _pid_is_alive(pid: Any) -> bool:
    try:
        value = int(pid)
    except (TypeError, ValueError):
        return False
    if value <= 1:
        return False
    try:
        os.kill(value, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    try:
        stat_fields = Path(f"/proc/{value}/stat").read_text(encoding="utf-8").split()
        return len(stat_fields) < 3 or stat_fields[2] != "Z"
    except OSError:
        return True


def _json_safe_loads(raw: str, fallback: Any) -> Any:
    try:
        return json.loads(raw)
    except Exception:
        return fallback


def _yaml_scalar(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if value is None:
        return "null"
    if isinstance(value, float) and value.is_integer():
        return str(int(value))
    if isinstance(value, (int, float)):
        return str(value)
    text = str(value)
    if text and re.fullmatch(r"[A-Za-z0-9_./:+,@-]+", text):
        return text
    return json.dumps(text, ensure_ascii=False)


def _update_simple_yaml_scalars(path: Path, updates: dict[str, Any]) -> None:
    """Atomically update existing scalar keys without discarding YAML comments."""
    path = path.expanduser().resolve()
    lines = path.read_text(encoding="utf-8").splitlines(keepends=True)
    wanted = {str(key): value for key, value in updates.items()}
    found: set[str] = set()
    parents: list[tuple[int, str]] = []
    rendered: list[str] = []

    for raw in lines:
        body = raw.rstrip("\r\n")
        newline = raw[len(body):]
        stripped = body.lstrip(" ")
        indent = len(body) - len(stripped)
        match = re.match(r"^([A-Za-z0-9_]+)\s*:\s*(.*)$", stripped)
        if not match or stripped.startswith("#"):
            rendered.append(raw)
            continue

        key, value_text = match.groups()
        while parents and indent <= parents[-1][0]:
            parents.pop()
        dotted = ".".join([part for _, part in parents] + [key])
        if dotted in wanted:
            rendered.append(f"{' ' * indent}{key}: {_yaml_scalar(wanted[dotted])}{newline or os.linesep}")
            found.add(dotted)
        else:
            rendered.append(raw)
        if not value_text.strip() or value_text.lstrip().startswith("#"):
            parents.append((indent, key))

    missing = sorted(set(wanted) - found)
    if missing:
        raise KeyError(f"配置文件缺少可持久化字段：{', '.join(missing)}")

    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}-{threading.get_ident()}")
    try:
        temporary.write_text("".join(rendered), encoding="utf-8")
        temporary.chmod(path.stat().st_mode)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


@dataclass
class DashboardState:
    mode: str = "match_rx"
    radio_side: str = "red"
    interference_level: int = 1
    # Manual/configured values are a persistent operator baseline. Referee
    # observations only update these effective runtime values.
    effective_radio_side: str = "red"
    effective_interference_level: int = 1
    # 智能模式：干扰等级/红蓝方接收参数完全跟随裁判串口(0x020E/0x0201)自动切换。
    # 关闭后进入手动模式，可在运行中动态设置红蓝方与干扰等级并热下发给两路 RX 节点。
    intelligent_mode: bool = True
    broadcast_gain: float = BROADCAST_RX_GAIN_DEFAULT
    broadcast_gain_min: float = BROADCAST_RX_GAIN_MIN
    broadcast_gain_max: float = BROADCAST_RX_GAIN_MAX
    sdr_uris: dict[str, str] = field(default_factory=lambda: {
        "rx1": "ip:192.168.2.1",
        "rx2": "ip:192.168.3.1",
    })
    referee: dict[str, Any] = field(default_factory=lambda: {
        "port": "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0",
        "baudrate": 115200,
        "sender_id": 9,
        "receiver_id": 0x8080,
        "bridge_topic": "/rm_gfsk_node/referee_bridge,/rm_gfsk_interference_node/referee_bridge",
        "radar_cmd_topic": "/rm_radar_algorithm/radar_cmd",
        "dry_run": False,
        "require_serial_open_on_start": False,
        "frame_timeout_sec": 2.0,
        "auto_send_invincible_targets": True,
        "invincible_targets_data_cmd_id": 0x0234,
        "invincible_targets_send_rate_hz": 3.0,
        "invincible_targets_freshness_sec": 1.0,
    })
    statuses: dict[str, Any] = field(default_factory=dict)
    status_first_seen_at: dict[str, float] = field(default_factory=dict)
    spectra: dict[str, Any] = field(default_factory=dict)
    filtered_spectra: dict[str, Any] = field(default_factory=dict)
    display_metrics: dict[str, Any] = field(default_factory=dict)
    frames: Deque[dict[str, Any]] = field(default_factory=lambda: deque(maxlen=200))
    bridge_outputs: Deque[dict[str, Any]] = field(default_factory=lambda: deque(maxlen=200))
    radar: dict[str, Any] = field(default_factory=dict)
    vision_radar: dict[str, Any] = field(default_factory=dict)
    sdr_checks: dict[str, Any] = field(default_factory=dict)
    events: Deque[dict[str, Any]] = field(default_factory=lambda: deque(maxlen=120))
    process_specs: dict[str, Any] = field(default_factory=dict)


class RadioDashboardNode(Node):
    def __init__(self, host: str = "127.0.0.1", port: int = 8765, web_root: Optional[Path] = None):
        super().__init__("rm_match_dashboard")
        self.host = host
        self.port = int(port)
        self.web_root = web_root or self._default_web_root()
        self.config_path = self._default_config_path()
        self._lock = threading.RLock()
        self._stream_condition = threading.Condition(self._lock)
        self.state = DashboardState()
        self._processes: dict[str, subprocess.Popen] = {}
        self._process_logs: dict[str, Any] = {}
        self._frame_sequence = 0
        self._stream_generation = 0
        self._radar_generation = 0
        self._sdr_check_lock = threading.Lock()
        self._load_env_defaults()

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # 手动模式下由面板直接热下发两路 RX 接收参数（与 referee 自动下发共用同一 topic）。
        rx_ctrl_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.broadcast_rx_setters_pub = self.create_publisher(String, "/rm_gfsk_node/setters_json", rx_ctrl_qos)
        self.interference_rx_setters_pub = self.create_publisher(String, "/rm_gfsk_interference_node/setters_json", rx_ctrl_qos)

        spectrum_qos = QoSProfile(depth=1)
        for node_name, role in [
            ("/rm_gfsk_node", "rx1"),
            ("/rm_gfsk_interference_node", "rx2"),
        ]:
            self.create_subscription(String, f"{node_name}/status", self._status_callback(role), 10)
            self.create_subscription(String, f"{node_name}/frames", self._frames_callback(role), 10)
            self.create_subscription(String, f"{node_name}/referee_bridge", self._bridge_callback(role), 10)
            self.create_subscription(String, f"{node_name}/spectrum", self._spectrum_callback(role), spectrum_qos)
            self.create_subscription(
                String,
                f"{node_name}/filtered_spectrum",
                self._spectrum_callback(role, tap="filtered"),
                spectrum_qos,
            )
        self.create_subscription(String, "/rm_referee_serial_node/status", self._status_callback("referee"), 10)
        self.create_subscription(String, "/rm_referee_serial_node/rx_frames", self._frames_callback("referee"), 10)
        self.create_subscription(String, "/rm_referee_serial_node/referee_bridge", self._bridge_callback("referee"), 10)
        self.create_subscription(
            String,
            "/rm_match_recorder/status",
            self._status_callback("recorder"),
            10,
        )
        self.create_subscription(
            String, "/rm_radar_algorithm/telemetry", self._radar_callback("vision_radar"), 10
        )
        self.create_subscription(
            String, "/rm_radar_integration/status", self._radar_callback("radar_integration"), 10
        )

        self.create_timer(3.0, self._schedule_sdr_check)
        self.create_timer(1.0, self._refresh_process_status)
        self._server = self._make_server()
        self._server_thread = threading.Thread(target=self._server.serve_forever, name="rm-radio-dashboard-http", daemon=True)
        self._server_thread.start()
        self._event("dashboard", f"控制台已启动：http://{self.host}:{self.port}/")
        self.get_logger().info(f"dashboard listening at http://{self.host}:{self.port}/")

    def _default_web_root(self) -> Path:
        for env_name in ("RM_RADIO_MATCH_WEB_ROOT", "RM_RADIO_DASHBOARD_WEB_ROOT"):
            configured = os.environ.get(env_name, "").strip()
            if configured:
                candidate = Path(configured).expanduser().resolve()
                if candidate.exists():
                    return candidate
        for candidate in self._candidate_workspace_roots():
            app_web = candidate / "apps" / "match_rx" / "web"
            if app_web.exists():
                return app_web
        try:
            share = Path(get_package_share_directory("rm_radio_ros"))
            candidate = share / "apps" / "match_rx" / "web"
            if candidate.exists():
                return candidate
        except PackageNotFoundError:
            pass
        return self._workspace_root() / "apps" / "match_rx" / "web"

    def _default_config_path(self) -> Path:
        configured = os.environ.get("MATCH_RX_CONFIG", "").strip()
        if configured:
            return Path(configured).expanduser().resolve()
        return (self._workspace_root() / "apps" / "match_rx" / "config.yaml").resolve()

    def _load_env_defaults(self) -> None:
        self.state.radio_side = os.environ.get("RM_RADIO_SIDE", self.state.radio_side).strip().lower() or "red"
        try:
            self.state.interference_level = int(os.environ.get("INTERFERENCE_LEVEL", self.state.interference_level))
        except ValueError:
            self.state.interference_level = 1
        try:
            configured_max = float(os.environ.get("BROADCAST_RX_GAIN_MAX", BROADCAST_RX_GAIN_MAX))
        except ValueError:
            configured_max = BROADCAST_RX_GAIN_MAX
        self.state.broadcast_gain_max = min(
            max(configured_max, self.state.broadcast_gain_min),
            BROADCAST_RX_GAIN_MAX,
        )
        try:
            configured_gain = float(os.environ.get("BROADCAST_RX_GAIN", BROADCAST_RX_GAIN_DEFAULT))
        except ValueError:
            configured_gain = BROADCAST_RX_GAIN_DEFAULT
        self.state.broadcast_gain = min(
            max(configured_gain, self.state.broadcast_gain_min),
            self.state.broadcast_gain_max,
        )
        self.state.effective_radio_side = self.state.radio_side
        self.state.effective_interference_level = self.state.interference_level
        self.state.intelligent_mode = os.environ.get(
            "RM_RADIO_INTELLIGENT_MODE",
            str(self.state.intelligent_mode),
        ).strip().lower() in {"1", "true", "yes", "on"}
        self.state.sdr_uris["rx1"] = os.environ.get("RX1_URI", os.environ.get("RX_URI", self.state.sdr_uris["rx1"]))
        self.state.sdr_uris["rx2"] = os.environ.get("RX2_URI", self.state.sdr_uris["rx2"])
        self.state.referee["port"] = os.environ.get("REFEREE_PORT", self.state.referee["port"])
        self.state.referee["baudrate"] = int(os.environ.get("REFEREE_BAUDRATE", self.state.referee["baudrate"]))
        self.state.referee["sender_id"] = int(os.environ.get(
            "RADAR_SENDER_ID",
            109 if self.state.radio_side == "blue" else self.state.referee["sender_id"],
        ))
        self.state.referee["receiver_id"] = int(os.environ.get("REFEREE_RECEIVER_ID", self.state.referee["receiver_id"]))
        self.state.referee["bridge_topic"] = os.environ.get("REFEREE_BRIDGE_TOPIC", self.state.referee["bridge_topic"])
        self.state.referee["radar_cmd_topic"] = os.environ.get("RADAR_CMD_TOPIC", self.state.referee["radar_cmd_topic"])
        self.state.referee["require_serial_open_on_start"] = os.environ.get(
            "REQUIRE_REFEREE_SERIAL_OPEN_ON_START",
            str(self.state.referee["require_serial_open_on_start"]),
        ).strip().lower() in {"1", "true", "yes", "on"}
        self.state.referee["frame_timeout_sec"] = max(
            0.0,
            float(os.environ.get("REFEREE_FRAME_TIMEOUT_SEC", self.state.referee["frame_timeout_sec"])),
        )
        self.state.referee["auto_send_invincible_targets"] = os.environ.get(
            "AUTO_SEND_INVINCIBLE_TARGETS",
            str(self.state.referee["auto_send_invincible_targets"]),
        ).strip().lower() in {"1", "true", "yes", "on"}
        self.state.referee["invincible_targets_data_cmd_id"] = int(str(os.environ.get(
            "INVINCIBLE_TARGETS_DATA_CMD_ID",
            self.state.referee["invincible_targets_data_cmd_id"],
        )), 0)
        self.state.referee["invincible_targets_send_rate_hz"] = max(
            0.1,
            float(os.environ.get(
                "INVINCIBLE_TARGETS_SEND_RATE_HZ",
                self.state.referee["invincible_targets_send_rate_hz"],
            )),
        )
        self.state.referee["invincible_targets_freshness_sec"] = max(
            0.1,
            float(os.environ.get(
                "INVINCIBLE_TARGETS_FRESHNESS_SEC",
                self.state.referee["invincible_targets_freshness_sec"],
            )),
        )
        for name, env_name in (
            ("rx", "RM_RADIO_SUPERVISED_RX_PID"),
            ("referee", "RM_RADIO_SUPERVISED_REFEREE_PID"),
        ):
            raw_pid = os.environ.get(env_name, "").strip()
            if not raw_pid.isdigit():
                continue
            self.state.process_specs[name] = {
                "name": name,
                "running": _pid_is_alive(raw_pid),
                "pid": int(raw_pid),
                "returncode": None,
                "externally_managed": True,
                "owner": "match_rx_start",
            }

    def _make_server(self) -> ThreadingHTTPServer:
        dashboard = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def do_GET(self):
                dashboard._handle_get(self)

            def do_POST(self):
                dashboard._handle_post(self)

            def log_message(self, fmt, *args):
                dashboard.get_logger().debug(fmt % args)

        return ThreadingHTTPServer((self.host, self.port), Handler)

    def _status_callback(self, role: str) -> Callable[[String], None]:
        def callback(msg: String) -> None:
            data = _json_safe_loads(msg.data, {"raw": msg.data})
            with self._lock:
                received_at = _now()
                self.state.status_first_seen_at.setdefault(role, received_at)
                self.state.statuses[role] = {
                    "timestamp": received_at,
                    "data": data,
                }
                if role == "referee" and self.state.intelligent_mode and isinstance(data, dict):
                    self._sync_effective_from_referee_unlocked(data)

        return callback

    def _sync_effective_from_referee_unlocked(self, data: Optional[dict[str, Any]] = None) -> bool:
        if data is None:
            entry = self.state.statuses.get("referee") or {}
            data = entry.get("data") if isinstance(entry.get("data"), dict) else {}
        changed = False
        side = str(data.get("effective_radio_side") or "").strip().lower()
        if side in ("red", "blue") and side != self.state.effective_radio_side:
            self.state.effective_radio_side = side
            changed = True
        for key in ("rx_interference_level", "confirmed_referee_level", "interference_level"):
            try:
                level = int(data.get(key))
            except (TypeError, ValueError):
                continue
            if level in (1, 2, 3):
                if level != self.state.effective_interference_level:
                    self.state.effective_interference_level = level
                    changed = True
                break
        if changed:
            # A 0x0201 side change and a 0x020E level change are independent.
            # Always publish a complete pair so neither RX can remain tuned to
            # the previous side merely because the other value did not change.
            self._publish_effective_rx_setters_unlocked()
        return changed

    def _frames_callback(self, role: str) -> Callable[[String], None]:
        def callback(msg: String) -> None:
            data = _json_safe_loads(msg.data, {"raw": msg.data})
            with self._stream_condition:
                self._frame_sequence = int(getattr(self, "_frame_sequence", 0)) + 1
                self.state.frames.appendleft({
                    "seq": self._frame_sequence,
                    "timestamp": _now(),
                    "role": role,
                    "frame": data,
                })
                self._stream_generation = int(getattr(self, "_stream_generation", 0)) + 1
                self._stream_condition.notify_all()

        return callback

    def _bridge_callback(self, role: str) -> Callable[[String], None]:
        def callback(msg: String) -> None:
            data = _json_safe_loads(msg.data, {"raw": msg.data})
            with self._lock:
                self.state.bridge_outputs.appendleft({
                    "timestamp": _now(),
                    "role": role,
                    "output": data,
                })

        return callback

    def _radar_callback(self, role: str) -> Callable[[String], None]:
        def callback(msg: String) -> None:
            data = _json_safe_loads(msg.data, {"raw": msg.data})
            with self._stream_condition:
                received_at = _now()
                self.state.status_first_seen_at.setdefault(role, received_at)
                entry = {"timestamp": received_at, "data": data}
                self.state.statuses[role] = entry
                if role == "radar_integration":
                    self.state.radar = entry
                else:
                    self.state.vision_radar = entry
                self._radar_generation = int(getattr(self, "_radar_generation", 0)) + 1
                self._stream_condition.notify_all()

        return callback

    def _spectrum_callback(self, role: str, tap: str = "raw") -> Callable[[String], None]:
        def callback(msg: String) -> None:
            data = _json_safe_loads(msg.data, {"raw": msg.data})
            with self._stream_condition:
                target = self.state.filtered_spectra if tap == "filtered" else self.state.spectra
                target[role] = {
                    "timestamp": _now(),
                    "data": data,
                }
                self._stream_generation = int(getattr(self, "_stream_generation", 0)) + 1
                self._stream_condition.notify_all()

        return callback

    def _event(self, kind: str, message: str, detail: Optional[dict[str, Any]] = None) -> None:
        with self._lock:
            self.state.events.appendleft({
                "timestamp": _now(),
                "kind": kind,
                "message": message,
                "detail": detail or {},
            })

    def _check_sdrs(self) -> None:
        with self._lock:
            roles = self._sdr_roles_unlocked()
            uris = {role: self.state.sdr_uris.get(role, "") for role in roles}
        checks = {}
        iio_info = shutil.which("iio_info")
        for role, uri in uris.items():
            result: dict[str, Any] = {
                "uri": uri,
                "checked_at": _now(),
                "available": None,
                "message": "",
            }
            if not uri:
                result.update({"available": False, "message": "未配置 URI"})
            elif iio_info is None:
                result.update({"available": None, "message": "未找到 iio_info，无法检查"})
            else:
                try:
                    completed = subprocess.run(
                        [iio_info, "-u", uri],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.PIPE,
                        text=True,
                        timeout=1.5,
                        check=False,
                    )
                    if completed.returncode == 0:
                        result.update({"available": True, "message": "可连接"})
                    else:
                        stderr = (completed.stderr or "").strip()
                        result.update({"available": False, "message": stderr[-160:] or "连接失败"})
                except subprocess.TimeoutExpired:
                    result.update({"available": False, "message": "检查超时"})
                except Exception as exc:
                    result.update({"available": False, "message": str(exc)})
            checks[role] = result
        with self._lock:
            self.state.sdr_checks = checks

    def _schedule_sdr_check(self) -> None:
        # iio_info can block for up to 1.5 s per SDR. Running it in the ROS
        # executor used to pause spectrum subscription callbacks every 3 s.
        lock = self._sdr_check_lock
        if not lock.acquire(blocking=False):
            return

        def worker() -> None:
            try:
                self._check_sdrs()
            finally:
                lock.release()

        threading.Thread(
            target=worker,
            name="rm-dashboard-sdr-check",
            daemon=True,
        ).start()

    def _sdr_roles_unlocked(self) -> tuple[str, ...]:
        return ("rx1", "rx2")

    def _workspace_root(self) -> Path:
        for candidate in self._candidate_workspace_roots():
            if (candidate / "src" / "rm_radio_ros" / "package.xml").exists():
                return candidate
        return Path(__file__).resolve().parents[4]

    @staticmethod
    def _candidate_workspace_roots() -> list[Path]:
        return [candidate for candidate in Path(__file__).resolve().parents]

    def _log_dir(self) -> Path:
        runtime_dir = os.environ.get("RM_RADIO_RUNTIME_LOG_DIR", "").strip()
        if runtime_dir:
            path = Path(runtime_dir).expanduser()
        else:
            root = os.environ.get(
                "RM_RADIO_LOG_ROOT",
                "~/.local/state/shark-radio/runtime-logs",
            )
            path = Path(root).expanduser() / f"match_dashboard_{time.strftime('%Y%m%d_%H%M%S')}"
        path.mkdir(parents=True, exist_ok=True)
        return path

    def _base_env(self, process_name: str = "") -> dict[str, str]:
        env = dict(os.environ)
        env.setdefault("ROS_LOG_DIR", "/tmp/ros_log")
        env.setdefault("ROS_HOME", "/tmp/ros_home")
        env.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
        env.setdefault("XDG_CACHE_HOME", "/tmp")
        env["RM_RADIO_SIDE"] = self.state.radio_side
        env["INTERFERENCE_LEVEL"] = str(self.state.interference_level)
        env["RX1_URI"] = self.state.sdr_uris["rx1"]
        env["RX2_URI"] = self.state.sdr_uris["rx2"]
        if process_name == "rx":
            env.setdefault("RM_RADIO_RX_TRACKING", "true")
        return env

    @staticmethod
    def _shell_quote(value: Any) -> str:
        import shlex

        return shlex.quote(str(value))

    def _bash_command(self, args: list[str]) -> list[str]:
        quoted = " ".join(self._shell_quote(arg) for arg in args)
        setup = "source /opt/ros/humble/setup.bash"
        install_setup = self._workspace_root() / "install" / "setup.bash"
        if install_setup.exists():
            setup += f" && source {self._shell_quote(install_setup)}"
        else:
            setup += f" && export PYTHONPATH={self._shell_quote(self._workspace_root() / 'src' / 'rm_radio_ros')}:$PYTHONPATH"
        return ["bash", "-lc", f"{setup} && cd {self._shell_quote(self._workspace_root())} && {quoted}"]

    def _rx_launch_command_unlocked(self) -> list[str]:
        flowgraph_dir = self._workspace_root() / "src" / "rm_radio_ros" / "flowgraphs" / "gfsk"
        return [
            "ros2", "launch", "rm_radio_ros", "rx2.launch.py",
            f"radio_side:={self.state.radio_side}",
            f"rx1_uri:={self.state.sdr_uris['rx1']}",
            f"rx2_uri:={self.state.sdr_uris['rx2']}",
            f"interference_level:={self.state.interference_level}",
            f"flowgraph_dir:={flowgraph_dir}",
            "air_extractor_max_access_hamming:=3",
            "air_extractor_allow_inverted:=true",
            "setters_apply_settle_sec:=0.15",
            "disable_gui_sinks:=true",
            "qt_platform:=offscreen",
        ]

    def _referee_launch_command_unlocked(self) -> list[str]:
        referee = self.state.referee
        return [
            "ros2", "launch", "rm_radio_ros", "referee_serial.launch.py",
            f"dry_run:={'true' if bool(referee.get('dry_run')) else 'false'}",
            f"require_serial_open_on_start:={'true' if bool(referee.get('require_serial_open_on_start')) else 'false'}",
            f"frame_timeout_sec:={max(0.0, float(referee.get('frame_timeout_sec', 2.0)))}",
            f"port:={referee.get('port', '')}",
            f"baudrate:={int(referee.get('baudrate', 115200))}",
            f"sender_id:={int(referee.get('sender_id', 9))}",
            f"receiver_id:={int(referee.get('receiver_id', 0x8080))}",
            f"radar_cmd_topic:={referee.get('radar_cmd_topic', '/rm_radar_algorithm/radar_cmd')}",
            f"bridge_topic:={referee.get('bridge_topic', '/rm_gfsk_node/referee_bridge,/rm_gfsk_interference_node/referee_bridge')}",
            "password_verify_cooldown_sec:=10.0",
            f"auto_send_invincible_targets:={'true' if bool(referee.get('auto_send_invincible_targets', True)) else 'false'}",
            f"invincible_targets_data_cmd_id:={int(referee.get('invincible_targets_data_cmd_id', 0x0234))}",
            f"invincible_targets_send_rate_hz:={max(0.1, float(referee.get('invincible_targets_send_rate_hz', 3.0)))}",
            f"invincible_targets_freshness_sec:={max(0.1, float(referee.get('invincible_targets_freshness_sec', 1.0)))}",
            "auto_interference_level:=false",
            "auto_rx_interference_level:=true",
            "apply_referee_level_only_when_running:=false",
            "single_rx_auto_broadcast_after_level3:=false",
            "single_rx_auto_allow_return_to_interference:=true",
            "referee_level_confirm_count:=2",
            f"radio_side:={self.state.radio_side}",
            f"interference_level:={self.state.interference_level}",
            "interference_rx_control_topic:=/rm_gfsk_interference_node/setters_json",
        ]

    def _process_command_unlocked(self, name: str) -> list[str]:
        if name == "rx":
            return self._rx_launch_command_unlocked()
        if name == "referee":
            return self._referee_launch_command_unlocked()
        raise ValueError(f"未知链路：{name}")

    def _process_snapshot_unlocked(self) -> dict[str, Any]:
        out = {}
        for name in ("rx", "referee"):
            proc = self._processes.get(name)
            spec = self.state.process_specs.get(name, {})
            externally_managed = bool(spec.get("externally_managed"))
            running = (
                _pid_is_alive(spec.get("pid"))
                if externally_managed
                else proc is not None and proc.poll() is None
            )
            entry = {
                **spec,
                "running": running,
                "pid": proc.pid if proc is not None else spec.get("pid"),
                "returncode": None if proc is None else proc.poll(),
            }
            log_path = spec.get("log_path")
            if log_path:
                entry["log_tail"] = self._read_log_tail(Path(log_path))
            out[name] = entry
        return out

    @staticmethod
    def _read_log_tail(path: Path, max_lines: int = 18) -> list[str]:
        try:
            lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
            return lines[-max_lines:]
        except Exception:
            return []

    def _refresh_process_status(self) -> None:
        with self._lock:
            for name, proc in list(self._processes.items()):
                returncode = proc.poll()
                spec = self.state.process_specs.setdefault(name, {})
                spec["running"] = returncode is None
                spec["returncode"] = returncode
                if returncode is not None and not spec.get("reported_exit"):
                    spec["reported_exit"] = True
                    self._event("process", f"{name} 已退出", {"returncode": returncode})

    def _start_process(self, name: str) -> dict[str, Any]:
        with self._lock:
            spec = self.state.process_specs.get(name, {})
            if spec.get("externally_managed"):
                state = "正在运行" if _pid_is_alive(spec.get("pid")) else "等待一键启动器恢复"
                raise ValueError(f"{name} 由一键启动器托管（{state}），禁止重复启动")
            existing = self._processes.get(name)
            if existing is not None and existing.poll() is None:
                raise ValueError(f"{name} 已在运行")
            command = self._process_command_unlocked(name)
            log_path = self._log_dir() / f"{name}_{time.strftime('%Y%m%d_%H%M%S')}.log"
            env = self._base_env(name)
            log_file = log_path.open("w", encoding="utf-8", buffering=1)
            wrapped = self._bash_command(command)
            proc = subprocess.Popen(
                wrapped,
                cwd=str(self._workspace_root()),
                env=env,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
            self._processes[name] = proc
            self._process_logs[name] = log_file
            self.state.process_specs[name] = {
                "name": name,
                "running": True,
                "pid": proc.pid,
                "started_at": _now(),
                "command": command,
                "log_path": str(log_path),
                "returncode": None,
                "reported_exit": False,
            }
        self._event("process", f"{name} 已启动", {"pid": proc.pid, "log_path": str(log_path)})
        return self._snapshot()

    def _stop_process(self, name: str) -> dict[str, Any]:
        with self._lock:
            spec = self.state.process_specs.get(name, {})
            if spec.get("externally_managed"):
                raise ValueError(f"{name} 由一键启动器托管，请在启动终端按 Ctrl+C 统一停止")
            proc = self._processes.get(name)
            if proc is None or proc.poll() is not None:
                self.state.process_specs.setdefault(name, {})["running"] = False
                return self._snapshot()
            os.killpg(proc.pid, signal.SIGTERM)
        try:
            proc.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=3.0)
        with self._lock:
            spec = self.state.process_specs.setdefault(name, {})
            spec["running"] = False
            spec["returncode"] = proc.returncode
            log_file = self._process_logs.pop(name, None)
            if log_file is not None:
                try:
                    log_file.close()
                except Exception:
                    pass
        self._event("process", f"{name} 已停止", {"returncode": proc.returncode})
        return self._snapshot()

    def _stop_all_processes(self) -> None:
        for name in list(self._processes):
            try:
                self._stop_process(name)
            except Exception:
                pass

    def _snapshot(self) -> dict[str, Any]:
        with self._lock:
            now = _now()
            return {
                "mode": self.state.mode,
                "radio_side": self.state.radio_side,
                "interference_level": self.state.interference_level,
                "configured": {
                    "radio_side": self.state.radio_side,
                    "interference_level": self.state.interference_level,
                },
                "effective": {
                    "radio_side": self.state.effective_radio_side,
                    "interference_level": self.state.effective_interference_level,
                },
                "intelligent_mode": self.state.intelligent_mode,
                "sdr_uris": dict(self.state.sdr_uris),
                "referee": dict(self.state.referee),
                "statuses": self.state.statuses,
                "spectra": self.state.spectra,
                "filtered_spectra": self.state.filtered_spectra,
                "rx1_gain": self._rx1_gain_snapshot_unlocked(),
                "display_metrics": dict(self.state.display_metrics),
                "reception_rates": self._reception_rates_unlocked(now=now),
                "health": self._health_unlocked(now=now),
                "radar": self.state.radar,
                "vision_radar": self.state.vision_radar,
                # Streaming data has a dedicated incremental endpoint. Keep a
                # short compatibility tail here so a 2-second state refresh
                # never serializes hundreds of large information-wave frames.
                "frames": list(self.state.frames)[:12],
                "bridge_outputs": list(self.state.bridge_outputs)[:12],
                "stream": {
                    "frame_sequence": int(getattr(self, "_frame_sequence", 0)),
                },
                "sdr_checks": self.state.sdr_checks,
                "events": list(self.state.events),
                "processes": self._process_snapshot_unlocked(),
                "runtime": {
                    "log_root": os.environ.get("RM_RADIO_RUNTIME_LOG_DIR")
                    or os.environ.get(
                        "RM_RADIO_LOG_ROOT",
                        "~/.local/state/shark-radio/runtime-logs",
                    ),
                    "recording_root": os.environ.get(
                        "RM_RADIO_RECORDING_ROOT",
                        "~/.local/share/shark-radio/match-records",
                    ),
                    "web_root": str(getattr(self, "web_root", "")),
                    "match_locked": True,
                    "config_path": str(getattr(self, "config_path", "")),
                },
                "frequency_plan": self._frequency_plan_unlocked(),
            }

    def _reception_rates_unlocked(
        self,
        *,
        now: Optional[float] = None,
        window_sec: float = 1.0,
    ) -> dict[str, float]:
        """Return decoded RX frame rates without depending on browser streams."""
        current = _now() if now is None else float(now)
        window = max(0.1, float(window_sec))
        cutoff = current - window
        counts = {"rx1": 0, "rx2": 0}
        for item in self.state.frames:
            role = item.get("role")
            if role not in counts:
                continue
            try:
                timestamp = float(item.get("timestamp", 0.0))
            except (TypeError, ValueError):
                continue
            if cutoff <= timestamp <= current:
                counts[role] += 1
        return {
            "window_sec": window,
            "rx1_hz": counts["rx1"] / window,
            "rx2_hz": counts["rx2"] / window,
        }

    def _rx1_gain_snapshot_unlocked(self) -> dict[str, Any]:
        entry = self.state.statuses.get("rx1") or {}
        payload = entry.get("data") if isinstance(entry.get("data"), dict) else {}
        runtime = payload.get("rx_gain") if isinstance(payload.get("rx_gain"), dict) else {}
        actual = runtime.get("gain_db")
        if actual is None:
            last_setters = payload.get("last_setters") if isinstance(payload.get("last_setters"), dict) else {}
            actual = last_setters.get("Gain")
        return {
            "configured_db": float(self.state.broadcast_gain),
            "min_db": float(self.state.broadcast_gain_min),
            "max_db": float(self.state.broadcast_gain_max),
            "actual_db": actual,
            "mode": runtime.get("mode", "manual"),
        }

    @staticmethod
    def _number_or_none(value: Any) -> Optional[float]:
        try:
            number = float(value)
            return number if math.isfinite(number) else None
        except (TypeError, ValueError):
            return None

    def _role_uptime_unlocked(self, role: str, now: float) -> float:
        entry = self.state.statuses.get(role) or {}
        first_seen = self.state.status_first_seen_at.get(role, entry.get("timestamp"))
        age = self._age_or_none(first_seen, now)
        return 0.0 if age is None else age

    @staticmethod
    def _frame_command_hex(frame: Any) -> Optional[str]:
        if not isinstance(frame, dict):
            return None
        value = frame.get("cmd_id")
        if value is None:
            value = frame.get("cmd_hex")
        try:
            command = int(str(value), 0) if isinstance(value, str) else int(value)
        except (TypeError, ValueError):
            return None
        return f"0x{command & 0xFFFF:04X}"

    def _latest_frames_by_command_unlocked(self, role: str) -> dict[str, float]:
        latest: dict[str, float] = {}
        for item in self.state.frames:
            if item.get("role") != role:
                continue
            command = self._frame_command_hex(item.get("frame"))
            timestamp = self._number_or_none(item.get("timestamp"))
            if command is None or timestamp is None:
                continue
            latest[command] = max(timestamp, latest.get(command, 0.0))
        return latest

    def _frame_rate_health_unlocked(self, role: str, label: str, rate: float, window: float, now: float) -> dict[str, Any]:
        entry = self.state.statuses.get(role) or {}
        data = entry.get("data") if isinstance(entry.get("data"), dict) else {}
        if not entry:
            return {"state": "unknown", "last_seen_age_sec": None, "message": f"尚未收到{label}节点状态，无法评估帧率"}
        if data.get("started") is not True:
            return {"state": "unknown", "last_seen_age_sec": None, "message": f"{label}节点未运行，帧率待检测"}
        if rate <= 0.0 and self._role_uptime_unlocked(role, now) < RX_STARTUP_GRACE_SEC:
            return {"state": "unknown", "last_seen_age_sec": 0.0, "message": f"{label}节点启动中，等待首帧"}
        below_threshold = rate < RECEPTION_FRAME_RATE_WARN_THRESHOLD_HZ
        return {
            "state": "warn" if below_threshold else "ok",
            "last_seen_age_sec": 0.0,
            "message": (
                f"{label} {window:g}s 平均帧率 {rate:.1f} 帧/s，"
                f"{'低于' if below_threshold else '达到'} {RECEPTION_FRAME_RATE_WARN_THRESHOLD_HZ:g} 帧/s 门限"
            ),
        }

    def _command_completeness_health_unlocked(self, role: str, now: float) -> dict[str, Any]:
        entry = self.state.statuses.get(role) or {}
        data = entry.get("data") if isinstance(entry.get("data"), dict) else {}
        label = "信息波" if role == "rx1" else "干扰波"
        if not entry or data.get("started") is not True:
            return {"state": "unknown", "last_seen_age_sec": None, "message": f"{label}节点未就绪，业务帧待检测"}
        latest = self._latest_frames_by_command_unlocked(role)
        expected = EXPECTED_BROADCAST_COMMANDS if role == "rx1" else {EXPECTED_INTERFERENCE_COMMAND: "密钥"}
        uptime = self._role_uptime_unlocked(role, now)
        missing = [command for command in expected if command not in latest]
        stale = {
            command: max(0.0, now - latest[command])
            for command in expected
            if command in latest and now - latest[command] > RX_EXPECTED_FRAME_STALE_SEC
        }
        if not latest and uptime < RX_STARTUP_GRACE_SEC:
            return {"state": "unknown", "last_seen_age_sec": 0.0, "message": f"{label}等待首轮业务帧"}
        if not missing and not stale:
            return {
                "state": "ok",
                "last_seen_age_sec": max((now - latest[command] for command in expected), default=0.0),
                "message": f"{label}预期业务帧均在 {RX_EXPECTED_FRAME_STALE_SEC:g} 秒内到达",
            }
        descriptions = [f"{command} {expected[command]}未到达" for command in missing]
        descriptions.extend(f"{command} {expected[command]}已中断 {age:.1f} 秒" for command, age in stale.items())
        ages = list(stale.values())
        return {
            # A healthy receiver can legitimately have no transmitter on air.
            # Missing or stale traffic is therefore capped at WARN; node/SDR/
            # decoder failures are reported by their dedicated health checks.
            "state": "warn",
            "last_seen_age_sec": max(ages) if ages else None,
            "message": f"{label}业务帧异常：" + "；".join(descriptions),
        }

    def _tuning_health_unlocked(self, role: str, now: float) -> dict[str, Any]:
        entry = self.state.statuses.get(role) or {}
        data = entry.get("data") if isinstance(entry.get("data"), dict) else {}
        label = "RX1 信息波" if role == "rx1" else "RX2 干扰波"
        if not entry or data.get("started") is not True:
            return {"state": "unknown", "last_seen_age_sec": None, "message": f"{label}未运行，调谐参数待检测"}
        side = self.state.effective_radio_side if self.state.effective_radio_side in ("red", "blue") else "red"
        level = self.state.effective_interference_level if self.state.effective_interference_level in (1, 2, 3) else 1
        expected = (
            broadcast_rx_setters_for_side(side, self.state.broadcast_gain)
            if role == "rx1"
            else interference_rx_setters_for_side_level(side, level)
        )
        setters = data.get("last_setters") if isinstance(data.get("last_setters"), dict) else {}
        actual_center = None
        for key in ("cen_f", "center_f", "center_F"):
            actual_center = self._number_or_none(setters.get(key))
            if actual_center is not None:
                break
        expected_center = self._number_or_none(expected.get("cen_f"))
        if actual_center is None:
            state = "unknown" if self._role_uptime_unlocked(role, now) < RX_STARTUP_GRACE_SEC else "warn"
            return {"state": state, "last_seen_age_sec": None, "message": f"{label}尚未回报实际中心频率"}
        if expected_center is not None and abs(actual_center - expected_center) > 1.0:
            return {
                "state": "bad",
                "last_seen_age_sec": None,
                "message": f"{label}频点不一致：实际 {actual_center / 1e6:.3f} MHz，期望 {expected_center / 1e6:.3f} MHz（{side} L{level}）",
            }
        if role == "rx1":
            gain = data.get("rx_gain") if isinstance(data.get("rx_gain"), dict) else {}
            actual_gain = self._number_or_none(gain.get("gain_db"))
            if actual_gain is None:
                actual_gain = self._number_or_none(setters.get("Gain"))
            if actual_gain is not None and abs(actual_gain - self.state.broadcast_gain) > 1.1:
                return {
                    "state": "warn",
                    "last_seen_age_sec": None,
                    "message": f"RX1 实际增益 {actual_gain:.1f} dB，与面板设置 {self.state.broadcast_gain:.1f} dB 不一致",
                }
        return {"state": "ok", "last_seen_age_sec": None, "message": f"{label}频点与有效阵营/等级一致"}

    def _signal_health_unlocked(self, role: str, now: float) -> dict[str, Any]:
        entry = self.state.statuses.get(role) or {}
        data = entry.get("data") if isinstance(entry.get("data"), dict) else {}
        label = "RX1 信息波" if role == "rx1" else "RX2 干扰波"
        decoder = data.get("decoder") if isinstance(data.get("decoder"), dict) else {}
        iq = decoder.get("iq_diagnostics") if isinstance(decoder.get("iq_diagnostics"), dict) else {}
        if not entry or data.get("started") is not True:
            return {"state": "unknown", "last_seen_age_sec": None, "message": f"{label}未运行，IQ 信号质量待检测"}
        if not iq:
            state = "unknown" if self._role_uptime_unlocked(role, now) < RX_STARTUP_GRACE_SEC else "warn"
            return {"state": state, "last_seen_age_sec": None, "message": f"{label}运行中但尚无 IQ 诊断样本"}
        signal_state = str(iq.get("signal_state") or "unknown")
        clip_fraction = max(0.0, self._number_or_none(iq.get("clip_fraction")) or 0.0)
        headroom = self._number_or_none(iq.get("headroom_db"))
        rms = self._number_or_none(iq.get("rms_dbfs"))
        if signal_state == "clipped" or clip_fraction > 0.0:
            return {
                "state": "bad",
                "last_seen_age_sec": None,
                "message": f"{label} ADC 输入削顶 {clip_fraction * 100:.3f}%（余量 {headroom:.1f} dB），请降低增益" if headroom is not None else f"{label} ADC 输入削顶 {clip_fraction * 100:.3f}%，请降低增益",
            }
        if signal_state == "warning":
            return {
                "state": "warn",
                "last_seen_age_sec": None,
                "message": f"{label}接近削顶，ADC 余量仅 {headroom:.1f} dB" if headroom is not None else f"{label}接近削顶",
            }
        latest_frames = self._latest_frames_by_command_unlocked(role)
        no_recent_frame = not latest_frames or now - max(latest_frames.values()) > RX_EXPECTED_FRAME_STALE_SEC
        spectrum = self.state.spectra.get(role) or {}
        spectrum_data = spectrum.get("data") if isinstance(spectrum.get("data"), dict) else {}
        peak = self._number_or_none(spectrum_data.get("peak_dbfs"))
        noise = self._number_or_none(spectrum_data.get("noise_floor_dbfs"))
        if no_recent_frame and self._role_uptime_unlocked(role, now) >= RX_STARTUP_GRACE_SEC:
            if peak is not None and noise is not None and peak - noise < 6.0:
                return {"state": "warn", "last_seen_age_sec": None, "message": f"{label}未检测到可辨识载波：峰值仅高于噪声 {peak - noise:.1f} dB"}
            if rms is not None and rms < -85.0:
                return {"state": "warn", "last_seen_age_sec": None, "message": f"{label}输入信号过弱（RMS {rms:.1f} dBFS）且无有效帧"}
        return {"state": "ok", "last_seen_age_sec": None, "message": f"{label} ADC 输入范围正常"}

    def _decode_quality_health_unlocked(self, role: str, now: float) -> dict[str, Any]:
        entry = self.state.statuses.get(role) or {}
        data = entry.get("data") if isinstance(entry.get("data"), dict) else {}
        label = "RX1 信息波" if role == "rx1" else "RX2 干扰波"
        pipeline = data.get("reception_pipeline") if isinstance(data.get("reception_pipeline"), dict) else {}
        if not entry or data.get("started") is not True:
            return {"state": "unknown", "last_seen_age_sec": None, "message": f"{label}未运行，解码质量待检测"}
        if not pipeline:
            state = "unknown" if self._role_uptime_unlocked(role, now) < RX_STARTUP_GRACE_SEC else "warn"
            return {"state": state, "last_seen_age_sec": None, "message": f"{label}运行中但尚无解码流水线诊断"}
        suspected = int(pipeline.get("suspected_frames") or 0)
        if pipeline.get("estimate_available") is not True:
            return {"state": "unknown", "last_seen_age_sec": None, "message": f"{label}最近 10 秒样本不足（候选帧 {suspected}）"}
        loss = max(0.0, self._number_or_none(pipeline.get("end_to_end_loss_rate")) or 0.0)
        rejection = max(0.0, self._number_or_none(pipeline.get("crc_rejection_rate")) or 0.0)
        sequence = pipeline.get("candidate_sequence") if isinstance(pipeline.get("candidate_sequence"), dict) else {}
        duplicates = int(sequence.get("duplicate_frames") or 0)
        out_of_order = int(sequence.get("out_of_order_frames") or 0)
        worst_rate = max(loss, rejection)
        state = "bad" if worst_rate > RX_LOSS_BAD_RATE else "warn" if worst_rate > RX_LOSS_WARN_RATE or duplicates + out_of_order >= 3 else "ok"
        details = f"总丢包 {loss * 100:.1f}% · CRC/长度拒绝 {rejection * 100:.1f}%"
        if duplicates or out_of_order:
            details += f" · 重复 {duplicates} / 乱序 {out_of_order}"
        return {"state": state, "last_seen_age_sec": 0.0, "message": f"{label}最近 10 秒：{details}"}

    def _spectrum_health_unlocked(self, role: str, now: float) -> dict[str, Any]:
        label = "RX1 信息波" if role == "rx1" else "RX2 干扰波"
        status = self.state.statuses.get(role) or {}
        data = status.get("data") if isinstance(status.get("data"), dict) else {}
        if not status or data.get("started") is not True:
            return {"state": "unknown", "last_seen_age_sec": None, "message": f"{label}未运行，频谱待检测"}
        entry = self.state.spectra.get(role) or {}
        age = self._age_or_none(entry.get("timestamp"), now)
        if age is None:
            state = "unknown" if self._role_uptime_unlocked(role, now) < RX_STARTUP_GRACE_SEC else "warn"
            return {"state": state, "last_seen_age_sec": None, "message": f"{label}尚未收到频谱数据"}
        state = "bad" if age > 10.0 else "warn" if age > 3.0 else "ok"
        return {"state": state, "last_seen_age_sec": age, "message": f"{label}频谱{'已中断' if state != 'ok' else '更新正常'} {age:.1f} 秒"}

    def _referee_stability_health_unlocked(self, now: float) -> dict[str, Any]:
        entry = self.state.statuses.get("referee") or {}
        data = entry.get("data") if isinstance(entry.get("data"), dict) else {}
        if not entry:
            return {"state": "unknown", "last_seen_age_sec": None, "message": "尚未收到裁判串口状态"}
        reconnects = max(0, int(data.get("serial_open_count") or 0) - 1)
        close_age = self._age_or_none(data.get("serial_last_close_time"), now)
        if reconnects and close_age is not None and close_age <= 30.0:
            state = "bad" if reconnects >= 3 else "warn"
            return {"state": state, "last_seen_age_sec": close_age, "message": f"裁判串口近期发生重连，累计 {reconnects} 次，最近一次关闭在 {close_age:.1f} 秒前"}
        return {"state": "ok", "last_seen_age_sec": close_age, "message": f"裁判串口连接稳定（重连 {reconnects} 次）"}

    def _radar_output_health_unlocked(self) -> dict[str, Any]:
        entry = self.state.statuses.get("radar_integration") or {}
        data = entry.get("data") if isinstance(entry.get("data"), dict) else {}
        if data.get("schema") != "shark.radar.fusion.v1":
            return {"state": "unknown", "last_seen_age_sec": None, "message": "等待雷达融合输出状态"}
        tx = data.get("tx_0305") if isinstance(data.get("tx_0305"), dict) else {}
        if not data.get("send_allowed", False):
            reason = str(data.get("send_block_reason") or "无有效坐标")
            vision = data.get("vision") if isinstance(data.get("vision"), dict) else {}
            radio = data.get("radio") if isinstance(data.get("radio"), dict) else {}
            sources_online = bool(vision.get("online") or radio.get("fresh"))
            state = "bad" if reason == "configured_side_mismatch" or sources_online else "warn"
            return {"state": state, "last_seen_age_sec": None, "message": f"0x0305 坐标发送已暂停：{reason}"}
        ack = tx.get("last_ack") if isinstance(tx.get("last_ack"), dict) else {}
        if ack and ack.get("written") is False and ack.get("dry_run") is not True:
            return {"state": "bad", "last_seen_age_sec": None, "message": f"0x0305 最近一次串口写入失败：{ack.get('error') or '未写入'}"}
        requests = int(tx.get("request_count") or 0)
        successes = int(tx.get("success_count") or 0)
        actual_rate = max(0.0, self._number_or_none(tx.get("actual_rate_hz")) or 0.0)
        configured_rate = max(0.1, self._number_or_none(tx.get("configured_rate_hz")) or 4.8)
        if requests < 2:
            return {"state": "unknown", "last_seen_age_sec": None, "message": "0x0305 已允许发送，等待发送率样本"}
        if successes == 0 or actual_rate <= 0.01:
            return {"state": "bad", "last_seen_age_sec": None, "message": f"0x0305 已请求 {requests} 次，但尚无成功串口写入"}
        if actual_rate < configured_rate * 0.5:
            return {"state": "warn", "last_seen_age_sec": None, "message": f"0x0305 实际发送率 {actual_rate:.2f} Hz，低于配置 {configured_rate:.2f} Hz 的一半"}
        return {"state": "ok", "last_seen_age_sec": None, "message": f"0x0305 发送正常：{actual_rate:.2f} Hz，成功 {successes}/{requests}"}

    def _coordinate_coverage_health_unlocked(self) -> dict[str, Any]:
        entry = self.state.statuses.get("radar_integration") or {}
        data = entry.get("data") if isinstance(entry.get("data"), dict) else {}
        if data.get("schema") != "shark.radar.fusion.v1":
            return {"state": "unknown", "last_seen_age_sec": None, "message": "等待融合坐标覆盖率"}
        robots = data.get("robots") if isinstance(data.get("robots"), dict) else {}
        if not robots:
            return {"state": "unknown", "last_seen_age_sec": None, "message": "融合节点尚未发布机器人坐标"}
        own_prefix = "R" if str(data.get("side") or "red").lower() == "red" else "B"
        valid_total = 0
        valid_opponents = 0
        for name, item in robots.items():
            item = item if isinstance(item, dict) else {}
            final = item.get("final") if isinstance(item.get("final"), dict) else {}
            if final.get("valid") is True:
                valid_total += 1
                if not str(name).startswith(own_prefix):
                    valid_opponents += 1
        vision = data.get("vision") if isinstance(data.get("vision"), dict) else {}
        if valid_total == 0 and vision.get("online") is True:
            return {"state": "bad", "last_seen_age_sec": None, "message": "视觉在线，但所有机器人坐标均无效，无法生成 0x0305"}
        if valid_opponents == 0:
            return {"state": "warn", "last_seen_age_sec": None, "message": f"当前有效坐标 {valid_total} 个，但没有有效对方机器人坐标"}
        if valid_total < 3:
            return {"state": "warn", "last_seen_age_sec": None, "message": f"机器人有效坐标覆盖偏低：{valid_total}/12，对方 {valid_opponents}/6"}
        return {"state": "ok", "last_seen_age_sec": None, "message": f"机器人有效坐标 {valid_total}/12，对方 {valid_opponents}/6"}

    def _vision_performance_health_unlocked(self, now: float) -> dict[str, Any]:
        entry = self.state.statuses.get("vision_radar") or {}
        data = entry.get("data") if isinstance(entry.get("data"), dict) else {}
        vision = data.get("vision") if isinstance(data.get("vision"), dict) else {}
        if data.get("schema") != "shark.radar.telemetry.v1":
            return {"state": "unknown", "last_seen_age_sec": None, "message": "视觉性能待相机就绪后检测"}
        if not vision.get("camera_ready", False):
            return {"state": "bad", "last_seen_age_sec": None, "message": "视觉相机未就绪，无法评估处理性能"}
        fps = max(
            0.0,
            self._number_or_none(vision.get("processing_fps"))
            or self._number_or_none(vision.get("fps"))
            or 0.0,
        )
        camera_fps = self._number_or_none(vision.get("camera_fps"))
        if camera_fps is not None:
            camera_fps = max(0.0, camera_fps)
        inference_ms = max(0.0, self._number_or_none(vision.get("inference_ms")) or 0.0)
        if fps <= 0.0:
            state = "unknown" if self._role_uptime_unlocked("vision_radar", now) < RX_STARTUP_GRACE_SEC else "bad"
            return {"state": state, "last_seen_age_sec": None, "message": "视觉相机已就绪但没有有效帧率样本"}
        rate_summary = (
            f"相机 {camera_fps:.1f} FPS / 处理 {fps:.1f} FPS / {inference_ms:.1f} ms"
            if camera_fps is not None
            else f"处理 {fps:.1f} FPS / {inference_ms:.1f} ms"
        )
        if camera_fps is not None and camera_fps < 5.0:
            return {"state": "bad", "last_seen_age_sec": None, "message": f"相机采集严重偏低：{rate_summary}"}
        if camera_fps is not None and camera_fps < 10.0:
            return {"state": "warn", "last_seen_age_sec": None, "message": f"相机采集性能偏低：{rate_summary}"}
        if fps < 5.0 or inference_ms > 250.0:
            return {"state": "bad", "last_seen_age_sec": None, "message": f"视觉处理严重变慢：{rate_summary}"}
        if fps < 10.0 or inference_ms > 100.0:
            return {"state": "warn", "last_seen_age_sec": None, "message": f"视觉处理性能偏低：{rate_summary}"}
        return {"state": "ok", "last_seen_age_sec": None, "message": f"视觉性能正常：{rate_summary}"}

    def _display_health_unlocked(self, now: float) -> dict[str, Any]:
        metrics = self.state.display_metrics
        age = self._age_or_none(metrics.get("received_at"), now)
        if not metrics or age is None or age > 10.0:
            return {"state": "unknown", "last_seen_age_sec": age, "message": "无线电监控页未活动，前端性能待检测"}
        problems: list[str] = []
        state = "ok"
        stream_state = str(metrics.get("stream_state") or "unknown")
        active_page = str(metrics.get("active_page") or "radio")
        latency = self._number_or_none(metrics.get("latency_ms")) or 0.0
        render_fps = self._number_or_none(metrics.get("render_fps")) or 0.0
        dropped = int(metrics.get("dropped_rows_delta") or 0) if active_page == "radio" else 0
        if stream_state != "open":
            state = "warn"
            problems.append(f"SSE {stream_state}")
        low_render_error = active_page == "radio" and render_fps > 0.0 and render_fps < 10.0
        low_render_warn = active_page == "radio" and render_fps > 0.0 and render_fps < 20.0
        if latency > 1500.0 or low_render_error or dropped >= 10:
            state = "bad"
        elif latency > 500.0 or low_render_warn or dropped > 0:
            state = "warn" if state != "bad" else state
        if latency > 500.0:
            problems.append(f"延迟 {latency:.0f} ms")
        if low_render_warn:
            problems.append(f"渲染 {render_fps:.0f} FPS")
        if dropped > 0:
            problems.append(f"本周期丢弃瀑布行 {dropped}")
        page_label = "雷达页" if active_page == "radar" else "无线电监控页"
        return {"state": state, "last_seen_age_sec": age, "message": f"{page_label}前端显示正常" if not problems else f"{page_label}前端显示异常：" + "；".join(problems)}

    def _health_unlocked(self, *, now: Optional[float] = None) -> dict[str, Any]:
        now = _now() if now is None else float(now)
        modules = {
            "rx1": self._status_health("rx1", now),
            "rx2": self._status_health("rx2", now),
            "referee": self._status_health("referee", now),
            "recorder": self._status_health("recorder", now),
            "radar_integration": self._status_health("radar_integration", now),
            "vision_radar": self._status_health("vision_radar", now),
        }
        rates = self._reception_rates_unlocked(now=now)
        window = float(rates["window_sec"])
        for role, label in (("rx1", "信息波"), ("rx2", "干扰波")):
            rate = float(rates[f"{role}_hz"])
            modules[f"{role}_frame_rate"] = self._frame_rate_health_unlocked(role, label, rate, window, now)
            modules[f"{role}_business_frames"] = self._command_completeness_health_unlocked(role, now)
            modules[f"{role}_tuning"] = self._tuning_health_unlocked(role, now)
            modules[f"{role}_signal"] = self._signal_health_unlocked(role, now)
            modules[f"{role}_decode_quality"] = self._decode_quality_health_unlocked(role, now)
            modules[f"{role}_spectrum"] = self._spectrum_health_unlocked(role, now)
        modules["referee_stability"] = self._referee_stability_health_unlocked(now)
        modules["radar_output"] = self._radar_output_health_unlocked()
        modules["coordinate_coverage"] = self._coordinate_coverage_health_unlocked()
        modules["vision_performance"] = self._vision_performance_health_unlocked(now)
        modules["display_performance"] = self._display_health_unlocked(now)
        for role in ("rx1", "rx2"):
            check = self.state.sdr_checks.get(role, {})
            available = check.get("available")
            if available is False:
                modules[f"{role}_sdr"] = {
                    "state": "bad",
                    "last_seen_age_sec": self._age_or_none(check.get("checked_at"), now),
                    "message": check.get("message") or "SDR 不可达",
                }
            elif available is True:
                modules[f"{role}_sdr"] = {
                    "state": "ok",
                    "last_seen_age_sec": self._age_or_none(check.get("checked_at"), now),
                    "message": check.get("message") or "SDR 可达",
                }
            else:
                modules[f"{role}_sdr"] = {
                    "state": "unknown",
                    "last_seen_age_sec": self._age_or_none(check.get("checked_at"), now),
                    "message": check.get("message") or "尚未检查 SDR",
                }
        rank = {"bad": 3, "warn": 2, "unknown": 1, "ok": 0}
        worst = max((entry["state"] for entry in modules.values()), key=lambda state: rank.get(state, 0))
        return {"state": worst, "modules": modules, "updated_at": now}

    @staticmethod
    def _age_or_none(timestamp: Any, now: float) -> Optional[float]:
        try:
            if not timestamp:
                return None
            return max(0.0, now - float(timestamp))
        except Exception:
            return None

    def _status_health(self, role: str, now: float) -> dict[str, Any]:
        entry = self.state.statuses.get(role) or {}
        timestamp = entry.get("timestamp")
        age = self._age_or_none(timestamp, now)
        data = entry.get("data") if isinstance(entry.get("data"), dict) else {}
        if not entry:
            return {"state": "unknown", "last_seen_age_sec": None, "message": "尚未收到模块状态，等待检测"}
        last_error = str(data.get("last_error") or "").strip()
        if last_error:
            stale_suffix = f"（状态已 {age:.1f} 秒未更新）" if age is not None and age > STATUS_STALE_WARN_SEC else ""
            return {"state": "bad", "last_seen_age_sec": age, "message": last_error + stale_suffix}
        if age is not None and age > STATUS_STALE_BAD_SEC:
            return {"state": "bad", "last_seen_age_sec": age, "message": f"模块状态已中断 {age:.1f} 秒"}
        if age is not None and age > STATUS_STALE_WARN_SEC:
            return {"state": "warn", "last_seen_age_sec": age, "message": f"模块状态已 {age:.1f} 秒未更新"}
        if role == "referee":
            dry_run = bool(data.get("dry_run", False))
            serial_open = bool(data.get("serial_open", False))
            if dry_run:
                return {"state": "warn", "last_seen_age_sec": age, "message": "裁判串口 dry-run"}
            if not serial_open:
                return {"state": "bad", "last_seen_age_sec": age, "message": "裁判串口未打开"}
            return {"state": "ok", "last_seen_age_sec": age, "message": "裁判串口在线"}
        if role == "vision_radar":
            if data.get("schema") != "shark.radar.telemetry.v1":
                return {"state": "bad", "last_seen_age_sec": age, "message": "视觉遥测协议无效"}
            vision = data.get("vision") if isinstance(data.get("vision"), dict) else {}
            if not vision.get("camera_ready", False):
                return {"state": "bad", "last_seen_age_sec": age, "message": "视觉相机未就绪"}
            return {"state": "ok", "last_seen_age_sec": age, "message": "视觉遥测在线"}
        if role == "radar_integration":
            if data.get("schema") != "shark.radar.fusion.v1":
                return {"state": "bad", "last_seen_age_sec": age, "message": "融合状态协议无效"}
            if data.get("side_mismatch"):
                return {"state": "bad", "last_seen_age_sec": age, "message": "阵营配置与裁判不一致"}
            # This module card describes the ROS bridge itself.  A missing radio
            # fusion input is reported separately by fusion.health/radio.fresh;
            # treating that as a bridge failure made a healthy ROS path look bad.
            return {"state": "ok", "last_seen_age_sec": age, "message": "ROS 融合桥接在线"}
        if data.get("started") is True:
            return {"state": "ok", "last_seen_age_sec": age, "message": "节点运行中"}
        if data.get("loaded") is True:
            return {"state": "warn", "last_seen_age_sec": age, "message": "flowgraph 已加载但未启动"}
        return {"state": "bad", "last_seen_age_sec": age, "message": "节点未运行"}

    def _frequency_plan_unlocked(self) -> dict[str, Any]:
        side = self.state.effective_radio_side if self.state.effective_radio_side in BROADCAST_FREQUENCIES else "red"
        level = self.state.effective_interference_level if self.state.effective_interference_level in (1, 2, 3) else 1
        return {
            "broadcast_hz": BROADCAST_FREQUENCIES[side],
            "interference": {
                "level": level,
                **interference_setters_for_side_level(side, level),
            },
        }

    def _update_config(self, data: dict[str, Any]) -> dict[str, Any]:
        if not isinstance(data, dict):
            raise ValueError("请求体必须是 JSON object")
        persist = data.get("persist") is True
        with self._lock:
            mode = str(data.get("mode", "match_rx")).strip()
            if mode != "match_rx":
                raise ValueError("比赛 RX Panel 只支持 match_rx 模式；实验室 TX 请使用 apps/lab_tx")
            self.state.mode = "match_rx"

            manual_rx_dirty = False
            gain_dirty = False
            intelligent_was_enabled = self.state.intelligent_mode
            if "intelligent_mode" in data:
                new_intelligent = bool(data["intelligent_mode"])
                if self.state.intelligent_mode and not new_intelligent:
                    manual_rx_dirty = True
                self.state.intelligent_mode = new_intelligent

            if "radio_side" in data:
                side = str(data["radio_side"]).strip().lower()
                if side not in ("red", "blue"):
                    raise ValueError("radio_side 必须是 red 或 blue")
                if side != self.state.radio_side:
                    manual_rx_dirty = True
                self.state.radio_side = side

            if "interference_level" in data:
                level = int(data["interference_level"])
                if level not in (1, 2, 3):
                    raise ValueError("interference_level 必须是 1/2/3")
                if level != self.state.interference_level:
                    manual_rx_dirty = True
                self.state.interference_level = level

            if "broadcast_gain" in data:
                gain = float(data["broadcast_gain"])
                if not self.state.broadcast_gain_min <= gain <= self.state.broadcast_gain_max:
                    raise ValueError(
                        f"broadcast_gain 必须在 {self.state.broadcast_gain_min:g}"
                        f"..{self.state.broadcast_gain_max:g} dB"
                    )
                if gain != self.state.broadcast_gain:
                    gain_dirty = True
                self.state.broadcast_gain = gain

            sdr_uris = data.get("sdr_uris")
            if isinstance(sdr_uris, dict):
                for role in ("rx1", "rx2"):
                    if role in sdr_uris:
                        self.state.sdr_uris[role] = str(sdr_uris[role]).strip()

            referee = data.get("referee")
            if isinstance(referee, dict):
                merged_referee = dict(self.state.referee)
                for key in ("port", "bridge_topic", "radar_cmd_topic"):
                    if key in referee:
                        merged_referee[key] = str(referee[key]).strip()
                for key in ("baudrate", "sender_id", "receiver_id", "invincible_targets_data_cmd_id"):
                    if key in referee:
                        merged_referee[key] = int(referee[key])
                if "frame_timeout_sec" in referee:
                    merged_referee["frame_timeout_sec"] = max(0.0, float(referee["frame_timeout_sec"]))
                for key in ("invincible_targets_send_rate_hz", "invincible_targets_freshness_sec"):
                    if key in referee:
                        merged_referee[key] = max(0.1, float(referee[key]))
                if "dry_run" in referee:
                    merged_referee["dry_run"] = bool(referee["dry_run"])
                if "require_serial_open_on_start" in referee:
                    merged_referee["require_serial_open_on_start"] = bool(
                        referee["require_serial_open_on_start"]
                    )
                if "auto_send_invincible_targets" in referee:
                    merged_referee["auto_send_invincible_targets"] = bool(
                        referee["auto_send_invincible_targets"]
                    )
                self.state.referee = merged_referee

            forbidden = sorted({"tx_type", "tx_enabled", "tx_config"} & set(data))
            if forbidden:
                raise ValueError("比赛 RX Panel 不接受 TX 配置：" + ", ".join(forbidden))

            if persist:
                self._persist_config_unlocked()

            if manual_rx_dirty and not self.state.intelligent_mode:
                self._publish_manual_rx_setters_unlocked()
                gain_dirty = False
            elif not intelligent_was_enabled and self.state.intelligent_mode:
                self._sync_effective_from_referee_unlocked()
            if gain_dirty:
                self._publish_broadcast_gain_unlocked()

        if persist:
            self._event("config", f"比赛接收配置已保存到 {self.config_path}")
        else:
            self._event("config", "比赛接收配置已更新")
        return self._snapshot()

    def _persist_config_unlocked(self) -> None:
        config_path = getattr(self, "config_path", None)
        if not isinstance(config_path, Path):
            config_path = self._default_config_path()
            self.config_path = config_path
        _update_simple_yaml_scalars(config_path, {
            "radio.intelligent_mode": self.state.intelligent_mode,
            "radio.side": self.state.radio_side,
            "radio.interference_level": self.state.interference_level,
            "radio.rx1_uri": self.state.sdr_uris["rx1"],
            "radio.rx2_uri": self.state.sdr_uris["rx2"],
            "radio.broadcast_gain": self.state.broadcast_gain,
            "referee.port": self.state.referee["port"],
            "referee.baudrate": self.state.referee["baudrate"],
        })

    def _publish_manual_rx_setters_unlocked(self) -> dict[str, Any]:
        """手动模式：按当前 side/level 计算两路 RX 接收参数并热下发到各自 setters_json。"""
        side = self.state.radio_side if self.state.radio_side in ("red", "blue") else "red"
        level = int(self.state.interference_level) if int(self.state.interference_level) in (1, 2, 3) else 1
        self.state.effective_radio_side = side
        self.state.effective_interference_level = level
        broadcast_setters = broadcast_rx_setters_for_side(side, self.state.broadcast_gain)
        interference_setters = interference_rx_setters_for_side_level(side, level)
        bmsg = String(); bmsg.data = json.dumps(broadcast_setters, ensure_ascii=False)
        imsg = String(); imsg.data = json.dumps(interference_setters, ensure_ascii=False)
        self.broadcast_rx_setters_pub.publish(bmsg)
        self.interference_rx_setters_pub.publish(imsg)
        self._event("rx_manual", f"手动下发 RX 参数：{side} L{level}")
        return {"side": side, "level": level, "broadcast": broadcast_setters, "interference": interference_setters}

    def _publish_effective_rx_setters_unlocked(self) -> dict[str, Any]:
        """智能模式：将裁判识别的阵营和干扰等级同时下发到两路 RX。"""
        side = self.state.effective_radio_side
        if side not in ("red", "blue"):
            side = self.state.radio_side if self.state.radio_side in ("red", "blue") else "red"
        level = int(self.state.effective_interference_level)
        if level not in (1, 2, 3):
            level = int(self.state.interference_level)
        if level not in (1, 2, 3):
            level = 1
        broadcast_setters = broadcast_rx_setters_for_side(side, self.state.broadcast_gain)
        interference_setters = interference_rx_setters_for_side_level(side, level)
        bmsg = String(); bmsg.data = json.dumps(broadcast_setters, ensure_ascii=False)
        imsg = String(); imsg.data = json.dumps(interference_setters, ensure_ascii=False)
        self.broadcast_rx_setters_pub.publish(bmsg)
        self.interference_rx_setters_pub.publish(imsg)
        self._event("rx_intelligent", f"裁判自动下发 RX 参数：{side} L{level}")
        return {"side": side, "level": level, "broadcast": broadcast_setters, "interference": interference_setters}

    def _publish_broadcast_gain_unlocked(self) -> dict[str, Any]:
        side = (
            self.state.effective_radio_side
            if self.state.intelligent_mode
            else self.state.radio_side
        )
        if side not in ("red", "blue"):
            side = "red"
        setters = broadcast_rx_setters_for_side(side, self.state.broadcast_gain)
        msg = String()
        msg.data = json.dumps(setters, ensure_ascii=False)
        self.broadcast_rx_setters_pub.publish(msg)
        self._event(
            "rx1_gain",
            f"下发 RX1 manual gain={self.state.broadcast_gain:g} dB"
            f"（范围 {self.state.broadcast_gain_min:g}..{self.state.broadcast_gain_max:g} dB）",
        )
        return setters

    def _stream_snapshot(
        self,
        *,
        after_sequence: int = 0,
        spectrum_after: Optional[dict[str, float]] = None,
        frame_limit: int = 24,
        spectrum_tap: str = "raw",
    ) -> dict[str, Any]:
        spectrum_after = spectrum_after or {}
        spectrum_tap = "filtered" if str(spectrum_tap).strip().lower() == "filtered" else "raw"
        with self._lock:
            spectrum_source = (
                self.state.filtered_spectra
                if spectrum_tap == "filtered"
                else self.state.spectra
            )
            spectra = {
                role: item
                for role, item in spectrum_source.items()
                if float(item.get("timestamp", 0.0) or 0.0)
                > float(spectrum_after.get(role, 0.0) or 0.0)
            }
            frames = [
                item
                for item in self.state.frames
                if item.get("role") in {"rx1", "rx2"}
                and int(item.get("seq", 0) or 0) > int(after_sequence)
            ][: max(1, min(int(frame_limit), 64))]
            return {
                "ok": True,
                "spectra": spectra,
                "frames": frames,
                "frame_sequence": int(getattr(self, "_frame_sequence", 0)),
                "generation": int(getattr(self, "_stream_generation", 0)),
                "spectrum_tap": spectrum_tap,
                "now": _now(),
            }

    def _wait_stream_snapshot(
        self,
        *,
        after_generation: int,
        after_sequence: int,
        spectrum_after: dict[str, float],
        spectrum_tap: str = "raw",
        timeout: float = 15.0,
    ) -> dict[str, Any]:
        with self._stream_condition:
            self._stream_condition.wait_for(
                lambda: int(getattr(self, "_stream_generation", 0)) > int(after_generation),
                timeout=max(0.01, float(timeout)),
            )
            return self._stream_snapshot(
                after_sequence=after_sequence,
                spectrum_after=spectrum_after,
                spectrum_tap=spectrum_tap,
            )

    def _send_spectrum_stream(
        self,
        handler: BaseHTTPRequestHandler,
        query: dict[str, list[str]],
    ) -> None:
        after_sequence = int(self._query_number(query, "after_seq", 0.0))
        spectrum_after = {
            "rx1": self._query_number(query, "rx1_after", 0.0),
            "rx2": self._query_number(query, "rx2_after", 0.0),
        }
        spectrum_tap = "filtered" if (query.get("tap") or ["raw"])[0].strip().lower() == "filtered" else "raw"
        last_generation = int(self._query_number(query, "generation", 0.0))
        handler.send_response(HTTPStatus.OK)
        handler.send_header("Content-Type", "text/event-stream; charset=utf-8")
        handler.send_header("Cache-Control", "no-cache, no-store")
        handler.send_header("Connection", "keep-alive")
        handler.send_header("X-Accel-Buffering", "no")
        handler.end_headers()
        handler.wfile.write(b"retry: 1000\n\n")
        handler.wfile.flush()

        try:
            # Send the current latest-only state immediately, then block on a
            # condition variable. No polling thread and no response backlog.
            payload = self._stream_snapshot(
                after_sequence=after_sequence,
                spectrum_after=spectrum_after,
                spectrum_tap=spectrum_tap,
            )
            while True:
                generation = int(payload.get("generation", last_generation))
                changed = bool(payload.get("spectra") or payload.get("frames"))
                if changed:
                    body = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
                    handler.wfile.write(f"data: {body}\n\n".encode("utf-8"))
                    handler.wfile.flush()
                    after_sequence = max(after_sequence, int(payload.get("frame_sequence", 0)))
                    for role, item in payload.get("spectra", {}).items():
                        spectrum_after[role] = max(
                            float(spectrum_after.get(role, 0.0)),
                            float(item.get("timestamp", 0.0) or 0.0),
                        )
                elif generation <= last_generation:
                    handler.wfile.write(b": keepalive\n\n")
                    handler.wfile.flush()
                last_generation = max(last_generation, generation)
                payload = self._wait_stream_snapshot(
                    after_generation=last_generation,
                    after_sequence=after_sequence,
                    spectrum_after=spectrum_after,
                    spectrum_tap=spectrum_tap,
                    timeout=15.0,
                )
        except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
            return

    def _radar_stream_snapshot(self) -> dict[str, Any]:
        with self._lock:
            now = _now()
            return {
                "ok": True,
                "generation": int(getattr(self, "_radar_generation", 0)),
                "now": now,
                "radar": self.state.radar,
                "vision_radar": self.state.vision_radar,
                "reception_rates": self._reception_rates_unlocked(now=now),
                "health": self._health_unlocked(now=now),
            }

    def _wait_radar_stream_snapshot(self, after_generation: int, timeout: float = 15.0) -> dict[str, Any]:
        with self._stream_condition:
            self._stream_condition.wait_for(
                lambda: int(getattr(self, "_radar_generation", 0)) > int(after_generation),
                timeout=max(0.01, float(timeout)),
            )
            return self._radar_stream_snapshot()

    def _send_radar_stream(
        self,
        handler: BaseHTTPRequestHandler,
        query: dict[str, list[str]],
    ) -> None:
        last_generation = int(self._query_number(query, "generation", 0.0))
        handler.send_response(HTTPStatus.OK)
        handler.send_header("Content-Type", "text/event-stream; charset=utf-8")
        handler.send_header("Cache-Control", "no-cache, no-store")
        handler.send_header("Connection", "keep-alive")
        handler.send_header("X-Accel-Buffering", "no")
        handler.end_headers()
        handler.wfile.write(b"retry: 1000\n\n")
        handler.wfile.flush()
        try:
            payload = self._radar_stream_snapshot()
            while True:
                generation = int(payload.get("generation", last_generation))
                if generation > last_generation or last_generation == 0:
                    body = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
                    handler.wfile.write(f"data: {body}\n\n".encode("utf-8"))
                    handler.wfile.flush()
                else:
                    handler.wfile.write(b": keepalive\n\n")
                    handler.wfile.flush()
                last_generation = max(last_generation, generation)
                payload = self._wait_radar_stream_snapshot(last_generation, timeout=15.0)
        except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
            return

    def _update_client_metrics(self, data: dict[str, Any]) -> dict[str, Any]:
        numeric_fields = (
            "render_fps",
            "rx1_hz",
            "rx2_hz",
            "latency_ms",
            "transport_ms",
            "dropped_waterfall_rows",
        )
        metrics: dict[str, Any] = {}
        for key in numeric_fields:
            try:
                value = float(data.get(key, 0.0))
            except (TypeError, ValueError):
                value = 0.0
            metrics[key] = min(max(value, 0.0), 100_000.0)
        metrics["stream_state"] = str(data.get("stream_state", "unknown"))[:32]
        metrics["active_page"] = "radar" if str(data.get("active_page")) == "radar" else "radio"
        metrics["received_at"] = _now()
        with self._lock:
            previous_dropped = int(self.state.display_metrics.get("dropped_waterfall_rows", 0) or 0)
            current_dropped = int(metrics.get("dropped_waterfall_rows", 0) or 0)
            metrics["dropped_rows_delta"] = (
                current_dropped - previous_dropped
                if current_dropped >= previous_dropped
                else current_dropped
            )
            self.state.display_metrics = metrics
        return {"ok": True, "metrics": metrics}

    @staticmethod
    def _query_number(query: dict[str, list[str]], name: str, default: float = 0.0) -> float:
        try:
            return float((query.get(name) or [default])[0])
        except (TypeError, ValueError):
            return float(default)

    def _handle_get(self, handler: BaseHTTPRequestHandler) -> None:
        parsed = urlparse(handler.path)
        if parsed.path == "/api/state":
            self._send_json(handler, self._snapshot())
            return
        if parsed.path == "/api/spectrum":
            query = parse_qs(parsed.query)
            self._send_json(
                handler,
                self._stream_snapshot(
                    after_sequence=int(self._query_number(query, "after_seq", 0.0)),
                    spectrum_after={
                        "rx1": self._query_number(query, "rx1_after", 0.0),
                        "rx2": self._query_number(query, "rx2_after", 0.0),
                    },
                    spectrum_tap=(query.get("tap") or ["raw"])[0],
                ),
            )
            return
        if parsed.path == "/api/spectrum-stream":
            self._send_spectrum_stream(handler, parse_qs(parsed.query))
            return
        if parsed.path == "/api/radar-stream":
            self._send_radar_stream(handler, parse_qs(parsed.query))
            return
        if parsed.path == "/api/performance":
            with self._lock:
                metrics = dict(self.state.display_metrics)
            self._send_json(
                handler,
                {
                    "ok": True,
                    "metrics": metrics,
                    "generation": int(getattr(self, "_stream_generation", 0)),
                    "now": _now(),
                },
            )
            return
        if parsed.path == "/api/check_sdr":
            self._check_sdrs()
            self._send_json(handler, self._snapshot().get("sdr_checks", {}))
            return
        path = parsed.path if parsed.path != "/" else "/index.html"
        self._send_static(handler, path)

    def _handle_post(self, handler: BaseHTTPRequestHandler) -> None:
        parsed = urlparse(handler.path)
        try:
            data = self._read_json_body(handler)
            if parsed.path == "/api/client-metrics":
                self._send_json(handler, self._update_client_metrics(data))
                return
            if parsed.path == "/api/config":
                self._send_json(handler, self._update_config(data))
                return
            if parsed.path == "/api/start":
                self._update_config(data)
                name = str(data.get("name", "")).strip()
                self._send_json(handler, {"ok": True, "state": self._start_process(name)})
                return
            if parsed.path == "/api/stop":
                name = str(data.get("name", "")).strip()
                self._send_json(handler, {"ok": True, "state": self._stop_process(name)})
                return
            if parsed.path == "/api/clear":
                with self._lock:
                    self.state.frames.clear()
                    self.state.bridge_outputs.clear()
                    self.state.spectra.clear()
                    self.state.filtered_spectra.clear()
                    self.state.events.clear()
                self._send_json(handler, {"ok": True})
                return
            self._send_json(handler, {"ok": False, "error": "unknown endpoint"}, HTTPStatus.NOT_FOUND)
        except Exception as exc:
            self._event("error", str(exc))
            self._send_json(handler, {"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)

    @staticmethod
    def _read_json_body(handler: BaseHTTPRequestHandler) -> dict[str, Any]:
        length = int(handler.headers.get("Content-Length", "0") or "0")
        raw = handler.rfile.read(length).decode("utf-8") if length else "{}"
        data = json.loads(raw or "{}")
        if not isinstance(data, dict):
            raise ValueError("JSON body must be an object")
        return data

    @staticmethod
    def _send_json(handler: BaseHTTPRequestHandler, data: Any, status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(data, ensure_ascii=False).encode("utf-8")
        handler.send_response(int(status))
        handler.send_header("Content-Type", "application/json; charset=utf-8")
        handler.send_header("Cache-Control", "no-store")
        handler.send_header("Content-Length", str(len(body)))
        handler.end_headers()
        handler.wfile.write(body)

    def _send_static(self, handler: BaseHTTPRequestHandler, url_path: str) -> None:
        clean = url_path.lstrip("/")
        clean_path = Path(clean)
        if not clean or clean_path.is_absolute() or ".." in clean_path.parts:
            clean = "index.html"
            clean_path = Path(clean)
        root = self.web_root.resolve()
        target_path = self.web_root / clean_path
        target = target_path.resolve()
        try:
            target.relative_to(root)
        except ValueError:
            try:
                target_path.relative_to(self.web_root)
            except ValueError:
                self._send_json(handler, {"ok": False, "error": "invalid path"}, HTTPStatus.BAD_REQUEST)
                return
        if target_path.is_dir():
            target_path = target_path / "index.html"
            target = target_path.resolve()
        if not target_path.exists() or not target_path.is_file():
            self._send_json(handler, {"ok": False, "error": "invalid path"}, HTTPStatus.BAD_REQUEST)
            return
        content_type = {
            ".html": "text/html; charset=utf-8",
            ".css": "text/css; charset=utf-8",
            ".js": "application/javascript; charset=utf-8",
            ".json": "application/json; charset=utf-8",
            ".jpg": "image/jpeg",
            ".jpeg": "image/jpeg",
            ".png": "image/png",
        }.get(target_path.suffix, "application/octet-stream")
        body = target_path.read_bytes()
        handler.send_response(HTTPStatus.OK)
        handler.send_header("Content-Type", content_type)
        handler.send_header("Cache-Control", "no-store")
        handler.send_header("Content-Length", str(len(body)))
        handler.end_headers()
        handler.wfile.write(body)

    def destroy_node(self) -> bool:
        self._stop_all_processes()
        for log_file in list(self._process_logs.values()):
            try:
                log_file.close()
            except Exception:
                pass
        try:
            self._server.shutdown()
            self._server.server_close()
        except Exception:
            pass
        return super().destroy_node()


def main(argv: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="RoboMaster radio local web dashboard")
    parser.add_argument("--host", default=os.environ.get("RM_RADIO_DASHBOARD_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("RM_RADIO_DASHBOARD_PORT", "8765")))
    parser.add_argument(
        "--web-root",
        default=os.environ.get("RM_RADIO_MATCH_WEB_ROOT", os.environ.get("RM_RADIO_DASHBOARD_WEB_ROOT", "")),
    )
    args, _ros_args = parser.parse_known_args(argv)

    web_root = Path(args.web_root).expanduser().resolve() if args.web_root else None
    rclpy.init()
    node: Optional[RadioDashboardNode] = None
    try:
        node = RadioDashboardNode(host=args.host, port=args.port, web_root=web_root)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
