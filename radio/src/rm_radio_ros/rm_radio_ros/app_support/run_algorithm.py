#!/usr/bin/env python3
import argparse
import json
import os
import runpy
import struct
import sys
import threading
import time
from pathlib import Path


def _truthy(value: str) -> bool:
    return value.strip().lower() not in {"0", "false", "no", "off"}


def _float_env(name: str, default: float) -> float:
    raw = os.environ.get(name, str(default)).strip()
    try:
        return float(raw)
    except ValueError:
        return float(default)


def _topic_list(raw: str) -> list[str]:
    return [topic.strip() for topic in str(raw).replace(";", ",").split(",") if topic.strip()]


def _u16(value) -> int:
    try:
        clean = int(value)
    except (TypeError, ValueError):
        clean = 0
    return max(0, min(clean, 0xFFFF))


def radar_info_to_0305_payload(message: dict) -> bytes | None:
    if not isinstance(message, dict) or message.get("type") != "RadarInfoToClient":
        return None
    payload = message.get("payload")
    if not isinstance(payload, dict):
        return None
    robots = payload.get("RadarSingleRobotInfo")
    if not isinstance(robots, list):
        return None

    values: list[int] = []
    has_nonzero_position = False
    for index in range(12):
        robot = robots[index] if index < len(robots) and isinstance(robots[index], dict) else {}
        x = _u16(robot.get("target_pos_x", 0))
        y = _u16(robot.get("target_pos_y", 0))
        has_nonzero_position = has_nonzero_position or x != 0 or y != 0
        values.extend([x, y])

    if not has_nonzero_position:
        return None
    return struct.pack("<24H", *values)


def is_valid_0305_payload(payload: bytes) -> bool:
    if len(payload) != 48:
        return False
    return any(payload)


def _patch_algorithm_protocol_structures(serial_protocol, referee_comm=None) -> None:
    import ctypes

    class DartStatData(ctypes.LittleEndianStructure):
        _pack_ = 1
        _fields_ = [
            ("dart_remaining_time", ctypes.c_uint8),
            ("recent_hit_target", ctypes.c_uint16, 3),
            ("accumulated_hit_count", ctypes.c_uint16, 3),
            ("selected_target", ctypes.c_uint16, 3),
            ("reserve", ctypes.c_uint16, 7),
        ]

    class RadarInfoData(ctypes.LittleEndianStructure):
        _pack_ = 1
        _fields_ = [
            ("double_vulnerability_count", ctypes.c_uint8, 2),
            ("is_double_vulnerability", ctypes.c_uint8, 1),
            ("own_encryption_level", ctypes.c_uint8, 2),
            ("can_change_password", ctypes.c_uint8, 1),
            ("reserve", ctypes.c_uint8, 2),
        ]

    serial_protocol.DartStatData = DartStatData
    serial_protocol.RadarInfoData = RadarInfoData
    serial_protocol.DartStatusMessage.STRUCT_CLASS = DartStatData
    serial_protocol.RadarInfoMessage.STRUCT_CLASS = RadarInfoData

    if referee_comm is not None:
        referee_comm.DartStatData = DartStatData
        referee_comm.RadarInfoData = RadarInfoData
        referee_comm.DartStatusMessage.STRUCT_CLASS = DartStatData
        referee_comm.RadarInfoMessage.STRUCT_CLASS = RadarInfoData


def _patch_referee_serial() -> None:
    if not _truthy(os.environ.get("RM_ALGO_DISABLE_REFEREE_SERIAL", "1")):
        return

    from driver.referee import serial_comm
    from driver.referee import referee_comm
    from driver.referee import serial_protocol
    from driver.referee.referee_comm import FACTION
    from std_msgs.msg import String

    _patch_algorithm_protocol_structures(serial_protocol, referee_comm)

    serial_state = serial_comm.SerialState
    manager_cls = serial_comm.RefereeSerialManager
    tx_topic = os.environ.get("RM_ALGO_REFEREE_TX_TOPIC", "/rm_referee_serial_node/send_frame")
    rx_topic = os.environ.get("RM_ALGO_REFEREE_RX_TOPIC", "/rm_referee_serial_node/rx_frames")
    radio_bridge_topics = _topic_list(
        os.environ.get(
            "RM_ALGO_RADIO_BRIDGE_TOPIC",
            os.environ.get("REFEREE_BRIDGE_TOPIC", "/rm_gfsk_node/referee_bridge"),
        )
    )
    bridge_enabled = _truthy(os.environ.get("RM_ALGO_REFEREE_BRIDGE", "1"))
    forward_radar_to_sentry = _truthy(os.environ.get("RM_ALGO_FORWARD_RADAR_TO_SENTRY", "0"))
    forward_unknown_faction = _truthy(os.environ.get("RM_ALGO_FORWARD_WHEN_FACTION_UNKNOWN", "0"))
    radar_client_rate_hz = max(_float_env("RM_ALGO_RADAR_CLIENT_RATE_HZ", 0.0), 0.0)
    radar_client_max_age_sec = max(_float_env("RM_ALGO_RADAR_CLIENT_MAX_AGE_SEC", 0.45), 0.0)
    decoded_radar_client_max_age_sec = max(_float_env("RM_ALGO_DECODED_RADAR_CLIENT_MAX_AGE_SEC", 1.0), 0.0)

    def _initial_faction():
        sender_id = os.environ.get("RADAR_SENDER_ID", "").strip()
        if sender_id:
            try:
                return FACTION.BLUE if int(sender_id, 0) >= 100 else FACTION.RED
            except ValueError:
                pass
        side = os.environ.get("RM_RADIO_SIDE", "").strip().lower()
        if side == "blue":
            return FACTION.BLUE
        if side == "red":
            return FACTION.RED
        return FACTION.UNKONWN

    def _ensure_bridge(self):
        if not bridge_enabled:
            return False
        if getattr(self, "_rm_radio_bridge_ready", False):
            return True

        import rclpy
        from rclpy.executors import MultiThreadedExecutor

        self._rm_radio_tx_pub = self.create_publisher(String, tx_topic, 10)
        self._rm_radio_rx_sub = self.create_subscription(
            String, rx_topic, lambda msg: _handle_rx_frame(self, msg), 50
        )
        self._rm_radio_bridge_subs = [
            self.create_subscription(String, topic, lambda msg: _handle_radio_bridge_message(self, msg), 50)
            for topic in radio_bridge_topics
        ]
        self._rm_radio_executor = MultiThreadedExecutor(num_threads=1)
        self._rm_radio_executor.add_node(self)
        self._rm_radio_executor_thread = threading.Thread(
            target=self._rm_radio_executor.spin,
            name="rm-radio-referee-bridge",
            daemon=True,
        )
        self._rm_radio_executor_thread.start()
        self._rm_radio_bridge_ready = True
        self._rm_radio_last_rx_time = 0.0
        self._rm_radio_drop_counts = {}
        self._rm_radio_latest_0305_payload = None
        self._rm_radio_latest_0305_time = 0.0
        self._rm_radio_latest_decoded_0305_payload = None
        self._rm_radio_latest_decoded_0305_time = 0.0
        self._rm_radio_latest_decoded_0305_summary = {}
        self._rm_radio_0305_lock = threading.Lock()
        self._rm_radio_0305_stop_event = threading.Event()
        if radar_client_rate_hz > 0.0:
            self._rm_radio_0305_thread = threading.Thread(
                target=lambda: _radar_client_tx_loop(self),
                name="rm-radio-0305-5hz",
                daemon=True,
            )
            self._rm_radio_0305_thread.start()
        self.get_logger().warning(
            "Algorithm physical referee serial is disabled; forwarding referee TX/RX through "
            f"rm_radio_ros topics tx={tx_topic}, rx={rx_topic}. "
            f"0x0305 scheduler={radar_client_rate_hz:.2f}Hz "
            f"decoded_priority_topics={radio_bridge_topics} "
            f"decoded_max_age={decoded_radar_client_max_age_sec:.2f}s "
            f"algorithm_max_age={radar_client_max_age_sec:.2f}s."
        )
        return rclpy.ok()

    def _handle_rx_frame(self, msg):
        try:
            payload = json.loads(msg.data)
            cmd_id = int(payload["cmd_id"], 0) if isinstance(payload["cmd_id"], str) else int(payload["cmd_id"])
            data = bytes.fromhex(str(payload.get("data_hex", "")))
        except Exception as exc:  # noqa: BLE001 - runtime bridge input
            self.get_logger().warning(f"Invalid rm_radio_ros rx frame ignored: {exc}")
            return

        self._rm_radio_last_rx_time = time.monotonic()
        key = f"0x{cmd_id:04x}"
        for cb_func in list(getattr(self, "cb_funcs", {}).get(key, [])):
            try:
                cb_func(cmd_id, data)
            except Exception as exc:  # noqa: BLE001 - keep other callbacks alive
                self.get_logger().warning(f"Algorithm referee callback failed for {key}: {exc}")

    def _handle_radio_bridge_message(self, msg):
        try:
            bridge_payload = json.loads(msg.data)
            payload = radar_info_to_0305_payload(bridge_payload)
            if payload is None:
                return
            robots = bridge_payload.get("payload", {}).get("RadarSingleRobotInfo", [])
            nonzero_count = sum(
                1
                for robot in robots[:12]
                if isinstance(robot, dict)
                and (_u16(robot.get("target_pos_x", 0)) != 0 or _u16(robot.get("target_pos_y", 0)) != 0)
            )
        except Exception as exc:  # noqa: BLE001 - runtime bridge input
            self.get_logger().warning(f"Invalid decoded radar bridge payload ignored: {exc}")
            return

        lock = getattr(self, "_rm_radio_0305_lock", None)
        if lock is None:
            return
        with lock:
            self._rm_radio_latest_decoded_0305_payload = payload
            self._rm_radio_latest_decoded_0305_time = time.monotonic()
            self._rm_radio_latest_decoded_0305_summary = {
                "timestamp": bridge_payload.get("timestamp"),
                "nonzero_robot_count": nonzero_count,
            }

    def _decode_frame(data):
        if not isinstance(data, bytes) or len(data) < 9 or data[0] != 0xA5:
            raise ValueError("not a RoboMaster referee frame")
        data_length = struct.unpack("<H", data[1:3])[0]
        frame_length = 5 + 2 + data_length + 2
        if len(data) < frame_length:
            raise ValueError(f"incomplete referee frame: got {len(data)}, need {frame_length}")
        cmd_id = struct.unpack("<H", data[5:7])[0]
        payload = data[7 : 7 + data_length]
        return cmd_id, payload

    def _is_unknown_faction(self):
        faction = getattr(self, "faction", None)
        return getattr(faction, "name", "") == "UNKONWN"

    def _record_drop(self, reason):
        counts = getattr(self, "_rm_radio_drop_counts", {})
        count = counts.get(reason, 0) + 1
        counts[reason] = count
        self._rm_radio_drop_counts = counts
        if count in (1, 10) or count % 100 == 0:
            self.get_logger().warning(f"Algorithm referee frame not forwarded: {reason} (count={count})")

    def _publish_tx_payload(self, cmd_id, payload, source="RM2025-Radar-Algorithm", **extra):
        msg = String()
        data = {
            "cmd_id": cmd_id,
            "data_hex": payload.hex(),
            "source": source,
        }
        data.update(extra)
        msg.data = json.dumps(data, ensure_ascii=False)
        self._rm_radio_tx_pub.publish(msg)

    def _cache_radar_client_payload(self, payload):
        lock = getattr(self, "_rm_radio_0305_lock", None)
        if lock is None:
            return False
        with lock:
            self._rm_radio_latest_0305_payload = bytes(payload)
            self._rm_radio_latest_0305_time = time.monotonic()
        return True

    def _radar_client_tx_loop(self):
        interval = 1.0 / radar_client_rate_hz
        next_tx = time.monotonic()
        stop_event = getattr(self, "_rm_radio_0305_stop_event", None)
        while stop_event is not None and not stop_event.is_set():
            next_tx += interval
            wait_s = next_tx - time.monotonic()
            if wait_s > 0 and stop_event.wait(wait_s):
                return
            if wait_s < -interval:
                next_tx = time.monotonic()

            lock = getattr(self, "_rm_radio_0305_lock", None)
            if lock is None:
                continue
            now = time.monotonic()
            with lock:
                decoded_payload = getattr(self, "_rm_radio_latest_decoded_0305_payload", None)
                decoded_time = float(getattr(self, "_rm_radio_latest_decoded_0305_time", 0.0))
                decoded_summary = dict(getattr(self, "_rm_radio_latest_decoded_0305_summary", {}) or {})
                algorithm_payload = getattr(self, "_rm_radio_latest_0305_payload", None)
                algorithm_time = float(getattr(self, "_rm_radio_latest_0305_time", 0.0))

            source = None
            payload = None
            payload_time = 0.0
            extra = {}
            decoded_age = now - decoded_time if decoded_time > 0.0 else None
            algorithm_age = now - algorithm_time if algorithm_time > 0.0 else None
            if (
                decoded_payload is not None
                and decoded_time > 0.0
                and (decoded_radar_client_max_age_sec <= 0.0 or decoded_age <= decoded_radar_client_max_age_sec)
            ):
                source = "rm_radio_ros:decoded_0x0A01_priority"
                payload = decoded_payload
                payload_time = decoded_time
                extra["decoded_summary"] = decoded_summary
                if algorithm_age is not None:
                    extra["algorithm_payload_age_sec"] = round(algorithm_age, 3)
            elif algorithm_payload is not None and algorithm_time > 0.0:
                source = "RM2025-Radar-Algorithm:5Hz_scheduler"
                payload = algorithm_payload
                payload_time = algorithm_time
                if decoded_age is not None:
                    extra["decoded_payload_age_sec"] = round(decoded_age, 3)

            if payload is None or payload_time <= 0.0:
                continue
            age = now - payload_time
            max_age = decoded_radar_client_max_age_sec if source == "rm_radio_ros:decoded_0x0A01_priority" else radar_client_max_age_sec
            if max_age > 0.0 and age > max_age:
                _record_drop(self, f"0x0305 latest payload stale ({age:.3f}s old)")
                continue
            should_forward, reason = _should_forward(self, 0x0305, payload)
            if not should_forward:
                _record_drop(self, reason)
                continue
            _publish_tx_payload(
                self,
                0x0305,
                payload,
                source=source,
                scheduled=True,
                payload_age_sec=round(age, 3),
                **extra,
            )

    def _should_forward(self, cmd_id, payload):
        if cmd_id == 0x0301 and len(payload) >= 6:
            data_cmd_id = struct.unpack("<H", payload[0:2])[0]
            if data_cmd_id == 0x0121:
                return False, "0x0121 is sent by /rm_radar_algorithm/radar_cmd via rm_referee_serial_node"
            if data_cmd_id == 0x0233 and not forward_radar_to_sentry:
                return False, "0x0233 radar-to-sentry forwarding is disabled"
            if data_cmd_id == 0x0233 and _is_unknown_faction(self) and not forward_unknown_faction:
                return False, "0x0233 radar-to-sentry frame blocked until robot faction is known"
        if cmd_id == 0x0305 and _is_unknown_faction(self) and not forward_unknown_faction:
            return False, "0x0305 client map frame blocked until robot faction is known"
        if cmd_id == 0x0305 and not is_valid_0305_payload(payload):
            return False, "0x0305 client map frame blocked because payload is empty or malformed"
        return True, ""

    def start_noop(self):
        if getattr(self, "rx_task", None) and self.rx_task.is_alive():
            print("Referee serial manager is already running")
            return False
        self.rx_task_stop_event = threading.Event()
        self.rx_task = None
        self.state = serial_state.CLOSED
        if _is_unknown_faction(self):
            self.faction = _initial_faction()
            self.get_logger().warning(
                f"Algorithm faction initialized from field config: {getattr(self.faction, 'name', self.faction)}"
            )
        _ensure_bridge(self)
        return True

    def close_noop(self):
        if getattr(self, "rx_task_stop_event", None):
            self.rx_task_stop_event.set()
        stop_event = getattr(self, "_rm_radio_0305_stop_event", None)
        if stop_event is not None:
            stop_event.set()
        radar_client_thread = getattr(self, "_rm_radio_0305_thread", None)
        if radar_client_thread is not None and radar_client_thread.is_alive():
            radar_client_thread.join(timeout=1.0)
        executor = getattr(self, "_rm_radio_executor", None)
        if executor is not None:
            try:
                executor.shutdown()
            except Exception:
                pass
        thread = getattr(self, "_rm_radio_executor_thread", None)
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        try:
            self.close_log_file()
        except Exception:
            pass
        self.state = serial_state.CLOSED
        print("Algorithm physical referee serial disabled; no serial port closed")

    def tx_noop(self, data):
        if not _ensure_bridge(self):
            return False
        try:
            cmd_id, payload = _decode_frame(data)
            should_forward, reason = _should_forward(self, cmd_id, payload)
            if not should_forward:
                _record_drop(self, reason)
                return True
            if cmd_id == 0x0305 and radar_client_rate_hz > 0.0:
                if not _cache_radar_client_payload(self, payload):
                    _record_drop(self, "0x0305 scheduler unavailable")
                    return False
                return True
            _publish_tx_payload(self, cmd_id, payload)
            return True
        except Exception as exc:  # noqa: BLE001 - keep algorithm runtime alive
            self.get_logger().warning(f"Failed to forward algorithm referee frame: {exc}")
            return False

    def is_connected_noop(self):
        last_rx = float(getattr(self, "_rm_radio_last_rx_time", 0.0))
        return last_rx > 0.0 and time.monotonic() - last_rx < 2.0

    manager_cls.start = start_noop
    manager_cls.close = close_noop
    manager_cls.tx = tx_noop
    manager_cls.is_connected = is_connected_noop


def _normalize_runtime_mode(value: str | None) -> str:
    mode = (value or "full").strip().lower().replace("-", "_")
    if mode in {"", "auto"}:
        return "full"
    return mode


def _option_value(argv: list[str], option: str, default: str) -> str:
    for index, token in enumerate(argv):
        if token == option and index + 1 < len(argv):
            return argv[index + 1]
        prefix = f"{option}="
        if token.startswith(prefix):
            return token[len(prefix) :]
    return default


def _load_yaml(path: Path) -> dict:
    try:
        import yaml

        with path.open("r", encoding="utf-8") as file:
            data = yaml.safe_load(file) or {}
        return data if isinstance(data, dict) else {}
    except Exception as exc:  # noqa: BLE001 - fallback must keep the process alive
        print(f"[rm_radio_ros][WARN] Failed to read algorithm config {path}: {exc}", file=sys.stderr, flush=True)
        return {}


def _config_path_from_args(algo_dir: Path, algo_args: list[str]) -> Path:
    config_arg = _option_value(algo_args, "--config", "config/params.yaml")
    path = Path(config_arg)
    return path if path.is_absolute() else algo_dir / path


def _referee_config(algo_dir: Path, algo_args: list[str]) -> tuple[str, int, Path]:
    config_path = _config_path_from_args(algo_dir, algo_args)
    config = _load_yaml(config_path)
    referee_cfg = config.get("referee", {}) if isinstance(config.get("referee", {}), dict) else {}
    port = os.environ.get("REFEREE_PORT") or referee_cfg.get("port") or "/dev/ttyUSB0"
    raw_baudrate = os.environ.get("REFEREE_BAUDRATE") or referee_cfg.get("baudrate") or 115200
    try:
        baudrate = int(raw_baudrate)
    except (TypeError, ValueError):
        baudrate = 115200
    return str(port), baudrate, config_path


def _run_full_algorithm(main_py: Path, algo_args: list[str]) -> None:
    sys.argv = [str(main_py), *algo_args]
    runpy.run_path(str(main_py), run_name="__main__")


def _run_bridge_only(algo_dir: Path, algo_args: list[str], reason: str = "") -> None:
    import rclpy
    from driver.referee.referee_comm import RefereeCommManager

    port, baudrate, config_path = _referee_config(algo_dir, algo_args)
    detail = f" Reason: {reason}" if reason else ""
    print(
        "[rm_radio_ros][WARN] RM_ALGO_RUNTIME_MODE=bridge_only active; "
        f"vision inference is disabled, config={config_path}, referee_port={port}, baudrate={baudrate}.{detail}",
        file=sys.stderr,
        flush=True,
    )

    try:
        if not rclpy.ok():
            rclpy.init(args=None)
    except RuntimeError as exc:
        if "already initialized" not in str(exc).lower():
            raise

    referee = RefereeCommManager(port=port, baudrate=baudrate)
    if not referee.start():
        raise SystemExit("Failed to start RefereeCommManager in bridge_only fallback mode")
    referee.get_logger().warning(
        "Algorithm bridge_only fallback is active; CUDA/TensorRT inference is unavailable, "
        "so camera and detector/tracker initialization are skipped."
    )

    try:
        while rclpy.ok():
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received, stopping algorithm bridge_only fallback...", flush=True)
    finally:
        try:
            referee.close()
            message_thread = getattr(referee, "message_daemon_thread", None)
            if message_thread is not None and message_thread.is_alive():
                message_thread.join(timeout=1.0)
        finally:
            if rclpy.ok():
                rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--algo-dir", required=True)
    parser.add_argument("algo_args", nargs=argparse.REMAINDER)
    args = parser.parse_args()

    algo_dir = Path(args.algo_dir).resolve()
    main_py = algo_dir / "main.py"
    if not main_py.is_file():
        raise SystemExit(f"Algorithm entry not found: {main_py}")

    os.chdir(algo_dir)
    sys.path.insert(0, str(algo_dir))
    os.environ.setdefault("RADAR_LD_FIXED", "1")

    _patch_referee_serial()

    algo_args = args.algo_args
    if algo_args and algo_args[0] == "--":
        algo_args = algo_args[1:]

    runtime_mode = _normalize_runtime_mode(os.environ.get("RM_ALGO_RUNTIME_MODE", "full"))
    if runtime_mode in {"bridge_only", "noop", "fallback"}:
        _run_bridge_only(algo_dir, algo_args)
        return
    if runtime_mode != "full":
        raise SystemExit(
            f"Unsupported RM_ALGO_RUNTIME_MODE={runtime_mode}; expected full or bridge_only."
        )

    _run_full_algorithm(main_py, algo_args)


if __name__ == "__main__":
    main()
