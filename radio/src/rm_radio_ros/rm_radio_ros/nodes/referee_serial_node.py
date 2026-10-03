from __future__ import annotations

import json
import os
import time
from collections import deque
from collections.abc import Iterable
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from ..core.invincible_targets import (
    INVINCIBLE_TARGETS_DATA_CMD_ID,
    WIRE_STATUS_ROBOT_NUMBERS,
    InvincibleTargetsMessage,
    fallback_invincible_target_statuses,
    invincible_target_receivers,
    invincible_target_statuses,
)
from ..core.referee_bridge import RefereeBridge
from ..core.rm_protocol import (
    RefereeFrame,
    RefereeFrameAssembler,
    build_radar_cmd_payload,
    build_referee_frame,
    build_robot_interaction_frame,
    radar_sender_id_for_side,
)
from ..core.radio_config import (
    broadcast_rx_setters_for_side,
    interference_rx_setters_for_side_level,
    interference_setters_for_side_level,
    normalize_interference_level,
)


def _node_int_parameter(node: Node, name: str, default: int) -> int:
    try:
        return int(node.get_parameter(name).value)
    except Exception:  # noqa: BLE001 - permits unit tests on uninitialized instances
        return int(default)


def _node_float_parameter(node: Node, name: str, default: float) -> float:
    try:
        return float(node.get_parameter(name).value)
    except Exception:  # noqa: BLE001 - permits unit tests on uninitialized instances
        return float(default)


def _node_bool_parameter(node: Node, name: str, default: bool) -> bool:
    try:
        return bool(node.get_parameter(name).value)
    except Exception:  # noqa: BLE001 - permits unit tests on uninitialized instances
        return bool(default)


def _bytes_from_hex(value: str) -> bytes:
    clean = ''.join(str(value).split())
    if clean.startswith('0x') or clean.startswith('0X'):
        clean = clean[2:]
    return bytes.fromhex(clean)


def _bytes_from_json_value(value: object) -> bytes:
    if value is None:
        return b''
    if isinstance(value, str):
        return _bytes_from_hex(value)
    if isinstance(value, Iterable):
        return bytes(int(item) & 0xFF for item in value)
    raise ValueError(f'cannot convert value to bytes: {value!r}')


def _valid_0305_payload(data: bytes) -> bool:
    return len(data) == 48 and any(data)


class OptionalSerialPort:
    def __init__(self, port: str, baudrate: int, timeout: float, exclusive: bool = True):
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout = float(timeout)
        self.exclusive = bool(exclusive)
        self._serial = None
        self._lock_file = None
        self.last_error = ''
        self.open_count = 0
        self.close_count = 0
        self.last_open_time = 0.0
        self.last_close_time = 0.0

    @property
    def is_open(self) -> bool:
        serial_obj = self._serial
        if serial_obj is None:
            return False
        try:
            return bool(getattr(serial_obj, 'is_open', True))
        except Exception:  # noqa: BLE001 - stale serial objects should be treated as closed elsewhere
            return True

    def _lock_path(self) -> Path:
        # The application-facing udev name must remain the lock identity even
        # while the USB device is absent.  realpath() changes from
        # /dev/ttyUSB<N> to /dev/shark-referee when the symlink disappears,
        # which lets two bridge processes acquire different locks during a
        # disconnect/reconnect race.
        identity = os.path.abspath(os.path.expanduser(self.port))
        clean = identity.replace('/', '_').replace('\\', '_').strip('_') or 'unknown'
        return Path('/tmp') / f'rm_radio_referee_serial_{clean}.lock'

    def _acquire_lock(self) -> bool:
        if not self.exclusive or self._lock_file is not None:
            return True
        try:
            import fcntl

            path = self._lock_path()
            handle = path.open('w', encoding='utf-8')
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            handle.seek(0)
            handle.truncate()
            handle.write(f'pid={os.getpid()} port={self.port}\n')
            handle.flush()
            self._lock_file = handle
            return True
        except BlockingIOError:
            self.last_error = f'serial port already locked by another rm_radio_ros bridge: {self.port}'
            return False
        except Exception as exc:  # noqa: BLE001 - lock support is best-effort but explicit
            self.last_error = f'failed to lock serial port {self.port}: {exc}'
            return False

    def _release_lock(self) -> None:
        handle = self._lock_file
        self._lock_file = None
        if handle is None:
            return
        try:
            import fcntl

            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        finally:
            handle.close()

    def _close_serial_handle(self) -> None:
        serial_obj = self._serial
        self._serial = None
        if serial_obj is None:
            return
        try:
            serial_obj.close()
        except Exception:
            pass
        self.close_count += 1
        self.last_close_time = time.time()

    def open(self) -> bool:
        if self._serial is not None and self.is_open:
            return True
        if self._serial is not None:
            self._close_serial_handle()
        if not self._acquire_lock():
            return False
        try:
            import serial  # type: ignore

            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
                write_timeout=self.timeout,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
                # Also ask the kernel to reject opens through an alias such as
                # /dev/ttyUSB0 while this handle is alive.
                exclusive=self.exclusive,
            )
            self.open_count += 1
            self.last_open_time = time.time()
            self.last_error = ''
            return True
        except Exception as exc:  # noqa: BLE001 - optional runtime dependency/hardware
            self.last_error = f'serial unavailable: {exc}'
            self._serial = None
            # Keep process ownership while the adapter is unplugged.  Releasing
            # the lock here lets a duplicate node win the reconnect race and
            # leaves the original bridge permanently reporting "already
            # locked".  close() releases it when the owning node really exits.
            return False

    def write(self, data: bytes) -> bool:
        if not self.open():
            return False
        try:
            written = self._serial.write(data)
            if written is not None and int(written) != len(data):
                raise IOError(f'partial serial write: {written}/{len(data)} bytes')
            self._serial.flush()
            self.last_error = ''
            return True
        except Exception as exc:  # noqa: BLE001
            self.last_error = f'serial write failed: {exc}'
            self._close_serial_handle()
            return False

    def read_available(self, max_bytes: int = 4096) -> bytes:
        if not self.open():
            return b''
        try:
            waiting = int(getattr(self._serial, 'in_waiting', 0))
            count = max(1, min(int(max_bytes), waiting if waiting > 0 else 1))
            data = self._serial.read(count)
            self.last_error = ''
            return data
        except Exception as exc:  # noqa: BLE001
            self.last_error = f'serial read failed: {exc}'
            self._close_serial_handle()
            return b''

    def close(self) -> None:
        try:
            self._close_serial_handle()
        finally:
            self._release_lock()


class RefereeSerialNode(Node):
    """Referee-system serial bridge for the official 0xA5 UART protocol."""

    def __init__(self):
        super().__init__('rm_referee_serial_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('dry_run', True)
        self.declare_parameter('bridge_topic', '/rm_gfsk_node/referee_bridge')
        self.declare_parameter('radar_cmd_topic', '/rm_radar_algorithm/radar_cmd')
        self.declare_parameter('sender_id', 9)
        self.declare_parameter('receiver_id', 0x8080)
        self.declare_parameter('auto_send_radar_cmd', True)
        self.declare_parameter('password_verify_cooldown_sec', 10.0)
        self.declare_parameter('require_can_change_password_for_update', True)
        self.declare_parameter('radar_client_max_rate_hz', 5.0)
        self.declare_parameter('auto_send_invincible_targets', True)
        self.declare_parameter('invincible_targets_data_cmd_id', INVINCIBLE_TARGETS_DATA_CMD_ID)
        self.declare_parameter('invincible_targets_send_rate_hz', 3.0)
        self.declare_parameter('invincible_targets_freshness_sec', 1.0)
        self.declare_parameter('auto_interference_level', False)
        self.declare_parameter('auto_rx_interference_level', True)
        self.declare_parameter('apply_referee_level_only_when_running', False)
        self.declare_parameter('single_rx_auto_broadcast_after_level3', False)
        self.declare_parameter('single_rx_auto_allow_return_to_interference', True)
        self.declare_parameter('referee_level_confirm_count', 2)
        self.declare_parameter('radio_side', 'red')
        self.declare_parameter('auto_detect_radio_side', True)
        self.declare_parameter('interference_level', 1)
        self.declare_parameter('interference_control_topic', '/rm_ganraoyuan_node/setters_json')
        self.declare_parameter('interference_rx_control_topic', '/rm_gfsk_interference_node/setters_json')
        self.declare_parameter('read_period_sec', 0.01)
        self.declare_parameter('status_period_sec', 1.0)
        self.declare_parameter('frame_timeout_sec', 2.0)
        self.declare_parameter('exclusive_port_lock', True)
        self.declare_parameter('require_serial_open_on_start', False)

        self.seq = 0
        self.tx_count = 0
        self.rx_count = 0
        self.dry_run_count = 0
        self.last_radar_cmd_value = 0
        self.algorithm_request_ids = deque(maxlen=256)
        self.algorithm_request_acks: dict[str, dict] = {}
        self.last_algorithm_request: Optional[dict] = None
        self.last_algorithm_ack: Optional[dict] = None
        self.last_password_verify_time = 0.0
        self.last_password_verify = ''
        self.password_verify_suppressed_count = 0
        self.last_password_verify_suppression = ''
        self.last_can_change_password: Optional[bool] = None
        self.last_radar_decision_sync: Optional[dict] = None
        self.last_game_status: Optional[dict] = None
        self.last_game_robot_hp: Optional[dict] = None
        self.last_robot_status: Optional[dict] = None
        self.detected_robot_id: Optional[int] = None
        self.detected_radio_side: Optional[str] = None
        self.detected_radar_sender_id: Optional[int] = None
        self.last_0305_tx_time = 0.0
        self.blocked_0305_rate_count = 0
        self.last_invincible_main_status: Optional[dict[str, int]] = None
        self.last_invincible_source_frame_sequence: Optional[int] = None
        self.last_invincible_source_received_at: Optional[float] = None
        self._last_invincible_source_monotonic: Optional[float] = None
        self.invincible_targets_source_count = 0
        self.invincible_targets_batch_count = 0
        self.invincible_targets_frame_request_count = 0
        self.invincible_targets_frame_success_count = 0
        self.invincible_targets_frame_dry_run_count = 0
        self.last_invincible_targets_batch: Optional[dict] = None
        self._pending_invincible_targets_batch: Optional[dict] = None
        self.last_error = ''
        self.last_tx: Optional[dict] = None
        self.last_rx: Optional[dict] = None
        self.last_valid_frame_at: Optional[float] = None
        self._last_valid_frame_monotonic: Optional[float] = None
        self._frame_watchdog_armed_at: Optional[float] = None
        self._frame_watchdog_error = ''
        self.interference_level = self._initial_interference_level()
        self.interference_update_count = 0
        self.last_interference_update: Optional[dict] = None
        self.last_interference_setters: Optional[dict] = None
        self.rx_interference_level = self.interference_level
        self.rx_interference_update_count = 0
        self.suppressed_rx_interference_update_count = 0
        self.last_rx_interference_update: Optional[dict] = None
        self.last_rx_interference_setters: Optional[dict] = None
        self.last_suppressed_rx_interference_update: Optional[dict] = None
        self.single_rx_mode = 'interference'
        self.single_rx_switch_count = 0
        self.last_single_rx_switch: Optional[dict] = None
        self.referee_level_candidate: Optional[int] = None
        self.referee_level_candidate_count = 0
        self.confirmed_referee_level = self.interference_level
        self.last_referee_level_observation: Optional[dict] = None
        self.last_confirmed_referee_level_update: Optional[dict] = None
        self.suppressed_referee_level_before_running_count = 0
        self.last_suppressed_referee_level_before_running: Optional[dict] = None
        self.assembler = RefereeFrameAssembler(max_buffer_size=16384)
        self.referee_bridge = RefereeBridge()
        self.serial_port = OptionalSerialPort(
            str(self.get_parameter('port').value),
            int(self.get_parameter('baudrate').value),
            timeout=0.001,
            exclusive=bool(self.get_parameter('exclusive_port_lock').value),
        )
        self._enforce_serial_start_policy()

        self.tx_pub = self.create_publisher(String, '~/tx_frames', 10)
        self.rx_pub = self.create_publisher(String, '~/rx_frames', 10)
        self.referee_bridge_pub = self.create_publisher(String, '~/referee_bridge', 10)
        self.raw_rx_pub = self.create_publisher(String, '~/raw_rx', 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        game_status_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.game_status_pub = self.create_publisher(String, '~/game_status', game_status_qos)
        interference_control_topic = str(self.get_parameter('interference_control_topic').value).strip()
        interference_control_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.interference_control_pub = self.create_publisher(
            String,
            interference_control_topic,
            interference_control_qos,
        )
        interference_rx_control_topic = str(self.get_parameter('interference_rx_control_topic').value).strip()
        self.interference_rx_control_pub = self.create_publisher(
            String,
            interference_rx_control_topic,
            interference_control_qos,
        )
        self.create_subscription(String, '~/send_frame', self._handle_send_frame, 10)
        self.create_subscription(String, '~/send_interaction', self._handle_send_interaction, 10)
        self.create_subscription(String, '~/send_radar_cmd', self._handle_send_radar_cmd, 10)
        bridge_topics = self._parse_topic_list(str(self.get_parameter('bridge_topic').value))
        radar_cmd_topic = str(self.get_parameter('radar_cmd_topic').value)
        for bridge_topic in bridge_topics:
            self.create_subscription(String, bridge_topic, self._handle_bridge_message, 10)
        self.create_subscription(String, radar_cmd_topic, self._handle_algorithm_radar_cmd, 10)

        self.create_timer(max(float(self.get_parameter('read_period_sec').value), 0.001), self._read_serial)
        self.create_timer(max(float(self.get_parameter('status_period_sec').value), 0.1), self._publish_status)
        if bool(self.get_parameter('auto_send_invincible_targets').value):
            invincible_rate_per_receiver = min(
                max(float(self.get_parameter('invincible_targets_send_rate_hz').value), 0.1),
                4.0,
            )
            invincible_frame_rate = invincible_rate_per_receiver * 6.0
            self.create_timer(1.0 / invincible_frame_rate, self._send_invincible_targets_tick)
        frame_timeout_sec = max(float(self.get_parameter('frame_timeout_sec').value), 0.0)
        frame_watchdog_period = min(max(frame_timeout_sec / 4.0, 0.1), 0.5) if frame_timeout_sec else 1.0
        self.create_timer(frame_watchdog_period, self._check_frame_watchdog)
        self.get_logger().info(f'subscribed bridge topics {bridge_topics}')
        self.get_logger().info(f'subscribed radar_cmd topic {radar_cmd_topic}')
        self.get_logger().info(f'interference level control topic {interference_control_topic}')
        self.get_logger().info(f'interference RX control topic {interference_rx_control_topic}')
        if bool(self.get_parameter('auto_send_invincible_targets').value):
            self.get_logger().info(
                f'invincible-target broadcast enabled: data_cmd_id='
                f'0x{_node_int_parameter(self, "invincible_targets_data_cmd_id", INVINCIBLE_TARGETS_DATA_CMD_ID):04X}, '
                f'rate={invincible_rate_per_receiver:.1f}Hz x 6 receivers '
                f'({invincible_frame_rate:.1f} frames/s round-robin)'
            )
        if bool(self.get_parameter('single_rx_auto_broadcast_after_level3').value):
            self.get_logger().warning(
                'single RX auto-switch enabled: interference RX will switch to broadcast after referee level reaches 3'
            )

    def _enforce_serial_start_policy(self) -> None:
        if bool(self.get_parameter('dry_run').value):
            return
        if not bool(self.get_parameter('require_serial_open_on_start').value):
            return
        if self.serial_port.open():
            return
        detail = self.serial_port.last_error or 'unknown serial open error'
        raise RuntimeError(
            f'referee serial fail-fast: could not open {self.serial_port.port}: {detail}'
        )

    def _next_seq(self) -> int:
        value = self.seq & 0xFF
        self.seq = (self.seq + 1) & 0xFF
        return value

    @staticmethod
    def _parse_topic_list(raw: str) -> list[str]:
        topics = [topic.strip() for topic in str(raw).replace(';', ',').split(',')]
        return [topic for topic in topics if topic]

    def _initial_interference_level(self) -> int:
        try:
            return normalize_interference_level(int(self.get_parameter('interference_level').value))
        except Exception as exc:  # noqa: BLE001 - keep node alive and surface config error
            self.last_error = f'invalid initial interference_level: {exc}'
            self.get_logger().warning(self.last_error)
            return 1

    def _send_bytes(self, frame: bytes, meta: dict) -> dict:
        payload = dict(meta)
        payload.update({
            'timestamp': time.time(),
            'frame_hex': frame.hex(),
            'frame_length': len(frame),
            'dry_run': bool(self.get_parameter('dry_run').value),
            'written': False,
        })
        if bool(self.get_parameter('dry_run').value):
            self.dry_run_count += 1
            self.last_error = ''
        else:
            payload['written'] = self.serial_port.write(frame)
            if payload['written']:
                self.tx_count += 1
                self.last_error = ''
            else:
                self.last_error = self.serial_port.last_error
                self.get_logger().warning(self.last_error)
        self.last_tx = payload
        self._publish_json(self.tx_pub, payload)
        return payload

    def _commit_radar_cmd_after_send(self, result: dict) -> None:
        if not (result.get('written') is True or result.get('dry_run') is True):
            return
        try:
            radar_cmd = int(result.get('radar_cmd', self.last_radar_cmd_value))
        except (TypeError, ValueError):
            return
        if radar_cmd > int(getattr(self, 'last_radar_cmd_value', 0)):
            self.last_radar_cmd_value = radar_cmd

    def _handle_send_frame(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            cmd_id = int(payload['cmd_id'], 0) if isinstance(payload['cmd_id'], str) else int(payload['cmd_id'])
            data = _bytes_from_json_value(payload.get('data_hex', payload.get('data', '')))
            seq = int(payload['seq']) if 'seq' in payload else self._next_seq()
            if self._is_raw_radar_cmd_frame(cmd_id, data):
                self.last_error = 'blocked raw 0x0301/0x0121 send_frame; use ~/send_radar_cmd'
                self.get_logger().warning(self.last_error)
                return
            if cmd_id == 0x0305 and not _valid_0305_payload(data):
                self.last_error = 'blocked invalid 0x0305 send_frame payload'
                self.get_logger().warning(self.last_error)
                return
            if cmd_id == 0x0305 and not self._allow_0305_send():
                return
            frame = build_referee_frame(cmd_id, data, seq=seq)
        except Exception as exc:  # noqa: BLE001
            self.last_error = f'invalid send_frame payload: {exc}'
            self.get_logger().warning(self.last_error)
            return
        self._send_bytes(frame, {'kind': 'frame', 'cmd_id': cmd_id, 'cmd_hex': f'0x{cmd_id:04X}', 'seq': seq})

    def _handle_send_interaction(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            frame, meta = self._build_interaction_from_payload(payload)
        except Exception as exc:  # noqa: BLE001
            self.last_error = f'invalid send_interaction payload: {exc}'
            self.get_logger().warning(self.last_error)
            return
        result = self._send_bytes(frame, meta)
        self._commit_radar_cmd_after_send(result)

    def _handle_send_radar_cmd(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            frame, meta = self._build_radar_cmd_from_payload(payload)
        except Exception as exc:  # noqa: BLE001
            self.last_error = f'invalid send_radar_cmd payload: {exc}'
            self.get_logger().warning(self.last_error)
            return
        result = self._send_bytes(frame, meta)
        self._commit_radar_cmd_after_send(result)

    def _handle_algorithm_radar_cmd(self, msg: String) -> None:
        request_id = ''
        try:
            payload = self._parse_algorithm_radar_cmd_message(msg.data)
            request_id = str(payload.get('request_id', '')).strip()
            if request_id and request_id in getattr(self, 'algorithm_request_acks', {}):
                duplicate_ack = dict(self.algorithm_request_acks[request_id])
                duplicate_ack.update({
                    'timestamp': time.time(),
                    'duplicate': True,
                    'source': 'algorithm_radar_cmd_topic',
                })
                self.last_algorithm_ack = duplicate_ack
                self._publish_json(self.tx_pub, duplicate_ack)
                return
            frame, meta = self._build_radar_cmd_from_payload(payload)
        except Exception as exc:  # noqa: BLE001
            self.last_error = f'invalid algorithm radar_cmd payload: {exc}'
            self.get_logger().warning(self.last_error)
            if not request_id:
                try:
                    raw_payload = json.loads(str(msg.data))
                    request_id = str(raw_payload.get('request_id', '')).strip() if isinstance(raw_payload, dict) else ''
                except Exception:  # noqa: BLE001
                    request_id = ''
            if request_id:
                ack = {
                    'kind': 'radar_cmd_0121',
                    'source': 'algorithm_radar_cmd_topic',
                    'request_id': request_id,
                    'timestamp': time.time(),
                    'radar_cmd': int(getattr(self, 'last_radar_cmd_value', 0)),
                    'written': False,
                    'dry_run': bool(_node_bool_parameter(self, 'dry_run', True)),
                    'duplicate': False,
                    'error': self.last_error,
                }
                self._remember_algorithm_ack(request_id, ack)
                self.last_algorithm_ack = ack
                self._publish_json(self.tx_pub, ack)
            return
        meta['source'] = 'algorithm_radar_cmd_topic'
        if request_id:
            meta['request_id'] = request_id
        self.last_algorithm_request = dict(meta)
        result = self._send_bytes(frame, meta)
        self._commit_radar_cmd_after_send(result)
        result['radar_cmd'] = int(getattr(self, 'last_radar_cmd_value', result.get('radar_cmd', 0)))
        result['duplicate'] = False
        self.last_algorithm_ack = dict(result)
        if request_id:
            self._remember_algorithm_ack(request_id, result)

    def _remember_algorithm_ack(self, request_id: str, ack: dict) -> None:
        if not hasattr(self, 'algorithm_request_ids'):
            self.algorithm_request_ids = deque(maxlen=256)
        if not hasattr(self, 'algorithm_request_acks'):
            self.algorithm_request_acks = {}
        if request_id not in self.algorithm_request_acks:
            if len(self.algorithm_request_ids) == self.algorithm_request_ids.maxlen:
                evicted = self.algorithm_request_ids.popleft()
                self.algorithm_request_acks.pop(evicted, None)
            self.algorithm_request_ids.append(request_id)
        self.algorithm_request_acks[request_id] = dict(ack)

    def _handle_bridge_message(self, msg: String) -> None:
        try:
            bridge_payload = json.loads(msg.data)
            if not isinstance(bridge_payload, dict):
                raise ValueError('bridge payload must be an object')
        except Exception as exc:  # noqa: BLE001
            self.last_error = f'invalid bridge message: {exc}'
            self.get_logger().warning(self.last_error)
            return

        if bridge_payload.get('type') == 'RadarEnemyBuffStatus':
            self._observe_invincible_targets_bridge(bridge_payload)

        if (
            bridge_payload.get('type') != 'RadarCommand0121'
            or not bool(self.get_parameter('auto_send_radar_cmd').value)
        ):
            return
        try:
            radar_payload = bridge_payload.get('payload') or {}
            if not self._should_send_bridge_radar_command(radar_payload):
                return
            frame, meta = self._build_radar_cmd_from_payload(
                radar_payload,
                preserve_last_radar_cmd=True,
            )
        except Exception as exc:  # noqa: BLE001
            self.last_error = f'bridge radar command failed: {exc}'
            self.get_logger().warning(self.last_error)
            return
        result = self._send_bytes(frame, meta)
        self._commit_radar_cmd_after_send(result)

    def _observe_invincible_targets_bridge(self, bridge_payload: dict) -> bool:
        if not _node_bool_parameter(self, 'auto_send_invincible_targets', True):
            return False
        try:
            payload = bridge_payload.get('payload')
            if not isinstance(payload, dict):
                raise ValueError('RadarEnemyBuffStatus payload must be an object')
            status = payload.get('robot_main_status')
            if not isinstance(status, dict):
                raise ValueError('RadarEnemyBuffStatus.robot_main_status must be an object')
            # Strictly validate all five official 0x0A05 fields before making
            # this snapshot authoritative.  The returned IDs are intentionally
            # discarded here because side can hot-switch before the send tick.
            invincible_target_statuses(status, self._effective_radio_side())
            clean_status = {
                name: int(status[name])
                for name, _robot_number in WIRE_STATUS_ROBOT_NUMBERS
            }
            source_sequence = int(bridge_payload.get('seq', 0)) & 0xFF
        except Exception as exc:  # noqa: BLE001
            self.last_error = f'ignored invalid RadarEnemyBuffStatus bridge: {exc}'
            self.get_logger().warning(self.last_error)
            return False

        self.last_invincible_main_status = clean_status
        self.last_invincible_source_frame_sequence = source_sequence
        self.last_invincible_source_received_at = time.time()
        self._last_invincible_source_monotonic = time.monotonic()
        self.invincible_targets_source_count = int(
            getattr(self, 'invincible_targets_source_count', 0)
        ) + 1
        return True

    def _invincible_targets_snapshot(self, now_monotonic: Optional[float] = None) -> dict:
        now = time.monotonic() if now_monotonic is None else float(now_monotonic)
        last = getattr(self, '_last_invincible_source_monotonic', None)
        age = max(0.0, now - float(last)) if last is not None else None
        freshness = max(_node_float_parameter(self, 'invincible_targets_freshness_sec', 1.0), 0.1)
        radio_fresh = age is not None and age <= freshness
        side = self._effective_radio_side()
        target_statuses = fallback_invincible_target_statuses(side)
        if radio_fresh:
            try:
                target_statuses = invincible_target_statuses(
                    getattr(self, 'last_invincible_main_status', None) or {},
                    side,
                )
            except ValueError:
                radio_fresh = False
                target_statuses = fallback_invincible_target_statuses(side)
        target_ids = tuple(robot_id for robot_id, invincible in target_statuses if invincible)
        return {
            'radio_side': side,
            'radio_fresh': radio_fresh,
            'fallback_all_alive': not radio_fresh,
            'source_age_sec': age,
            'freshness_sec': freshness,
            'source_frame_sequence': (
                int(getattr(self, 'last_invincible_source_frame_sequence', 0)) & 0xFF
                if radio_fresh
                else 0xFF
            ),
            'target_ids': list(target_ids),
            'target_statuses': [
                {'robot_id': robot_id, 'invincible': invincible}
                for robot_id, invincible in target_statuses
            ],
            'receivers': list(invincible_target_receivers(side)),
        }

    def _send_invincible_targets_tick(self) -> Optional[dict]:
        if not _node_bool_parameter(self, 'auto_send_invincible_targets', True):
            return None
        data_cmd_id = _node_int_parameter(
            self,
            'invincible_targets_data_cmd_id',
            INVINCIBLE_TARGETS_DATA_CMD_ID,
        )
        if not 0x0200 <= data_cmd_id <= 0x02FF:
            self.last_error = (
                f'invincible_targets_data_cmd_id must be in 0x0200..0x02FF, got 0x{data_cmd_id:04X}'
            )
            self.get_logger().warning(self.last_error)
            return None

        pending = getattr(self, '_pending_invincible_targets_batch', None)
        if not isinstance(pending, dict):
            snapshot = self._invincible_targets_snapshot()
            message = InvincibleTargetsMessage.from_target_statuses(
                tuple(
                    (int(entry['robot_id']), bool(entry['invincible']))
                    for entry in snapshot['target_statuses']
                )
            )
            pending = {
                'snapshot': snapshot,
                'user_data': message.to_bytes(),
                'sender_id': self._effective_sender_id(),
                'data_cmd_id': data_cmd_id,
                'started_at': time.time(),
                'results': [],
            }
            self._pending_invincible_targets_batch = pending

        snapshot = pending['snapshot']
        user_data = bytes(pending['user_data'])
        sender_id = int(pending['sender_id'])
        batch_data_cmd_id = int(pending['data_cmd_id'])
        results = pending['results']
        receiver_id = int(snapshot['receivers'][len(results)])
        frame_sequence = self._next_seq()
        frame = build_robot_interaction_frame(
            batch_data_cmd_id,
            sender_id,
            receiver_id,
            user_data,
            seq=frame_sequence,
        )
        meta = {
            'kind': 'radar_invincible_targets',
            'cmd_id': 0x0301,
            'cmd_hex': '0x0301',
            'data_cmd_id': batch_data_cmd_id,
            'data_cmd_hex': f'0x{batch_data_cmd_id:04X}',
            'sender_id': sender_id,
            'receiver_id': receiver_id,
            'source_frame_sequence': int(snapshot['source_frame_sequence']),
            'radio_fresh': bool(snapshot['radio_fresh']),
            'fallback_all_alive': bool(snapshot['fallback_all_alive']),
            'invincible_target_ids': list(snapshot['target_ids']),
            'invincible_target_statuses': list(snapshot['target_statuses']),
            'invincible_mask': user_data[0],
            'source_age_sec': snapshot['source_age_sec'],
            'user_data_hex': user_data.hex(),
            'user_data_length': len(user_data),
            'seq': frame_sequence,
        }
        result = self._send_bytes(frame, meta)
        results.append(result)

        self.invincible_targets_frame_request_count = int(
            getattr(self, 'invincible_targets_frame_request_count', 0)
        ) + 1
        self.invincible_targets_frame_success_count = int(
            getattr(self, 'invincible_targets_frame_success_count', 0)
        ) + int(result.get('written') is True)
        self.invincible_targets_frame_dry_run_count = int(
            getattr(self, 'invincible_targets_frame_dry_run_count', 0)
        ) + int(result.get('dry_run') is True)

        if len(results) < len(snapshot['receivers']):
            return {
                **snapshot,
                'schema': 'shark.radar.invincible_target_states.v1',
                'batch_complete': False,
                'sent_receiver_id': receiver_id,
                'remaining_receivers': len(snapshot['receivers']) - len(results),
            }

        self.invincible_targets_batch_count = int(
            getattr(self, 'invincible_targets_batch_count', 0)
        ) + 1
        batch = dict(snapshot)
        batch.update({
            'schema': 'shark.radar.invincible_target_states.v1',
            'batch_complete': True,
            'timestamp': time.time(),
            'started_at': pending['started_at'],
            'data_cmd_id': batch_data_cmd_id,
            'data_cmd_hex': f'0x{batch_data_cmd_id:04X}',
            'user_data_hex': user_data.hex(),
            'frame_results': [
                {
                    'receiver_id': result.get('receiver_id'),
                    'written': result.get('written'),
                    'dry_run': result.get('dry_run'),
                }
                for result in results
            ],
        })
        self.last_invincible_targets_batch = batch
        self._pending_invincible_targets_batch = None
        return batch

    def _should_send_bridge_radar_command(self, payload: dict) -> bool:
        password_cmd = int(payload.get('password_cmd', 0))
        password = payload.get('password')
        if password_cmd != 2 or password is None:
            return True

        now = time.monotonic()
        cooldown = max(_node_float_parameter(self, 'password_verify_cooldown_sec', 10.0), 0.0)
        last_time = float(getattr(self, 'last_password_verify_time', 0.0))
        if last_time > 0.0 and now - last_time < cooldown:
            self.password_verify_suppressed_count = int(getattr(self, 'password_verify_suppressed_count', 0)) + 1
            remaining = cooldown - (now - last_time)
            self.last_password_verify_suppression = (
                f'suppressed password verification during {cooldown:.1f}s protocol cooldown '
                f'({remaining:.1f}s remaining)'
            )
            if self.password_verify_suppressed_count in (1, 10) or self.password_verify_suppressed_count % 100 == 0:
                self.get_logger().warning(self.last_password_verify_suppression)
            return False

        self.last_password_verify_time = now
        self.last_password_verify = str(password)
        self.password_verify_suppressed_count = 0
        self.last_password_verify_suppression = ''
        return True

    def _password_verify_status(self) -> dict:
        cooldown = max(_node_float_parameter(self, 'password_verify_cooldown_sec', 10.0), 0.0)
        last_time = float(getattr(self, 'last_password_verify_time', 0.0))
        elapsed = max(0.0, time.monotonic() - last_time) if last_time > 0.0 else cooldown
        remaining = max(0.0, cooldown - elapsed)
        return {
            'last_password_verify': str(getattr(self, 'last_password_verify', '') or ''),
            'password_verify_cooldown_sec': cooldown,
            'password_verify_cooldown_remaining_sec': remaining,
            'password_verify_ready': remaining <= 0.0,
        }

    def _build_interaction_from_payload(self, payload: dict) -> tuple[bytes, dict]:
        data_cmd_id = payload.get('data_cmd_id', payload.get('sub_cmd_id'))
        if data_cmd_id is None:
            raise ValueError('data_cmd_id is required')
        data_cmd_id = int(data_cmd_id, 0) if isinstance(data_cmd_id, str) else int(data_cmd_id)
        if data_cmd_id == 0x0121:
            raise ValueError('0x0121 radar command must use ~/send_radar_cmd so protocol guards are applied')
        sender_id = int(payload.get('sender_id', self._effective_sender_id()))
        receiver_id = int(payload.get('receiver_id', _node_int_parameter(self, 'receiver_id', 0x8080)))
        user_data = _bytes_from_json_value(payload.get('user_data_hex', payload.get('user_data', '')))
        seq = int(payload['seq']) if 'seq' in payload else self._next_seq()
        frame = build_robot_interaction_frame(data_cmd_id, sender_id, receiver_id, user_data, seq=seq)
        return frame, {
            'kind': 'robot_interaction',
            'cmd_id': 0x0301,
            'cmd_hex': '0x0301',
            'data_cmd_id': data_cmd_id,
            'data_cmd_hex': f'0x{data_cmd_id:04X}',
            'sender_id': sender_id,
            'receiver_id': receiver_id,
            'user_data_hex': user_data.hex(),
            'seq': seq,
        }

    def _build_radar_cmd_from_payload(
        self,
        payload: dict,
        preserve_last_radar_cmd: bool = False,
    ) -> tuple[bytes, dict]:
        has_explicit_radar_cmd = 'radar_cmd' in payload or 'request_count' in payload
        password_present = 'password' in payload or 'password_cmd' in payload
        payload_radar_cmd = int(payload.get('radar_cmd', payload.get('request_count', 0)))
        if (
            password_present
            and (not has_explicit_radar_cmd or payload_radar_cmd == 0)
            and int(getattr(self, 'last_radar_cmd_value', 0)) > 0
        ):
            radar_cmd = int(getattr(self, 'last_radar_cmd_value', 0))
        elif preserve_last_radar_cmd and (
            not has_explicit_radar_cmd or (password_present and payload_radar_cmd == 0)
        ):
            radar_cmd = int(getattr(self, 'last_radar_cmd_value', 0))
        else:
            radar_cmd = payload_radar_cmd
        password_cmd = int(payload.get('password_cmd', 0))
        password = payload.get('password') if password_present else None
        self._validate_radar_cmd_transition(radar_cmd, password_present=password_present)
        self._validate_password_update_allowed(password_cmd)
        sender_id = int(payload.get('sender_id', self._effective_sender_id()))
        receiver_id = int(payload.get('receiver_id', _node_int_parameter(self, 'receiver_id', 0x8080)))
        seq = int(payload['seq']) if 'seq' in payload else self._next_seq()
        user_data = build_radar_cmd_payload(radar_cmd, password_cmd, password)
        frame = build_robot_interaction_frame(0x0121, sender_id, receiver_id, user_data, seq=seq)
        meta = {
            'kind': 'radar_cmd_0121',
            'cmd_id': 0x0301,
            'cmd_hex': '0x0301',
            'data_cmd_id': 0x0121,
            'data_cmd_hex': '0x0121',
            'sender_id': sender_id,
            'receiver_id': receiver_id,
            'radar_cmd': radar_cmd,
            'user_data_hex': user_data.hex(),
            'user_data_length': len(user_data),
            'seq': seq,
        }
        if password_present:
            meta['password_cmd'] = password_cmd
            meta['password'] = password
        meta['preserve_last_radar_cmd'] = preserve_last_radar_cmd
        return frame, meta

    @staticmethod
    def _is_raw_radar_cmd_frame(cmd_id: int, data: bytes) -> bool:
        return int(cmd_id) == 0x0301 and len(data) >= 2 and int.from_bytes(data[:2], 'little') == 0x0121

    def _allow_0305_send(self) -> bool:
        configured_rate = _node_float_parameter(self, 'radar_client_max_rate_hz', 5.0)
        max_rate_hz = min(max(float(configured_rate), 0.001), 5.0)
        now = time.monotonic()
        last = float(getattr(self, 'last_0305_tx_time', 0.0))
        min_interval = 1.0 / max_rate_hz
        if last > 0.0 and now - last < min_interval:
            self.blocked_0305_rate_count = int(getattr(self, 'blocked_0305_rate_count', 0)) + 1
            remaining = min_interval - (now - last)
            self.last_error = (
                f'blocked 0x0305 above {max_rate_hz:.1f}Hz protocol limit '
                f'({remaining:.3f}s remaining)'
            )
            if self.blocked_0305_rate_count in (1, 10) or self.blocked_0305_rate_count % 100 == 0:
                self.get_logger().warning(self.last_error)
            return False
        self.last_0305_tx_time = now
        self.blocked_0305_rate_count = 0
        return True

    def _validate_radar_cmd_transition(self, radar_cmd: int, password_present: bool = False) -> None:
        if not 0 <= int(radar_cmd) <= 0xFF:
            raise ValueError(f'radar_cmd must be in uint8 range 0..255, got {radar_cmd!r}')
        last = int(getattr(self, 'last_radar_cmd_value', 0))
        if int(radar_cmd) < last:
            raise ValueError(f'radar_cmd must not decrease: last={last}, requested={radar_cmd}')
        if int(radar_cmd) > last + 1:
            raise ValueError(f'radar_cmd must increase by exactly 1: last={last}, requested={radar_cmd}')

    def _validate_password_update_allowed(self, password_cmd: int) -> None:
        if int(password_cmd) != 1:
            return
        if not _node_bool_parameter(self, 'require_can_change_password_for_update', True):
            return
        if getattr(self, 'last_can_change_password', None) is True:
            return
        raise ValueError('password_cmd=1 blocked until referee 0x020E can_change_password=true')

    def _parse_algorithm_radar_cmd_message(self, data: str) -> dict:
        raw = str(data).strip()
        if not raw:
            raise ValueError('empty radar_cmd message')
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError:
            payload = {'radar_cmd': int(raw, 0)}
        if isinstance(payload, int):
            payload = {'radar_cmd': payload}
        if not isinstance(payload, dict):
            raise ValueError('radar_cmd message must be an integer or JSON object')
        if 'action' in payload:
            if payload.get('schema') not in {
                'transistor.radar.command.v1',
                # Keep old recordings and upstream tools replayable.
                'shark.radar.command.v1',
            }:
                raise ValueError('unsupported radar command schema')
            if payload.get('action') != 'trigger_double_vulnerability':
                raise ValueError(f"unsupported radar command action: {payload.get('action')!r}")
            request_id = str(payload.get('request_id', '')).strip()
            if not request_id or len(request_id) > 128:
                raise ValueError('semantic radar command requires a request_id up to 128 characters')
            payload['radar_cmd'] = int(getattr(self, 'last_radar_cmd_value', 0)) + 1
        if 'request_count' in payload and 'radar_cmd' not in payload:
            payload['radar_cmd'] = payload['request_count']
        return payload

    def _read_serial(self) -> None:
        if bool(self.get_parameter('dry_run').value):
            return
        data = self.serial_port.read_available()
        if not data:
            if self.serial_port.last_error:
                self.last_error = self.serial_port.last_error
            return
        self._publish_json(self.raw_rx_pub, {
            'timestamp': time.time(),
            'raw_hex': data.hex(),
            'raw_length': len(data),
            'port': str(self.get_parameter('port').value),
        })
        for frame in self.assembler.push_bytes(data):
            self._mark_valid_frame_received()
            self.rx_count += 1
            payload = self._frame_to_payload(frame)
            self.last_rx = payload
            self._handle_rx_payload(payload)
            self._publish_json(self.rx_pub, payload)
            for output in self.referee_bridge.process_frames([frame]):
                self._publish_json(self.referee_bridge_pub, output)

    def _mark_valid_frame_received(self) -> None:
        now_monotonic = time.monotonic()
        self._last_valid_frame_monotonic = now_monotonic
        self.last_valid_frame_at = time.time()
        self._frame_watchdog_armed_at = now_monotonic
        if getattr(self, '_frame_watchdog_error', ''):
            self._frame_watchdog_error = ''
            self.get_logger().info('referee frame stream recovered')

    def _check_frame_watchdog(self) -> bool:
        timeout_sec = max(_node_float_parameter(self, 'frame_timeout_sec', 2.0), 0.0)
        dry_run = _node_bool_parameter(self, 'dry_run', True)
        if timeout_sec <= 0.0 or dry_run or not bool(getattr(self.serial_port, 'is_open', False)):
            self._frame_watchdog_armed_at = None
            self._frame_watchdog_error = ''
            return False

        now = time.monotonic()
        armed_at = getattr(self, '_frame_watchdog_armed_at', None)
        if armed_at is None:
            self._frame_watchdog_armed_at = now
            return False
        last_frame = getattr(self, '_last_valid_frame_monotonic', None)
        reference = max(float(armed_at), float(last_frame)) if last_frame is not None else float(armed_at)
        age_sec = max(0.0, now - reference)

        if age_sec < timeout_sec:
            return False
        if not getattr(self, '_frame_watchdog_error', ''):
            self._frame_watchdog_error = (
                f'referee frame timeout: no valid frame received for {timeout_sec:g}s'
            )
            self.get_logger().error(self._frame_watchdog_error)
        return True

    def _frame_watchdog_status(self) -> dict:
        timeout_sec = max(_node_float_parameter(self, 'frame_timeout_sec', 2.0), 0.0)
        now = time.monotonic()
        last_frame = getattr(self, '_last_valid_frame_monotonic', None)
        armed_at = getattr(self, '_frame_watchdog_armed_at', None)
        reference = (
            max(float(armed_at), float(last_frame))
            if armed_at is not None and last_frame is not None
            else last_frame if last_frame is not None else armed_at
        )
        return {
            'frame_timeout_sec': timeout_sec,
            'frame_timed_out': bool(getattr(self, '_frame_watchdog_error', '')),
            'last_valid_frame_age_sec': (
                max(0.0, now - float(last_frame)) if last_frame is not None else None
            ),
            'frame_silence_age_sec': max(0.0, now - float(reference)) if reference is not None else None,
            'last_valid_frame_at': getattr(self, 'last_valid_frame_at', None),
            'frame_watchdog_error': str(getattr(self, '_frame_watchdog_error', '') or ''),
        }

    def _frame_to_payload(self, frame: RefereeFrame) -> dict:
        payload = frame.to_dict()
        payload.update({
            'timestamp': time.time(),
            'raw_hex': frame.raw.hex(),
            'raw_length': len(frame.raw),
        })
        return payload

    def _handle_rx_payload(self, payload: dict) -> None:
        if int(payload.get('cmd_id', 0)) == 0x0001:
            parsed = payload.get('parsed') or {}
            game_status = parsed.get('game_status') or {}
            if game_status.get('valid_game_status_length', False):
                event = dict(game_status)
                event.update({
                    'timestamp': payload.get('timestamp', time.time()),
                    'rx_seq': payload.get('seq'),
                    'rx_raw_hex': payload.get('raw_hex'),
                })
                self.last_game_status = event
                game_status_pub = getattr(self, 'game_status_pub', None)
                if game_status_pub is not None:
                    self._publish_json(game_status_pub, event)
            return
        if int(payload.get('cmd_id', 0)) == 0x0003:
            parsed = payload.get('parsed') or {}
            game_robot_hp = parsed.get('game_robot_hp') or {}
            if game_robot_hp.get('valid_game_robot_hp_length', False):
                event = dict(game_robot_hp)
                event.update({
                    'timestamp': payload.get('timestamp', time.time()),
                    'rx_seq': payload.get('seq'),
                    'rx_raw_hex': payload.get('raw_hex'),
                })
                self.last_game_robot_hp = event
            return
        if int(payload.get('cmd_id', 0)) == 0x0201:
            parsed = payload.get('parsed') or {}
            robot_status = parsed.get('robot_status') or {}
            if robot_status.get('valid_robot_status_length', False):
                self._record_robot_status(robot_status, payload)
            return
        if int(payload.get('cmd_id', 0)) != 0x020E:
            return
        parsed = payload.get('parsed') or {}
        radar_info = parsed.get('radar_decision_sync') or {}
        if not radar_info.get('valid_radar_info_length', False):
            return
        event = dict(radar_info)
        event.update({
            'timestamp': payload.get('timestamp', time.time()),
            'rx_seq': payload.get('seq'),
            'rx_raw_hex': payload.get('raw_hex'),
        })
        self.last_radar_decision_sync = event
        self.last_can_change_password = bool(radar_info.get('can_change_password', False))
        level = radar_info.get('own_encryption_level')
        if not self._observe_referee_level(level, payload):
            return
        self._maybe_switch_single_rx_to_broadcast(level, payload)
        self._update_interference_level_from_referee(level, payload)
        self._update_rx_interference_level_from_referee(level, payload)

    def _auto_detect_radio_side_enabled(self) -> bool:
        return bool(_node_bool_parameter(self, 'auto_detect_radio_side', True))

    def _effective_radio_side(self) -> str:
        if self._auto_detect_radio_side_enabled():
            detected_side = getattr(self, 'detected_radio_side', None)
            if detected_side in ('red', 'blue'):
                return str(detected_side)
        return str(self.get_parameter('radio_side').value).strip().lower()

    def _effective_sender_id(self) -> int:
        if self._auto_detect_radio_side_enabled():
            detected_sender_id = getattr(self, 'detected_radar_sender_id', None)
            if detected_sender_id is not None:
                return int(detected_sender_id)
        return _node_int_parameter(self, 'sender_id', 9)

    def _record_robot_status(self, robot_status: dict, source_payload: Optional[dict] = None) -> None:
        status = dict(robot_status)
        status['timestamp'] = time.time()
        if source_payload is not None:
            status['rx_seq'] = source_payload.get('seq')
            status['rx_raw_hex'] = source_payload.get('raw_hex')
        self.last_robot_status = status

        side = status.get('radio_side')
        robot_id = status.get('robot_id')
        if side not in ('red', 'blue'):
            return
        self.detected_robot_id = int(robot_id)
        self.detected_radio_side = str(side)
        self.detected_radar_sender_id = radar_sender_id_for_side(str(side))

    def _match_running_for_referee_level(self) -> bool:
        if not bool(_node_bool_parameter(self, 'apply_referee_level_only_when_running', False)):
            return True
        game_status = getattr(self, 'last_game_status', None) or {}
        return bool(game_status.get('match_running', False) or game_status.get('game_progress') == 4)

    def _observe_referee_level(self, level: object, source_payload: Optional[dict] = None) -> bool:
        try:
            clean_level = normalize_interference_level(int(level))
        except Exception as exc:  # noqa: BLE001
            self.last_error = f'ignore invalid 0x020E referee level {level!r}: {exc}'
            self.get_logger().warning(self.last_error)
            return False

        if not self._match_running_for_referee_level():
            self.suppressed_referee_level_before_running_count = int(
                getattr(self, 'suppressed_referee_level_before_running_count', 0)
            ) + 1
            self.last_suppressed_referee_level_before_running = {
                'timestamp': time.time(),
                'source': '0x020E',
                'level': clean_level,
                'reason': 'match_not_running',
            }
            if source_payload is not None:
                self.last_suppressed_referee_level_before_running['rx_seq'] = source_payload.get('seq')
                self.last_suppressed_referee_level_before_running['rx_raw_hex'] = source_payload.get('raw_hex')
            self.get_logger().warning(
                f'ignored 0x020E referee level {clean_level}; match is not running yet'
            )
            return False

        if getattr(self, 'referee_level_candidate', None) == clean_level:
            self.referee_level_candidate_count = int(getattr(self, 'referee_level_candidate_count', 0)) + 1
        else:
            self.referee_level_candidate = clean_level
            self.referee_level_candidate_count = 1

        required_count = max(1, _node_int_parameter(self, 'referee_level_confirm_count', 2))
        confirmed = self.referee_level_candidate_count >= required_count
        self.last_referee_level_observation = {
            'timestamp': time.time(),
            'source': '0x020E',
            'level': clean_level,
            'candidate_count': int(self.referee_level_candidate_count),
            'required_count': int(required_count),
            'confirmed': bool(confirmed),
        }
        if source_payload is not None:
            self.last_referee_level_observation['rx_seq'] = source_payload.get('seq')
            self.last_referee_level_observation['rx_raw_hex'] = source_payload.get('raw_hex')
        if not confirmed:
            return False

        previous_level = int(getattr(self, 'confirmed_referee_level', int(getattr(self, 'interference_level', 1))))
        self.confirmed_referee_level = clean_level
        self.last_confirmed_referee_level_update = {
            'timestamp': time.time(),
            'source': '0x020E',
            'previous_level': previous_level,
            'level': clean_level,
            'candidate_count': int(self.referee_level_candidate_count),
            'required_count': int(required_count),
        }
        if source_payload is not None:
            self.last_confirmed_referee_level_update['rx_seq'] = source_payload.get('seq')
            self.last_confirmed_referee_level_update['rx_raw_hex'] = source_payload.get('raw_hex')
        return True

    def _maybe_switch_single_rx_to_broadcast(self, level: object, source_payload: Optional[dict] = None) -> bool:
        if not bool(_node_bool_parameter(self, 'single_rx_auto_broadcast_after_level3', False)):
            return False
        try:
            clean_level = normalize_interference_level(int(level))
        except Exception as exc:  # noqa: BLE001
            self.last_error = f'ignore invalid 0x020E single RX level {level!r}: {exc}'
            self.get_logger().warning(self.last_error)
            return False
        if clean_level < 3:
            if (
                getattr(self, 'single_rx_mode', 'interference') == 'broadcast'
                and bool(_node_bool_parameter(self, 'single_rx_auto_allow_return_to_interference', True))
            ):
                return self._publish_single_rx_interference_switch(clean_level, source_payload=source_payload)
            return False
        if getattr(self, 'single_rx_mode', 'interference') == 'broadcast':
            return False
        return self._publish_single_rx_broadcast_switch(clean_level, source_payload=source_payload)

    def _publish_single_rx_broadcast_switch(self, level: int = 3, source_payload: Optional[dict] = None) -> bool:
        try:
            side = self._effective_radio_side()
            setters = broadcast_rx_setters_for_side(side)
        except Exception as exc:  # noqa: BLE001
            self.last_error = f'failed to map broadcast RX setters: {exc}'
            self.get_logger().warning(self.last_error)
            return False

        setters['rx_profile'] = 'broadcast'
        setters['interference_level'] = int(level)
        msg = String()
        msg.data = json.dumps(setters, ensure_ascii=False)
        self.interference_rx_control_pub.publish(msg)
        self.single_rx_mode = 'broadcast'
        self.single_rx_switch_count = int(getattr(self, 'single_rx_switch_count', 0)) + 1
        self.last_single_rx_switch = {
            'timestamp': time.time(),
            'source': '0x020E',
            'mode': 'broadcast',
            'radio_side': str(side).strip().lower(),
            'setters': dict(setters),
        }
        if source_payload is not None:
            self.last_single_rx_switch['rx_seq'] = source_payload.get('seq')
            self.last_single_rx_switch['rx_raw_hex'] = source_payload.get('raw_hex')
        self.last_error = ''
        self.get_logger().warning(
            f'0x020E level reached 3; switched single RX to broadcast setters={setters}'
        )
        return True

    def _publish_single_rx_interference_switch(self, level: int, source_payload: Optional[dict] = None) -> bool:
        try:
            side = self._effective_radio_side()
            setters = interference_rx_setters_for_side_level(side, level)
        except Exception as exc:  # noqa: BLE001
            self.last_error = f'failed to map interference RX setters: {exc}'
            self.get_logger().warning(self.last_error)
            return False

        setters['rx_profile'] = 'interference'
        setters['interference_level'] = int(level)
        msg = String()
        msg.data = json.dumps(setters, ensure_ascii=False)
        self.interference_rx_control_pub.publish(msg)
        self.single_rx_mode = 'interference'
        self.single_rx_switch_count = int(getattr(self, 'single_rx_switch_count', 0)) + 1
        self.rx_interference_level = int(level)
        self.rx_interference_update_count = int(getattr(self, 'rx_interference_update_count', 0)) + 1
        self.last_rx_interference_setters = dict(setters)
        self.last_rx_interference_update = {
            'timestamp': time.time(),
            'source': '0x020E',
            'level': int(level),
            'radio_side': str(side).strip().lower(),
            'setters': dict(setters),
        }
        self.last_single_rx_switch = {
            'timestamp': time.time(),
            'source': '0x020E',
            'mode': 'interference',
            'level': int(level),
            'radio_side': str(side).strip().lower(),
            'setters': dict(setters),
        }
        if source_payload is not None:
            self.last_rx_interference_update['rx_seq'] = source_payload.get('seq')
            self.last_rx_interference_update['rx_raw_hex'] = source_payload.get('raw_hex')
            self.last_single_rx_switch['rx_seq'] = source_payload.get('seq')
            self.last_single_rx_switch['rx_raw_hex'] = source_payload.get('raw_hex')
        self.last_error = ''
        self.get_logger().warning(
            f'0x020E level returned to {level}; switched single RX back to interference setters={setters}'
        )
        return True

    def _update_interference_level_from_referee(self, level: object, source_payload: Optional[dict] = None) -> bool:
        if not bool(_node_bool_parameter(self, 'auto_interference_level', False)):
            return False
        try:
            clean_level = normalize_interference_level(int(level))
        except Exception as exc:  # noqa: BLE001 - bad referee data should not kill the node
            self.last_error = f'ignore invalid 0x020E interference level {level!r}: {exc}'
            self.get_logger().warning(self.last_error)
            return False
        current_level = int(getattr(self, 'interference_level', 1))
        if clean_level == current_level:
            return False
        return self._publish_interference_level(clean_level, source_payload=source_payload)

    def _publish_interference_level(self, level: int, source_payload: Optional[dict] = None) -> bool:
        try:
            side = self._effective_radio_side()
            setters = interference_setters_for_side_level(side, level)
        except Exception as exc:  # noqa: BLE001 - report config errors through status
            self.last_error = f'failed to map interference level {level}: {exc}'
            self.get_logger().warning(self.last_error)
            return False

        msg = String()
        msg.data = json.dumps(setters, ensure_ascii=False)
        self.interference_control_pub.publish(msg)
        self.interference_level = int(level)
        self.interference_update_count = int(getattr(self, 'interference_update_count', 0)) + 1
        self.last_interference_setters = dict(setters)
        self.last_interference_update = {
            'timestamp': time.time(),
            'source': '0x020E',
            'level': int(level),
            'radio_side': str(side).strip().lower(),
            'setters': dict(setters),
        }
        if source_payload is not None:
            self.last_interference_update['rx_seq'] = source_payload.get('seq')
            self.last_interference_update['rx_raw_hex'] = source_payload.get('raw_hex')
        self.last_error = ''
        self.get_logger().warning(
            f'0x020E updated local interference level to {level}; setters={setters}'
        )
        return True

    def _update_rx_interference_level_from_referee(self, level: object, source_payload: Optional[dict] = None) -> bool:
        if not bool(_node_bool_parameter(self, 'auto_rx_interference_level', True)):
            return False
        try:
            clean_level = normalize_interference_level(int(level))
        except Exception as exc:  # noqa: BLE001
            self.last_error = f'ignore invalid 0x020E RX interference level {level!r}: {exc}'
            self.get_logger().warning(self.last_error)
            return False
        if (
            bool(_node_bool_parameter(self, 'single_rx_auto_broadcast_after_level3', False))
            and getattr(self, 'single_rx_mode', 'interference') == 'broadcast'
        ):
            self.suppressed_rx_interference_update_count = int(
                getattr(self, 'suppressed_rx_interference_update_count', 0)
            ) + 1
            self.last_suppressed_rx_interference_update = {
                'timestamp': time.time(),
                'source': '0x020E',
                'level': clean_level,
                'reason': 'single_rx_already_switched_to_broadcast',
            }
            if source_payload is not None:
                self.last_suppressed_rx_interference_update['rx_seq'] = source_payload.get('seq')
                self.last_suppressed_rx_interference_update['rx_raw_hex'] = source_payload.get('raw_hex')
            if (
                self.suppressed_rx_interference_update_count in (1, 10)
                or self.suppressed_rx_interference_update_count % 100 == 0
            ):
                self.get_logger().warning(
                    'ignored 0x020E interference RX retune because single RX is already in broadcast mode'
                )
            return False
        current_level = int(getattr(self, 'rx_interference_level', int(getattr(self, 'interference_level', 1))))
        if clean_level == current_level:
            return False
        return self._publish_rx_interference_level(clean_level, source_payload=source_payload)

    def _publish_rx_interference_level(self, level: int, source_payload: Optional[dict] = None) -> bool:
        try:
            side = self._effective_radio_side()
            setters = interference_rx_setters_for_side_level(side, level)
        except Exception as exc:  # noqa: BLE001
            self.last_error = f'failed to map RX interference level {level}: {exc}'
            self.get_logger().warning(self.last_error)
            return False

        setters['interference_level'] = int(level)
        msg = String()
        msg.data = json.dumps(setters, ensure_ascii=False)
        self.interference_rx_control_pub.publish(msg)
        self.rx_interference_level = int(level)
        self.rx_interference_update_count = int(getattr(self, 'rx_interference_update_count', 0)) + 1
        self.last_rx_interference_setters = dict(setters)
        self.last_rx_interference_update = {
            'timestamp': time.time(),
            'source': '0x020E',
            'level': int(level),
            'radio_side': str(side).strip().lower(),
            'setters': dict(setters),
        }
        if source_payload is not None:
            self.last_rx_interference_update['rx_seq'] = source_payload.get('seq')
            self.last_rx_interference_update['rx_raw_hex'] = source_payload.get('raw_hex')
        self.last_error = ''
        self.get_logger().warning(
            f'0x020E retuned interference RX to level {level}; setters={setters}'
        )
        return True

    def _publish_status(self) -> None:
        self._check_frame_watchdog()
        frame_watchdog = self._frame_watchdog_status()
        invincible_snapshot = self._invincible_targets_snapshot()
        invincible_snapshot.update({
            'enabled': _node_bool_parameter(self, 'auto_send_invincible_targets', True),
            'data_cmd_id': _node_int_parameter(
                self,
                'invincible_targets_data_cmd_id',
                INVINCIBLE_TARGETS_DATA_CMD_ID,
            ),
            'configured_rate_hz': min(
                max(_node_float_parameter(self, 'invincible_targets_send_rate_hz', 3.0), 0.1),
                4.0,
            ),
            'scheduled_frame_rate_hz': min(
                max(_node_float_parameter(self, 'invincible_targets_send_rate_hz', 3.0), 0.1),
                4.0,
            ) * 6.0,
            'pending_receiver_count': (
                6 - len((getattr(self, '_pending_invincible_targets_batch', None) or {}).get('results', []))
                if getattr(self, '_pending_invincible_targets_batch', None)
                else 0
            ),
            'source_count': int(getattr(self, 'invincible_targets_source_count', 0)),
            'batch_count': int(getattr(self, 'invincible_targets_batch_count', 0)),
            'frame_request_count': int(
                getattr(self, 'invincible_targets_frame_request_count', 0)
            ),
            'frame_success_count': int(
                getattr(self, 'invincible_targets_frame_success_count', 0)
            ),
            'frame_dry_run_count': int(
                getattr(self, 'invincible_targets_frame_dry_run_count', 0)
            ),
            'source_received_at': getattr(self, 'last_invincible_source_received_at', None),
            'last_batch': getattr(self, 'last_invincible_targets_batch', None),
        })
        raw_error = str(self.last_error or '')
        cooldown_prefix = 'suppressed password verification during '
        if raw_error.startswith(cooldown_prefix):
            # Compatibility with a node that retained the old classification:
            # protocol cooldown suppression is operational state, not a fault.
            self.last_password_verify_suppression = raw_error
            raw_error = ''
        effective_error = str(raw_error or frame_watchdog['frame_watchdog_error'])
        payload = {
            'node': self.get_name(),
            'port': str(self.get_parameter('port').value),
            'baudrate': int(self.get_parameter('baudrate').value),
            'dry_run': bool(self.get_parameter('dry_run').value),
            'bridge_topic': str(self.get_parameter('bridge_topic').value),
            'bridge_topics': self._parse_topic_list(str(self.get_parameter('bridge_topic').value)),
            'sender_id': int(self.get_parameter('sender_id').value),
            'receiver_id': int(self.get_parameter('receiver_id').value),
            'auto_send_radar_cmd': bool(self.get_parameter('auto_send_radar_cmd').value),
            'password_verify_suppressed_count': int(getattr(self, 'password_verify_suppressed_count', 0)),
            'last_password_verify_suppression': str(
                getattr(self, 'last_password_verify_suppression', '') or ''
            ),
            'require_can_change_password_for_update': bool(
                _node_bool_parameter(self, 'require_can_change_password_for_update', True)
            ),
            'last_can_change_password': getattr(self, 'last_can_change_password', None),
            'radar_client_max_rate_hz': min(
                max(_node_float_parameter(self, 'radar_client_max_rate_hz', 5.0), 0.001),
                5.0,
            ),
            'blocked_0305_rate_count': int(getattr(self, 'blocked_0305_rate_count', 0)),
            'invincible_targets': invincible_snapshot,
            'auto_interference_level': bool(self.get_parameter('auto_interference_level').value),
            'auto_rx_interference_level': bool(_node_bool_parameter(self, 'auto_rx_interference_level', True)),
            'apply_referee_level_only_when_running': bool(
                _node_bool_parameter(self, 'apply_referee_level_only_when_running', False)
            ),
            'single_rx_auto_broadcast_after_level3': bool(
                _node_bool_parameter(self, 'single_rx_auto_broadcast_after_level3', False)
            ),
            'single_rx_auto_allow_return_to_interference': bool(
                _node_bool_parameter(self, 'single_rx_auto_allow_return_to_interference', True)
            ),
            'single_rx_mode': str(getattr(self, 'single_rx_mode', 'interference')),
            'single_rx_switch_count': int(getattr(self, 'single_rx_switch_count', 0)),
            'referee_level_candidate': getattr(self, 'referee_level_candidate', None),
            'referee_level_candidate_count': int(getattr(self, 'referee_level_candidate_count', 0)),
            'confirmed_referee_level': int(
                getattr(self, 'confirmed_referee_level', int(getattr(self, 'interference_level', 1)))
            ),
            'radio_side': str(self.get_parameter('radio_side').value),
            'auto_detect_radio_side': bool(_node_bool_parameter(self, 'auto_detect_radio_side', True)),
            'effective_radio_side': self._effective_radio_side(),
            'effective_sender_id': self._effective_sender_id(),
            'detected_robot_id': getattr(self, 'detected_robot_id', None),
            'detected_radio_side': getattr(self, 'detected_radio_side', None),
            'detected_radar_sender_id': getattr(self, 'detected_radar_sender_id', None),
            'interference_level': int(getattr(self, 'interference_level', 1)),
            'rx_interference_level': int(
                getattr(self, 'rx_interference_level', int(getattr(self, 'interference_level', 1)))
            ),
            'interference_control_topic': str(self.get_parameter('interference_control_topic').value),
            'interference_rx_control_topic': str(self.get_parameter('interference_rx_control_topic').value),
            'interference_update_count': int(getattr(self, 'interference_update_count', 0)),
            'rx_interference_update_count': int(getattr(self, 'rx_interference_update_count', 0)),
            'suppressed_rx_interference_update_count': int(
                getattr(self, 'suppressed_rx_interference_update_count', 0)
            ),
            'suppressed_referee_level_before_running_count': int(
                getattr(self, 'suppressed_referee_level_before_running_count', 0)
            ),
            'tx_count': self.tx_count,
            'rx_count': self.rx_count,
            'dry_run_count': self.dry_run_count,
            'last_radar_cmd_value': int(getattr(self, 'last_radar_cmd_value', 0)),
            'algorithm_request_cache_size': len(getattr(self, 'algorithm_request_acks', {})),
            'serial_open': self.serial_port.is_open,
            'require_serial_open_on_start': bool(
                self.get_parameter('require_serial_open_on_start').value
            ),
            'serial_open_count': int(getattr(self.serial_port, 'open_count', 0)),
            'serial_close_count': int(getattr(self.serial_port, 'close_count', 0)),
            'last_error': effective_error,
        }
        payload.update(frame_watchdog)
        payload.update(self._password_verify_status())
        if getattr(self.serial_port, 'last_open_time', 0.0):
            payload['serial_last_open_time'] = float(self.serial_port.last_open_time)
        if getattr(self.serial_port, 'last_close_time', 0.0):
            payload['serial_last_close_time'] = float(self.serial_port.last_close_time)
        if self.last_tx is not None:
            payload['last_tx'] = self.last_tx
        if getattr(self, 'last_algorithm_request', None) is not None:
            payload['last_algorithm_request'] = self.last_algorithm_request
        if getattr(self, 'last_algorithm_ack', None) is not None:
            payload['last_algorithm_ack'] = self.last_algorithm_ack
        if self.last_rx is not None:
            payload['last_rx'] = self.last_rx
        if self.last_game_status is not None:
            payload['last_game_status'] = self.last_game_status
        if self.last_game_robot_hp is not None:
            payload['last_game_robot_hp'] = self.last_game_robot_hp
        if self.last_robot_status is not None:
            payload['last_robot_status'] = self.last_robot_status
        if self.last_radar_decision_sync is not None:
            payload['last_radar_decision_sync'] = self.last_radar_decision_sync
        if self.last_referee_level_observation is not None:
            payload['last_referee_level_observation'] = self.last_referee_level_observation
        if self.last_confirmed_referee_level_update is not None:
            payload['last_confirmed_referee_level_update'] = self.last_confirmed_referee_level_update
        if self.last_interference_update is not None:
            payload['last_interference_update'] = self.last_interference_update
        if self.last_interference_setters is not None:
            payload['last_interference_setters'] = self.last_interference_setters
        if self.last_rx_interference_update is not None:
            payload['last_rx_interference_update'] = self.last_rx_interference_update
        if self.last_rx_interference_setters is not None:
            payload['last_rx_interference_setters'] = self.last_rx_interference_setters
        if self.last_suppressed_rx_interference_update is not None:
            payload['last_suppressed_rx_interference_update'] = self.last_suppressed_rx_interference_update
        if self.last_suppressed_referee_level_before_running is not None:
            payload['last_suppressed_referee_level_before_running'] = (
                self.last_suppressed_referee_level_before_running
            )
        if self.last_single_rx_switch is not None:
            payload['last_single_rx_switch'] = self.last_single_rx_switch
        self._publish_json(self.status_pub, payload)

    @staticmethod
    def _publish_json(pub, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        pub.publish(msg)

    def destroy_node(self) -> bool:
        self.serial_port.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node: Optional[RefereeSerialNode] = None
    try:
        node = RefereeSerialNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
