from __future__ import annotations

import json
import socket
import threading
import time
import uuid
from collections import deque
from copy import deepcopy
from typing import Callable, Optional


RADAR_COMMAND_SCHEMA = 'transistor.radar.command.v1'


def normalize_transport_mode(value: object) -> str:
    mode = str(value or 'radio_ros').strip().lower()
    if mode not in ('radio_ros', 'legacy_serial'):
        raise ValueError(f'裁判通信模式必须是 radio_ros 或 legacy_serial，当前为 {value!r}')
    return mode


class RefereeTransport:
    @staticmethod
    def create(
        mode: str,
        side: str,
        on_referee_message: Optional[Callable] = None,
        radio_config: Optional[dict] = None,
    ):
        clean_mode = normalize_transport_mode(mode)
        if clean_mode == 'radio_ros':
            return RadioRosTransport(
                side=side,
                on_referee_message=on_referee_message,
                radio_config=radio_config,
            )
        return LegacySerialTransport(side=side)


class LegacySerialTransport:
    mode = 'legacy_serial'

    def __init__(self, side: str):
        self.side = str(side)

    def start(self) -> None:
        return None

    def close(self) -> None:
        return None

    def snapshot(self) -> dict:
        return {'mode': self.mode, 'connected': True}


class RadioRosTransport:
    """ROS2 JSON transport. It never imports pyserial or constructs referee frames."""

    mode = 'radio_ros'

    @staticmethod
    def _spin_node(node) -> None:
        """Run the executor without printing expected shutdown tracebacks."""
        try:
            import rclpy
            from rclpy.executors import ExternalShutdownException

            rclpy.spin(node)
        except (ExternalShutdownException, KeyboardInterrupt):
            pass

    def __init__(
        self,
        side: str,
        on_referee_message: Optional[Callable] = None,
        radio_config: Optional[dict] = None,
    ):
        self.side = str(side)
        self.on_referee_message = on_referee_message
        config = dict(radio_config or {})
        self._bridge_backend = str(config.get('bridge_backend', 'direct_ros')).strip().lower()
        if self._bridge_backend not in ('udp', 'direct_ros'):
            raise ValueError(
                'referee.bridge_backend must be udp or direct_ros, '
                f'got {self._bridge_backend!r}'
            )
        self._udp_bind_host = str(config.get('vision_bind_host', '127.0.0.1'))
        self._udp_bind_port = int(config.get('vision_bind_port', 37601))
        self._udp_bridge_host = str(config.get('bridge_host', '127.0.0.1'))
        self._udp_bridge_port = int(config.get('bridge_port', 37602))
        self._lock = threading.Lock()
        self._node = None
        self._spin_thread = None
        self._owns_rclpy = False
        self._telemetry_publisher = None
        self._command_publisher = None
        self._string_type = None
        self._udp_socket = None
        self._udp_stop = threading.Event()
        self._last_bridge_heartbeat_at: Optional[float] = None
        self._last_bridge_status: Optional[dict] = None
        self._pending: dict[str, dict] = {}
        self._successful = deque()
        self._strategy: dict[str, dict] = {}
        self._last_ack: Optional[dict] = None
        self._last_map_ack: Optional[dict] = None
        self._last_status: Optional[dict] = None
        self._last_status_received_at: Optional[float] = None
        self._last_referee_packet_at: Optional[float] = None
        self._last_error = ''

    def start(self) -> None:
        if self._node is not None:
            return
        if self._bridge_backend == 'udp':
            self._start_udp_bridge()
            return
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import String

            if not rclpy.ok():
                rclpy.init(args=None)
                self._owns_rclpy = True
            node = Node('transistor_vision_radar_transport')
            self._string_type = String
            self._telemetry_publisher = node.create_publisher(
                String, '/rm_radar_algorithm/telemetry', 10
            )
            self._command_publisher = node.create_publisher(
                String, '/rm_radar_algorithm/radar_cmd', 10
            )
            node.create_subscription(
                String, '/rm_referee_serial_node/referee_bridge', self._on_bridge_message, 10
            )
            node.create_subscription(
                String, '/rm_referee_serial_node/tx_frames', self._on_tx_message, 10
            )
            node.create_subscription(
                String, '/rm_referee_serial_node/status', self._on_status_message, 10
            )
            self._node = node
            self._spin_thread = threading.Thread(
                target=self._spin_node, args=(node,), name='radar-radio-ros', daemon=True
            )
            self._spin_thread.start()
            self._last_error = ''
        except Exception as exc:
            self._last_error = str(exc)
            self.close()
            raise RuntimeError(f'ROS2 无线电裁判通信启动失败: {exc}') from exc

    def _start_udp_bridge(self) -> None:
        """Bridge the Python 3.12 vision process to ROS2 Humble's Python 3.10."""
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            udp_socket.bind((self._udp_bind_host, self._udp_bind_port))
            udp_socket.settimeout(0.2)
        except Exception:
            udp_socket.close()
            raise
        self._udp_socket = udp_socket
        # Keep the existing connected contract: the local transport is ready
        # even before the ROS sidecar sends its first heartbeat.
        self._node = 'udp_bridge'
        self._udp_stop.clear()
        self._spin_thread = threading.Thread(
            target=self._udp_receive_loop,
            name='radar-radio-udp',
            daemon=True,
        )
        self._spin_thread.start()
        self._last_error = ''

    def _udp_receive_loop(self) -> None:
        udp_socket = self._udp_socket
        if udp_socket is None:
            return
        while not self._udp_stop.is_set():
            try:
                raw, _ = udp_socket.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                if not self._udp_stop.is_set():
                    self._last_error = '无线电 UDP 桥接接收中断'
                break
            try:
                envelope = json.loads(raw.decode('utf-8'))
                if not isinstance(envelope, dict):
                    raise ValueError('envelope must be a JSON object')
                channel = str(envelope.get('channel', ''))
                payload = envelope.get('payload')
                if not isinstance(payload, dict):
                    raise ValueError('payload must be a JSON object')
                if channel == 'referee_bridge':
                    self._process_bridge_payload(payload)
                elif channel == 'tx_frames':
                    self._process_tx_payload(payload)
                elif channel == 'status':
                    self._process_status_payload(payload)
                elif channel == 'bridge_status':
                    with self._lock:
                        self._last_bridge_status = deepcopy(payload)
                        self._last_bridge_heartbeat_at = time.time()
                else:
                    raise ValueError(f'unknown channel {channel!r}')
            except Exception as exc:
                self._last_error = f'无线电 UDP 桥接数据无效: {exc}'

    def _publish_udp(self, channel: str, payload: dict) -> None:
        if self._udp_socket is None:
            raise RuntimeError('radio UDP bridge has not been started')
        envelope = json.dumps(
            {'channel': channel, 'payload': payload},
            ensure_ascii=False,
            separators=(',', ':'),
        ).encode('utf-8')
        self._udp_socket.sendto(envelope, (self._udp_bridge_host, self._udp_bridge_port))

    @staticmethod
    def _json_payload(message) -> dict:
        payload = json.loads(message.data)
        if not isinstance(payload, dict):
            raise ValueError('ROS2 payload must be a JSON object')
        return payload

    def _on_bridge_message(self, message) -> None:
        try:
            self._process_bridge_payload(self._json_payload(message))
        except Exception as exc:
            self._last_error = f'裁判 ROS 桥接数据无效: {exc}'

    def _process_bridge_payload(self, payload: dict) -> None:
        output_type = str(payload.get('type', ''))
        with self._lock:
            if output_type:
                self._strategy[output_type] = deepcopy(payload)
            self._last_referee_packet_at = time.time()
        if self.on_referee_message is not None:
            self.on_referee_message(output_type, deepcopy(payload.get('payload') or {}))

    def _on_tx_message(self, message) -> None:
        try:
            self._process_tx_payload(self._json_payload(message))
        except Exception as exc:
            self._last_error = f'裁判 ROS ACK 无效: {exc}'

    def _process_tx_payload(self, payload: dict) -> None:
        try:
            cmd_id = int(payload.get('cmd_id', -1))
        except (TypeError, ValueError):
            cmd_id = -1
        if cmd_id == 0x0305 or str(payload.get('cmd_hex', '')).lower() == '0x0305':
            with self._lock:
                self._last_map_ack = deepcopy(payload)
        request_id = str(payload.get('request_id', '')).strip()
        if payload.get('source') != 'algorithm_radar_cmd_topic' or not request_id:
            return
        with self._lock:
            self._last_ack = deepcopy(payload)
            pending = self._pending.get(request_id)
            if pending is None:
                return
            pending['ack'] = deepcopy(payload)
            pending['ack_at'] = time.time()
            if payload.get('written') is True:
                self._successful.append(deepcopy(payload))
            self._pending.pop(request_id, None)

    def _on_status_message(self, message) -> None:
        try:
            self._process_status_payload(self._json_payload(message))
        except Exception as exc:
            self._last_error = f'裁判串口状态无效: {exc}'

    def _process_status_payload(self, payload: dict) -> None:
        with self._lock:
            self._last_status = deepcopy(payload)
            self._last_status_received_at = time.time()

    def _publish_json(self, publisher, payload: dict) -> None:
        if publisher is None or self._string_type is None:
            raise RuntimeError('radio_ros transport has not been started')
        message = self._string_type()
        message.data = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
        publisher.publish(message)

    def publish_telemetry(self, telemetry: dict) -> None:
        if self._bridge_backend == 'udp':
            self._publish_udp('telemetry', telemetry)
        else:
            self._publish_json(self._telemetry_publisher, telemetry)

    def request_double_vulnerability(self, request_id: Optional[str] = None) -> Optional[str]:
        with self._lock:
            if self._pending:
                return None
        clean_id = str(request_id or uuid.uuid4()).strip()
        payload = {
            'schema': RADAR_COMMAND_SCHEMA,
            'action': 'trigger_double_vulnerability',
            'request_id': clean_id,
        }
        with self._lock:
            self._pending[clean_id] = {'requested_at': time.time(), 'payload': payload}
        try:
            if self._bridge_backend == 'udp':
                self._publish_udp('radar_cmd', payload)
            else:
                self._publish_json(self._command_publisher, payload)
        except Exception:
            with self._lock:
                self._pending.pop(clean_id, None)
            raise
        return clean_id

    def consume_successful_requests(self) -> list[dict]:
        with self._lock:
            values = list(self._successful)
            self._successful.clear()
        return values

    def snapshot(self) -> dict:
        with self._lock:
            status = deepcopy(self._last_status)
            now = time.time()
            status_age = (
                max(0.0, now - self._last_status_received_at)
                if self._last_status_received_at is not None
                else None
            )
            referee_status_error = str((status or {}).get('last_error') or '').strip()
            bridge_age = (
                max(0.0, now - self._last_bridge_heartbeat_at)
                if self._last_bridge_heartbeat_at is not None
                else None
            )
            if self._bridge_backend == 'udp':
                radio_online = bridge_age is not None and bridge_age <= 2.5
            else:
                radio_online = status_age is not None and status_age <= 2.0
            return {
                'mode': self.mode,
                'connected': self._node is not None,
                'bridge_backend': self._bridge_backend,
                'radio_online': radio_online,
                'bridge_status_age_sec': bridge_age,
                'bridge_status': deepcopy(self._last_bridge_status),
                'radio_status_age_sec': status_age,
                'last_referee_packet_at': self._last_referee_packet_at,
                'pending_requests': deepcopy(self._pending),
                'last_ack': deepcopy(self._last_ack),
                'last_map_ack': deepcopy(self._last_map_ack),
                'referee_status': status,
                'strategy': deepcopy(self._strategy),
                'last_error': self._last_error or referee_status_error,
            }

    def close(self) -> None:
        node = self._node
        self._node = None
        self._udp_stop.set()
        udp_socket = self._udp_socket
        self._udp_socket = None
        if udp_socket is not None:
            try:
                udp_socket.close()
            except OSError:
                pass
        spin_thread = self._spin_thread
        try:
            import rclpy

            if self._owns_rclpy and rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        if node is not None and node != 'udp_bridge':
            try:
                node.destroy_node()
            except Exception:
                pass
        self._spin_thread = None
        if (
            spin_thread is not None
            and spin_thread is not threading.current_thread()
            and spin_thread.is_alive()
        ):
            spin_thread.join(timeout=1.0)
