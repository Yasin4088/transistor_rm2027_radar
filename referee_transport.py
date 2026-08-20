from __future__ import annotations

import json
import threading
import time
import uuid
from collections import deque
from copy import deepcopy
from typing import Callable, Optional


RADAR_COMMAND_SCHEMA = 'shark.radar.command.v1'


def normalize_transport_mode(value: object) -> str:
    mode = str(value or 'radio_ros').strip().lower()
    if mode not in ('radio_ros', 'legacy_serial'):
        raise ValueError(f'裁判通信模式必须是 radio_ros 或 legacy_serial，当前为 {value!r}')
    return mode


class RefereeTransport:
    @staticmethod
    def create(mode: str, side: str, on_referee_message: Optional[Callable] = None):
        clean_mode = normalize_transport_mode(mode)
        if clean_mode == 'radio_ros':
            return RadioRosTransport(side=side, on_referee_message=on_referee_message)
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

    def __init__(self, side: str, on_referee_message: Optional[Callable] = None):
        self.side = str(side)
        self.on_referee_message = on_referee_message
        self._lock = threading.Lock()
        self._node = None
        self._spin_thread = None
        self._owns_rclpy = False
        self._telemetry_publisher = None
        self._command_publisher = None
        self._string_type = None
        self._pending: dict[str, dict] = {}
        self._successful = deque()
        self._strategy: dict[str, dict] = {}
        self._last_ack: Optional[dict] = None
        self._last_status: Optional[dict] = None
        self._last_status_received_at: Optional[float] = None
        self._last_referee_packet_at: Optional[float] = None
        self._last_error = ''

    def start(self) -> None:
        if self._node is not None:
            return
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import String

            if not rclpy.ok():
                rclpy.init(args=None)
                self._owns_rclpy = True
            node = Node('shark_vision_radar_transport')
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

    @staticmethod
    def _json_payload(message) -> dict:
        payload = json.loads(message.data)
        if not isinstance(payload, dict):
            raise ValueError('ROS2 payload must be a JSON object')
        return payload

    def _on_bridge_message(self, message) -> None:
        try:
            payload = self._json_payload(message)
            output_type = str(payload.get('type', ''))
            with self._lock:
                if output_type:
                    self._strategy[output_type] = deepcopy(payload)
                self._last_referee_packet_at = time.time()
            if self.on_referee_message is not None:
                self.on_referee_message(output_type, deepcopy(payload.get('payload') or {}))
        except Exception as exc:
            self._last_error = f'裁判 ROS 桥接数据无效: {exc}'

    def _on_tx_message(self, message) -> None:
        try:
            self._process_tx_payload(self._json_payload(message))
        except Exception as exc:
            self._last_error = f'裁判 ROS ACK 无效: {exc}'

    def _process_tx_payload(self, payload: dict) -> None:
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
            payload = self._json_payload(message)
            with self._lock:
                self._last_status = deepcopy(payload)
                self._last_status_received_at = time.time()
        except Exception as exc:
            self._last_error = f'裁判串口状态无效: {exc}'

    def _publish_json(self, publisher, payload: dict) -> None:
        if publisher is None or self._string_type is None:
            raise RuntimeError('radio_ros transport has not been started')
        message = self._string_type()
        message.data = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
        publisher.publish(message)

    def publish_telemetry(self, telemetry: dict) -> None:
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
            return {
                'mode': self.mode,
                'connected': self._node is not None,
                'radio_online': status_age is not None and status_age <= 2.0,
                'radio_status_age_sec': status_age,
                'last_referee_packet_at': self._last_referee_packet_at,
                'pending_requests': deepcopy(self._pending),
                'last_ack': deepcopy(self._last_ack),
                'referee_status': status,
                'strategy': deepcopy(self._strategy),
                'last_error': self._last_error or referee_status_error,
            }

    def close(self) -> None:
        node = self._node
        self._node = None
        try:
            import rclpy

            if self._owns_rclpy and rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        thread = self._spin_thread
        self._spin_thread = None
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
