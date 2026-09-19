from __future__ import annotations

import json
import socket
import time
from typing import Callable

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


BRIDGE_STATUS_SCHEMA = 'transistor.radio.bridge-status.v1'


class VisionUdpBridgeNode(Node):
    """Bridge the vision venv to ROS2 without sharing a Python ABI.

    ROS2 Humble is installed for Python 3.10 while the vision process uses a
    Python 3.12 virtual environment.  Localhost UDP keeps that boundary small:
    datagrams contain one JSON envelope and never leave the host by default.
    """

    def __init__(self) -> None:
        super().__init__('transistor_vision_udp_bridge')
        self.declare_parameter('listen_host', '127.0.0.1')
        self.declare_parameter('listen_port', 37602)
        self.declare_parameter('vision_host', '127.0.0.1')
        self.declare_parameter('vision_port', 37601)
        self.declare_parameter('telemetry_topic', '/rm_radar_algorithm/telemetry')
        self.declare_parameter('radar_cmd_topic', '/rm_radar_algorithm/radar_cmd')
        self.declare_parameter('referee_bridge_topic', '/rm_referee_serial_node/referee_bridge')
        self.declare_parameter('referee_tx_topic', '/rm_referee_serial_node/tx_frames')
        self.declare_parameter('referee_status_topic', '/rm_referee_serial_node/status')
        self.declare_parameter('broadcast_status_topic', '/rm_gfsk_node/status')
        self.declare_parameter('interference_status_topic', '/rm_gfsk_interference_node/status')
        self.declare_parameter('fusion_status_topic', '/rm_radar_integration/status')

        self.listen_address = (
            str(self.get_parameter('listen_host').value),
            int(self.get_parameter('listen_port').value),
        )
        self.vision_address = (
            str(self.get_parameter('vision_host').value),
            int(self.get_parameter('vision_port').value),
        )
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(self.listen_address)
        self.socket.setblocking(False)

        self.telemetry_pub = self.create_publisher(
            String, str(self.get_parameter('telemetry_topic').value), 10
        )
        self.radar_cmd_pub = self.create_publisher(
            String, str(self.get_parameter('radar_cmd_topic').value), 10
        )
        self.create_subscription(
            String,
            str(self.get_parameter('referee_bridge_topic').value),
            self._forward_to_vision('referee_bridge'),
            10,
        )
        self.module_status: dict[str, dict] = {}
        self.module_status_at: dict[str, float] = {}
        for name, parameter in (
            ('broadcast_rx', 'broadcast_status_topic'),
            ('interference_rx', 'interference_status_topic'),
            ('fusion', 'fusion_status_topic'),
        ):
            self.create_subscription(
                String,
                str(self.get_parameter(parameter).value),
                self._cache_module_status(name),
                10,
            )
        self.create_subscription(
            String,
            str(self.get_parameter('referee_tx_topic').value),
            self._forward_to_vision('tx_frames'),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('referee_status_topic').value),
            self._forward_to_vision('status'),
            10,
        )
        self.received_counts = {'telemetry': 0, 'radar_cmd': 0}
        self.forwarded_counts = {
            'referee_bridge': 0,
            'tx_frames': 0,
            'status': 0,
        }
        self.last_error = ''
        self.create_timer(0.01, self._receive_tick)
        self.create_timer(1.0, self._heartbeat_tick)
        self.get_logger().info(
            f'vision UDP bridge listening on {self.listen_address[0]}:{self.listen_address[1]} '
            f'and forwarding to {self.vision_address[0]}:{self.vision_address[1]}'
        )

    @staticmethod
    def _decode_json_object(raw: bytes) -> dict:
        payload = json.loads(raw.decode('utf-8'))
        if not isinstance(payload, dict):
            raise ValueError('UDP datagram must contain a JSON object')
        return payload

    @staticmethod
    def _ros_message(payload: dict) -> String:
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
        return message

    def _send_to_vision(self, channel: str, payload: dict) -> None:
        envelope = json.dumps(
            {'channel': channel, 'payload': payload},
            ensure_ascii=False,
            separators=(',', ':'),
        ).encode('utf-8')
        self.socket.sendto(envelope, self.vision_address)

    def _forward_to_vision(self, channel: str) -> Callable[[String], None]:
        def callback(message: String) -> None:
            try:
                payload = self._decode_json_object(message.data.encode('utf-8'))
                self._send_to_vision(channel, payload)
                self.forwarded_counts[channel] += 1
                self.last_error = ''
            except Exception as exc:  # noqa: BLE001 - report malformed ROS payloads
                self.last_error = f'{channel} forward failed: {exc}'
                self.get_logger().warning(self.last_error)

        return callback

    def _cache_module_status(self, name: str) -> Callable[[String], None]:
        def callback(message: String) -> None:
            try:
                self.module_status[name] = self._decode_json_object(message.data.encode('utf-8'))
                self.module_status_at[name] = time.monotonic()
            except Exception as exc:  # noqa: BLE001
                self.last_error = f'{name} status invalid: {exc}'

        return callback

    def _receive_tick(self) -> None:
        # Drain a bounded batch so a telemetry burst cannot starve ROS timers.
        for _ in range(64):
            try:
                raw, _ = self.socket.recvfrom(65535)
            except BlockingIOError:
                return
            except OSError as exc:
                self.last_error = f'UDP receive failed: {exc}'
                return
            try:
                envelope = self._decode_json_object(raw)
                channel = str(envelope.get('channel', ''))
                payload = envelope.get('payload')
                if not isinstance(payload, dict):
                    raise ValueError('envelope payload must be a JSON object')
                if channel == 'telemetry':
                    self.telemetry_pub.publish(self._ros_message(payload))
                elif channel == 'radar_cmd':
                    self.radar_cmd_pub.publish(self._ros_message(payload))
                else:
                    raise ValueError(f'unsupported vision channel {channel!r}')
                self.received_counts[channel] += 1
                self.last_error = ''
            except Exception as exc:  # noqa: BLE001 - keep bridge alive and observable
                self.last_error = f'invalid vision datagram: {exc}'
                self.get_logger().warning(self.last_error)

    def _heartbeat_tick(self) -> None:
        now_monotonic = time.monotonic()
        self._send_to_vision('bridge_status', {
            'schema': BRIDGE_STATUS_SCHEMA,
            'timestamp': time.time(),
            'listen_address': f'{self.listen_address[0]}:{self.listen_address[1]}',
            'vision_address': f'{self.vision_address[0]}:{self.vision_address[1]}',
            'received_counts': dict(self.received_counts),
            'forwarded_counts': dict(self.forwarded_counts),
            'modules': {
                name: {
                    'age_sec': max(0.0, now_monotonic - self.module_status_at[name]),
                    'status': status,
                }
                for name, status in self.module_status.items()
            },
            'last_error': self.last_error,
        })

    def destroy_node(self) -> bool:
        try:
            self.socket.close()
        except OSError:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionUdpBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
