from __future__ import annotations

import json
import secrets
import string
import time
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_PASSWORD_ALPHABET = ''.join(
    ch for ch in string.ascii_letters + string.digits if ch not in '0O1Il'
)


def generate_password(length: int = 6, alphabet: str = DEFAULT_PASSWORD_ALPHABET) -> str:
    clean_length = int(length)
    if clean_length != 6:
        raise ValueError('radar password length must be exactly 6')
    clean_alphabet = ''.join(dict.fromkeys(str(alphabet)))
    if len(clean_alphabet) < 16:
        raise ValueError('password alphabet must contain at least 16 unique characters')
    if any(not ch.isascii() or not ch.isalnum() for ch in clean_alphabet):
        raise ValueError('password alphabet must contain only ASCII letters or digits')
    return ''.join(secrets.choice(clean_alphabet) for _ in range(clean_length))


class AutoPasswordNode(Node):
    """Set a new radar password when referee 0x020E says password changes are allowed."""

    def __init__(self):
        super().__init__('rm_auto_password_node')
        self.declare_parameter('enabled', True)
        self.declare_parameter('status_topic', '/rm_referee_serial_node/status')
        self.declare_parameter('send_topic', '/rm_referee_serial_node/send_radar_cmd')
        self.declare_parameter('alphabet', DEFAULT_PASSWORD_ALPHABET)
        self.declare_parameter('period_sec', 0.5)
        self.declare_parameter('min_interval_sec', 5.0)

        self.sent_count = 0
        self.last_error = ''
        self.last_password = ''
        self.last_send_time = 0.0
        self.last_allowed: Optional[bool] = None
        self.pending_allowed_edge = False

        status_topic = str(self.get_parameter('status_topic').value)
        send_topic = str(self.get_parameter('send_topic').value)
        self.send_pub = self.create_publisher(String, send_topic, 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.create_subscription(String, status_topic, self._handle_referee_status, 10)
        self.create_timer(max(float(self.get_parameter('period_sec').value), 0.1), self._publish_status)
        self.get_logger().info(f'auto password status_topic={status_topic} send_topic={send_topic}')

    def _handle_referee_status(self, msg: String) -> None:
        if not bool(self.get_parameter('enabled').value):
            return
        try:
            payload = json.loads(msg.data)
        except Exception as exc:  # noqa: BLE001
            self.last_error = f'invalid referee status JSON: {exc}'
            return
        allowed_raw = payload.get('last_can_change_password')
        allowed = allowed_raw is True
        if allowed and self.last_allowed is not True:
            self.pending_allowed_edge = True
        self.last_allowed = allowed
        if allowed and self.pending_allowed_edge:
            self._send_new_password()

    def _send_new_password(self) -> None:
        now = time.monotonic()
        min_interval = max(float(self.get_parameter('min_interval_sec').value), 0.0)
        if self.last_send_time > 0.0 and now - self.last_send_time < min_interval:
            return
        try:
            password = generate_password(6, str(self.get_parameter('alphabet').value))
        except Exception as exc:  # noqa: BLE001
            self.last_error = f'password generation failed: {exc}'
            self.get_logger().warning(self.last_error)
            return

        msg = String()
        msg.data = json.dumps(
            {
                'password_cmd': 1,
                'password': password,
                'source': 'rm_auto_password_node',
            },
            ensure_ascii=False,
        )
        self.send_pub.publish(msg)
        self.sent_count += 1
        self.last_password = password
        self.last_send_time = now
        self.pending_allowed_edge = False
        self.last_error = ''
        self.get_logger().warning('sent automatic radar password update')

    def _publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                'node': self.get_name(),
                'enabled': bool(self.get_parameter('enabled').value),
                'sent_count': self.sent_count,
                'last_allowed': self.last_allowed,
                'pending_allowed_edge': self.pending_allowed_edge,
                'last_password': self.last_password,
                'last_error': self.last_error,
            },
            ensure_ascii=False,
        )
        self.status_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node: Optional[AutoPasswordNode] = None
    try:
        node = AutoPasswordNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
