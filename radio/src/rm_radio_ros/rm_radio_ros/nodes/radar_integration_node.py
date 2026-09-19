from __future__ import annotations

import json
import time
from collections import deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ..core.double_vulnerability import (
    AutoDoubleVulnerabilityDecision,
    DoubleVulnerabilityConfig,
    TriggerZoneMask,
)
from ..core.radar_integration import RadarFusionState


def _default_trigger_mask_path() -> str:
    filename = 'double_vulnerability_trigger_mask.npz'
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = Path(get_package_share_directory('rm_radio_ros')) / 'masks' / filename
        if installed.exists():
            return str(installed)
    except Exception:  # noqa: BLE001 - source-tree tests do not require ament
        pass
    return str(Path(__file__).resolve().parents[2] / 'assets' / filename)


class RadarIntegrationNode(Node):
    """Fuse positions, send 0x0305, and make radio-side radar decisions."""

    def __init__(self):
        super().__init__('rm_radar_integration')
        self.declare_parameter('radio_side', 'red')
        self.declare_parameter('auto_sync_side', True)
        self.declare_parameter('telemetry_topic', '/rm_radar_algorithm/telemetry')
        self.declare_parameter(
            'bridge_topics',
            '/rm_gfsk_node/referee_bridge,/rm_gfsk_interference_node/referee_bridge,'
            '/rm_referee_serial_node/referee_bridge',
        )
        self.declare_parameter('referee_status_topic', '/rm_referee_serial_node/status')
        self.declare_parameter('referee_tx_topic', '/rm_referee_serial_node/tx_frames')
        self.declare_parameter('send_frame_topic', '/rm_referee_serial_node/send_frame')
        self.declare_parameter('send_rate_hz', 4.8)
        self.declare_parameter('state_rate_hz', 20.0)
        self.declare_parameter('vision_timeout_sec', 0.5)
        self.declare_parameter('radio_timeout_sec', 1.0)
        self.declare_parameter('radio_ghost_sec', 2.0)
        self.declare_parameter('radar_cmd_topic', '/rm_radar_algorithm/radar_cmd')
        self.declare_parameter('auto_double_vulnerability_enabled', False)
        self.declare_parameter('auto_double_vulnerability_mask_path', '')
        self.declare_parameter('auto_double_vulnerability_zone_dwell_sec', 3.0)
        self.declare_parameter('auto_double_vulnerability_hero_hp_drop_amount', 100)
        self.declare_parameter('auto_double_vulnerability_hero_hp_window_sec', 3.0)
        self.declare_parameter('auto_double_vulnerability_endgame_sec', 30.0)
        self.declare_parameter('auto_double_vulnerability_referee_timeout_sec', 2.5)

        self.fusion = RadarFusionState(
            str(self.get_parameter('radio_side').value),
            auto_sync_side=bool(self.get_parameter('auto_sync_side').value),
            vision_timeout_sec=float(self.get_parameter('vision_timeout_sec').value),
            radio_timeout_sec=float(self.get_parameter('radio_timeout_sec').value),
            radio_ghost_sec=float(self.get_parameter('radio_ghost_sec').value),
        )
        self.last_error = ''
        self.last_referee_status: dict = {}
        self.last_0305_ack: dict = {}
        self.last_auto_double_ack: dict = {}
        self.auto_double_error = ''
        self.send_request_count = 0
        self.send_success_count = 0
        self.send_dry_run_count = 0
        self.last_send_request_time = 0.0
        self.last_send_success_time = 0.0
        self._last_send_monotonic = 0.0
        self._successful_send_times: deque[float] = deque(maxlen=32)

        configured_mask_path = str(
            self.get_parameter('auto_double_vulnerability_mask_path').value
        ).strip()
        mask_path = (
            _default_trigger_mask_path()
            if configured_mask_path.lower() in ('', 'bundled', 'default')
            else configured_mask_path
        )
        trigger_mask = None
        try:
            trigger_mask = TriggerZoneMask.load(mask_path)
        except Exception as exc:  # noqa: BLE001 - fail closed and expose diagnostics
            self.auto_double_error = f'failed to load double vulnerability mask: {exc}'
            self.get_logger().error(self.auto_double_error)
        self.auto_double = AutoDoubleVulnerabilityDecision(
            trigger_mask,
            DoubleVulnerabilityConfig(
                enabled=bool(
                    self.get_parameter('auto_double_vulnerability_enabled').value
                ),
                zone_dwell_sec=float(
                    self.get_parameter('auto_double_vulnerability_zone_dwell_sec').value
                ),
                hero_hp_drop_amount=int(
                    self.get_parameter(
                        'auto_double_vulnerability_hero_hp_drop_amount'
                    ).value
                ),
                hero_hp_window_sec=float(
                    self.get_parameter(
                        'auto_double_vulnerability_hero_hp_window_sec'
                    ).value
                ),
                endgame_trigger_sec=float(
                    self.get_parameter('auto_double_vulnerability_endgame_sec').value
                ),
                referee_data_timeout_sec=float(
                    self.get_parameter(
                        'auto_double_vulnerability_referee_timeout_sec'
                    ).value
                ),
            ),
        )
        self.send_pub = self.create_publisher(
            String, str(self.get_parameter('send_frame_topic').value), 10
        )
        self.radar_cmd_pub = self.create_publisher(
            String, str(self.get_parameter('radar_cmd_topic').value), 10
        )
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.create_subscription(
            String,
            str(self.get_parameter('telemetry_topic').value),
            self._handle_vision,
            10,
        )
        for topic in self._topic_list(str(self.get_parameter('bridge_topics').value)):
            self.create_subscription(String, topic, self._handle_bridge, 10)
        self.create_subscription(
            String,
            str(self.get_parameter('referee_status_topic').value),
            self._handle_referee_status,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('referee_tx_topic').value),
            self._handle_referee_tx,
            10,
        )
        rate = min(max(float(self.get_parameter('send_rate_hz').value), 0.1), 4.8)
        state_rate = min(max(float(self.get_parameter('state_rate_hz').value), 1.0), 30.0)
        self.create_timer(1.0 / rate, self._send_tick)
        self.create_timer(1.0 / state_rate, self._status_tick)
        self.get_logger().info(
            f'radar integration ready: side={self.fusion.own_side}, '
            f'state_rate={state_rate:.1f}Hz, 0x0305={rate:.2f}Hz, '
            f'auto_double={self.auto_double.config.enabled}'
        )

    @staticmethod
    def _topic_list(raw: str) -> list[str]:
        return [value.strip() for value in str(raw).replace(';', ',').split(',') if value.strip()]

    @staticmethod
    def _decode(message: String) -> dict:
        payload = json.loads(message.data)
        if not isinstance(payload, dict):
            raise ValueError('message must contain a JSON object')
        return payload

    def _handle_vision(self, message: String) -> None:
        try:
            self.fusion.update_vision(self._decode(message), time.monotonic())
            self.last_error = ''
        except Exception as exc:  # noqa: BLE001 - surface malformed external telemetry
            self.last_error = f'invalid vision telemetry: {exc}'
            self.get_logger().warning(self.last_error)

    def _handle_bridge(self, message: String) -> None:
        try:
            self.fusion.update_radio_bridge(self._decode(message), time.monotonic())
            self.last_error = ''
        except Exception as exc:  # noqa: BLE001
            self.last_error = f'invalid referee bridge message: {exc}'
            self.get_logger().warning(self.last_error)

    def _handle_referee_status(self, message: String) -> None:
        try:
            payload = self._decode(message)
            self.last_referee_status = payload
            detected_side = payload.get('detected_radio_side')
            effective_side = detected_side
            if payload.get('auto_detect_radio_side') is False:
                effective_side = payload.get('effective_radio_side')
            previous_side = self.fusion.own_side
            if self.fusion.set_referee_side(detected_side, effective_side):
                self.get_logger().info(
                    f'radar side hot-synced: {previous_side} -> {self.fusion.own_side} '
                    f'(source={self.fusion.side_source})'
                )
        except Exception as exc:  # noqa: BLE001
            self.last_error = f'invalid referee status: {exc}'

    def _handle_referee_tx(self, message: String) -> None:
        try:
            payload = self._decode(message)
        except Exception as exc:  # noqa: BLE001
            self.last_error = f'invalid referee TX acknowledgement: {exc}'
            return
        try:
            cmd_id = int(payload.get('cmd_id', -1))
        except (TypeError, ValueError):
            return
        request_id = str(payload.get('request_id', ''))
        if (
            payload.get('kind') == 'radar_cmd_0121'
            and request_id.startswith('auto-double-vulnerability-')
        ):
            self.last_auto_double_ack = payload
        if cmd_id != 0x0305:
            return
        self.last_0305_ack = payload
        now = time.time()
        if payload.get('written') is True:
            self.send_success_count += 1
            self.last_send_success_time = now
            self._successful_send_times.append(now)
        elif payload.get('dry_run') is True:
            self.send_dry_run_count += 1

    def _actual_send_rate(self, now: float) -> float:
        while self._successful_send_times and now - self._successful_send_times[0] > 2.0:
            self._successful_send_times.popleft()
        if len(self._successful_send_times) < 2:
            return 0.0
        span = self._successful_send_times[-1] - self._successful_send_times[0]
        return (len(self._successful_send_times) - 1) / span if span > 0.0 else 0.0

    def _decision_referee_status(self, fusion_snapshot: dict) -> dict:
        status = dict(self.last_referee_status)
        strategy = fusion_snapshot.get('strategy')
        if not isinstance(strategy, dict):
            return status
        for output_type, status_key in (
            ('GameStatus', 'last_game_status'),
            ('GameRobotHP', 'last_game_robot_hp'),
            ('RadarDecisionSync', 'last_radar_decision_sync'),
        ):
            event = strategy.get(output_type)
            if not isinstance(event, dict) or not isinstance(event.get('payload'), dict):
                continue
            payload = dict(event['payload'])
            payload['timestamp'] = event.get('timestamp', payload.get('timestamp'))
            status[status_key] = payload
        return status

    def _send_tick(self) -> None:
        monotonic_now = time.monotonic()
        wall_now = time.time()
        # ROS timers may catch up with back-to-back callbacks after executor stalls.
        # Keep a second, monotonic protocol guard so no burst can exceed official 5 Hz.
        if self._last_send_monotonic and monotonic_now - self._last_send_monotonic < 1.0 / 5.0:
            return
        snapshot = self.fusion.snapshot(monotonic_now)
        if snapshot['send_allowed']:
            message = String()
            message.data = json.dumps({
                'cmd_id': '0x0305',
                'data_hex': snapshot['payload_0305_hex'],
                'source': 'radar_integration',
            })
            self.send_pub.publish(message)
            self.send_request_count += 1
            self.last_send_request_time = wall_now
            self._last_send_monotonic = monotonic_now

    def _status_tick(self) -> None:
        monotonic_now = time.monotonic()
        wall_now = time.time()
        snapshot = self.fusion.snapshot(monotonic_now)
        decision_referee_status = self._decision_referee_status(snapshot)
        try:
            command = self.auto_double.evaluate(
                now=wall_now,
                fusion=snapshot,
                referee_status=decision_referee_status,
            )
            if command is not None:
                message = String()
                message.data = json.dumps(
                    command, ensure_ascii=False, separators=(',', ':')
                )
                self.radar_cmd_pub.publish(message)
                self.get_logger().warning(
                    'automatic double vulnerability requested: '
                    f"reason={command['reason']}, request_id={command['request_id']}"
                )
        except Exception as exc:  # noqa: BLE001 - decision failure must not stop 0x0305
            self.auto_double_error = f'auto double vulnerability decision failed: {exc}'
            self.get_logger().error(self.auto_double_error)

        snapshot.update({
            'node': self.get_name(),
            'timestamp': wall_now,
            'health': (
                'bad' if snapshot['side_mismatch']
                else 'warn' if not snapshot['vision']['online'] or not snapshot['radio']['fresh']
                else 'ok'
            ),
            'last_error': self.last_error,
            'state_rate_hz': min(
                max(float(self.get_parameter('state_rate_hz').value), 1.0), 30.0
            ),
            'tx_0305': {
                'configured_rate_hz': min(
                    max(float(self.get_parameter('send_rate_hz').value), 0.1), 4.8
                ),
                'actual_rate_hz': self._actual_send_rate(wall_now),
                'request_count': self.send_request_count,
                'success_count': self.send_success_count,
                'dry_run_count': self.send_dry_run_count,
                'last_request_time': self.last_send_request_time or None,
                'last_success_time': self.last_send_success_time or None,
                'last_ack': self.last_0305_ack or None,
            },
            'referee_serial': self.last_referee_status,
            'auto_double_vulnerability': {
                **self.auto_double.status(),
                'last_ack': self.last_auto_double_ack or None,
                'last_error': self.auto_double_error,
            },
        })
        status_message = String()
        status_message.data = json.dumps(snapshot, ensure_ascii=False, separators=(',', ':'))
        self.status_pub.publish(status_message)


def main() -> None:
    rclpy.init()
    node = RadarIntegrationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
