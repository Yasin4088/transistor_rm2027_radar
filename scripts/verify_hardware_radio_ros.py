#!/usr/bin/env python3
"""Verify the live RX -> decode -> fusion -> referee-output ROS2 path."""

from __future__ import annotations

import argparse
import json
import time
from typing import Any

import rclpy
from std_msgs.msg import String


def _decode(message: String) -> dict[str, Any]:
    value = json.loads(message.data)
    if not isinstance(value, dict):
        raise ValueError('ROS message is not a JSON object')
    return value


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout-sec', type=float, default=45.0)
    parser.add_argument(
        '--expect-written',
        action='store_true',
        help='require a real referee serial write instead of the default dry-run ACK',
    )
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('rm_hardware_radio_loop_verifier')
    state: dict[str, Any] = {
        'decoded_0a01_count': 0,
        'bridge_radar_info_count': 0,
        'fusion_radio_source_count': 0,
        'fusion_radio_fresh': False,
        'tx_0305_ack': None,
        'last_frame': None,
        'last_bridge': None,
        'last_fusion': None,
        'decode_errors': [],
    }

    def callback(kind: str):
        def receive(message: String) -> None:
            try:
                payload = _decode(message)
                if kind == 'frame':
                    state['last_frame'] = payload
                    if str(payload.get('cmd_hex', '')).upper() == '0X0A01':
                        state['decoded_0a01_count'] += 1
                elif kind == 'bridge':
                    state['last_bridge'] = payload
                    if payload.get('type') == 'RadarInfoToClient':
                        state['bridge_radar_info_count'] += 1
                elif kind == 'fusion':
                    state['last_fusion'] = payload
                    source_counts = payload.get('source_counts') or {}
                    state['fusion_radio_source_count'] = max(
                        int(state['fusion_radio_source_count']),
                        int(source_counts.get('radio', 0)),
                    )
                    radio = payload.get('radio') or {}
                    state['fusion_radio_fresh'] = bool(
                        state['fusion_radio_fresh'] or radio.get('fresh')
                    )
                elif kind == 'tx':
                    try:
                        cmd_id = int(payload.get('cmd_id', -1))
                    except (TypeError, ValueError):
                        cmd_id = -1
                    if cmd_id == 0x0305:
                        state['tx_0305_ack'] = payload
            except Exception as exc:  # noqa: BLE001 - report malformed live data
                errors = state['decode_errors']
                if len(errors) < 10:
                    errors.append(f'{kind}: {exc}')

        return receive

    node.create_subscription(String, '/rm_gfsk_node/frames', callback('frame'), 20)
    node.create_subscription(
        String, '/rm_gfsk_node/referee_bridge', callback('bridge'), 20
    )
    node.create_subscription(
        String, '/rm_radar_integration/status', callback('fusion'), 20
    )
    node.create_subscription(
        String, '/rm_referee_serial_node/tx_frames', callback('tx'), 20
    )

    def complete() -> bool:
        ack = state.get('tx_0305_ack') or {}
        acknowledged = (
            ack.get('written') is True
            if args.expect_written
            else ack.get('dry_run') is True
        )
        return bool(
            state['decoded_0a01_count'] > 0
            and state['bridge_radar_info_count'] > 0
            and state['fusion_radio_source_count'] > 0
            and state['fusion_radio_fresh']
            and acknowledged
        )

    deadline = time.monotonic() + max(args.timeout_sec, 1.0)
    try:
        while time.monotonic() < deadline and not complete():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    summary = {
        'ok': complete(),
        'expected_referee_mode': 'written' if args.expect_written else 'dry_run',
        **state,
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True))
    if complete():
        return 0
    print(
        'hardware loop incomplete: require decoded 0x0A01, RadarInfoToClient, '
        'fresh radio-selected fusion coordinates, and a matching 0x0305 ACK',
        flush=True,
    )
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
