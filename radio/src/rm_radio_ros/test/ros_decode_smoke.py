#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
import time

import rclpy
from std_msgs.msg import String

from rm_radio_ros.core.rm_protocol import ACCESS_CODES, build_referee_frame, bytes_to_bits


def air_chunks(data: bytes, size: int = 15) -> bytes:
    chunks = []
    for offset in range(0, len(data), size):
        chunk = bytearray(data[offset : offset + size])
        if len(chunk) < size:
            chunk.extend(b'\x00' * (size - len(chunk)))
        chunks.append(bytes(chunk))
    return b''.join(chunks)


def main() -> int:
    rclpy.init()
    node = rclpy.create_node('rm_decode_smoke')
    received = []
    bridge_received = []

    def on_frame(msg: String) -> None:
        received.append(json.loads(msg.data))

    def on_bridge(msg: String) -> None:
        bridge_received.append(json.loads(msg.data))

    node.create_subscription(String, '/rm_gfsk_node/frames', on_frame, 10)
    node.create_subscription(String, '/rm_gfsk_node/referee_bridge', on_bridge, 10)
    payload_publisher = node.create_publisher(String, '/rm_gfsk_node/air_payload_hex', 10)
    bits_publisher = node.create_publisher(String, '/rm_gfsk_node/demod_bits_hex', 10)

    deadline = time.monotonic() + 8.0
    while (
        (payload_publisher.get_subscription_count() == 0 or bits_publisher.get_subscription_count() == 0)
        and time.monotonic() < deadline
    ):
        rclpy.spin_once(node, timeout_sec=0.1)

    frame = build_referee_frame(0x0A06, b'ABC123')
    payload_msg = String()
    position_frame = build_referee_frame(0x0A01, bytes(range(24)))
    payload_msg.data = air_chunks(position_frame + frame).hex()

    bit_frame = build_referee_frame(0x0A06, b'BIT123')
    packet = ACCESS_CODES['broadcast'] + b'\x00\x0F\x00\x0F' + bit_frame
    bits_msg = String()
    bits_msg.data = bytes(bytes_to_bits(packet)).hex()

    for _ in range(5):
        payload_publisher.publish(payload_msg)
        bits_publisher.publish(bits_msg)
        rclpy.spin_once(node, timeout_sec=0.1)
        passwords = {item.get('parsed', {}).get('password') for item in received}
        bridge_types = {item.get('type') for item in bridge_received}
        if {'ABC123', 'BIT123'} <= passwords and {'RadarInfoToClient', 'RadarCommand0121'} <= bridge_types:
            break

    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        passwords = {item.get('parsed', {}).get('password') for item in received}
        bridge_types = {item.get('type') for item in bridge_received}
        if {'ABC123', 'BIT123'} <= passwords and {'RadarInfoToClient', 'RadarCommand0121'} <= bridge_types:
            break
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()

    passwords = {item.get('parsed', {}).get('password') for item in received}
    if 'ABC123' not in passwords:
        print('no decoded frame received from air_payload_hex', file=sys.stderr)
        return 1
    if 'BIT123' not in passwords:
        print('no decoded frame received from demod_bits_hex', file=sys.stderr)
        return 1

    if any(item.get('cmd_hex') != '0x0A06' for item in received):
        unexpected = [item for item in received if item.get('cmd_hex') not in {'0x0A01', '0x0A06'}]
        if unexpected:
            print(f'unexpected cmd: {received}', file=sys.stderr)
            return 1

    bridge_types = {item.get('type') for item in bridge_received}
    if 'RadarInfoToClient' not in bridge_types:
        print(f'no RadarInfoToClient bridge output: {bridge_received}', file=sys.stderr)
        return 1
    if 'RadarCommand0121' not in bridge_types:
        print(f'no RadarCommand0121 bridge output: {bridge_received}', file=sys.stderr)
        return 1

    print(json.dumps({'frames': received, 'bridge': bridge_received}, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
