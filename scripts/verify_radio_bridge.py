#!/usr/bin/env python3
"""Exercise vision -> UDP -> ROS2 fusion -> referee dry-run -> UDP."""

from __future__ import annotations

import json
import time

from output.referee_transport import RadioRosTransport
from output.vision_telemetry import build_vision_telemetry


def main() -> int:
    transport = RadioRosTransport(
        'R',
        radio_config={
            'bridge_backend': 'udp',
            'vision_bind_host': '127.0.0.1',
            'vision_bind_port': 37601,
            'bridge_host': '127.0.0.1',
            'bridge_port': 37602,
        },
    )
    transport.start()
    command_sent = False
    deadline = time.time() + 8.0
    try:
        while time.time() < deadline:
            telemetry = build_vision_telemetry(
                side='R',
                send_map={'B1': (1234, 567)},
                valid_names={'B1'},
                guess_list={},
                occlusion_names=set(),
                camera_ready=True,
                fps=30.0,
                inference_ms=10.0,
                filter_type='integration_test',
            )
            transport.publish_telemetry(telemetry)
            snapshot = transport.snapshot()
            if snapshot['radio_online'] and not command_sent:
                command_sent = bool(
                    transport.request_double_vulnerability('integrated-dry-run')
                )
            ack = snapshot.get('last_ack') or {}
            map_ack = snapshot.get('last_map_ack') or {}
            bridge_status = snapshot.get('bridge_status') or {}
            modules = bridge_status.get('modules') or {}
            received_counts = bridge_status.get('received_counts') or {}
            if (
                command_sent
                and ack.get('request_id') == 'integrated-dry-run'
                and int(map_ack.get('cmd_id', -1)) == 0x0305
                and int(received_counts.get('radar_cmd', 0)) >= 1
                and {'broadcast_rx', 'interference_rx', 'fusion'} <= set(modules)
            ):
                result = {
                    'radio_online': snapshot['radio_online'],
                    'bridge_received': received_counts,
                    'module_names': sorted(modules),
                    'radar_command_ack': {
                        'request_id': ack.get('request_id'),
                        'dry_run': ack.get('dry_run'),
                        'written': ack.get('written'),
                    },
                    'map_0305_ack': {
                        'cmd_id': map_ack.get('cmd_id'),
                        'dry_run': map_ack.get('dry_run'),
                        'written': map_ack.get('written'),
                        'frame_length': map_ack.get('frame_length'),
                    },
                }
                print(json.dumps(result, ensure_ascii=False, indent=2))
                return 0
            time.sleep(0.05)
    finally:
        transport.close()
    raise RuntimeError('integrated radio loop did not produce heartbeat, command ACK, and 0x0305 ACK')


if __name__ == '__main__':
    raise SystemExit(main())
