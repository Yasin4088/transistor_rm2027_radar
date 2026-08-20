import json
import time
import unittest

from referee_transport import RadioRosTransport, normalize_transport_mode
from vision_telemetry import build_vision_telemetry, classify_legacy_target_names


class FakeString:
    def __init__(self):
        self.data = ''


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(json.loads(message.data))


class RefereeTransportTests(unittest.TestCase):
    def test_mode_requires_explicit_legacy_serial(self):
        self.assertEqual(normalize_transport_mode(None), 'radio_ros')
        self.assertEqual(normalize_transport_mode('legacy_serial'), 'legacy_serial')
        with self.assertRaises(ValueError):
            normalize_transport_mode('serial')

    def test_semantic_request_is_uuid_deduplicated_while_pending_and_ack_driven(self):
        transport = RadioRosTransport('R')
        transport._string_type = FakeString
        transport._command_publisher = FakePublisher()

        request_id = transport.request_double_vulnerability('request-1')
        self.assertEqual(request_id, 'request-1')
        self.assertIsNone(transport.request_double_vulnerability('request-2'))
        self.assertEqual(transport.consume_successful_requests(), [])

        transport._process_tx_payload({
            'source': 'algorithm_radar_cmd_topic',
            'request_id': 'request-1',
            'radar_cmd': 1,
            'written': True,
        })
        success = transport.consume_successful_requests()
        self.assertEqual(success[0]['radar_cmd'], 1)
        self.assertTrue(success[0]['written'])

    def test_failed_ack_does_not_create_success(self):
        transport = RadioRosTransport('B')
        transport._string_type = FakeString
        transport._command_publisher = FakePublisher()
        transport.request_double_vulnerability('failed')
        transport._process_tx_payload({
            'source': 'algorithm_radar_cmd_topic',
            'request_id': 'failed',
            'radar_cmd': 1,
            'written': False,
        })
        self.assertEqual(transport.consume_successful_requests(), [])

    def test_radio_online_requires_a_recent_wireless_status_heartbeat(self):
        transport = RadioRosTransport('R')
        self.assertFalse(transport.snapshot()['radio_online'])

        transport._last_status = {'serial_open': True}
        transport._last_status_received_at = time.time()
        snapshot = transport.snapshot()
        self.assertTrue(snapshot['radio_online'])
        self.assertLessEqual(snapshot['radio_status_age_sec'], 0.1)

        transport._last_status_received_at = time.time() - 2.01
        self.assertFalse(transport.snapshot()['radio_online'])

    def test_snapshot_surfaces_referee_frame_watchdog_error(self):
        transport = RadioRosTransport('R')
        transport._last_status = {
            'serial_open': True,
            'frame_timed_out': True,
            'last_error': 'referee frame timeout: no valid frame received for 2s',
        }
        transport._last_status_received_at = time.time()

        snapshot = transport.snapshot()

        self.assertTrue(snapshot['radio_online'])
        self.assertEqual(
            snapshot['last_error'],
            'referee frame timeout: no valid frame received for 2s',
        )

    def test_telemetry_uses_only_protocol_robot_roles_and_fixed_states(self):
        telemetry = build_vision_telemetry(
            side='R',
            send_map={'B1': (1200, 650), 'R1': (200, 100), 'B5': (1, 1)},
            valid_names={'B1', 'R1'},
            guess_list={'B1': False, 'R1': False},
            occlusion_names={'R1'},
            camera_ready=True,
            fps=18.5,
            camera_fps=29.2,
            inference_ms=42.0,
            filter_type='sliding_window',
            source_time=100.0,
        )
        self.assertEqual(telemetry['schema'], 'transistor.radar.telemetry.v1')
        self.assertEqual(len(telemetry['robots']), 12)
        self.assertNotIn('B5', telemetry['robots'])
        self.assertEqual(telemetry['robots']['B1']['state'], 'measured')
        self.assertEqual(telemetry['robots']['R1']['state'], 'occlusion_hold')
        self.assertEqual(telemetry['vision']['camera_fps'], 29.2)
        self.assertEqual(telemetry['vision']['processing_fps'], 18.5)
        self.assertEqual(telemetry['vision']['fps'], 18.5)

    def test_legacy_fresh_measurement_is_not_labeled_as_occlusion_hold(self):
        valid_names, occlusion_names = classify_legacy_target_names(
            measured_names={'B1'},
            hold_cache_names={'B1', 'R1'},
        )

        self.assertEqual(valid_names, {'B1', 'R1'})
        self.assertEqual(occlusion_names, {'R1'})


if __name__ == '__main__':
    unittest.main()
