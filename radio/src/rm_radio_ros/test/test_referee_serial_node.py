import json
import sys
import types

import pytest

rclpy = pytest.importorskip('rclpy')

from rm_radio_ros.nodes.referee_serial_node import OptionalSerialPort, RefereeSerialNode, _valid_0305_payload
from rm_radio_ros.core.invincible_targets import InvincibleTargetsMessage
from rm_radio_ros.core.rm_protocol import RefereeFrameAssembler, build_referee_frame


class FakeParameter:
    def __init__(self, value):
        self.value = value


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg.data)


class FakeLogger:
    def __init__(self):
        self.warnings = []
        self.errors = []
        self.infos = []

    def warning(self, msg):
        self.warnings.append(msg)

    def error(self, msg):
        self.errors.append(msg)

    def info(self, msg):
        self.infos.append(msg)


class FakeSerialStartPort:
    def __init__(self, opens: bool):
        self.opens = opens
        self.port = '/dev/ttyFAIL'
        self.last_error = 'permission denied'
        self.open_calls = 0

    def open(self):
        self.open_calls += 1
        return self.opens


def _fake_get_parameter(values):
    def get_parameter(name):
        if name not in values:
            raise KeyError(name)
        return FakeParameter(values[name])

    return get_parameter


def _frame_payload(cmd_id, data):
    frame = RefereeFrameAssembler().push_bytes(build_referee_frame(cmd_id, data))[0]
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    return RefereeSerialNode._frame_to_payload(node, frame)


def test_serial_start_policy_is_optional_and_skipped_in_dry_run():
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    port = FakeSerialStartPort(opens=False)
    node.serial_port = port
    values = {'dry_run': False, 'require_serial_open_on_start': False}
    node.get_parameter = _fake_get_parameter(values)

    RefereeSerialNode._enforce_serial_start_policy(node)
    assert port.open_calls == 0

    values.update({'dry_run': True, 'require_serial_open_on_start': True})
    RefereeSerialNode._enforce_serial_start_policy(node)
    assert port.open_calls == 0


def test_serial_start_policy_fails_fast_when_required_open_fails():
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.serial_port = FakeSerialStartPort(opens=False)
    node.get_parameter = _fake_get_parameter({
        'dry_run': False,
        'require_serial_open_on_start': True,
    })

    with pytest.raises(RuntimeError, match='fail-fast.*permission denied'):
        RefereeSerialNode._enforce_serial_start_policy(node)


def test_serial_start_policy_keeps_successfully_opened_port():
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.serial_port = FakeSerialStartPort(opens=True)
    node.get_parameter = _fake_get_parameter({
        'dry_run': False,
        'require_serial_open_on_start': True,
    })

    RefereeSerialNode._enforce_serial_start_policy(node)

    assert node.serial_port.open_calls == 1


def test_referee_frame_watchdog_errors_after_two_seconds_and_recovers(monkeypatch):
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.serial_port = types.SimpleNamespace(is_open=True)
    node.get_parameter = _fake_get_parameter({'dry_run': False, 'frame_timeout_sec': 2.0})
    logger = FakeLogger()
    node.get_logger = lambda: logger
    node.last_valid_frame_at = None
    node._last_valid_frame_monotonic = None
    node._frame_watchdog_armed_at = None
    node._frame_watchdog_error = ''
    clock = {'monotonic': 100.0, 'wall': 1000.0}
    monkeypatch.setattr(
        'rm_radio_ros.nodes.referee_serial_node.time.monotonic',
        lambda: clock['monotonic'],
    )
    monkeypatch.setattr(
        'rm_radio_ros.nodes.referee_serial_node.time.time',
        lambda: clock['wall'],
    )

    assert RefereeSerialNode._check_frame_watchdog(node) is False
    clock['monotonic'] = 101.99
    assert RefereeSerialNode._check_frame_watchdog(node) is False
    clock['monotonic'] = 102.0
    assert RefereeSerialNode._check_frame_watchdog(node) is True
    assert node._frame_watchdog_error == 'referee frame timeout: no valid frame received for 2s'
    assert logger.errors == [node._frame_watchdog_error]

    clock.update(monotonic=103.0, wall=1003.0)
    RefereeSerialNode._mark_valid_frame_received(node)
    assert node._frame_watchdog_error == ''
    assert node.last_valid_frame_at == 1003.0
    assert logger.infos == ['referee frame stream recovered']
    assert RefereeSerialNode._frame_watchdog_status(node)['frame_timed_out'] is False

    clock['monotonic'] = 105.0
    assert RefereeSerialNode._check_frame_watchdog(node) is True
    assert logger.errors == [
        'referee frame timeout: no valid frame received for 2s',
        'referee frame timeout: no valid frame received for 2s',
    ]


def test_referee_frame_watchdog_rearms_after_serial_reconnect(monkeypatch):
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.serial_port = types.SimpleNamespace(is_open=True)
    node.get_parameter = _fake_get_parameter({'dry_run': False, 'frame_timeout_sec': 2.0})
    node.get_logger = lambda: FakeLogger()
    node.last_valid_frame_at = 1000.0
    node._last_valid_frame_monotonic = 100.0
    node._frame_watchdog_armed_at = 100.0
    node._frame_watchdog_error = 'old timeout'
    clock = {'now': 200.0}
    monkeypatch.setattr(
        'rm_radio_ros.nodes.referee_serial_node.time.monotonic',
        lambda: clock['now'],
    )

    node.serial_port.is_open = False
    assert RefereeSerialNode._check_frame_watchdog(node) is False
    assert node._frame_watchdog_error == ''

    node.serial_port.is_open = True
    assert RefereeSerialNode._check_frame_watchdog(node) is False
    clock['now'] = 201.99
    assert RefereeSerialNode._check_frame_watchdog(node) is False
    clock['now'] = 202.0
    assert RefereeSerialNode._check_frame_watchdog(node) is True


def test_serial_node_builds_radar_0121_frame_from_bridge_payload():
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.seq = 0
    node.last_radar_cmd_value = 0

    frame, meta = RefereeSerialNode._build_radar_cmd_from_payload(
        node,
        {'radar_cmd': 1, 'password_cmd': 2, 'password': 'ABC123', 'sender_id': 9, 'receiver_id': 0x8080},
    )
    decoded = RefereeFrameAssembler().push_bytes(frame)[0]
    parsed = decoded.to_dict()['parsed']

    assert meta['kind'] == 'radar_cmd_0121'
    assert meta['data_cmd_id'] == 0x0121
    assert decoded.cmd_id == 0x0301
    assert parsed['robot_interaction']['receiver_id'] == 0x8080
    assert parsed['radar_cmd']['password'] == 'ABC123'


def test_serial_node_builds_password_update_0121_frame():
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.seq = 0
    node.last_radar_cmd_value = 0
    node.last_can_change_password = True

    frame, meta = RefereeSerialNode._build_radar_cmd_from_payload(
        node,
        {'radar_cmd': 0, 'password_cmd': 1, 'password': 'NEK001', 'sender_id': 9, 'receiver_id': 0x8080},
    )
    decoded = RefereeFrameAssembler().push_bytes(frame)[0]
    parsed = decoded.to_dict()['parsed']['radar_cmd']

    assert meta['data_cmd_id'] == 0x0121
    assert meta['password_cmd'] == 1
    assert meta['user_data_hex'] == '00014e454b303031'
    assert parsed['password_cmd'] == 1
    assert parsed['password'] == 'NEK001'


def test_serial_node_builds_generic_0212_interaction_frame():
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.seq = 0

    payload = json.loads('{"data_cmd_id":"0x0212","sender_id":9,"receiver_id":7,"user_data_hex":"010203"}')
    frame, meta = RefereeSerialNode._build_interaction_from_payload(node, payload)
    decoded = RefereeFrameAssembler().push_bytes(frame)[0]
    parsed = decoded.to_dict()['parsed']['robot_interaction']

    assert meta['data_cmd_id'] == 0x0212
    assert decoded.cmd_id == 0x0301
    assert parsed['data_cmd_id'] == 0x0212
    assert parsed['sender_id'] == 9
    assert parsed['receiver_id'] == 7
    assert parsed['user_data_hex'] == '010203'


def _invincible_targets_node():
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.seq = 0
    node.detected_radio_side = None
    node.detected_radar_sender_id = None
    node.last_invincible_main_status = None
    node.last_invincible_source_frame_sequence = None
    node.last_invincible_source_received_at = None
    node._last_invincible_source_monotonic = None
    node.invincible_targets_source_count = 0
    node.invincible_targets_batch_count = 0
    node.invincible_targets_frame_request_count = 0
    node.invincible_targets_frame_success_count = 0
    node.invincible_targets_frame_dry_run_count = 0
    node.last_invincible_targets_batch = None
    node._pending_invincible_targets_batch = None
    node.last_error = ''
    node.get_parameter = _fake_get_parameter({
        'auto_send_invincible_targets': True,
        'invincible_targets_data_cmd_id': 0x0234,
        'invincible_targets_freshness_sec': 1.0,
        'auto_detect_radio_side': False,
        'radio_side': 'red',
        'sender_id': 9,
    })
    node.get_logger = lambda: FakeLogger()
    return node


def test_serial_node_accepts_only_complete_0a05_invincible_snapshot(monkeypatch):
    node = _invincible_targets_node()
    clock = {'monotonic': 100.0, 'wall': 1000.0}
    monkeypatch.setattr(
        'rm_radio_ros.nodes.referee_serial_node.time.monotonic',
        lambda: clock['monotonic'],
    )
    monkeypatch.setattr(
        'rm_radio_ros.nodes.referee_serial_node.time.time',
        lambda: clock['wall'],
    )
    bridge = {
        'type': 'RadarEnemyBuffStatus',
        'seq': 9,
        'payload': {
            'robot_main_status': {
                'opponent_hero': 2,
                'opponent_engineer': 0,
                'opponent_infantry_3': 3,
                'opponent_infantry_4': 1,
                'opponent_sentry': 0,
            },
        },
    }

    assert RefereeSerialNode._observe_invincible_targets_bridge(node, bridge) is True
    snapshot = RefereeSerialNode._invincible_targets_snapshot(node, now_monotonic=100.5)
    assert snapshot['radio_fresh'] is True
    assert snapshot['fallback_all_alive'] is False
    assert snapshot['source_frame_sequence'] == 9
    assert snapshot['target_ids'] == [101, 103]
    assert snapshot['target_statuses'] == [
        {'robot_id': 101, 'invincible': True},
        {'robot_id': 102, 'invincible': False},
        {'robot_id': 103, 'invincible': True},
        {'robot_id': 104, 'invincible': False},
        {'robot_id': 107, 'invincible': False},
    ]

    partial = json.loads(json.dumps(bridge))
    partial['payload']['robot_main_status'].pop('opponent_sentry')
    assert RefereeSerialNode._observe_invincible_targets_bridge(node, partial) is False
    assert node.invincible_targets_source_count == 1

    stale = RefereeSerialNode._invincible_targets_snapshot(node, now_monotonic=101.01)
    assert stale['radio_fresh'] is False
    assert stale['fallback_all_alive'] is True
    assert stale['target_ids'] == []
    assert stale['source_frame_sequence'] == 0xFF
    assert all(not entry['invincible'] for entry in stale['target_statuses'])


def test_serial_node_sends_one_shared_invincible_list_to_six_receivers(monkeypatch):
    node = _invincible_targets_node()
    node.last_invincible_main_status = {
        'opponent_hero': 0,
        'opponent_engineer': 2,
        'opponent_infantry_3': 0,
        'opponent_infantry_4': 3,
        'opponent_sentry': 0,
    }
    node.last_invincible_source_frame_sequence = 17
    node._last_invincible_source_monotonic = 100.0
    monkeypatch.setattr(
        'rm_radio_ros.nodes.referee_serial_node.time.monotonic',
        lambda: 100.25,
    )
    monkeypatch.setattr(
        'rm_radio_ros.nodes.referee_serial_node.time.time',
        lambda: 1000.0,
    )
    sent = []

    def fake_send(frame, meta):
        sent.append((frame, dict(meta)))
        return {**meta, 'written': True, 'dry_run': False}

    node._send_bytes = fake_send

    progress = [RefereeSerialNode._send_invincible_targets_tick(node) for _ in range(6)]
    batch = progress[-1]

    assert batch is not None
    assert all(item['batch_complete'] is False for item in progress[:-1])
    assert batch['batch_complete'] is True
    assert batch['radio_fresh'] is True
    assert batch['target_ids'] == [102, 104]
    assert [meta['receiver_id'] for _frame, meta in sent] == [1, 3, 4, 5, 6, 7]
    assert node.invincible_targets_frame_request_count == 6
    assert node.invincible_targets_frame_success_count == 6
    for frame, meta in sent:
        decoded = RefereeFrameAssembler().push_bytes(frame)[0]
        interaction = decoded.to_dict()['parsed']['robot_interaction']
        message = InvincibleTargetsMessage.from_bytes(bytes.fromhex(interaction['user_data_hex']))
        assert decoded.cmd_id == 0x0301
        assert interaction['data_cmd_id'] == 0x0234
        assert interaction['sender_id'] == 9
        assert interaction['receiver_id'] == meta['receiver_id']
        assert message.invincible_mask == 0b01010
        assert message.invincible_states == (False, True, False, True, False)


def test_serial_node_continuously_sends_all_not_invincible_without_fresh_0a05():
    node = _invincible_targets_node()
    sent = []
    node._send_bytes = lambda frame, meta: {
        **meta,
        'written': True,
        'dry_run': False,
        'captured': sent.append((frame, dict(meta))),
    }

    progress = [RefereeSerialNode._send_invincible_targets_tick(node) for _ in range(6)]

    assert progress[-1]['batch_complete'] is True
    assert progress[-1]['radio_fresh'] is False
    assert len(sent) == 6
    assert node.invincible_targets_frame_request_count == 6
    for frame, _meta in sent:
        decoded = RefereeFrameAssembler().push_bytes(frame)[0]
        interaction = decoded.to_dict()['parsed']['robot_interaction']
        message = InvincibleTargetsMessage.from_bytes(bytes.fromhex(interaction['user_data_hex']))
        assert message.invincible_mask == 0
        assert message.invincible_states == (False, False, False, False, False)


def test_serial_node_accepts_algorithm_request_count_payload():
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.seq = 0
    node.last_radar_cmd_value = 1

    payload = RefereeSerialNode._parse_algorithm_radar_cmd_message(node, '{"request_count":2}')
    frame, meta = RefereeSerialNode._build_radar_cmd_from_payload(node, payload)
    decoded = RefereeFrameAssembler().push_bytes(frame)[0]
    parsed = decoded.to_dict()['parsed']

    assert payload['radar_cmd'] == 2
    assert meta['user_data_hex'] == '0200000000000000'
    assert meta['user_data_length'] == 8
    assert parsed['radar_cmd'] == {
        'valid_radar_cmd_length': True,
        'radar_cmd': 2,
        'password_cmd': 0,
    }


def test_serial_node_accepts_plain_integer_algorithm_payload():
    node = RefereeSerialNode.__new__(RefereeSerialNode)

    payload = RefereeSerialNode._parse_algorithm_radar_cmd_message(node, '3')

    assert payload == {'radar_cmd': 3}


def test_serial_node_semantic_requests_generate_monotonic_radar_cmd():
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.last_radar_cmd_value = 0

    first = RefereeSerialNode._parse_algorithm_radar_cmd_message(
        node,
        '{"schema":"transistor.radar.command.v1","action":"trigger_double_vulnerability","request_id":"one"}',
    )
    assert first['radar_cmd'] == 1

    node.last_radar_cmd_value = 1
    second = RefereeSerialNode._parse_algorithm_radar_cmd_message(
        node,
        '{"schema":"shark.radar.command.v1","action":"trigger_double_vulnerability","request_id":"two"}',
    )
    assert second['radar_cmd'] == 2


def test_serial_node_only_commits_radar_cmd_after_success_or_dry_run():
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.last_radar_cmd_value = 0

    RefereeSerialNode._commit_radar_cmd_after_send(
        node, {'radar_cmd': 1, 'written': False, 'dry_run': False}
    )
    assert node.last_radar_cmd_value == 0

    RefereeSerialNode._commit_radar_cmd_after_send(
        node, {'radar_cmd': 1, 'written': True, 'dry_run': False}
    )
    assert node.last_radar_cmd_value == 1


def test_serial_node_remembers_uuid_ack_for_deduplication():
    from collections import deque

    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.algorithm_request_ids = deque(maxlen=2)
    node.algorithm_request_acks = {}

    RefereeSerialNode._remember_algorithm_ack(node, 'one', {'written': True, 'radar_cmd': 1})
    RefereeSerialNode._remember_algorithm_ack(node, 'two', {'written': False, 'radar_cmd': 2})
    RefereeSerialNode._remember_algorithm_ack(node, 'three', {'written': True, 'radar_cmd': 2})

    assert 'one' not in node.algorithm_request_acks
    assert node.algorithm_request_acks['two']['written'] is False
    assert node.algorithm_request_acks['three']['radar_cmd'] == 2


def test_serial_node_rejects_empty_or_malformed_0305_payloads():
    assert not _valid_0305_payload(b'')
    assert not _valid_0305_payload(b'\x01')
    assert not _valid_0305_payload(b'\x00' * 48)


def test_serial_node_accepts_nonzero_0305_payload_shape():
    payload = bytearray(48)
    payload[0] = 1

    assert _valid_0305_payload(bytes(payload))


def test_serial_node_password_bridge_preserves_last_radar_cmd():
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.seq = 0
    node.last_radar_cmd_value = 2

    frame, meta = RefereeSerialNode._build_radar_cmd_from_payload(
        node,
        {'radar_cmd': 0, 'password_cmd': 2, 'password': 'ABC123'},
        preserve_last_radar_cmd=True,
    )
    decoded = RefereeFrameAssembler().push_bytes(frame)[0]
    parsed = decoded.to_dict()['parsed']['radar_cmd']

    assert meta['radar_cmd'] == 2
    assert parsed['radar_cmd'] == 2
    assert parsed['password_cmd'] == 2
    assert parsed['password'] == 'ABC123'


def test_serial_node_manual_password_command_preserves_last_radar_cmd():
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.seq = 0
    node.last_radar_cmd_value = 3
    node.last_can_change_password = True

    frame, meta = RefereeSerialNode._build_radar_cmd_from_payload(
        node,
        {'radar_cmd': 0, 'password_cmd': 1, 'password': 'NEK001'},
    )
    decoded = RefereeFrameAssembler().push_bytes(frame)[0]
    parsed = decoded.to_dict()['parsed']['radar_cmd']

    assert meta['radar_cmd'] == 3
    assert parsed['radar_cmd'] == 3
    assert parsed['password_cmd'] == 1


def test_serial_node_rejects_radar_cmd_jump_or_decrease():
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.last_radar_cmd_value = 2

    with pytest.raises(ValueError, match='must not decrease'):
        RefereeSerialNode._validate_radar_cmd_transition(node, 1)

    with pytest.raises(ValueError, match='increase by exactly 1'):
        RefereeSerialNode._validate_radar_cmd_transition(node, 4)

    RefereeSerialNode._validate_radar_cmd_transition(node, 2, password_present=True)
    RefereeSerialNode._validate_radar_cmd_transition(node, 3)


def test_serial_node_requires_020e_permission_for_password_update(monkeypatch):
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.last_can_change_password = False

    monkeypatch.setattr(
        'rm_radio_ros.nodes.referee_serial_node._node_bool_parameter',
        lambda _node, _name, default: default,
    )

    with pytest.raises(ValueError, match='can_change_password=true'):
        RefereeSerialNode._validate_password_update_allowed(node, 1)

    node.last_can_change_password = True
    RefereeSerialNode._validate_password_update_allowed(node, 1)


def test_serial_node_blocks_raw_0121_send_frame():
    assert RefereeSerialNode._is_raw_radar_cmd_frame(0x0301, bytes([0x21, 0x01, 9, 0, 0x80, 0x80, 1]))
    assert not RefereeSerialNode._is_raw_radar_cmd_frame(0x0301, bytes([0x12, 0x02, 9, 0, 7, 0]))


def test_serial_node_limits_0305_rate(monkeypatch):
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.last_0305_tx_time = 0.0
    node.blocked_0305_rate_count = 0
    node.last_error = ''
    node.get_logger = lambda: type('Logger', (), {'warning': lambda _self, _msg: None})()

    monkeypatch.setattr('rm_radio_ros.nodes.referee_serial_node.time.monotonic', lambda: 100.0)
    monkeypatch.setattr(
        'rm_radio_ros.nodes.referee_serial_node._node_float_parameter',
        lambda _node, _name, _default: 5.0,
    )

    assert RefereeSerialNode._allow_0305_send(node)

    monkeypatch.setattr('rm_radio_ros.nodes.referee_serial_node.time.monotonic', lambda: 100.1)
    assert not RefereeSerialNode._allow_0305_send(node)
    assert '0x0305' in node.last_error

    monkeypatch.setattr('rm_radio_ros.nodes.referee_serial_node.time.monotonic', lambda: 100.21)
    assert RefereeSerialNode._allow_0305_send(node)


def test_serial_node_suppresses_password_verify_during_protocol_cooldown(monkeypatch):
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.last_password_verify_time = 100.0
    node.password_verify_suppressed_count = 0
    node.last_password_verify_suppression = ''
    node.last_error = ''
    node.get_logger = lambda: type('Logger', (), {'warning': lambda _self, _msg: None})()

    monkeypatch.setattr('rm_radio_ros.nodes.referee_serial_node.time.monotonic', lambda: 105.0)
    monkeypatch.setattr(
        'rm_radio_ros.nodes.referee_serial_node._node_float_parameter',
        lambda _node, _name, _default: 10.0,
    )

    ok = RefereeSerialNode._should_send_bridge_radar_command(
        node,
        {'radar_cmd': 0, 'password_cmd': 2, 'password': 'ABC123'},
    )

    assert not ok
    assert node.password_verify_suppressed_count == 1
    assert node.last_error == ''
    assert 'protocol cooldown' in node.last_password_verify_suppression


def test_password_verify_status_reports_remaining_and_ready(monkeypatch):
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.last_password_verify_time = 100.0
    node.last_password_verify = 'ABC123'
    monkeypatch.setattr('rm_radio_ros.nodes.referee_serial_node.time.monotonic', lambda: 106.5)
    monkeypatch.setattr(
        'rm_radio_ros.nodes.referee_serial_node._node_float_parameter',
        lambda _node, _name, _default: 10.0,
    )

    cooling = RefereeSerialNode._password_verify_status(node)
    assert cooling == {
        'last_password_verify': 'ABC123',
        'password_verify_cooldown_sec': 10.0,
        'password_verify_cooldown_remaining_sec': 3.5,
        'password_verify_ready': False,
    }

    monkeypatch.setattr('rm_radio_ros.nodes.referee_serial_node.time.monotonic', lambda: 111.0)
    ready = RefereeSerialNode._password_verify_status(node)
    assert ready['password_verify_cooldown_remaining_sec'] == 0.0
    assert ready['password_verify_ready'] is True


def test_serial_node_updates_interference_level_from_020e(monkeypatch):
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.interference_level = 1
    node.interference_update_count = 0
    node.last_error = ''
    node.last_can_change_password = None
    node.last_radar_decision_sync = None
    node.last_interference_update = None
    node.last_interference_setters = None
    node.rx_interference_level = 1
    node.rx_interference_update_count = 0
    node.last_rx_interference_update = None
    node.last_rx_interference_setters = None
    node.interference_control_pub = FakePublisher()
    node.interference_rx_control_pub = FakePublisher()
    node.get_parameter = _fake_get_parameter({
        'auto_interference_level': True,
        'auto_rx_interference_level': True,
        'referee_level_confirm_count': 2,
        'radio_side': 'red',
    })
    node.get_logger = lambda: FakeLogger()

    payload = _frame_payload(0x020E, bytes([(2 << 3) | (1 << 5)]))
    RefereeSerialNode._handle_rx_payload(node, payload)
    assert node.interference_update_count == 0
    assert node.rx_interference_update_count == 0
    assert node.last_referee_level_observation['candidate_count'] == 1
    assert node.last_referee_level_observation['confirmed'] is False

    RefereeSerialNode._handle_rx_payload(node, payload)

    assert node.interference_level == 2
    assert node.interference_update_count == 1
    assert node.last_can_change_password is True
    assert node.last_radar_decision_sync['own_encryption_level'] == 2
    assert json.loads(node.interference_control_pub.messages[-1]) == {
        'center_f': 432500000,
        'BW_ganrao': 860000,
    }
    assert node.rx_interference_level == 2
    assert node.rx_interference_update_count == 1
    rx_setters = json.loads(node.interference_rx_control_pub.messages[-1])
    assert rx_setters['cen_f'] == 432500000
    assert rx_setters['BW'] == 860000
    assert rx_setters['LowPass'] == 500000
    assert rx_setters['interference_level'] == 2
    assert node.last_interference_update['source'] == '0x020E'
    assert node.last_rx_interference_update['source'] == '0x020E'
    assert node.confirmed_referee_level == 2
    assert node.last_confirmed_referee_level_update['candidate_count'] == 2


def test_serial_node_records_0001_game_status():
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.last_game_status = None
    node.game_status_pub = FakePublisher()

    data = bytes([(4 << 4) | 1]) + (418).to_bytes(2, 'little') + (0).to_bytes(8, 'little')
    frame = RefereeFrameAssembler().push_bytes(build_referee_frame(0x0001, data))[0]

    RefereeSerialNode._handle_rx_payload(node, RefereeSerialNode._frame_to_payload(node, frame))

    assert node.last_game_status['game_progress'] == 4
    assert node.last_game_status['game_progress_name'] == 'running'
    assert node.last_game_status['match_running'] is True
    assert node.last_game_status['stage_remain_time'] == 418
    event = json.loads(node.game_status_pub.messages[-1])
    assert event['game_progress'] == 4
    assert event['match_running'] is True
    assert event['rx_raw_hex'] == frame.raw.hex()


def test_serial_node_records_0201_robot_status_for_auto_side():
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.last_robot_status = None
    node.detected_robot_id = None
    node.detected_radio_side = None
    node.detected_radar_sender_id = None

    data = (
        bytes([109, 1])
        + (400).to_bytes(2, 'little')
        + (500).to_bytes(2, 'little')
        + b'\x00' * 11
    )
    frame = RefereeFrameAssembler().push_bytes(build_referee_frame(0x0201, data))[0]

    RefereeSerialNode._handle_rx_payload(node, RefereeSerialNode._frame_to_payload(node, frame))

    assert node.last_robot_status['robot_id'] == 109
    assert node.last_robot_status['radio_side'] == 'blue'
    assert node.detected_robot_id == 109
    assert node.detected_radio_side == 'blue'
    assert node.detected_radar_sender_id == 109


def test_serial_node_effective_side_and_sender_follow_detected_referee_side():
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.detected_radio_side = 'blue'
    node.detected_radar_sender_id = 109
    node.get_parameter = _fake_get_parameter({
        'auto_detect_radio_side': True,
        'radio_side': 'red',
        'sender_id': 9,
    })

    assert RefereeSerialNode._effective_radio_side(node) == 'blue'
    assert RefereeSerialNode._effective_sender_id(node) == 109

    node.get_parameter = _fake_get_parameter({
        'auto_detect_radio_side': False,
        'radio_side': 'red',
        'sender_id': 9,
    })

    assert RefereeSerialNode._effective_radio_side(node) == 'red'
    assert RefereeSerialNode._effective_sender_id(node) == 9


def test_serial_node_suppresses_020e_level_before_match_running():
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.interference_level = 1
    node.rx_interference_level = 1
    node.rx_interference_update_count = 0
    node.single_rx_mode = 'interference'
    node.single_rx_switch_count = 0
    node.referee_level_candidate = None
    node.referee_level_candidate_count = 0
    node.confirmed_referee_level = 1
    node.last_game_status = {
        'game_progress': 1,
        'game_progress_name': 'preparation',
        'match_running': False,
    }
    node.last_referee_level_observation = None
    node.last_confirmed_referee_level_update = None
    node.last_error = ''
    node.last_can_change_password = None
    node.last_radar_decision_sync = None
    node.last_single_rx_switch = None
    node.suppressed_referee_level_before_running_count = 0
    node.last_suppressed_referee_level_before_running = None
    node.interference_rx_control_pub = FakePublisher()
    node.interference_control_pub = FakePublisher()
    node.get_parameter = _fake_get_parameter({
        'apply_referee_level_only_when_running': True,
        'auto_interference_level': False,
        'auto_rx_interference_level': True,
        'single_rx_auto_broadcast_after_level3': True,
        'single_rx_auto_allow_return_to_interference': True,
        'referee_level_confirm_count': 1,
        'radio_side': 'red',
    })
    logger = FakeLogger()
    node.get_logger = lambda: logger

    payload = _frame_payload(0x020E, bytes([0x38]))
    RefereeSerialNode._handle_rx_payload(node, payload)

    assert node.last_radar_decision_sync['own_encryption_level'] == 3
    assert node.last_can_change_password is True
    assert node.confirmed_referee_level == 1
    assert node.single_rx_mode == 'interference'
    assert node.single_rx_switch_count == 0
    assert node.interference_rx_control_pub.messages == []
    assert node.suppressed_referee_level_before_running_count == 1
    assert node.last_suppressed_referee_level_before_running['reason'] == 'match_not_running'
    assert 'not running yet' in logger.warnings[-1]

    node.last_game_status = {
        'game_progress': 4,
        'game_progress_name': 'running',
        'match_running': True,
    }
    RefereeSerialNode._handle_rx_payload(node, payload)

    assert node.confirmed_referee_level == 3
    assert node.single_rx_mode == 'broadcast'
    assert node.single_rx_switch_count == 1
    setters = json.loads(node.interference_rx_control_pub.messages[-1])
    assert setters['rx_profile'] == 'broadcast'


def test_serial_node_can_auto_retune_rx_without_tx_control(monkeypatch):
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.interference_level = 1
    node.rx_interference_level = 1
    node.rx_interference_update_count = 0
    node.last_error = ''
    node.last_rx_interference_update = None
    node.last_rx_interference_setters = None
    node.interference_rx_control_pub = FakePublisher()
    node.get_parameter = _fake_get_parameter({
        'auto_rx_interference_level': True,
        'radio_side': 'blue',
    })
    node.get_logger = lambda: FakeLogger()

    updated = RefereeSerialNode._update_rx_interference_level_from_referee(node, 3)

    assert updated is True
    assert node.rx_interference_level == 3
    setters = json.loads(node.interference_rx_control_pub.messages[-1])
    assert setters['cen_f'] == 434320000
    assert setters['interference_level'] == 3


def test_serial_node_does_not_retune_single_rx_after_broadcast_switch(monkeypatch):
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.single_rx_mode = 'broadcast'
    node.suppressed_rx_interference_update_count = 0
    node.last_suppressed_rx_interference_update = None
    node.rx_interference_level = 1
    node.rx_interference_update_count = 0
    node.last_error = ''
    node.interference_rx_control_pub = FakePublisher()
    node.get_parameter = _fake_get_parameter({
        'auto_rx_interference_level': True,
        'single_rx_auto_broadcast_after_level3': True,
        'radio_side': 'red',
    })
    node.get_logger = lambda: FakeLogger()

    updated = RefereeSerialNode._update_rx_interference_level_from_referee(node, 1)

    assert updated is False
    assert node.rx_interference_update_count == 0
    assert node.interference_rx_control_pub.messages == []
    assert node.suppressed_rx_interference_update_count == 1
    assert node.last_suppressed_rx_interference_update['reason'] == 'single_rx_already_switched_to_broadcast'


def test_serial_node_ignores_single_bad_level3_before_single_rx_switch():
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.interference_level = 1
    node.interference_update_count = 0
    node.rx_interference_level = 1
    node.rx_interference_update_count = 0
    node.single_rx_mode = 'interference'
    node.single_rx_switch_count = 0
    node.referee_level_candidate = None
    node.referee_level_candidate_count = 0
    node.confirmed_referee_level = 1
    node.last_referee_level_observation = None
    node.last_confirmed_referee_level_update = None
    node.last_error = ''
    node.last_can_change_password = None
    node.last_radar_decision_sync = None
    node.last_single_rx_switch = None
    node.last_interference_update = None
    node.last_interference_setters = None
    node.interference_control_pub = FakePublisher()
    node.interference_rx_control_pub = FakePublisher()
    node.get_parameter = _fake_get_parameter({
        'auto_interference_level': False,
        'auto_rx_interference_level': True,
        'single_rx_auto_broadcast_after_level3': True,
        'single_rx_auto_allow_return_to_interference': True,
        'referee_level_confirm_count': 2,
        'radio_side': 'red',
    })
    node.get_logger = lambda: FakeLogger()

    RefereeSerialNode._handle_rx_payload(node, _frame_payload(0x020E, bytes([0x38])))

    assert node.single_rx_mode == 'interference'
    assert node.single_rx_switch_count == 0
    assert node.interference_rx_control_pub.messages == []
    assert node.last_referee_level_observation['level'] == 3
    assert node.last_referee_level_observation['confirmed'] is False

    RefereeSerialNode._handle_rx_payload(node, _frame_payload(0x020E, bytes([0x28])))

    assert node.single_rx_mode == 'interference'
    assert node.single_rx_switch_count == 0
    assert node.interference_rx_control_pub.messages == []
    assert node.last_referee_level_observation['level'] == 1
    assert node.last_referee_level_observation['candidate_count'] == 1


def test_serial_node_switches_single_rx_after_confirmed_level3():
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.interference_level = 1
    node.rx_interference_level = 1
    node.rx_interference_update_count = 0
    node.single_rx_mode = 'interference'
    node.single_rx_switch_count = 0
    node.referee_level_candidate = None
    node.referee_level_candidate_count = 0
    node.confirmed_referee_level = 1
    node.last_referee_level_observation = None
    node.last_confirmed_referee_level_update = None
    node.last_error = ''
    node.last_can_change_password = None
    node.last_radar_decision_sync = None
    node.last_single_rx_switch = None
    node.interference_rx_control_pub = FakePublisher()
    node.interference_control_pub = FakePublisher()
    node.get_parameter = _fake_get_parameter({
        'auto_interference_level': False,
        'auto_rx_interference_level': True,
        'single_rx_auto_broadcast_after_level3': True,
        'single_rx_auto_allow_return_to_interference': True,
        'referee_level_confirm_count': 2,
        'radio_side': 'blue',
    })
    node.get_logger = lambda: FakeLogger()
    payload = _frame_payload(0x020E, bytes([0x38]))

    RefereeSerialNode._handle_rx_payload(node, payload)
    RefereeSerialNode._handle_rx_payload(node, payload)

    assert node.confirmed_referee_level == 3
    assert node.single_rx_mode == 'broadcast'
    assert node.single_rx_switch_count == 1
    setters = json.loads(node.interference_rx_control_pub.messages[-1])
    assert setters['rx_profile'] == 'broadcast'
    assert setters['interference_level'] == 3
    assert setters['cen_f'] == 433920000
    assert node.last_single_rx_switch['mode'] == 'broadcast'


def test_serial_node_returns_single_rx_to_interference_after_confirmed_level1():
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.interference_level = 1
    node.rx_interference_level = 3
    node.rx_interference_update_count = 0
    node.single_rx_mode = 'broadcast'
    node.single_rx_switch_count = 1
    node.referee_level_candidate = 3
    node.referee_level_candidate_count = 2
    node.confirmed_referee_level = 3
    node.last_referee_level_observation = None
    node.last_confirmed_referee_level_update = None
    node.last_error = ''
    node.last_can_change_password = None
    node.last_radar_decision_sync = None
    node.last_rx_interference_update = None
    node.last_rx_interference_setters = None
    node.last_single_rx_switch = {'mode': 'broadcast'}
    node.interference_rx_control_pub = FakePublisher()
    node.interference_control_pub = FakePublisher()
    node.get_parameter = _fake_get_parameter({
        'auto_interference_level': False,
        'auto_rx_interference_level': True,
        'single_rx_auto_broadcast_after_level3': True,
        'single_rx_auto_allow_return_to_interference': True,
        'referee_level_confirm_count': 2,
        'radio_side': 'red',
    })
    node.get_logger = lambda: FakeLogger()
    payload = _frame_payload(0x020E, bytes([0x28]))

    RefereeSerialNode._handle_rx_payload(node, payload)
    assert node.single_rx_mode == 'broadcast'

    RefereeSerialNode._handle_rx_payload(node, payload)

    assert node.confirmed_referee_level == 1
    assert node.single_rx_mode == 'interference'
    assert node.single_rx_switch_count == 2
    assert node.rx_interference_level == 1
    assert node.rx_interference_update_count == 1
    setters = json.loads(node.interference_rx_control_pub.messages[-1])
    assert setters['rx_profile'] == 'interference'
    assert setters['cen_f'] == 432200000
    assert setters['BW'] == 940000
    assert node.last_single_rx_switch['mode'] == 'interference'


def test_serial_node_can_keep_single_rx_broadcast_when_return_disabled():
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.interference_level = 1
    node.rx_interference_level = 3
    node.rx_interference_update_count = 0
    node.single_rx_mode = 'broadcast'
    node.single_rx_switch_count = 1
    node.referee_level_candidate = 3
    node.referee_level_candidate_count = 2
    node.confirmed_referee_level = 3
    node.last_referee_level_observation = None
    node.last_confirmed_referee_level_update = None
    node.last_error = ''
    node.last_can_change_password = None
    node.last_radar_decision_sync = None
    node.last_single_rx_switch = {'mode': 'broadcast'}
    node.interference_rx_control_pub = FakePublisher()
    node.interference_control_pub = FakePublisher()
    node.get_parameter = _fake_get_parameter({
        'auto_interference_level': False,
        'auto_rx_interference_level': True,
        'single_rx_auto_broadcast_after_level3': True,
        'single_rx_auto_allow_return_to_interference': False,
        'referee_level_confirm_count': 2,
        'radio_side': 'red',
    })
    node.get_logger = lambda: FakeLogger()
    payload = _frame_payload(0x020E, bytes([0x28]))

    RefereeSerialNode._handle_rx_payload(node, payload)
    RefereeSerialNode._handle_rx_payload(node, payload)

    assert node.confirmed_referee_level == 1
    assert node.single_rx_mode == 'broadcast'
    assert node.single_rx_switch_count == 1
    assert node.interference_rx_control_pub.messages == []


def test_serial_node_ignores_same_interference_level(monkeypatch):
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.interference_level = 1
    node.interference_update_count = 0
    node.last_error = ''
    node.interference_control_pub = FakePublisher()
    node.get_parameter = _fake_get_parameter({
        'auto_interference_level': True,
        'radio_side': 'red',
    })
    node.get_logger = lambda: FakeLogger()

    updated = RefereeSerialNode._update_interference_level_from_referee(node, 1)

    assert updated is False
    assert node.interference_update_count == 0
    assert node.interference_control_pub.messages == []


def test_serial_node_accepts_multiple_bridge_topics():
    assert RefereeSerialNode._parse_topic_list('/a,/b ; /c') == ['/a', '/b', '/c']


def test_optional_serial_port_lock_blocks_duplicate_bridge():
    first = OptionalSerialPort('/tmp/rm_radio_test_referee_uart', 115200, 0.001, exclusive=True)
    second = OptionalSerialPort('/tmp/rm_radio_test_referee_uart', 115200, 0.001, exclusive=True)
    try:
        assert first._acquire_lock()
        assert not second._acquire_lock()
        assert 'already locked' in second.last_error
    finally:
        first.close()
        second.close()


def test_optional_serial_port_lock_identity_survives_symlink_disconnect(tmp_path):
    target = tmp_path / 'ttyUSB-test'
    alias = tmp_path / 'stable-referee-port'
    target.touch()
    alias.symlink_to(target)

    port = OptionalSerialPort(str(alias), 115200, 0.001, exclusive=True)
    connected_lock_path = port._lock_path()
    alias.unlink()

    assert port._lock_path() == connected_lock_path


def test_optional_serial_port_keeps_ownership_and_recovers_when_device_returns(monkeypatch):
    class RecoveringSerial:
        EIGHTBITS = 8
        PARITY_NONE = 'N'
        STOPBITS_ONE = 1
        available = False

        def __init__(self, **_kwargs):
            if not self.available:
                raise OSError(2, 'No such file or directory')
            self.is_open = True

        def close(self):
            self.is_open = False

    monkeypatch.setitem(sys.modules, 'serial', types.SimpleNamespace(
        Serial=RecoveringSerial,
        EIGHTBITS=RecoveringSerial.EIGHTBITS,
        PARITY_NONE=RecoveringSerial.PARITY_NONE,
        STOPBITS_ONE=RecoveringSerial.STOPBITS_ONE,
    ))
    first = OptionalSerialPort('/tmp/rm_radio_test_missing_referee_uart', 115200, 0.001, exclusive=True)
    second = OptionalSerialPort('/tmp/rm_radio_test_missing_referee_uart', 115200, 0.001, exclusive=True)
    try:
        assert not first.open()
        assert 'serial unavailable' in first.last_error
        assert not second.open()
        assert 'already locked' in second.last_error
        RecoveringSerial.available = True
        assert first.open()
        assert first.is_open
        assert not second.open()
        assert 'already locked' in second.last_error
    finally:
        first.close()
        second.close()


def test_optional_serial_port_closes_stale_handle_after_write_failure(monkeypatch):
    class FakeSerial:
        instances = []
        fail_next_write = True

        EIGHTBITS = 8
        PARITY_NONE = 'N'
        STOPBITS_ONE = 1

        def __init__(self, **_kwargs):
            self.is_open = True
            self.closed = False
            self.write_calls = 0
            FakeSerial.instances.append(self)

        def write(self, _data):
            self.write_calls += 1
            if FakeSerial.fail_next_write:
                FakeSerial.fail_next_write = False
                raise OSError(5, 'Input/output error')
            return len(_data)

        def flush(self):
            pass

        def close(self):
            self.closed = True
            self.is_open = False

    monkeypatch.setitem(sys.modules, 'serial', types.SimpleNamespace(
        Serial=FakeSerial,
        EIGHTBITS=FakeSerial.EIGHTBITS,
        PARITY_NONE=FakeSerial.PARITY_NONE,
        STOPBITS_ONE=FakeSerial.STOPBITS_ONE,
    ))

    port = OptionalSerialPort('/tmp/rm_radio_test_reconnect_uart', 115200, 0.001, exclusive=False)

    assert not port.write(b'\x01')
    assert 'serial write failed' in port.last_error
    assert FakeSerial.instances[0].closed
    assert not port.is_open
    assert port.close_count == 1

    assert port.write(b'\x02')
    assert len(FakeSerial.instances) == 2
    assert port.is_open
    assert port.open_count == 2


def test_optional_serial_port_closes_stale_handle_after_read_failure(monkeypatch):
    class FakeSerial:
        EIGHTBITS = 8
        PARITY_NONE = 'N'
        STOPBITS_ONE = 1

        def __init__(self, **_kwargs):
            self.is_open = True
            self.in_waiting = 1
            self.closed = False

        def read(self, _count):
            raise OSError(5, 'Input/output error')

        def close(self):
            self.closed = True
            self.is_open = False

    monkeypatch.setitem(sys.modules, 'serial', types.SimpleNamespace(
        Serial=FakeSerial,
        EIGHTBITS=FakeSerial.EIGHTBITS,
        PARITY_NONE=FakeSerial.PARITY_NONE,
        STOPBITS_ONE=FakeSerial.STOPBITS_ONE,
    ))

    port = OptionalSerialPort('/tmp/rm_radio_test_read_reconnect_uart', 115200, 0.001, exclusive=False)

    assert port.read_available() == b''
    assert 'serial read failed' in port.last_error
    assert not port.is_open
    assert port.close_count == 1


def test_optional_serial_port_treats_partial_write_as_failure(monkeypatch):
    class FakeSerial:
        EIGHTBITS = 8
        PARITY_NONE = 'N'
        STOPBITS_ONE = 1

        def __init__(self, **_kwargs):
            self.is_open = True
            self.closed = False

        def write(self, _data):
            return 1

        def flush(self):
            pass

        def close(self):
            self.closed = True
            self.is_open = False

    monkeypatch.setitem(sys.modules, 'serial', types.SimpleNamespace(
        Serial=FakeSerial,
        EIGHTBITS=FakeSerial.EIGHTBITS,
        PARITY_NONE=FakeSerial.PARITY_NONE,
        STOPBITS_ONE=FakeSerial.STOPBITS_ONE,
    ))

    port = OptionalSerialPort('/tmp/rm_radio_test_partial_write_uart', 115200, 0.001, exclusive=False)

    assert not port.write(b'\x01\x02')
    assert 'partial serial write' in port.last_error
    assert not port.is_open


def _level_escalation_node(confirm_count, side='red'):
    node = RefereeSerialNode.__new__(RefereeSerialNode)
    node.interference_level = 1
    node.rx_interference_level = 1
    node.rx_interference_update_count = 0
    node.single_rx_mode = 'interference'
    node.single_rx_switch_count = 0
    node.referee_level_candidate = None
    node.referee_level_candidate_count = 0
    node.confirmed_referee_level = 1
    node.detected_radio_side = None
    node.last_referee_level_observation = None
    node.last_confirmed_referee_level_update = None
    node.last_error = ''
    node.last_can_change_password = None
    node.last_radar_decision_sync = None
    node.last_single_rx_switch = None
    node.last_interference_update = None
    node.last_interference_setters = None
    node.last_rx_interference_update = None
    node.last_rx_interference_setters = None
    node.interference_control_pub = FakePublisher()
    node.interference_rx_control_pub = FakePublisher()
    node.get_parameter = _fake_get_parameter({
        'auto_interference_level': False,
        'auto_rx_interference_level': True,
        'single_rx_auto_broadcast_after_level3': False,
        'single_rx_auto_allow_return_to_interference': True,
        'referee_level_confirm_count': confirm_count,
        'apply_referee_level_only_when_running': False,
        'auto_detect_radio_side': True,
        'radio_side': side,
    })
    node.get_logger = lambda: FakeLogger()
    return node


def _feed_referee_level(node, level):
    # 0x020E radar_decision_sync carries the interference level in bits 3-4 of its single byte.
    RefereeSerialNode._handle_rx_payload(node, _frame_payload(0x020E, bytes([(int(level) & 0x3) << 3])))


def test_interrupted_level3_stream_escalates_with_confirm_count_2_not_8():
    """Regression for the match 'stuck at level 2': a jittery 0x020E L3 stream (one stray L2,
    longest consecutive L3 run = 3) must still escalate the RX to level 3 under the current
    referee_level_confirm_count=2. The old default of 8 never accumulates 8 consecutive L3 and
    stays stuck at level 2 -- this test pins the fix so it cannot silently regress."""
    sequence = [2] * 8 + [3, 3, 3, 2, 3, 3, 3]

    old = _level_escalation_node(confirm_count=8)
    for level in sequence:
        _feed_referee_level(old, level)
    assert old.confirmed_referee_level == 2  # old behaviour: stuck below L3 on a jittery stream

    current = _level_escalation_node(confirm_count=2)
    for level in sequence:
        _feed_referee_level(current, level)
    assert current.confirmed_referee_level == 3
    setters = json.loads(current.interference_rx_control_pub.messages[-1])
    assert setters['interference_level'] == 3
    assert setters['cen_f'] == 432_800_000  # L3 interference centre frequency (red)
    assert setters['BW'] == 250_000
    assert setters['LowPass'] == 160_000
