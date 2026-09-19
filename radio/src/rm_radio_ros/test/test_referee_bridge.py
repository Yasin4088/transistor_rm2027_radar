from rm_radio_ros.core.referee_bridge import (
    RefereeBridge,
    radar_command_from_password_frame,
    radar_info_from_position_frame,
)
from rm_radio_ros.core.rm_protocol import RefereeFrameAssembler, build_referee_frame
from types import SimpleNamespace

from rm_radio_ros.app_support.run_algorithm import (
    _patch_algorithm_protocol_structures,
    is_valid_0305_payload,
    radar_info_to_0305_payload,
)


def _frame(cmd_id, payload):
    assembler = RefereeFrameAssembler()
    return assembler.push_bytes(build_referee_frame(cmd_id, payload))[0]


def test_radar_info_to_client_shape_from_position_frame():
    frame = _frame(0x0A01, bytes(range(24)))
    radar_info = radar_info_from_position_frame(frame)

    payload = radar_info.to_dict()
    robots = payload['RadarSingleRobotInfo']
    assert payload['topic'] == 'RadarInfoToClient'
    assert len(robots) == 12
    assert robots[0] == {'target_pos_x': 256, 'target_pos_y': 770, 'is_high_light': 0}
    assert robots[5] == {'target_pos_x': 5396, 'target_pos_y': 5910, 'is_high_light': 0}
    assert robots[6:] == [{'target_pos_x': 0, 'target_pos_y': 0, 'is_high_light': 0}] * 6


def test_password_frame_to_0121_payload():
    frame = _frame(0x0A06, b'ABC123')
    command = radar_command_from_password_frame(frame)

    assert command.to_bytes() == b'\x00\x02ABC123'
    assert command.to_dict()['payload_hex'] == '0002414243313233'


def test_invalid_password_frame_is_not_bridged_to_0121():
    frame = _frame(0x0A06, b'ABC!@#')

    assert radar_command_from_password_frame(frame) is None


def test_bridge_emits_official_payloads():
    frames = [
        _frame(0x0A01, bytes(range(24))),
        _frame(0x0A02, bytes(range(12))),
        _frame(0x0A04, bytes([1, 0, 2, 0, 1, 0, 0, 0])),
        _frame(0x0A06, b'ABC123'),
    ]
    outputs = RefereeBridge().process_frames(frames)

    assert [output['type'] for output in outputs] == [
        'RadarInfoToClient',
        'RadarEnemyHp',
        'RadarEnemyMacroStatus',
        'RadarCommand0121',
    ]
    assert outputs[1]['payload']['opponent_engineer'] == 0x0302
    assert outputs[2]['payload']['remaining_coins'] == 1
    assert outputs[2]['payload']['total_coins'] == 2
    assert outputs[3]['payload']['password_cmd'] == 2


def test_bridge_includes_new_0a05_main_status_fields():
    payload = bytes(35) + bytes([5, 0, 1, 2, 3, 0])
    outputs = RefereeBridge().process_frames([_frame(0x0A05, payload)])

    assert len(outputs) == 1
    assert outputs[0]['type'] == 'RadarEnemyBuffStatus'
    assert outputs[0]['payload']['sentry_mode'] == 5
    assert outputs[0]['payload']['sentry_mode_name'] == 'enhanced_defensive'
    assert outputs[0]['payload']['robot_main_status']['opponent_engineer'] == 1
    assert outputs[0]['payload']['robot_main_status_names']['opponent_infantry_4'] == 'invincible_weakened'


def test_bridge_emits_referee_status_payloads():
    dart_packed = (2 & 0x07) | ((3 & 0x07) << 3) | ((4 & 0x07) << 6)
    frames = [
        _frame(0x0105, bytes([15]) + dart_packed.to_bytes(2, 'little')),
        _frame(0x020C, bytes([0x21, 0x08])),
    ]
    outputs = RefereeBridge().process_frames(frames)

    assert [output['type'] for output in outputs] == ['DartStatus', 'RadarMarkProgress']
    assert outputs[0]['payload']['selected_target'] == 4
    assert outputs[1]['payload']['enemy']['opponent_hero'] is True
    assert outputs[1]['payload']['enemy']['opponent_sentry'] is True
    assert outputs[1]['payload']['ally']['ally_sentry'] is True


def test_bridge_emits_v2_game_robot_hp_payload():
    values = [600, 300, 250, 240, -10, 400, 500, 1500, 450, 1400]
    payload = b''.join(
        value.to_bytes(2, 'little', signed=index == 4)
        for index, value in enumerate(values)
    )
    outputs = RefereeBridge().process_frames([_frame(0x0003, payload)])

    assert len(outputs) == 1
    assert outputs[0]['type'] == 'GameRobotHP'
    assert outputs[0]['payload']['ally_hero'] == 600
    assert outputs[0]['payload']['damage_difference'] == -10


def test_decoded_radar_info_can_build_protocol_0305_payload():
    frame = _frame(0x0A01, bytes(range(24)))
    radar_info = radar_info_from_position_frame(frame)
    bridge_payload = {
        'type': 'RadarInfoToClient',
        'payload': radar_info.to_dict(),
    }

    payload = radar_info_to_0305_payload(bridge_payload)

    assert payload is not None
    assert len(payload) == 48
    assert payload[:12] == bytes(range(12))
    assert payload[24:] == b'\x00' * 24


def test_decoded_radar_info_ignores_empty_positions_for_0305_fallback():
    bridge_payload = {
        'type': 'RadarInfoToClient',
        'payload': {
            'RadarSingleRobotInfo': [
                {'target_pos_x': 0, 'target_pos_y': 0, 'is_high_light': 0}
                for _ in range(12)
            ],
        },
    }

    assert radar_info_to_0305_payload(bridge_payload) is None


def test_0305_payload_validator_rejects_empty_or_malformed_payloads():
    assert not is_valid_0305_payload(b'')
    assert not is_valid_0305_payload(b'\x01')
    assert not is_valid_0305_payload(b'\x00' * 48)


def test_0305_payload_validator_accepts_nonzero_protocol_payload():
    payload = bytearray(48)
    payload[0] = 1

    assert is_valid_0305_payload(bytes(payload))


def test_algorithm_protocol_patch_decodes_0105_and_020e_bitfields():
    dart_message_cls = type('DartStatusMessage', (), {'STRUCT_CLASS': object})
    radar_info_message_cls = type('RadarInfoMessage', (), {'STRUCT_CLASS': object})
    serial_protocol = SimpleNamespace(
        DartStatusMessage=dart_message_cls,
        RadarInfoMessage=radar_info_message_cls,
    )
    referee_comm = SimpleNamespace(
        DartStatusMessage=dart_message_cls,
        RadarInfoMessage=radar_info_message_cls,
    )

    _patch_algorithm_protocol_structures(serial_protocol, referee_comm)

    dart_packed = (5 & 0x07) | ((3 & 0x07) << 3) | ((4 & 0x07) << 6) | (0x2A << 9)
    dart = serial_protocol.DartStatData.from_buffer_copy(bytes([12]) + dart_packed.to_bytes(2, 'little'))
    radar_info_byte = (2 << 0) | (1 << 2) | (3 << 3) | (1 << 5)
    radar_info = serial_protocol.RadarInfoData.from_buffer_copy(bytes([radar_info_byte]))

    assert serial_protocol.DartStatusMessage.STRUCT_CLASS is serial_protocol.DartStatData
    assert referee_comm.RadarInfoMessage.STRUCT_CLASS is serial_protocol.RadarInfoData
    assert dart.dart_remaining_time == 12
    assert dart.recent_hit_target == 5
    assert dart.accumulated_hit_count == 3
    assert dart.selected_target == 4
    assert dart.reserve == 0x2A
    assert radar_info.double_vulnerability_count == 2
    assert radar_info.is_double_vulnerability == 1
    assert radar_info.own_encryption_level == 3
    assert radar_info.can_change_password == 1
    assert radar_info.reserve == 0
