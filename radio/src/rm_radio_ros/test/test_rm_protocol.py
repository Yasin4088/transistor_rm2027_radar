import pytest

from rm_radio_ros.core.rm_protocol import (
    ACCESS_CODES,
    RADAR_AIR_DATA_LENGTHS,
    AccessCodeAirPacketExtractor,
    RefereeFrameAssembler,
    build_radar_cmd_payload,
    build_referee_frame,
    build_robot_interaction_data,
    build_robot_interaction_frame,
    bytes_to_bits,
    crc16_rm,
    crc8_rm,
    parse_robot_interaction_data,
    parse_dart_status,
    parse_game_robot_hp,
    parse_game_status,
    parse_radar_mark_progress,
    parse_radar_decision_sync,
    parse_robot_status,
    radar_sender_id_for_side,
    side_from_robot_id,
)
from rm_radio_ros.core.radio_config import (
    BROADCAST_RX_SEN_RE,
    GFSK_FLOWGRAPH_SPS,
    GFSK_TX_FLOWGRAPH_SAMPLE_RATE,
    GFSK_TX_FLOWGRAPH_SPS,
    INTERFERENCE_RX_SEN_RE,
    broadcast_rx_setters_for_side,
    interference_rx_setters_for_side_level,
    interference_setters_for_side_level,
)


def _air_chunks(data, size=15):
    chunks = []
    for offset in range(0, len(data), size):
        chunk = list(data[offset : offset + size])
        if len(chunk) < size:
            chunk.extend([0] * (size - len(chunk)))
        chunks.append(chunk)
    return chunks


def test_build_and_assemble_single_frame():
    frame = build_referee_frame(0x0A06, b'ABC123')
    assert frame[:7] == bytes([0xA5, 0x06, 0x00, 0x01, crc8_rm(frame[:4]), 0x06, 0x0A])
    assert crc16_rm(frame[:-2]) == frame[-2] | (frame[-1] << 8)

    assembler = RefereeFrameAssembler()
    frames = assembler.push_air_payload(_air_chunks(frame)[0])

    assert len(frames) == 1
    assert frames[0].cmd_id == 0x0A06
    assert frames[0].to_dict()['parsed']['password'] == 'ABC123'
    assert frames[0].to_dict()['parsed']['password_is_alnum_ascii'] is True


def test_parse_0a06_rejects_non_alnum_ascii_password_for_protocol_table():
    frame = build_referee_frame(0x0A06, b'ABC!@#')
    parsed = RefereeFrameAssembler().push_bytes(frame)[0].to_dict()['parsed']

    assert parsed['password'] == 'ABC!@#'
    assert parsed['password_is_alnum_ascii'] is False


def test_parse_020e_radar_decision_sync_bits_from_protocol_table():
    radar_info = (2 << 0) | (1 << 2) | (3 << 3) | (1 << 5)
    frame = build_referee_frame(0x020E, bytes([radar_info]))
    parsed = RefereeFrameAssembler().push_bytes(frame)[0].to_dict()['parsed']
    sync = parsed['radar_decision_sync']

    assert parsed['valid_radar_length'] is True
    assert sync == {
        'valid_radar_info_length': True,
        'radar_info': radar_info,
        'double_vulnerability_count': 2,
        'is_double_vulnerability': True,
        'own_encryption_level': 3,
        'opponent_interference_difficulty': 3,
        'can_change_password': True,
        'reserved': 0,
    }


def test_parse_0001_game_status_match_running_from_protocol_table():
    data = bytes([(4 << 4) | 1]) + (419).to_bytes(2, 'little') + (123456789).to_bytes(8, 'little')
    frame = build_referee_frame(0x0001, data)
    parsed = RefereeFrameAssembler().push_bytes(frame)[0].to_dict()['parsed']
    status = parsed['game_status']

    assert parsed['valid_radar_length'] is True
    assert status == {
        'valid_game_status_length': True,
        'game_type': 1,
        'game_progress': 4,
        'game_progress_name': 'running',
        'stage_remain_time': 419,
        'match_running': True,
        'sync_timestamp': 123456789,
    }


def test_parse_0003_game_robot_hp_from_v2_protocol_table():
    values = [600, 300, 250, 240, -321, 400, 500, 1500, 450, 1400]
    payload = b''.join(
        value.to_bytes(2, 'little', signed=index == 4)
        for index, value in enumerate(values)
    )
    frame = build_referee_frame(0x0003, payload)
    parsed = RefereeFrameAssembler().push_bytes(frame)[0].to_dict()['parsed']

    assert parsed['valid_radar_length'] is True
    assert parsed['game_robot_hp'] == {
        'valid_game_robot_hp_length': True,
        'ally_hero': 600,
        'ally_engineer': 300,
        'ally_infantry_3': 250,
        'ally_infantry_4': 240,
        'damage_difference': -321,
        'ally_sentry': 400,
        'ally_outpost': 500,
        'ally_base': 1500,
        'opponent_outpost': 450,
        'opponent_base': 1400,
    }
    assert parse_game_robot_hp(payload[:-1]) == {
        'valid_game_robot_hp_length': False
    }


def test_parse_0105_dart_status_bits_from_protocol_table():
    packed = (5 & 0x07) | ((3 & 0x07) << 3) | ((4 & 0x07) << 6) | (0x2A << 9)
    frame = build_referee_frame(0x0105, bytes([12]) + packed.to_bytes(2, 'little'))
    parsed = RefereeFrameAssembler().push_bytes(frame)[0].to_dict()['parsed']['dart_status']

    assert parsed == {
        'valid_dart_status_length': True,
        'dart_remaining_time': 12,
        'recent_hit_target': 5,
        'accumulated_hit_count': 3,
        'selected_target': 4,
        'reserved': 0x2A,
    }


def test_parse_0201_robot_status_and_side_from_protocol_table():
    payload = bytes([
        109,
        2,
        0x34, 0x12,
        0x78, 0x56,
        0x21, 0x43,
        0x65, 0x87,
        0x10, 0x00,
        0x00, 0x00, 0xC8, 0x41,
        0b00000111,
    ])
    frame = build_referee_frame(0x0201, payload)
    parsed = RefereeFrameAssembler().push_bytes(frame)[0].to_dict()['parsed']
    status = parsed['robot_status']

    assert parsed['valid_radar_length'] is True
    assert status['valid_robot_status_length'] is True
    assert status['robot_id'] == 109
    assert status['radio_side'] == 'blue'
    assert status['robot_level'] == 2
    assert status['current_hp'] == 0x1234
    assert status['maximum_hp'] == 0x5678
    assert status['shooter_barrel_cooling_value'] == 0x4321
    assert status['shooter_barrel_heat_limit'] == 0x8765
    assert status['chassis_power_limit'] == 16
    assert status['bullet_speed_limit'] == pytest.approx(25.0)
    assert status['power_management_gimbal_output'] is True
    assert status['power_management_chassis_output'] is True
    assert status['power_management_shooter_output'] is True


def test_robot_id_side_and_radar_sender_id_helpers():
    assert side_from_robot_id(9) == 'red'
    assert side_from_robot_id(109) == 'blue'
    assert side_from_robot_id(0) is None
    assert radar_sender_id_for_side('red') == 9
    assert radar_sender_id_for_side('blue') == 109
    assert parse_robot_status([]) == {'valid_robot_status_length': False}


def test_parse_0201_rejects_pre_v2_length():
    status = parse_robot_status(bytes(13))

    assert status['valid_robot_status_length'] is False


def test_parse_020c_radar_mark_progress_bits_from_protocol_table():
    progress_bits = (
        (1 << 0)
        | (1 << 5)
        | (1 << 7)
        | (1 << 11)
        | (1 << 12)
        | (1 << 15)
    )
    frame = build_referee_frame(0x020C, progress_bits.to_bytes(2, 'little'))
    parsed = RefereeFrameAssembler().push_bytes(frame)[0].to_dict()['parsed']['radar_mark_progress']

    assert parsed['valid_radar_mark_progress_length'] is True
    assert parsed['enemy']['opponent_hero'] is True
    assert parsed['enemy']['opponent_sentry'] is True
    assert parsed['ally']['ally_engineer'] is True
    assert parsed['ally']['ally_sentry'] is True
    assert parsed['ally']['ally_hero'] is False
    assert parsed['aerial_countermeasure'] == {
        'opponent_aerial_aimed_by_ally_radar': True,
        'opponent_aerial_countered': False,
        'ally_aerial_aimed_by_opponent_radar': False,
        'ally_aerial_countered': True,
    }


def test_parse_020e_rejects_wrong_length():
    parsed = parse_radar_decision_sync([])

    assert parsed == {'valid_radar_info_length': False}
    assert parse_game_status([]) == {'valid_game_status_length': False}
    assert parse_dart_status([]) == {'valid_dart_status_length': False}
    assert parse_radar_mark_progress([]) == {'valid_radar_mark_progress_length': False}


def test_interference_setters_follow_field_side_and_level_table():
    assert interference_setters_for_side_level('red', 1) == {'center_f': 432200000, 'BW_ganrao': 940000}
    assert interference_setters_for_side_level('red', 3) == {'center_f': 432800000, 'BW_ganrao': 250000}
    assert interference_setters_for_side_level('blue', 1) == {'center_f': 434920000, 'BW_ganrao': 940000}
    assert interference_setters_for_side_level('blue', 3) == {'center_f': 434320000, 'BW_ganrao': 250000}


def test_interference_rx_setters_follow_receiver_flowgraph_names():
    red_level_2 = interference_rx_setters_for_side_level('red', 2)
    blue_level_3 = interference_rx_setters_for_side_level('blue', 3)

    assert red_level_2['cen_f'] == 432500000
    assert red_level_2['center_F'] == red_level_2['cen_f']
    assert red_level_2['BW'] == 860000
    assert red_level_2['bw_re'] == 860000
    assert red_level_2['LowPass'] == 500000
    assert red_level_2['rx_tracking'] is True
    assert blue_level_3['cen_f'] == 434320000
    assert blue_level_3['center_F'] == blue_level_3['cen_f']
    assert blue_level_3['LowPass'] == 160000
    assert blue_level_3['rx_tracking'] is True


def test_broadcast_rx_setters_keep_strong_adjacent_channel_profile():
    setters = broadcast_rx_setters_for_side('red')

    assert setters['cen_f'] == 433200000
    assert setters['GainMode'] == 'manual'
    assert setters['Gain'] == 20
    assert setters['LowPass'] == 260000
    assert broadcast_rx_setters_for_side('red', -1)['Gain'] == -1
    assert broadcast_rx_setters_for_side('red', 73)['Gain'] == 73
    with pytest.raises(ValueError):
        broadcast_rx_setters_for_side('red', -2)
    with pytest.raises(ValueError):
        broadcast_rx_setters_for_side('red', 74)


def test_gfsk_timing_and_sensitivity_follow_v2_rule_book():
    assert GFSK_FLOWGRAPH_SPS == 94
    assert BROADCAST_RX_SEN_RE == 1.5628 / 2.0
    assert INTERFERENCE_RX_SEN_RE == {
        1: 2.8194 / 2.0,
        2: 2.5681 / 2.0,
        3: 0.6517 / 2.0,
    }
    assert GFSK_TX_FLOWGRAPH_SAMPLE_RATE == 1_000_000
    assert GFSK_TX_FLOWGRAPH_SPS == 47
    assert GFSK_TX_FLOWGRAPH_SPS / GFSK_TX_FLOWGRAPH_SAMPLE_RATE == pytest.approx(
        47 / 1_000_000,
        abs=1e-10,
    )


def test_assemble_broadcast_cycle_across_air_packets():
    cycle = b''.join([
        build_referee_frame(0x0A01, bytes(range(24))),
        build_referee_frame(0x0A02, bytes(range(12))),
        build_referee_frame(0x0A03, bytes(range(10))),
        build_referee_frame(0x0A04, bytes(range(8))),
        build_referee_frame(0x0A05, bytes(range(41))),
    ])
    assembler = RefereeFrameAssembler()
    frames = []
    for chunk in _air_chunks(cycle):
        frames.extend(assembler.push_air_payload(chunk))

    assert [frame.cmd_id for frame in frames] == [0x0A01, 0x0A02, 0x0A03, 0x0A04, 0x0A05]
    assert frames[0].to_dict()['parsed']['positions_cm']['hero'] == {'x': 256, 'y': 770}
    assert frames[3].to_dict()['parsed']['remaining_coins'] == 256
    assert frames[4].to_dict()['parsed']['sentry_mode'] == 35
    assert frames[4].to_dict()['parsed']['robot_main_status']['opponent_sentry'] == 40


def test_air_assembler_filters_non_radar_wireless_commands():
    assembler = RefereeFrameAssembler(allowed_cmd_lengths=RADAR_AIR_DATA_LENGTHS)
    raw = build_referee_frame(0x020E, b'\x08') + build_referee_frame(0x0A06, b'KEY123')

    frames = assembler.push_bytes(raw)

    assert [frame.cmd_id for frame in frames] == [0x0A06]
    assert frames[0].to_dict()['parsed']['password'] == 'KEY123'


def test_air_assembler_filters_wrong_radar_wireless_lengths():
    assembler = RefereeFrameAssembler(allowed_cmd_lengths=RADAR_AIR_DATA_LENGTHS)
    raw = build_referee_frame(0x0A06, b'ABC1234') + build_referee_frame(0x0A06, b'KEY123')

    frames = assembler.push_bytes(raw)

    assert [frame.cmd_id for frame in frames] == [0x0A06]
    assert frames[0].to_dict()['parsed']['password'] == 'KEY123'


def test_assembler_diagnostics_observe_rejections_without_repairing_frames():
    assembler = RefereeFrameAssembler(allowed_cmd_lengths=RADAR_AIR_DATA_LENGTHS)
    bad_crc8 = bytearray(build_referee_frame(0x0A06, b'CRC8NO'))
    bad_crc8[4] ^= 0x01
    bad_crc16 = bytearray(build_referee_frame(0x0A06, b'CRC16!'))
    bad_crc16[-1] ^= 0x01
    wrong_length = build_referee_frame(0x0A06, b'TOOLONG')
    valid = build_referee_frame(0x0A06, b'KEY123')

    frames = assembler.push_bytes(bytes(bad_crc8) + bytes(bad_crc16) + wrong_length + valid)

    assert [frame.data for frame in frames] == [b'KEY123']
    diagnostics = assembler.diagnostics()
    assert diagnostics['crc8_failures'] == 1
    assert diagnostics['crc16_failures'] == 1
    assert diagnostics['command_length_failures'] == 1
    assert diagnostics['crc16_valid_frames'] == 2
    assert diagnostics['valid_frames'] == 1
    assert [event['stage'] for event in assembler.pop_diagnostic_events()] == [
        'crc8_failure',
        'crc16_failure',
        'command_length_failure',
        'valid',
    ]


def test_assembler_clear_resets_diagnostic_counters_and_events():
    assembler = RefereeFrameAssembler()
    frame = bytearray(build_referee_frame(0x0A06, b'ABC123'))
    frame[-1] ^= 1
    assert assembler.push_bytes(frame) == []
    assert assembler.diagnostics()['crc16_failures'] == 1

    assembler.clear()

    assert assembler.diagnostics()['crc16_failures'] == 0
    assert assembler.pop_diagnostic_events() == []


def test_parse_radar_macro_status_bits_from_protocol_table():
    # bits: supply zone, central highland=2, trapezoid, fortress=3, outpost=1,
    # base, and all terrain-crossing RFID flags set.
    occupation_bits = (
        (1 << 0)
        | (2 << 1)
        | (1 << 3)
        | (3 << 4)
        | (1 << 6)
        | (1 << 8)
        | (1 << 9)
        | (1 << 10)
        | (1 << 11)
        | (1 << 12)
        | (1 << 13)
        | (1 << 14)
        | (1 << 15)
    )
    payload = bytes([0x34, 0x12, 0x78, 0x56]) + occupation_bits.to_bytes(4, 'little')
    frame = build_referee_frame(0x0A04, payload)
    parsed = RefereeFrameAssembler().push_bytes(frame)[0].to_dict()['parsed']

    assert parsed['remaining_coins'] == 0x1234
    assert parsed['total_coins'] == 0x5678
    assert parsed['occupation']['opponent_central_highland_status'] == 2
    assert parsed['occupation']['opponent_fortress_buff_zone_status'] == 3
    assert parsed['occupation']['opponent_outpost_buff_zone_status'] == 1
    assert parsed['occupation']['opponent_terrain_crossing_road_rfid'] is True


def test_parse_radar_buff_status_from_protocol_table():
    payload = bytes(range(35)) + bytes([6, 0, 1, 2, 3, 9])
    frame = build_referee_frame(0x0A05, payload)
    parsed = RefereeFrameAssembler().push_bytes(frame)[0].to_dict()['parsed']

    hero = parsed['buff_status']['opponent_hero']
    sentry = parsed['buff_status']['opponent_sentry']
    assert hero == {
        'hp_recovery_percent': 0,
        'shooting_heat_cooling': 0x0201,
        'defense_percent': 3,
        'negative_defense_percent': 4,
        'attack_percent': 0x0605,
    }
    assert sentry['hp_recovery_percent'] == 28
    assert sentry['shooting_heat_cooling'] == 0x1E1D
    assert sentry['attack_percent'] == 0x2221
    assert parsed['sentry_mode'] == 6
    assert parsed['sentry_mode_name'] == 'enhanced_mobile'
    assert parsed['robot_main_status'] == {
        'opponent_hero': 0,
        'opponent_engineer': 1,
        'opponent_infantry_3': 2,
        'opponent_infantry_4': 3,
        'opponent_sentry': 9,
    }
    assert parsed['robot_main_status_names'] == {
        'opponent_hero': 'alive',
        'opponent_engineer': 'destroyed',
        'opponent_infantry_3': 'invincible_not_weakened',
        'opponent_infantry_4': 'invincible_weakened',
        'opponent_sentry': 'unknown_9',
    }


def test_air_assembler_rejects_obsolete_36_byte_0a05_frame():
    assembler = RefereeFrameAssembler(allowed_cmd_lengths=RADAR_AIR_DATA_LENGTHS)
    obsolete = build_referee_frame(0x0A05, bytes(36))
    current = build_referee_frame(0x0A05, bytes(41))

    frames = assembler.push_bytes(obsolete + current)

    assert [len(frame.data) for frame in frames] == [41]


def test_assembler_resynchronizes_after_noise_and_bad_crc():
    good = build_referee_frame(0x0A06, b'KEY999')
    bad = bytearray(build_referee_frame(0x0A06, b'BAD999'))
    bad[-1] ^= 0xFF

    assembler = RefereeFrameAssembler()
    frames = assembler.push_bytes([0x00, 0xA5, 0x11]) + assembler.push_bytes(bad) + assembler.push_bytes(good)

    assert len(frames) == 1
    assert frames[0].to_dict()['parsed']['password'] == 'KEY999'


def test_extract_air_payload_from_demodulated_bits():
    frame = build_referee_frame(0x0A06, b'BIT123')
    air_payload = _air_chunks(frame)[0]
    packet = ACCESS_CODES['interference'] + b'\x00\x0F\x00\x0F' + bytes(air_payload)
    bit_bytes = bytes_to_bits(packet)

    extractor = AccessCodeAirPacketExtractor()
    payloads = extractor.push_bit_bytes([0, 1, 0] + bit_bytes[:80])
    payloads += extractor.push_bit_bytes(bit_bytes[80:])

    assert payloads == [bytes(air_payload)]
    assembler = RefereeFrameAssembler()
    frames = assembler.push_air_payload(payloads[0])
    assert frames[0].to_dict()['parsed']['password'] == 'BIT123'


def test_extract_air_payload_can_be_limited_to_one_access_code():
    frame = build_referee_frame(0x0A06, b'BIT123')
    air_payload = _air_chunks(frame)[0]
    packet = ACCESS_CODES['interference'] + b'\x00\x0F\x00\x0F' + bytes(air_payload)
    bit_bytes = bytes_to_bits(packet)

    payloads = AccessCodeAirPacketExtractor(access_codes=[ACCESS_CODES['broadcast']]).push_bit_bytes(bit_bytes)

    assert payloads == []


def test_extract_air_payload_tolerates_access_bit_errors_by_default():
    frame = build_referee_frame(0x0A06, b'ERR123')
    air_payload = _air_chunks(frame)[0]
    packet = ACCESS_CODES['interference'] + b'\x00\x0F\x00\x0F' + bytes(air_payload)
    bit_bytes = bytes_to_bits(packet)
    for index in (0, 17, 63):
        bit_bytes[index] = 1 - bit_bytes[index]

    payloads = AccessCodeAirPacketExtractor().push_bit_bytes(bit_bytes)

    assert payloads == [bytes(air_payload)]
    frames = RefereeFrameAssembler().push_air_payload(payloads[0])
    assert frames[0].to_dict()['parsed']['password'] == 'ERR123'


def test_extract_air_payload_handles_inverted_bits_by_default():
    frame = build_referee_frame(0x0A06, b'INV123')
    air_payload = _air_chunks(frame)[0]
    packet = ACCESS_CODES['interference'] + b'\x00\x0F\x00\x0F' + bytes(air_payload)
    bit_bytes = [1 - bit for bit in bytes_to_bits(packet)]

    payloads = AccessCodeAirPacketExtractor().push_bit_bytes(bit_bytes)

    assert payloads == [bytes(air_payload)]
    frames = RefereeFrameAssembler().push_air_payload(payloads[0])
    assert frames[0].to_dict()['parsed']['password'] == 'INV123'


def test_extract_air_payload_strict_mode_rejects_access_bit_errors():
    frame = build_referee_frame(0x0A06, b'BAD123')
    air_payload = _air_chunks(frame)[0]
    packet = ACCESS_CODES['interference'] + b'\x00\x0F\x00\x0F' + bytes(air_payload)
    bit_bytes = bytes_to_bits(packet)
    bit_bytes[0] = 1 - bit_bytes[0]

    payloads = AccessCodeAirPacketExtractor(max_access_hamming=0).push_bit_bytes(bit_bytes)

    assert payloads == []


def test_build_and_parse_robot_interaction_0212_payload():
    data = build_robot_interaction_data(0x0212, 9, 7, bytes(range(8)))
    parsed = parse_robot_interaction_data(data)

    assert data[:6] == bytes([0x12, 0x02, 0x09, 0x00, 0x07, 0x00])
    assert parsed['data_cmd_id'] == 0x0212
    assert parsed['sender_id'] == 9
    assert parsed['receiver_id'] == 7
    assert parsed['user_data_hex'] == '0001020304050607'


def test_build_robot_interaction_frame_0121_radar_cmd():
    user_data = build_radar_cmd_payload(radar_cmd=1, password_cmd=2, password='ABC123')
    raw = build_robot_interaction_frame(0x0121, 9, 0x8080, user_data, seq=7)
    frame = RefereeFrameAssembler().push_bytes(raw)[0]
    parsed = frame.to_dict()['parsed']

    assert frame.cmd_id == 0x0301
    assert frame.seq == 7
    assert parsed['robot_interaction']['data_cmd_id'] == 0x0121
    assert parsed['robot_interaction']['sender_id'] == 9
    assert parsed['robot_interaction']['receiver_id'] == 0x8080
    assert parsed['radar_cmd'] == {
        'valid_radar_cmd_length': True,
        'radar_cmd': 1,
        'password_cmd': 2,
        'password': 'ABC123',
        'password_is_alnum_ascii': True,
    }


def test_build_robot_interaction_frame_0121_radar_cmd_only():
    user_data = build_radar_cmd_payload(radar_cmd=2)
    raw = build_robot_interaction_frame(0x0121, 9, 0x8080, user_data, seq=8)
    frame = RefereeFrameAssembler().push_bytes(raw)[0]
    parsed = frame.to_dict()['parsed']

    assert user_data == b'\x02\x00\x00\x00\x00\x00\x00\x00'
    assert parsed['robot_interaction']['data_cmd_id'] == 0x0121
    assert parsed['radar_cmd'] == {
        'valid_radar_cmd_length': True,
        'radar_cmd': 2,
        'password_cmd': 0,
    }


def test_parse_radar_cmd_rejects_pre_v2_one_byte_payload():
    raw = build_robot_interaction_frame(0x0121, 9, 0x8080, b'\x02', seq=8)
    parsed = RefereeFrameAssembler().push_bytes(raw)[0].to_dict()['parsed']['radar_cmd']

    assert parsed == {
        'valid_radar_cmd_length': False,
        'radar_cmd': 2,
    }


def test_build_radar_cmd_payload_rejects_non_protocol_password_commands():
    with pytest.raises(ValueError, match='password_cmd'):
        build_radar_cmd_payload(radar_cmd=1, password_cmd=3, password='ABC123')

    with pytest.raises(ValueError, match='password is required'):
        build_radar_cmd_payload(radar_cmd=1, password_cmd=2)

    with pytest.raises(ValueError, match='uint8'):
        build_radar_cmd_payload(radar_cmd=256)
