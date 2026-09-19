from rm_radio_ros.core.virtual_link_test import _buff_payload, _default_broadcast_config, run_virtual_link


def test_v2_buff_payload_is_41_bytes_and_upgrades_old_raw_config():
    payload = _buff_payload(_default_broadcast_config())
    upgraded = _buff_payload({'buff_raw': list(range(36))})

    assert len(payload) == 41
    assert len(upgraded) == 41
    assert upgraded[1:3] == bytes([200, 0])  # Hero cooling is capped at +200/s.
    assert upgraded[35] == 6
    assert upgraded[36:] == bytes(5)


def test_red_tx_blue_rx_virtual_link_decodes_broadcast_and_all_interference_levels():
    result = run_virtual_link(tx_side='red', rx_side='blue')

    assert [frame['cmd_hex'] for frame in result['broadcast']['frames']] == [
        '0x0A01',
        '0x0A02',
        '0x0A03',
        '0x0A04',
        '0x0A05',
    ]
    assert result['broadcast']['air_packet_count'] == 10
    assert result['broadcast']['air_payload_count'] == 10
    assert [item['type'] for item in result['broadcast']['bridge_outputs']] == [
        'RadarInfoToClient',
        'RadarEnemyHp',
        'RadarEnemyBulletAllowance',
        'RadarEnemyMacroStatus',
        'RadarEnemyBuffStatus',
    ]

    assert result['frequency_plan']['broadcast_hz'] == 433200000
    assert result['frequency_plan']['interference'][1] == {'center_f': 432200000, 'BW_ganrao': 940000}
    assert result['frequency_plan']['interference'][2] == {'center_f': 432500000, 'BW_ganrao': 860000}
    assert result['frequency_plan']['interference'][3] == {'center_f': 432800000, 'BW_ganrao': 250000}

    for level, password in {1: 'R1L001', 2: 'R2L002', 3: 'R3L003'}.items():
        decoded = result['interference'][level]
        assert decoded['air_packet_count'] == 9
        assert decoded['air_payload_count'] == 9
        assert [frame['cmd_hex'] for frame in decoded['frames']] == ['0x0A06']
        assert decoded['frames'][0]['parsed']['password'] == password
        assert decoded['bridge_outputs'][0]['type'] == 'RadarCommand0121'
        assert decoded['bridge_outputs'][0]['payload']['password'] == password


def test_virtual_link_uses_custom_broadcast_and_password_with_optional_noise():
    result = run_virtual_link(
        tx_side='blue',
        rx_side='red',
        broadcast_config={
            'enabled_cmds': ['0x0A01', '0x0A02', '0x0A05'],
            'robots': {'opponent_hero': {'x': 2222, 'y': 1333}},
            'hp': {'opponent_hero': 321},
            'buffs': {'opponent_hero': {'attack_percent': 88}},
            'sentry_mode': 4,
            'robot_main_status': {'opponent_hero': 2, 'opponent_sentry': 1},
        },
        interference_password='ZXCV12',
        interference_level=3,
        bit_error_rate=0.0,
        random_seed=7,
    )

    assert result['frequency_plan']['broadcast_hz'] == 433920000
    assert [frame['cmd_hex'] for frame in result['broadcast']['frames']] == ['0x0A01', '0x0A02', '0x0A05']
    assert result['broadcast']['frames'][0]['parsed']['official_positions_cm']['opponent_hero'] == {'x': 2222, 'y': 1333}
    assert result['broadcast']['frames'][1]['parsed']['official_hp']['opponent_hero'] == 321
    assert result['broadcast']['frames'][2]['parsed']['buff_status']['opponent_hero']['attack_percent'] == 88
    assert result['broadcast']['frames'][2]['parsed']['sentry_mode'] == 4
    assert result['broadcast']['frames'][2]['parsed']['sentry_mode_name'] == 'enhanced_offensive'
    assert result['broadcast']['frames'][2]['parsed']['robot_main_status']['opponent_hero'] == 2
    assert result['broadcast']['frames'][2]['parsed']['robot_main_status']['opponent_sentry'] == 1
    assert list(result['interference']) == [3]
    assert result['interference'][3]['frames'][0]['parsed']['password'] == 'ZXCV12'
    assert result['broadcast']['bit_error_count'] == 0
