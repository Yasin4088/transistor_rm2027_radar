import struct

from rm_radio_ros.core.radar_integration import RadarFusionState, build_0305_payload


def _vision(side='red', **positions):
    robots = {}
    for name, position in positions.items():
        robots[name] = {
            'position_cm': {'x': position[0], 'y': position[1]},
            'valid': True,
            'state': 'measured',
            'confidence': 0.9,
            'observed_at': 100.0,
        }
    return {
        'schema': 'transistor.radar.telemetry.v1',
        'source_time': 100.0,
        'side': side,
        'vision': {'camera_ready': True, 'fps': 20.0},
        'robots': robots,
    }


def _radio(*positions):
    robots = [
        {'target_pos_x': position[0], 'target_pos_y': position[1], 'is_high_light': 0}
        for position in positions
    ]
    return {
        'type': 'RadarInfoToClient',
        'payload': {'RadarSingleRobotInfo': robots},
    }


def test_radio_is_fresh_at_0999_and_exactly_one_second_then_immediately_falls_back():
    state = RadarFusionState('red')
    state.update_vision(_vision(B1=(100, 200)), received_at=10.75)
    state.update_radio_bridge(_radio((900, 800)), received_at=10.0)

    assert state.snapshot(10.999)['robots']['B1']['final']['source'] == 'radio'
    assert state.snapshot(11.0)['robots']['B1']['final']['source'] == 'radio'

    expired = state.snapshot(11.000001)
    assert expired['robots']['B1']['final']['source'] == 'vision'
    assert expired['robots']['B1']['radio']['expired'] is True
    assert expired['robots']['B1']['radio']['ghost_visible'] is True

    removed = state.snapshot(13.000001)
    assert removed['robots']['B1']['radio']['ghost_visible'] is False


def test_blue_side_maps_radio_positions_to_red_opponents_and_keeps_allies_visual():
    state = RadarFusionState('blue')
    state.update_vision(_vision(side='blue', R1=(100, 200), B1=(300, 400)), received_at=10.0)
    state.update_radio_bridge(_radio((900, 800)), received_at=10.0)

    snapshot = state.snapshot(10.1)
    assert snapshot['robots']['R1']['final'] == {
        'position_cm': {'x': 900, 'y': 800}, 'valid': True, 'source': 'radio'
    }
    assert snapshot['robots']['B1']['final']['source'] == 'vision'


def test_stale_vision_keeps_fresh_radio_opponents_and_zeroes_allies():
    state = RadarFusionState('red')
    state.update_vision(_vision(B1=(100, 200), R1=(300, 400)), received_at=10.0)
    state.update_radio_bridge(_radio((900, 800)), received_at=10.5)

    snapshot = state.snapshot(10.6)
    assert snapshot['vision']['online'] is False
    assert snapshot['robots']['B1']['final']['position_cm'] == {'x': 900, 'y': 800}
    assert snapshot['robots']['R1']['final']['position_cm'] == {'x': 0, 'y': 0}
    assert snapshot['send_allowed'] is True


def test_all_stale_or_zero_suppresses_0305_and_side_mismatch_blocks_send():
    state = RadarFusionState('red', auto_sync_side=False)
    state.update_vision(_vision(B1=(100, 200)), received_at=10.0)
    stale = state.snapshot(12.0)
    assert stale['send_allowed'] is False
    assert stale['send_block_reason'] == 'all_positions_zero_or_stale'

    state.update_vision(_vision(B1=(100, 200)), received_at=20.0)
    state.set_detected_side('blue')
    mismatch = state.snapshot(20.1)
    assert mismatch['send_allowed'] is False
    assert mismatch['send_block_reason'] == 'configured_side_mismatch'


def test_referee_side_hot_sync_recovers_send_without_restart():
    state = RadarFusionState('blue')
    state.update_vision(_vision(side='blue', R1=(100, 200)), received_at=10.0)

    assert state.set_referee_side('red', 'red') is True
    state.update_vision(_vision(side='red', B1=(300, 400)), received_at=10.1)
    snapshot = state.snapshot(10.2)

    assert snapshot['configured_side'] == 'blue'
    assert snapshot['side'] == 'red'
    assert snapshot['side_source'] == 'referee'
    assert snapshot['configured_side_mismatch'] is True
    assert snapshot['side_mismatch'] is False
    assert snapshot['send_allowed'] is True


def test_vision_side_is_fallback_and_last_known_side_survives_sync_loss():
    state = RadarFusionState('blue')
    state.update_vision(_vision(side='red', B1=(100, 200)), received_at=10.0)
    assert state.own_side == 'red'
    assert state.side_source == 'vision'

    state.set_referee_side('blue', 'blue')
    assert state.own_side == 'blue'
    state.set_referee_side(None, None)
    assert state.own_side == 'blue'
    assert state.side_source == 'last_known'

    state.update_vision(_vision(side='red', B1=(300, 400)), received_at=11.0)
    assert state.own_side == 'red'
    assert state.side_source == 'vision'


def test_0305_payload_order_is_opponents_then_allies_for_red_and_is_48_bytes():
    positions = {
        'B1': {'x': 1, 'y': 2},
        'B2': {'x': 3, 'y': 4},
        'R1': {'x': 101, 'y': 102},
    }
    payload = build_0305_payload(positions, 'red')

    assert payload is not None
    assert len(payload) == 48
    values = struct.unpack('<24H', payload)
    assert values[:4] == (1, 2, 3, 4)
    assert values[12:14] == (101, 102)


def test_0305_blue_side_places_visual_allies_in_the_second_protocol_half():
    positions = {
        'R1': {'x': 11, 'y': 12},
        'B1': {'x': 1011, 'y': 1012},
        'B6': {'x': 1061, 'y': 1062},
    }
    payload = build_0305_payload(positions, 'blue')

    assert payload is not None
    values = struct.unpack('<24H', payload)
    assert values[:2] == (11, 12)  # 对方英雄：前 24 B
    assert values[12:14] == (1011, 1012)  # 己方英雄：后 24 B
    assert values[20:22] == (1061, 1062)  # 己方 6 号空中机器人
