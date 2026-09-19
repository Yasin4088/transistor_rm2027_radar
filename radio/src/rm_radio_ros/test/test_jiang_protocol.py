import importlib.util
import struct
import sys
import types
from pathlib import Path

import numpy as np

from rm_radio_ros.core.rm_protocol import RefereeFrameAssembler
from rm_radio_ros.core.radar_wireless_constraints import (
    BUFF_FIELD_MAX,
    BULLET_MAX,
    HP_MAX,
    POSITION_X_MAX_CM,
    POSITION_Y_MAX_CM,
    normalize_occupation_bits,
)


def _load_jiang():
    gnuradio = types.ModuleType('gnuradio')
    gr = types.ModuleType('gnuradio.gr')

    class BasicBlock:
        def __init__(self, *args, **kwargs):
            pass

        def message_port_register_out(self, *_args, **_kwargs):
            pass

        def message_port_pub(self, *_args, **_kwargs):
            pass

    gr.basic_block = BasicBlock
    gr.sync_block = BasicBlock
    gnuradio.gr = gr

    pmt = types.ModuleType('pmt')
    pmt.PMT_NIL = object()
    pmt.intern = lambda value: value
    pmt.init_u8vector = lambda _length, data: list(data)
    pmt.cons = lambda meta, data: (meta, data)

    sys.modules.setdefault('gnuradio', gnuradio)
    sys.modules.setdefault('gnuradio.gr', gr)
    sys.modules.setdefault('pmt', pmt)

    module_path = Path(__file__).resolve().parents[1] / 'flowgraphs' / 'ganraoyuan' / 'jiang.py'
    spec = importlib.util.spec_from_file_location('ganraoyuan_jiang_test', module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


jiang_mod = _load_jiang()


def _collect_payloads(generator, count):
    return [generator._messages()[2] for _ in range(count)]


def test_crc8_matches_official_little_endian_header():
    assert jiang_mod._crc8_rm([0xA5, 0x16, 0x00, 0x01]) == 0x06


def test_frame_uses_little_endian_length_cmd_and_whole_frame_crc16():
    generator = jiang_mod.jiang(com_id=[0x0A, 0x01], payload_size=24)
    frame = generator._build_frame([0x0A, 0x01], [0] * 24)
    next_frame = generator._build_frame([0x0A, 0x01], [0] * 24)

    assert frame[:7] == [0xA5, 0x18, 0x00, 0x01, jiang_mod._crc8_rm(frame[:4]), 0x01, 0x0A]
    assert next_frame[3] == 0x02
    expected_crc = jiang_mod._crc16_rm(frame[:-2])
    assert frame[-2:] == [expected_crc & 0xFF, expected_crc >> 8]


def test_broadcast_stream_is_1400_bytes_per_second_without_period_padding():
    generator = jiang_mod.jiang(
        Access_Code=[0x2F, 0x6F, 0x4C, 0x74, 0xB9, 0x14, 0x49, 0x2E],
        Period=100,
        command_cycle=[
            {'cmd_id': [0x0A, 0x01], 'payload_size': 24},
            {'cmd_id': [0x0A, 0x02], 'payload_size': 12},
            {'cmd_id': [0x0A, 0x03], 'payload_size': 10},
            {'cmd_id': [0x0A, 0x04], 'payload_size': 8},
            {'cmd_id': [0x0A, 0x05], 'payload_size': 41},
        ],
    )

    # Three 140-byte/100-ms cycles align to exactly 28 15-byte air payloads.
    payloads = _collect_payloads(generator, 28)
    assert all(len(payload) == 15 for payload in payloads)
    assert generator._publish_interval_seconds() == 15 / 1400
    assert generator._publish_interval_seconds() > 216 * 47 / 1_000_000
    assert payloads[0][:7] == [0xA5, 0x18, 0x00, 0x01, jiang_mod._crc8_rm(payloads[0][:4]), 0x01, 0x0A]
    assert payloads[2][3:10] == [0xA5, 0x0C, 0x00, 0x02, jiang_mod._crc8_rm([0xA5, 0x0C, 0x00, 0x02]), 0x02, 0x0A]

    assembler = RefereeFrameAssembler()
    frames = []
    for payload in payloads:
        frames.extend(assembler.push_air_payload(payload))
    assert [frame.cmd_id for frame in frames] == [
        0x0A01, 0x0A02, 0x0A03, 0x0A04, 0x0A05,
    ] * 3
    assert all(len(frame.data) == 41 for frame in frames if frame.cmd_id == 0x0A05)


def test_short_broadcast_stream_fills_period_to_1400_bytes_per_second(monkeypatch):
    monkeypatch.setattr(jiang_mod.random, 'getrandbits', lambda _bits: 0xCC)
    generator = jiang_mod.jiang(
        Access_Code=[0x2F, 0x6F, 0x4C, 0x74, 0xB9, 0x14, 0x49, 0x2E],
        Period=100,
        command_cycle=[
            {'cmd_id': [0x0A, 0x04], 'payload_size': 8},
        ],
    )

    payloads = _collect_payloads(generator, 28)

    assert len(payloads) == 28
    assert generator._publish_interval_seconds() == 15 / 1400
    assert payloads[0][:7] == [0xA5, 0x08, 0x00, 0x01, jiang_mod._crc8_rm(payloads[0][:4]), 0x04, 0x0A]
    assert payloads[1][2:] == [0xCC] * 13
    assert payloads[-1] == [0xCC] * 15

    assembler = RefereeFrameAssembler()
    frames = []
    for payload in payloads:
        frames.extend(assembler.push_air_payload(payload))
    assert [frame.cmd_id for frame in frames] == [0x0A04] * 3


def test_broadcast_auto_change_rebuilds_legal_command_payload_and_crc_each_period():
    jiang_mod.random.seed(2026)
    generator = jiang_mod.jiang(
        command_cycle=[
            {
                'cmd_id': [0x0A, 0x04],
                'payload_size': 8,
                'randomize_payload': True,
            },
        ],
    )

    first = generator._current_frames()
    second = generator._current_frames()

    assert first[7:-2] != second[7:-2]
    for frame in (first, second):
        remaining, total, occupation = struct.unpack('<HHI', bytes(frame[7:-2]))
        assert 0 <= remaining <= total <= 0xFFFF
        assert occupation == normalize_occupation_bits(occupation)
        crc = jiang_mod._crc16_rm(frame[:-2])
        assert frame[-2:] == [crc & 0xFF, crc >> 8]


def test_broadcast_auto_change_obeys_every_2026_field_range():
    generator = jiang_mod.jiang()
    specs = [
        ([0x0A, 0x01], 24),
        ([0x0A, 0x02], 12),
        ([0x0A, 0x03], 10),
        ([0x0A, 0x04], 8),
        ([0x0A, 0x05], 41),
    ]

    for _ in range(200):
        position = generator._random_payload_data_for_command(*specs[0])
        coords = struct.unpack('<12H', bytes(position))
        assert all(0 <= coords[index] <= POSITION_X_MAX_CM for index in range(0, 12, 2))
        assert all(0 <= coords[index] <= POSITION_Y_MAX_CM for index in range(1, 12, 2))

        hp = struct.unpack('<6H', bytes(generator._random_payload_data_for_command(*specs[1])))
        assert all(value <= maximum for value, maximum in zip(hp, HP_MAX.values()))
        assert hp[4] == 0

        bullets = struct.unpack('<5H', bytes(generator._random_payload_data_for_command(*specs[2])))
        assert all(value <= maximum for value, maximum in zip(bullets, BULLET_MAX.values()))

        remaining, total, occupation = struct.unpack(
            '<HHI', bytes(generator._random_payload_data_for_command(*specs[3]))
        )
        assert remaining <= total
        assert occupation == normalize_occupation_bits(occupation)

        buffs = generator._random_payload_data_for_command(*specs[4])
        for index, key in enumerate(BUFF_FIELD_MAX['attack_percent']):
            offset = index * 7
            recovery = buffs[offset]
            cooling = buffs[offset + 1] | (buffs[offset + 2] << 8)
            defense = buffs[offset + 3]
            negative = buffs[offset + 4]
            attack = buffs[offset + 5] | (buffs[offset + 6] << 8)
            assert recovery <= BUFF_FIELD_MAX['hp_recovery_percent'][key]
            assert cooling <= BUFF_FIELD_MAX['shooting_heat_cooling'][key]
            assert defense <= BUFF_FIELD_MAX['defense_percent'][key]
            assert negative <= BUFF_FIELD_MAX['negative_defense_percent'][key]
            assert attack <= BUFF_FIELD_MAX['attack_percent'][key]
        assert 1 <= buffs[35] <= 6
        assert all(0 <= value <= 3 for value in buffs[36:41])


def test_single_frame_messages_path_keeps_exact_15_byte_chunking():
    generator = jiang_mod.jiang(com_id=[0x0A, 0x06], payload_size=6)
    access, header, payload = generator._messages()

    assert len(access) == 8
    assert header == [0x00, 0x0F, 0x00, 0x0F]
    assert len(payload) == 15
    assert payload[:7] == [0xA5, 0x06, 0x00, 0x01, jiang_mod._crc8_rm(payload[:4]), 0x06, 0x0A]
    assert bytes(payload[7:13]) == b'ABC123'


def test_interference_stream_inserts_0a06_into_random_byte_stream_at_1350_bytes_per_second(monkeypatch):
    monkeypatch.setattr(jiang_mod.random, 'randrange', lambda _start, _stop: 7)
    monkeypatch.setattr(jiang_mod.random, 'getrandbits', lambda _bits: 0x5A)
    generator = jiang_mod.jiang(
        Access_Code=[0x16, 0xE8, 0xD3, 0x77, 0x15, 0x1C, 0x71, 0x2D],
        Period=100,
        com_id=[0x0A, 0x06],
        payload_size=6,
    )

    payloads = [generator._next_interference_payload() for _ in range(9)]

    assert len(payloads) == 9
    assert generator._interference_chunks_per_period() == 9
    assert generator._publish_interval_seconds() == 100 / 1000 / 9
    assert all(len(payload) == 15 for payload in payloads)
    assert payloads[0][:7] == [0x5A] * 7
    assert payloads[0][7:] == [0xA5, 0x06, 0x00, 0x01, jiang_mod._crc8_rm([0xA5, 0x06, 0x00, 0x01]), 0x06, 0x0A, ord('A')]
    assert payloads[1][:5] == [ord('B'), ord('C'), ord('1'), ord('2'), ord('3')]

    assembler = RefereeFrameAssembler()
    frames = []
    for payload in payloads:
        frames.extend(assembler.push_air_payload(payload))

    assert len(frames) == 1
    assert frames[0].cmd_id == 0x0A06
    assert frames[0].data == b'ABC123'


def test_scheduler_clocked_source_fills_every_item_and_inserts_explicit_idle_bits():
    symbol_rate = 1_000_000 / 47
    generator = jiang_mod.jiang(
        Access_Code=[0x16, 0xE8, 0xD3, 0x77, 0x15, 0x1C, 0x71, 0x2D],
        Period=100,
        com_id=[0x0A, 0x06],
        payload_size=6,
        symbol_rate=symbol_rate,
    )
    output = np.empty(round(symbol_rate), dtype=np.uint8)

    produced = generator.work([], [output])
    diagnostics = generator.get_stream_diagnostics()

    assert produced == output.size
    assert set(np.unique(output)).issubset({0, 1})
    assert diagnostics['mode'] == 'scheduler_clocked_continuous_bits'
    assert diagnostics['packets_started'] == 90
    assert diagnostics['packet_symbols_emitted'] == 90 * 216
    assert diagnostics['idle_symbols_emitted'] == output.size - 90 * 216
    assert diagnostics['idle_symbols_emitted'] > 0
    assert set(diagnostics['recent_packet_intervals']).issubset({236, 237})
    assert diagnostics['packet_rate_hz'] == 90.0


def test_scheduler_clocked_source_emits_air_bytes_msb_first():
    generator = jiang_mod.jiang(
        Access_Code=[0x2F, 0x6F, 0x4C, 0x74, 0xB9, 0x14, 0x49, 0x2E],
    )
    output = np.empty(12 * 8, dtype=np.uint8)

    assert generator.work([], [output]) == output.size

    reconstructed = []
    for offset in range(0, output.size, 8):
        value = 0
        for bit in output[offset : offset + 8]:
            value = (value << 1) | int(bit)
        reconstructed.append(value)
    assert reconstructed == [
        0x2F, 0x6F, 0x4C, 0x74, 0xB9, 0x14, 0x49, 0x2E,
        0x00, 0x0F, 0x00, 0x0F,
    ]


def test_interference_tx_flowgraph_uses_dynamic_jiang_stream_not_fixed_packet_source():
    rm_py = Path(__file__).resolve().parents[1] / 'flowgraphs' / 'ganraoyuan' / 'RM.py'
    source = rm_py.read_text(encoding='utf-8')

    assert 'fixed_interference_packet_source' not in source
    assert 'vector_source_b' not in source
    assert '_env_tx_buffer_size' in source
    assert "os.environ.get('RM_RADIO_TX_BUFFER_SIZE', '1048576')" in source
    assert 'fmcomms2_sink_fc32(self.interference_tx_uri, [True, True, False, False], tx_buffer_size, False)' in source
    assert 'blocks.throttle' not in source
    assert 'pdu_to_tagged_stream' not in source
    assert 'tagged_stream_mux' not in source
    assert 'self.connect((self.jiang_0, 0), (self.digital_gfsk_mod_0_0, 0))' in source
    assert 'do_unpack=False' in source
    assert 'symbol_rate=symbol_rate' in source
    assert '_DEFAULT_TX_SAMPLE_RATE = 1_000_000' in source
    assert '_OFFICIAL_SYMBOL_RATE = 1_000_000.0 / 47.0' in source
    assert 'self.sps = sps = _sps_for_sample_rate(sample_rate)' in source
