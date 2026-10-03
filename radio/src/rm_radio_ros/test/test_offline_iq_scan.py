import numpy as np

from rm_radio_ros.core.offline_iq_scan import (
    _access_patterns,
    _decode_bits_to_referee_frames,
    _merge_decoded_results,
    summarize_scan_result,
)
from rm_radio_ros.core.rm_protocol import (
    ACCESS_CODES,
    AIR_LENGTH_FIELD,
    RADAR_AIR_DATA_LENGTHS,
    RefereeFrameAssembler,
    bits_to_bytes,
    build_referee_frame,
    bytes_to_bits,
)


def _air_chunks(data, size=15):
    chunks = []
    for offset in range(0, len(data), size):
        chunk = list(data[offset : offset + size])
        if len(chunk) < size:
            chunk.extend([0] * (size - len(chunk)))
        chunks.append(chunk)
    return chunks


def test_decode_bits_to_referee_frames_from_air_packets():
    raw_frame = build_referee_frame(0x0A06, b'ABC123')
    bits = []
    for payload in _air_chunks(raw_frame):
        packet = ACCESS_CODES['interference'] + b'\x00\x0F\x00\x0F' + bytes(payload)
        bits.extend(bytes_to_bits(packet))

    decoded = _decode_bits_to_referee_frames(np.array(bits, dtype=np.uint8))

    assert decoded['air_payload_count'] == 1
    assert decoded['referee_frame_count'] == 1
    assert decoded['referee_frames'][0]['cmd_hex'] == '0x0A06'
    assert decoded['referee_frames'][0]['parsed']['password'] == 'ABC123'


def test_decode_bits_to_referee_frames_handles_inverted_bits():
    raw_frame = build_referee_frame(0x0A06, b'ABC123')
    packet = ACCESS_CODES['interference'] + b'\x00\x0F\x00\x0F' + bytes(_air_chunks(raw_frame)[0])
    inverted_bits = 1 - np.array(bytes_to_bits(packet), dtype=np.uint8)

    decoded = _decode_bits_to_referee_frames(inverted_bits)

    assert decoded['air_payload_count'] == 1
    assert decoded['air_payloads'][0]['bit_polarity'] == 'inverted_corrected'
    assert decoded['referee_frames'][0]['cmd_hex'] == '0x0A06'


def test_decode_bits_can_be_limited_to_broadcast_access():
    raw_frame = build_referee_frame(0x0A06, b'ABC123')
    packet = ACCESS_CODES['interference'] + b'\x00\x0F\x00\x0F' + bytes(_air_chunks(raw_frame)[0])
    bits = np.array(bytes_to_bits(packet), dtype=np.uint8)

    decoded = _decode_bits_to_referee_frames(bits, allowed_access_names=('broadcast',))

    assert decoded['air_payload_count'] == 0
    assert decoded['referee_frame_count'] == 0


def test_access_patterns_can_be_limited_to_one_profile():
    patterns = _access_patterns(('broadcast',))

    assert [name for name, _pattern in patterns] == ['broadcast']


def test_summarize_scan_result_keeps_first_decoded_frame():
    frame = {
        'cmd_hex': '0x0A06',
        'command': 'interference_password',
        'raw_hex': 'a5060001060a4142433132330000',
    }
    result = {
        'file': '/tmp/RX_BLUE_ganrao_1',
        'file_duration_seconds': 1.0,
        'start_second': 0.0,
        'best_access_match': {'access': 'interference', 'hamming_distance': 0, 'sps': 94.0, 'offset': 0.0},
        'decoded': {'air_payload_count': 2, 'referee_frame_count': 2, 'referee_frames': [frame, {'cmd_hex': '0x0A06', 'command': 'interference_password', 'raw_hex': 'b5'}]},
    }

    summary = summarize_scan_result(result, frame_limit=1)

    assert summary['file'] == 'RX_BLUE_ganrao_1'
    assert summary['commands'] == [{'cmd_hex': '0x0A06', 'command': 'interference_password'}]
    assert summary['first_referee_frame'] == frame
    assert summary['referee_frame_samples'] == [frame]
    assert summary['referee_frame_samples_truncated'] is True


def test_merge_decoded_results_deduplicates_frames_by_raw_hex():
    target = {
        'air_payloads': [],
        'referee_frames': [],
        'air_payload_count': 0,
        'referee_frame_count': 0,
        'truncated': False,
    }
    first = {
        'air_payloads': [{'access': 'broadcast', 'absolute_bit_index': 1, 'air_payload_hex': 'aa'}],
        'referee_frames': [{'raw_hex': 'a5', 'cmd_hex': '0x0A02'}],
    }
    second = {
        'air_payloads': [
            {'access': 'broadcast', 'absolute_bit_index': 1, 'air_payload_hex': 'aa'},
            {'access': 'broadcast', 'absolute_bit_index': 2, 'air_payload_hex': 'bb'},
        ],
        'referee_frames': [
            {'raw_hex': 'a5', 'cmd_hex': '0x0A02'},
            {'raw_hex': 'b5', 'cmd_hex': '0x0A03'},
        ],
    }

    _merge_decoded_results(target, first, max_payloads=8, max_frames=8)
    _merge_decoded_results(target, second, max_payloads=8, max_frames=8)

    assert target['air_payload_count'] == 2
    assert target['referee_frame_count'] == 2
    assert [frame['cmd_hex'] for frame in target['referee_frames']] == ['0x0A02', '0x0A03']


def _corrupt_payload_hex(frame_bytes, ber, rng):
    bits = np.unpackbits(np.frombuffer(bytes(frame_bytes), dtype=np.uint8))
    noisy = np.bitwise_xor(bits, (rng.random(bits.size) < ber).astype(np.uint8))
    return np.packbits(noisy).tobytes().hex()


def test_f5_time_diversity_recovers_password_frame_from_noisy_copies():
    from rm_radio_ros.core.offline_iq_scan import AIR_PAYLOAD_BYTES, _apply_repeated_frame_voting

    frame = build_referee_frame(0x0A06, b'R3L003', seq=3)
    assert len(frame) == AIR_PAYLOAD_BYTES  # password frame is exactly one air payload
    rng = np.random.default_rng(7)
    ber = 0.06  # per-bit BER at which a single 15-byte copy almost never passes CRC

    payloads = [{'air_payload_hex': _corrupt_payload_hex(frame, ber, rng)} for _ in range(12)]
    # interference TX pads with random-fill payloads; F5 must exclude them from the vote
    payloads += [
        {'air_payload_hex': bytes(rng.integers(0, 256, AIR_PAYLOAD_BYTES, dtype=np.uint8)).hex()}
        for _ in range(6)
    ]

    decoded = {
        'air_payloads': payloads,
        'referee_frames': [],
        'air_payload_count': len(payloads),
        'referee_frame_count': 0,
    }
    _apply_repeated_frame_voting(decoded, max_frames=32)

    assert decoded['referee_frame_count'] >= 1
    frame_dict = decoded['referee_frames'][0]
    assert frame_dict['cmd_hex'] == '0x0A06'
    assert frame_dict['parsed']['password'] == 'R3L003'
    assert frame_dict['source'] == 'time_diversity_vote'


def test_f5_voting_rejects_random_fill_and_requires_min_copies():
    from rm_radio_ros.core.offline_iq_scan import (
        AIR_PAYLOAD_BYTES,
        _apply_repeated_frame_voting,
        _vote_consensus_payloads,
    )

    rng = np.random.default_rng(1)
    fill = [
        {'air_payload_hex': bytes(rng.integers(0, 256, AIR_PAYLOAD_BYTES, dtype=np.uint8)).hex()}
        for _ in range(10)
    ]
    decoded = {'air_payloads': fill, 'referee_frames': [], 'air_payload_count': 10, 'referee_frame_count': 0}
    _apply_repeated_frame_voting(decoded, max_frames=32)
    assert decoded['referee_frame_count'] == 0  # random fill never passes CRC -> no fabricated frame

    frame = build_referee_frame(0x0A06, b'R3L003', seq=3)
    two = [{'air_payload_hex': bytes(frame).hex()}] * 2
    assert _vote_consensus_payloads(two, min_copies=3) == []  # too few copies -> no vote


# ---------------------------------------------------------------------------
# F1 time-diversity soft-symbol combining
# ---------------------------------------------------------------------------

_SPS_SIM = 47
_PASSWORD = b'R1L001'


def _interference_packet_symbols(password=_PASSWORD, seq=9):
    """Nominal +/-1 GFSK symbols for one full interference air packet (access + length + the
    15-byte 0x0A06 password frame), the constant frame the interference TX repeats at 10 Hz."""
    frame = build_referee_frame(0x0A06, password, seq=seq)
    assert len(frame) == 15  # the password frame is exactly one 15-byte air payload
    bits = bytes_to_bits(ACCESS_CODES['interference']) + bytes_to_bits(AIR_LENGTH_FIELD) + bytes_to_bits(frame)
    return np.where(np.array(bits) == 1, 1.0, -1.0)


def _soft_cumsum(packet_sym, n_copies, sigma, rng, gap=40):
    """Build the integrated phase (``cumsum``) of a symbol stream that embeds ``n_copies`` of the
    constant packet separated by random-fill, with per-sample Gaussian noise of std ``sigma``.
    Recovering symbols from this cumsum exercises the real ``_soft_symbols`` matched filter."""
    parts = [rng.choice([-1.0, 1.0], size=int(rng.integers(20, 60)))]
    for _ in range(int(n_copies)):
        parts.append(rng.choice([-1.0, 1.0], size=gap))
        parts.append(packet_sym.copy())
    parts.append(rng.choice([-1.0, 1.0], size=30))
    stream = np.concatenate(parts)
    pd = np.repeat(stream, _SPS_SIM).astype(np.float64) + rng.normal(0.0, sigma, stream.size * _SPS_SIM)
    return np.concatenate([[0.0], np.cumsum(pd)]), pd.size


def _password_recovered(payloads, password=_PASSWORD):
    """True iff any 15-byte payload assembles into a CRC-valid 0x0A06 frame with the password."""
    for payload in payloads:
        assembler = RefereeFrameAssembler(max_buffer_size=4096, allowed_cmd_lengths=RADAR_AIR_DATA_LENGTHS)
        for frame in assembler.push_air_payload(list(payload)):
            if frame.cmd_id == 0x0A06 and frame.data == password:
                return True
    return False


def _enrolled_hard_payloads(cumsum, sample_count, max_access_hamming=8):
    """Hard 15-byte payloads of every copy whose access code matches within the relaxed
    enrolment threshold -- the same pool F1 averages, fed hard to F5 for an apples-to-apples
    comparison."""
    from rm_radio_ros.core.offline_iq_scan import (
        ACCESS_BITS,
        AIR_PACKET_BITS,
        AIR_PAYLOAD_BYTES,
        LENGTH_BITS,
        _access_patterns,
        _match_distances,
        _soft_symbols,
    )

    symbols = _soft_symbols(cumsum, sample_count, _SPS_SIM, 0.0)
    hard = (symbols > 0).astype(np.uint8)
    payload_start = ACCESS_BITS + LENGTH_BITS
    payload_end = payload_start + AIR_PAYLOAD_BYTES * 8
    out, seen = [], set()
    for polarity in ('normal', 'inverted'):
        view = hard if polarity == 'normal' else (1 - hard).astype(np.uint8)
        for _name, pattern in _access_patterns(('interference',)):
            for bit_index in np.flatnonzero(_match_distances(view, pattern) <= int(max_access_hamming)):
                if int(bit_index) in seen:
                    continue
                end = int(bit_index) + AIR_PACKET_BITS
                if end > symbols.size:
                    continue
                seg = symbols[int(bit_index):end] if polarity == 'normal' else -symbols[int(bit_index):end]
                seg_hard = (seg > 0).astype(np.uint8)
                try:
                    payload = bits_to_bytes(seg_hard[payload_start:payload_end])
                except ValueError:
                    continue
                seen.add(int(bit_index))
                out.append({'air_payload_hex': payload.hex()})
    return out


def test_f1_soft_combine_beats_hard_vote_at_high_payload_ber():
    """At a per-symbol BER (~0.11) where no single 15-byte copy passes CRC and hard majority
    voting (F5) has collapsed, soft-symbol combining (F1) still recovers the password from the
    same enrolled copies. This is the ~2 dB soft-over-hard combining edge, end to end."""
    from rm_radio_ros.core.offline_iq_scan import _soft_combine_consensus_payloads, _vote_consensus_payloads

    packet_sym = _interference_packet_symbols()
    # Scale the per-sample noise for the V2 SPS=47 integration window so the
    # per-symbol SNR remains equivalent to the former SPS=52 test case.
    sigma = 4.75
    trials = 30
    single = f5 = f1 = 0
    for trial in range(trials):
        rng = np.random.default_rng(4000 + trial)
        cumsum, sample_count = _soft_cumsum(packet_sym, 12, sigma, rng)
        hard_payloads = _enrolled_hard_payloads(cumsum, sample_count)
        single += int(_password_recovered([bytes.fromhex(p['air_payload_hex']) for p in hard_payloads]))
        f5 += int(_password_recovered(_vote_consensus_payloads(hard_payloads, min_copies=3)))
        f1 += int(
            _password_recovered(
                _soft_combine_consensus_payloads(
                    cumsum, sample_count, sps=_SPS_SIM, offset=0.0,
                    allowed_access_names=('interference',), max_access_hamming=8,
                )
            )
        )
    assert single == 0           # a single noisy 15-byte copy never passes CRC at this BER
    assert f5 <= trials // 3     # hard majority vote has collapsed here (observed ~1/30)
    assert f1 >= 24              # soft combine still recovers the vast majority (observed ~29/30)
    assert f1 > f5 + 10          # and decisively beats hard voting on the identical copy pool


def test_f1_soft_combine_does_not_fabricate_frames_from_noise():
    """A pure-noise stream with no embedded packet must never yield a CRC-valid frame: access
    enrolment is selective and CRC is the sole arbiter, so F1 cannot fabricate a password."""
    from rm_radio_ros.core.offline_iq_scan import _soft_combine_consensus_payloads

    fabricated = 0
    for trial in range(20):
        rng = np.random.default_rng(5000 + trial)
        stream = rng.choice([-1.0, 1.0], size=4000)
        pd = np.repeat(stream, _SPS_SIM).astype(np.float64) + rng.normal(0.0, 1.0, stream.size * _SPS_SIM)
        cumsum = np.concatenate([[0.0], np.cumsum(pd)])
        out = _soft_combine_consensus_payloads(
            cumsum, pd.size, sps=_SPS_SIM, offset=0.0,
            allowed_access_names=('interference',), max_access_hamming=8,
        )
        fabricated += int(_password_recovered(out))
    assert fabricated == 0


def test_f1_soft_combine_requires_min_copies():
    """Below the minimum copy count there is no diversity to exploit -> no consensus payload."""
    from rm_radio_ros.core.offline_iq_scan import _soft_combine_consensus_payloads

    packet_sym = _interference_packet_symbols()
    rng = np.random.default_rng(99)
    cumsum, sample_count = _soft_cumsum(packet_sym, 2, 0.5, rng)  # only 2 copies present
    out = _soft_combine_consensus_payloads(
        cumsum, sample_count, sps=_SPS_SIM, offset=0.0,
        allowed_access_names=('interference',), max_access_hamming=8, min_copies=3,
    )
    assert out == []


def _broadcast_packet_symbols(payload):
    """Nominal +/-1 symbols for a broadcast-access air packet carrying a fixed 15-byte payload.
    Used only to stress the per-access grouping: these never close CRC inside one air payload."""
    assert len(payload) == 15
    bits = bytes_to_bits(ACCESS_CODES['broadcast']) + bytes_to_bits(AIR_LENGTH_FIELD) + bytes_to_bits(payload)
    return np.where(np.array(bits) == 1, 1.0, -1.0)


def _mixed_soft_cumsum(interference_sym, broadcast_sym, n_interf, n_broadcast, sigma, rng, gap=40):
    """A symbol stream that interleaves interference-password copies with MORE broadcast copies,
    so a naive pooled average would let the broadcast majority flip the password bits."""
    parts = [rng.choice([-1.0, 1.0], size=int(rng.integers(20, 60)))]
    blocks = [interference_sym] * int(n_interf) + [broadcast_sym] * int(n_broadcast)
    order = rng.permutation(len(blocks))
    for idx in order:
        parts.append(rng.choice([-1.0, 1.0], size=gap))
        parts.append(blocks[idx].copy())
    parts.append(rng.choice([-1.0, 1.0], size=30))
    stream = np.concatenate(parts)
    pd = np.repeat(stream, _SPS_SIM).astype(np.float64) + rng.normal(0.0, sigma, stream.size * _SPS_SIM)
    return np.concatenate([[0.0], np.cumsum(pd)]), pd.size


def test_f1_soft_combine_groups_by_access_and_survives_broadcast_majority():
    """A window with 3 interference-password copies and 6 broadcast copies, scanned WITHOUT an
    access filter (allowed_access_names=None, the offline/CLI default), must still recover the
    password: copies are averaged per access code, so the broadcast majority cannot outvote and
    corrupt the interference consensus."""
    from rm_radio_ros.core.offline_iq_scan import _soft_combine_consensus_payloads

    interference_sym = _interference_packet_symbols()
    broadcast_sym = _broadcast_packet_symbols(bytes(range(1, 16)))
    recovered = 0
    for trial in range(10):
        rng = np.random.default_rng(7000 + trial)
        cumsum, sample_count = _mixed_soft_cumsum(interference_sym, broadcast_sym, 3, 6, 0.5, rng)
        out = _soft_combine_consensus_payloads(
            cumsum, sample_count, sps=_SPS_SIM, offset=0.0,
            allowed_access_names=None, max_access_hamming=8,
        )
        recovered += int(_password_recovered(out))
    assert recovered == 10  # per-access grouping keeps the interference consensus intact


def _gfsk_iq(packet_sym, n_copies, noise_amp, rng, gap=40, sps=_SPS_SIM, bt=0.35, deviation=0.30):
    """Synthesize compliant 2-GFSK IQ (Gaussian BT=0.35) carrying ``n_copies`` of the constant
    interference packet, plus complex AWGN. Feeding this through ``scan_iq_samples`` exercises
    the full real chain: FM discriminator -> access search -> diversity combine -> CRC."""
    span = 4
    taps_t = np.arange(-span * sps, span * sps + 1, dtype=np.float64)
    gauss_sigma = np.sqrt(np.log(2.0)) / (2.0 * np.pi * bt) * sps
    taps = np.exp(-0.5 * (taps_t / gauss_sigma) ** 2)
    taps /= taps.sum()

    parts = [rng.choice([-1.0, 1.0], size=int(rng.integers(20, 60)))]
    for _ in range(int(n_copies)):
        parts.append(rng.choice([-1.0, 1.0], size=gap))
        parts.append(packet_sym.copy())
    parts.append(rng.choice([-1.0, 1.0], size=30))
    stream = np.concatenate(parts)
    nrz = np.repeat(stream, sps).astype(np.float64)
    shaped = np.convolve(nrz, taps, mode='same')
    phase = np.cumsum(deviation * shaped)
    iq = np.exp(1j * phase)
    noise = rng.normal(0.0, noise_amp, iq.size) + 1j * rng.normal(0.0, noise_amp, iq.size)
    return (iq + noise).astype(np.complex64)


def _scan_password(iq, vote):
    from rm_radio_ros.core.offline_iq_scan import scan_iq_samples

    result = scan_iq_samples(
        iq, sample_rate=1_000_000.0, sps_values=(45.0, 46.0, 47.0, 48.0, 49.0),
        offset_step=2, low_pass_hz=None, frequency_shift_hz=0.0,
        allowed_access_names=('interference',), aggregate_decode=True, decode=True,
        decode_threshold=3, max_decoded_payloads=4096, max_decoded_frames=512,
        vote_repeated_frames=vote,
    )
    frames = result['decoded']['referee_frames']
    recovered = any(
        f['cmd_hex'] == '0x0A06' and f.get('parsed', {}).get('password') == 'R1L001'
        for f in frames
    )
    soft = any(f.get('source') == 'time_diversity_soft' for f in frames)
    return recovered, soft


def test_time_diversity_recovers_password_from_noisy_gfsk_iq_end_to_end():
    """End-to-end on synthesized 2-GFSK IQ + AWGN: in a noise window where the direct decode
    almost never recovers the password, enabling time diversity recovers it in the clear
    majority of captures, and the F1 soft-combine path is the one doing the recovery."""
    packet_sym = _interference_packet_symbols()
    noise_amp = 0.65
    trials = 16
    direct = diversity = 0
    soft_path_used = False
    for trial in range(trials):
        rng = np.random.default_rng(6000 + trial)
        iq = _gfsk_iq(packet_sym, 15, noise_amp, rng)
        ok_off, _ = _scan_password(iq, vote=False)
        ok_on, soft = _scan_password(iq, vote=True)
        direct += int(ok_off)
        diversity += int(ok_on)
        soft_path_used = soft_path_used or soft
    assert direct <= 2                  # direct decode almost never recovers in this window
    assert diversity >= trials // 2     # time diversity recovers a clear majority
    assert diversity >= direct + 8      # and recovers many captures the direct path cannot
    assert soft_path_used               # F1 soft combine specifically produced the recovery
