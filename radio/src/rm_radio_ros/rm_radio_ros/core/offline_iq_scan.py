from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

import numpy as np

from .rm_protocol import (
    ACCESS_CODES,
    RADAR_AIR_DATA_LENGTHS,
    RefereeFrameAssembler,
    bytes_to_bits,
    bits_to_bytes,
)


AIR_LENGTH_FIELD = b'\x00\x0F\x00\x0F'
AIR_PAYLOAD_BYTES = 15
AIR_PACKET_BITS = 8 * (8 + 4 + AIR_PAYLOAD_BYTES)
ACCESS_BITS = 8 * 8
LENGTH_BITS = 8 * 4


def _moving_average_symbols(
    cumsum: np.ndarray,
    sample_count: int,
    sps: float,
    offset: float,
    symbol_count: int,
    half_symbol_width: float,
) -> np.ndarray:
    centers = (float(offset) + (np.arange(symbol_count) + 0.5) * float(sps)).astype(np.int64)
    half_width = max(1, int(float(sps) * float(half_symbol_width)))
    valid = (centers - half_width >= 0) & (centers + half_width < sample_count)
    centers = centers[valid]
    return (cumsum[centers + half_width] - cumsum[centers - half_width]) / (2 * half_width)


def _bits_from_iq(
    cumsum: np.ndarray,
    sample_count: int,
    sps: float,
    offset: float,
    half_symbol_width: float,
) -> np.ndarray:
    symbol_count = max(0, int((sample_count - 200 - float(offset)) / float(sps)))
    symbols = _moving_average_symbols(cumsum, sample_count, sps, offset, symbol_count, half_symbol_width)
    return (symbols > 0).astype(np.uint8)


def _best_match(bits: np.ndarray, pattern: Iterable[int]) -> Tuple[int, int]:
    pat = np.array(list(pattern), dtype=np.uint8)
    if len(bits) < len(pat):
        return len(pat), -1
    matches = (
        np.convolve((bits == 1).astype(np.int16), pat[::-1], mode='valid')
        + np.convolve((bits == 0).astype(np.int16), (1 - pat)[::-1], mode='valid')
    )
    index = int(np.argmax(matches))
    distance = len(pat) - int(matches[index])
    return distance, index


def _match_distances(bits: np.ndarray, pattern: Iterable[int]) -> np.ndarray:
    pat = np.array(list(pattern), dtype=np.uint8)
    if len(bits) < len(pat):
        return np.array([], dtype=np.int16)
    matches = (
        np.convolve((bits == 1).astype(np.int16), pat[::-1], mode='valid')
        + np.convolve((bits == 0).astype(np.int16), (1 - pat)[::-1], mode='valid')
    )
    return (len(pat) - matches).astype(np.int16)


def _absolute_bit_index(local_bit_index: int, start_second: float, sample_rate: float, sps: float) -> int:
    return int(round(float(start_second) * float(sample_rate) / float(sps))) + int(local_bit_index)


def _lowpass_fir_taps(low_pass_hz: float, sample_rate: float, tap_count: int) -> np.ndarray:
    if float(low_pass_hz) <= 0:
        raise ValueError('low_pass_hz must be positive')
    nyquist = float(sample_rate) / 2.0
    if float(low_pass_hz) >= nyquist:
        raise ValueError(f'low_pass_hz must be below Nyquist ({nyquist:g} Hz)')
    taps_n = max(3, int(tap_count))
    if taps_n % 2 == 0:
        taps_n += 1
    center = (taps_n - 1) / 2.0
    n = np.arange(taps_n, dtype=np.float64) - center
    taps = 2.0 * float(low_pass_hz) / float(sample_rate) * np.sinc(2.0 * float(low_pass_hz) * n / float(sample_rate))
    taps *= np.hamming(taps_n)
    taps /= np.sum(taps)
    return taps.astype(np.float64)


def _apply_lowpass(iq: np.ndarray, low_pass_hz: float, sample_rate: float, filter_taps: int) -> np.ndarray:
    taps = _lowpass_fir_taps(low_pass_hz, sample_rate, filter_taps)
    try:
        from scipy import signal  # type: ignore

        filtered = signal.lfilter(taps, [1.0], iq)
    except Exception:
        filtered = np.convolve(iq, taps, mode='full')[: iq.size]
    delay = len(taps) - 1
    if filtered.size > delay:
        filtered = filtered[delay:]
    return filtered.astype(np.complex64, copy=False)


def _condition_iq(
    iq: np.ndarray,
    *,
    sample_rate: float,
    start_sample: int = 0,
    frequency_shift_hz: float = 0.0,
    low_pass_hz: Optional[float] = None,
    filter_taps: int = 513,
) -> np.ndarray:
    conditioned = iq.astype(np.complex64, copy=False)
    if float(frequency_shift_hz) != 0.0:
        sample_index = int(start_sample) + np.arange(conditioned.size, dtype=np.float64)
        mixer = np.exp(-1j * 2.0 * np.pi * float(frequency_shift_hz) * sample_index / float(sample_rate))
        conditioned = (conditioned * mixer).astype(np.complex64, copy=False)
    if low_pass_hz is not None and float(low_pass_hz) > 0.0:
        conditioned = _apply_lowpass(conditioned, float(low_pass_hz), float(sample_rate), int(filter_taps))
    return conditioned


def _normalize_allowed_access_names(allowed_access_names: Optional[Iterable[str]]) -> Optional[set[str]]:
    if allowed_access_names is None:
        return None
    allowed = {str(name).replace('_inverted', '').strip().lower() for name in allowed_access_names}
    allowed.discard('')
    unknown = sorted(allowed - set(ACCESS_CODES))
    if unknown:
        raise ValueError(f'unknown access name(s): {", ".join(unknown)}')
    return allowed or None


def _access_patterns(allowed_access_names: Optional[Iterable[str]] = None) -> List[Tuple[str, List[int]]]:
    allowed = _normalize_allowed_access_names(allowed_access_names)
    return [
        (name, bytes_to_bits(access))
        for name, access in ACCESS_CODES.items()
        if allowed is None or name in allowed
    ]


def _extract_air_payloads(
    bits: np.ndarray,
    *,
    start_second: float,
    sample_rate: float,
    sps: float,
    max_access_hamming: int,
    max_payloads: int,
    allowed_access_names: Optional[Iterable[str]] = None,
) -> List[dict]:
    payloads: List[dict] = []
    seen = set()
    variants = [
        ('normal', bits),
        ('inverted_corrected', (1 - bits).astype(np.uint8)),
    ]
    for bit_polarity, corrected_bits in variants:
        for access_name, pattern in _access_patterns(allowed_access_names):
            distances = _match_distances(corrected_bits, pattern)
            if distances.size == 0:
                continue
            for bit_index in np.flatnonzero(distances <= int(max_access_hamming)):
                packet_end = int(bit_index) + AIR_PACKET_BITS
                if packet_end > len(corrected_bits):
                    continue
                length_start = int(bit_index) + ACCESS_BITS
                length_end = length_start + LENGTH_BITS
                try:
                    length_field = bits_to_bytes(corrected_bits[length_start:length_end])
                except ValueError:
                    continue
                if length_field != AIR_LENGTH_FIELD:
                    continue
                payload_start = length_end
                payload_end = payload_start + AIR_PAYLOAD_BYTES * 8
                try:
                    payload = bits_to_bytes(corrected_bits[payload_start:payload_end])
                except ValueError:
                    continue

                key = (bit_polarity, access_name, int(bit_index), payload)
                if key in seen:
                    continue
                seen.add(key)
                payloads.append(
                    {
                        'access': access_name,
                        'bit_polarity': bit_polarity,
                        'access_hamming_distance': int(distances[int(bit_index)]),
                        'bit_index': int(bit_index),
                        'absolute_bit_index': _absolute_bit_index(
                            int(bit_index),
                            start_second,
                            sample_rate,
                            sps,
                        ),
                        'sps': float(sps),
                        'air_payload_hex': payload.hex(),
                    }
                )
                if len(payloads) >= int(max_payloads):
                    return sorted(payloads, key=lambda item: item['bit_index'])
    return sorted(payloads, key=lambda item: item['bit_index'])


def _decode_bits_to_referee_frames(
    bits: np.ndarray,
    *,
    start_second: float = 0.0,
    sample_rate: float = 2_000_000.0,
    sps: float = 94.0,
    max_access_hamming: int = 3,
    max_payloads: int = 64,
    max_frames: int = 32,
    allowed_access_names: Optional[Iterable[str]] = None,
) -> dict:
    air_payloads = _extract_air_payloads(
        bits,
        start_second=start_second,
        sample_rate=sample_rate,
        sps=sps,
        max_access_hamming=max_access_hamming,
        max_payloads=max_payloads,
        allowed_access_names=allowed_access_names,
    )
    assembler = RefereeFrameAssembler(max_buffer_size=16384)
    frames = []
    for payload in air_payloads:
        decoded = assembler.push_air_payload(bytes.fromhex(payload['air_payload_hex']))
        for frame in decoded:
            frame_dict = frame.to_dict()
            frame_dict['raw_hex'] = frame.raw.hex()
            frame_dict['source_air_payload_bit_index'] = payload['bit_index']
            frame_dict['source_air_payload_absolute_bit_index'] = payload['absolute_bit_index']
            frames.append(frame_dict)
            if len(frames) >= int(max_frames):
                return {
                    'air_payloads': air_payloads,
                    'referee_frames': frames,
                    'air_payload_count': len(air_payloads),
                    'referee_frame_count': len(frames),
                    'truncated': True,
                }
    return {
        'air_payloads': air_payloads,
        'referee_frames': frames,
        'air_payload_count': len(air_payloads),
        'referee_frame_count': len(frames),
        'truncated': False,
    }


def _merge_decoded_results(target: dict, decoded: dict, max_payloads: int, max_frames: int) -> None:
    seen_payloads = {
        (
            payload.get('bit_polarity'),
            payload.get('access'),
            payload.get('absolute_bit_index'),
            payload.get('air_payload_hex'),
        )
        for payload in target['air_payloads']
    }
    for payload in decoded.get('air_payloads', []):
        key = (
            payload.get('bit_polarity'),
            payload.get('access'),
            payload.get('absolute_bit_index'),
            payload.get('air_payload_hex'),
        )
        if key in seen_payloads:
            continue
        if len(target['air_payloads']) >= int(max_payloads):
            target['truncated'] = True
            break
        seen_payloads.add(key)
        target['air_payloads'].append(payload)

    seen_frames = {frame.get('raw_hex') for frame in target['referee_frames']}
    for frame in decoded.get('referee_frames', []):
        key = frame.get('raw_hex')
        if key in seen_frames:
            continue
        if len(target['referee_frames']) >= int(max_frames):
            target['truncated'] = True
            break
        seen_frames.add(key)
        target['referee_frames'].append(frame)

    target['air_payload_count'] = len(target['air_payloads'])
    target['referee_frame_count'] = len(target['referee_frames'])


def summarize_scan_result(result: dict, frame_limit: int = 8) -> dict:
    match = result.get('best_access_match') or {}
    decoded = result.get('decoded') or {}
    frames = decoded.get('referee_frames') or []
    commands = []
    seen_commands = set()
    for frame in frames:
        key = (frame.get('cmd_hex'), frame.get('command'))
        if key in seen_commands:
            continue
        seen_commands.add(key)
        commands.append({'cmd_hex': frame.get('cmd_hex'), 'command': frame.get('command')})
    return {
        'file': Path(str(result.get('file', ''))).name,
        'file_duration_seconds': result.get('file_duration_seconds', result.get('seconds_scanned')),
        'best_access': match.get('access'),
        'best_access_hamming_distance': match.get('hamming_distance'),
        'best_window_start_second': result.get('start_second'),
        'best_sps': match.get('sps'),
        'best_offset': match.get('offset'),
        'air_payload_count': decoded.get('air_payload_count', 0),
        'referee_frame_count': decoded.get('referee_frame_count', 0),
        'commands': commands,
        'first_referee_frame': frames[0] if frames else None,
        'referee_frame_samples': frames[: max(0, int(frame_limit))],
        'referee_frame_samples_truncated': len(frames) > max(0, int(frame_limit)),
    }


def scan_iq_file(
    path: Path,
    max_seconds: float = 2.0,
    sample_rate: float = 2_000_000.0,
    sps_values: Iterable[float] = (92.0, 92.5, 93.0, 93.5, 94.0),
    offset_step: int = 4,
    start_second: float = 0.0,
    decode: bool = True,
    decode_threshold: int = 3,
    max_decoded_payloads: int = 64,
    max_decoded_frames: int = 32,
    aggregate_decode: bool = False,
    frequency_shift_hz: float = 0.0,
    low_pass_hz: Optional[float] = None,
    filter_taps: int = 513,
    allowed_access_names: Optional[Iterable[str]] = None,
) -> dict:
    max_samples = int(float(max_seconds) * float(sample_rate))
    start_sample = max(0, int(float(start_second) * float(sample_rate)))
    iq = np.fromfile(path, dtype=np.complex64, count=max_samples, offset=start_sample * np.dtype(np.complex64).itemsize)
    if iq.size < 1024:
        raise ValueError(f'IQ file is too short: {path}')

    result = scan_iq_samples(
        iq,
        label=str(path),
        sample_rate=sample_rate,
        start_second=start_second,
        sps_values=sps_values,
        offset_step=offset_step,
        decode=decode,
        decode_threshold=decode_threshold,
        max_decoded_payloads=max_decoded_payloads,
        max_decoded_frames=max_decoded_frames,
        aggregate_decode=aggregate_decode,
        frequency_shift_hz=frequency_shift_hz,
        low_pass_hz=low_pass_hz,
        filter_taps=filter_taps,
        start_sample=start_sample,
        allowed_access_names=allowed_access_names,
    )
    result['file'] = str(path)
    return result


def _vote_consensus_payloads(
    air_payloads: List[dict],
    *,
    min_copies: int = 3,
    payload_bytes: int = AIR_PAYLOAD_BYTES,
    cluster_hamming_bits: int = 20,
) -> List[bytes]:
    """F5 time diversity: recover a consensus 15-byte air payload by per-bit majority vote
    across repeated noisy copies of the SAME constant official frame (e.g. the interference
    password, retransmitted unchanged at 10 Hz). The vote is taken inside a Hamming cluster
    around the most frequent copy, so unrelated random-fill payloads do not bias it.

    Returns at most one consensus payload; the caller must still CRC-check it through the
    assembler (CRC stays the sole arbiter -- no fabricated frames). Meaningful only when one
    official frame dominates the pool (interference profile), not for cycling info commands."""
    raws = [
        bytes.fromhex(p['air_payload_hex'])
        for p in air_payloads
        if len(p.get('air_payload_hex', '')) == 2 * int(payload_bytes)
    ]
    if len(raws) < int(min_copies):
        return []
    bit_rows = np.array(
        [np.unpackbits(np.frombuffer(r, dtype=np.uint8)) for r in raws], dtype=np.uint8
    )
    from collections import Counter

    seed_hex = Counter(r.hex() for r in raws).most_common(1)[0][0]
    seed = np.unpackbits(np.frombuffer(bytes.fromhex(seed_hex), dtype=np.uint8))
    hamming = np.count_nonzero(bit_rows != seed, axis=1)
    cluster = bit_rows[hamming <= int(cluster_hamming_bits)]
    if cluster.shape[0] < int(min_copies):
        return []
    voted = (cluster.mean(axis=0) >= 0.5).astype(np.uint8)
    return [np.packbits(voted).tobytes()]


def _apply_repeated_frame_voting(decoded: dict, *, max_frames: int) -> None:
    """Decode the F5 consensus payload(s) and append any NEW CRC-valid referee frame to
    ``decoded``. CRC8+CRC16 (via the assembler) remain the sole gate; only frames that pass
    are added, tagged with source='time_diversity_vote'."""
    consensus_payloads = _vote_consensus_payloads(decoded.get('air_payloads', []))
    if not consensus_payloads:
        return
    seen = {frame.get('raw_hex') for frame in decoded['referee_frames']}
    assembler = RefereeFrameAssembler(max_buffer_size=4096, allowed_cmd_lengths=RADAR_AIR_DATA_LENGTHS)
    for payload in consensus_payloads:
        for frame in assembler.push_air_payload(list(payload)):
            raw_hex = frame.raw.hex()
            if raw_hex in seen or len(decoded['referee_frames']) >= int(max_frames):
                continue
            seen.add(raw_hex)
            frame_dict = frame.to_dict()
            frame_dict['raw_hex'] = raw_hex
            frame_dict['source'] = 'time_diversity_vote'
            decoded['referee_frames'].append(frame_dict)
    decoded['referee_frame_count'] = len(decoded['referee_frames'])


def _soft_symbols(
    cumsum: np.ndarray,
    sample_count: int,
    sps: float,
    offset: float,
    half_symbol_width: float = 0.35,
) -> np.ndarray:
    """Return the float soft symbols (matched-filter-like FM-discriminator averages) on one
    (sps, offset) grid -- the same quantity ``_bits_from_iq`` hardens with ``> 0``. F1 keeps
    these floats so repeated copies can be combined before the bit decision is made."""
    symbol_count = max(0, int((sample_count - 200 - float(offset)) / float(sps)))
    return _moving_average_symbols(cumsum, sample_count, sps, offset, symbol_count, half_symbol_width)


def _soft_combine_consensus_payloads(
    cumsum: np.ndarray,
    sample_count: int,
    *,
    sps: float,
    offset: float,
    allowed_access_names: Optional[Iterable[str]] = None,
    max_access_hamming: int = 8,
    half_symbol_width: float = 0.35,
    min_copies: int = 3,
    max_length_hamming: int = 8,
) -> List[bytes]:
    """F1 time diversity: equal-gain *soft-symbol* combining across repeated copies of the SAME
    constant official frame (e.g. the interference password retransmitted unchanged at 10 Hz).

    Each detected copy is aligned on one shared (sps, offset) symbol grid -- so copies are
    integer-bit shifts of each other and line up sample-for-sample -- then polarity-corrected
    (an inverted copy has its soft symbols negated) and averaged. Averaging the LLR-like soft
    symbols *before* the bit decision recovers weaker copies than hard per-bit majority voting
    (F5): confident symbols outvote noisy ones continuously rather than one-bit-one-vote, which
    is the ~2 dB combining edge soft decisions hold over hard decisions.

    Enrolment is deliberately decoupled from the strict CRC-decode threshold: copies are
    accepted at a *relaxed* access Hamming distance (``max_access_hamming``) so that copies too
    weak to decode on their own still join the average -- exactly the regime where diversity
    helps. False enrolment is still negligible: a random 64-bit window matching the access code
    within 8 bits has probability ~1e-8, and the 32-bit ``00 0F 00 0F`` length-field gate culls
    the rest. The consensus payload is then CRC-checked by the caller through the assembler, so
    CRC remains the sole arbiter -- no fabricated frames.

    Copies are grouped and averaged *per access code*: only segments that matched the SAME
    access code are combined together, so an unfiltered call (``allowed_access_names=None``) that
    sees both the cycling broadcast wave and the constant interference wave never averages the
    two into one vector. One consensus payload is emitted per access group with enough copies;
    in practice only the constant interference password closes CRC inside a single 15-byte air
    payload, so the broadcast group's hypothesis is harmlessly dropped by the assembler."""
    symbols = _soft_symbols(cumsum, sample_count, float(sps), float(offset), half_symbol_width)
    if symbols.size < AIR_PACKET_BITS:
        return []
    hard = (symbols > 0).astype(np.uint8)
    length_pattern = np.array(bytes_to_bits(AIR_LENGTH_FIELD), dtype=np.uint8)
    payload_start = ACCESS_BITS + LENGTH_BITS
    payload_end = payload_start + AIR_PAYLOAD_BYTES * 8

    # polarity-corrected soft packet vectors, grouped by the access code each copy matched, so
    # only copies of the same official frame are ever averaged together.
    groups: dict = {}
    enrolled: set = set()  # (access_name, start) already taken -- also blocks the opposite
    #                        polarity matching the same window, which would cancel the average
    for polarity in ('normal', 'inverted'):
        view = hard if polarity == 'normal' else (1 - hard).astype(np.uint8)
        for name, pattern in _access_patterns(allowed_access_names):
            distances = _match_distances(view, pattern)
            if distances.size == 0:
                continue
            for bit_index in np.flatnonzero(distances <= int(max_access_hamming)):
                start = int(bit_index)
                if (name, start) in enrolled:
                    continue
                end = start + AIR_PACKET_BITS
                if end > symbols.size:
                    continue
                seg = symbols[start:end].astype(np.float64)
                if polarity == 'inverted':
                    seg = -seg
                seg_hard = (seg > 0).astype(np.uint8)
                length_bits = seg_hard[ACCESS_BITS:ACCESS_BITS + LENGTH_BITS]
                if int(np.count_nonzero(length_bits != length_pattern)) > int(max_length_hamming):
                    continue
                enrolled.add((name, start))
                groups.setdefault(name, []).append(seg)

    consensus: List[bytes] = []
    for _name, segments in groups.items():
        if len(segments) < int(min_copies):
            continue
        combined = np.mean(np.stack(segments, axis=0), axis=0)
        combined_hard = (combined > 0).astype(np.uint8)
        try:
            consensus.append(bits_to_bytes(combined_hard[payload_start:payload_end]))
        except ValueError:
            continue
    return consensus


def _apply_soft_diversity(
    decoded: dict,
    cumsum: np.ndarray,
    sample_count: int,
    *,
    sps: float,
    offset: float,
    allowed_access_names: Optional[Iterable[str]] = None,
    max_access_hamming: int = 8,
    max_frames: int = 32,
) -> None:
    """Decode the F1 soft-combined consensus payload and append any NEW CRC-valid referee frame
    to ``decoded``. CRC8+CRC16 (via the assembler) remain the sole gate; only frames that pass
    are added, tagged with source='time_diversity_soft'."""
    consensus_payloads = _soft_combine_consensus_payloads(
        cumsum,
        sample_count,
        sps=float(sps),
        offset=float(offset),
        allowed_access_names=allowed_access_names,
        max_access_hamming=int(max_access_hamming),
    )
    if not consensus_payloads:
        return
    seen = {frame.get('raw_hex') for frame in decoded['referee_frames']}
    for payload in consensus_payloads:
        assembler = RefereeFrameAssembler(max_buffer_size=4096, allowed_cmd_lengths=RADAR_AIR_DATA_LENGTHS)
        for frame in assembler.push_air_payload(list(payload)):
            raw_hex = frame.raw.hex()
            if raw_hex in seen or len(decoded['referee_frames']) >= int(max_frames):
                continue
            seen.add(raw_hex)
            frame_dict = frame.to_dict()
            frame_dict['raw_hex'] = raw_hex
            frame_dict['source'] = 'time_diversity_soft'
            decoded['referee_frames'].append(frame_dict)
    decoded['referee_frame_count'] = len(decoded['referee_frames'])


def scan_iq_samples(
    iq: np.ndarray,
    *,
    label: str = 'iq_samples',
    sample_rate: float = 2_000_000.0,
    start_second: float = 0.0,
    sps_values: Iterable[float] = (92.0, 92.5, 93.0, 93.5, 94.0),
    offset_step: int = 4,
    decode: bool = True,
    decode_threshold: int = 3,
    max_decoded_payloads: int = 64,
    max_decoded_frames: int = 32,
    aggregate_decode: bool = False,
    frequency_shift_hz: float = 0.0,
    low_pass_hz: Optional[float] = None,
    filter_taps: int = 513,
    start_sample: Optional[int] = None,
    allowed_access_names: Optional[Iterable[str]] = None,
    vote_repeated_frames: bool = False,
) -> dict:
    iq = np.asarray(iq, dtype=np.complex64)
    if iq.size < 1024:
        raise ValueError(f'IQ sample window is too short: {iq.size}')

    if start_sample is None:
        start_sample = max(0, int(float(start_second) * float(sample_rate)))
    iq = _condition_iq(
        iq,
        sample_rate=sample_rate,
        start_sample=start_sample,
        frequency_shift_hz=frequency_shift_hz,
        low_pass_hz=low_pass_hz,
        filter_taps=filter_taps,
    )
    iq = iq - np.mean(iq)
    phase_delta = np.angle(iq[1:] * np.conj(iq[:-1])).astype(np.float32)
    phase_delta = phase_delta - float(np.mean(phase_delta))
    cumsum = np.concatenate([[0.0], np.cumsum(phase_delta, dtype=np.float64)])

    patterns: List[Tuple[str, List[int]]] = []
    for name, pattern in _access_patterns(allowed_access_names):
        patterns.append((name, pattern))
        patterns.append((f'{name}_inverted', [1 - bit for bit in pattern]))

    best = None
    best_decode = None
    aggregate_decoded = {
        'air_payloads': [],
        'referee_frames': [],
        'air_payload_count': 0,
        'referee_frame_count': 0,
        'truncated': False,
    }
    for sps in sps_values:
        for offset in range(0, int(max(sps_values)) + 1, int(offset_step)):
            bits = _bits_from_iq(cumsum, len(phase_delta), float(sps), float(offset), 0.35)
            best_distance_for_bits = None
            for name, pattern in patterns:
                distance, index = _best_match(bits, pattern)
                if best_distance_for_bits is None or distance < best_distance_for_bits:
                    best_distance_for_bits = distance
                candidate = {
                    'access': name,
                    'hamming_distance': distance,
                    'bit_index': index,
                    'absolute_bit_index': index,
                    'sps': float(sps),
                    'offset': float(offset),
                }
                if best is None or candidate['hamming_distance'] < best['hamming_distance']:
                    best = candidate
                    if index >= 0:
                        start = index
                        end = min(len(bits), start + 64 + 32 + 120)
                        try:
                            candidate['matched_bytes'] = bits_to_bytes(bits[start:end]).hex()
                        except ValueError:
                            pass
            if decode and best_distance_for_bits is not None and best_distance_for_bits <= int(decode_threshold):
                decoded = _decode_bits_to_referee_frames(
                    bits,
                    start_second=start_second,
                    sample_rate=sample_rate,
                    sps=float(sps),
                    max_access_hamming=decode_threshold,
                    max_payloads=max_decoded_payloads,
                    max_frames=max_decoded_frames,
                    allowed_access_names=allowed_access_names,
                )
                decoded['sps'] = float(sps)
                decoded['offset'] = float(offset)
                decoded['best_access_hamming_distance'] = int(best_distance_for_bits)
                if aggregate_decode:
                    _merge_decoded_results(
                        aggregate_decoded,
                        decoded,
                        max_decoded_payloads,
                        max_decoded_frames,
                    )
                if (
                    best_decode is None
                    or decoded['referee_frame_count'] > best_decode['referee_frame_count']
                    or (
                        decoded['referee_frame_count'] == best_decode['referee_frame_count']
                        and decoded['air_payload_count'] > best_decode['air_payload_count']
                    )
                    or (
                        decoded['referee_frame_count'] == best_decode['referee_frame_count']
                        and decoded['air_payload_count'] == best_decode['air_payload_count']
                        and decoded['best_access_hamming_distance']
                        < best_decode['best_access_hamming_distance']
                    )
                ):
                    best_decode = decoded

    final_decoded = aggregate_decoded if aggregate_decode else (best_decode or aggregate_decoded)
    if decode and vote_repeated_frames:
        # Time-diversity recovery for repeated constant frames (interference password). Both
        # paths are CRC-gated through the assembler; CRC remains the sole arbiter. F1 soft
        # combine runs first on the lowest single-copy access-Hamming grid (works even when no
        # single copy produced a CRC frame), then F5 hard majority vote fills in from payloads.
        if best is not None and float(best.get('sps') or 0.0) > 1.0:
            _apply_soft_diversity(
                final_decoded,
                cumsum,
                len(phase_delta),
                sps=float(best['sps']),
                offset=float(best.get('offset') or 0.0),
                allowed_access_names=allowed_access_names,
                max_access_hamming=max(8, int(decode_threshold)),
                max_frames=max_decoded_frames,
            )
        _apply_repeated_frame_voting(final_decoded, max_frames=max_decoded_frames)
    result = {
        'file': str(label),
        'sample_rate': float(sample_rate),
        'start_second': float(start_second),
        'samples_scanned': int(iq.size),
        'seconds_scanned': float(iq.size) / float(sample_rate),
        'best_access_match': best,
        'frequency_shift_hz': float(frequency_shift_hz),
        'low_pass_hz': None if low_pass_hz is None else float(low_pass_hz),
        'filter_taps': int(filter_taps),
    }
    if decode:
        result['decode_threshold'] = int(decode_threshold)
        result['decoded'] = final_decoded
    return result


def scan_iq_file_windows(
    path: Path,
    window_seconds: float = 2.0,
    step_seconds: float = 2.0,
    sample_rate: float = 2_000_000.0,
    sps_values: Iterable[float] = (92.0, 92.5, 93.0, 93.5, 94.0),
    offset_step: int = 4,
    decode: bool = True,
    decode_threshold: int = 3,
    max_decoded_payloads: int = 64,
    max_decoded_frames: int = 32,
    aggregate_decode: bool = False,
    frequency_shift_hz: float = 0.0,
    low_pass_hz: Optional[float] = None,
    filter_taps: int = 513,
    allowed_access_names: Optional[Iterable[str]] = None,
    vote_repeated_frames: bool = False,
) -> dict:
    file_samples = path.stat().st_size // np.dtype(np.complex64).itemsize
    if file_samples < 1024:
        raise ValueError(f'IQ file is too short: {path}')
    duration = float(file_samples) / float(sample_rate)
    window_seconds = max(0.1, float(window_seconds))
    step_seconds = max(0.1, float(step_seconds))

    starts = list(np.arange(0.0, max(duration - 0.1, 0.0), step_seconds))
    if not starts:
        starts = [0.0]

    best_result = None
    best_access_result = None
    decoded_aggregate = {
        'air_payloads': [],
        'referee_frames': [],
        'air_payload_count': 0,
        'referee_frame_count': 0,
        'truncated': False,
    }
    scanned = 0
    for start_second in starts:
        result = scan_iq_file(
            path,
            max_seconds=min(window_seconds, max(duration - start_second, 0.0)),
            sample_rate=sample_rate,
            sps_values=sps_values,
            offset_step=offset_step,
            start_second=start_second,
            decode=decode,
            decode_threshold=decode_threshold,
            max_decoded_payloads=max_decoded_payloads,
            max_decoded_frames=max_decoded_frames,
            aggregate_decode=aggregate_decode,
            frequency_shift_hz=frequency_shift_hz,
            low_pass_hz=low_pass_hz,
            filter_taps=filter_taps,
            allowed_access_names=allowed_access_names,
        )
        scanned += result['samples_scanned']
        match = result.get('best_access_match') or {}
        if match.get('bit_index', -1) >= 0:
            match['absolute_bit_index'] = _absolute_bit_index(
                int(match['bit_index']),
                start_second,
                sample_rate,
                float(match['sps']),
            )
        if (
            best_access_result is None
            or match.get('hamming_distance', 10**9)
            < (best_access_result.get('best_access_match') or {}).get('hamming_distance', 10**9)
        ):
            best_access_result = result
        if decode:
            decoded = result.get('decoded') or {}
            window_decoded = {
                'air_payloads': [],
                'referee_frames': [],
                'air_payload_count': 0,
                'referee_frame_count': 0,
                'truncated': bool(decoded.get('truncated', False)),
            }
            for payload in decoded.get('air_payloads', []):
                payload = dict(payload)
                payload['window_start_second'] = float(start_second)
                window_decoded['air_payloads'].append(payload)
            for frame in decoded.get('referee_frames', []):
                frame = dict(frame)
                frame['window_start_second'] = float(start_second)
                window_decoded['referee_frames'].append(frame)
            window_decoded['air_payload_count'] = len(window_decoded['air_payloads'])
            window_decoded['referee_frame_count'] = len(window_decoded['referee_frames'])
            _merge_decoded_results(
                decoded_aggregate,
                window_decoded,
                max_decoded_payloads,
                max_decoded_frames,
            )
            decoded_aggregate['truncated'] = decoded_aggregate['truncated'] or bool(decoded.get('truncated', False))
        if (
            best_result is None
            or (result.get('decoded') or {}).get('referee_frame_count', 0)
            > (best_result.get('decoded') or {}).get('referee_frame_count', 0)
            or (
                (result.get('decoded') or {}).get('referee_frame_count', 0)
                == (best_result.get('decoded') or {}).get('referee_frame_count', 0)
                and match.get('hamming_distance', 10**9)
                < (best_result.get('best_access_match') or {}).get('hamming_distance', 10**9)
            )
        ):
            best_result = result

    if best_result is None:
        best_result = {
            'file': str(path),
            'sample_rate': float(sample_rate),
            'start_second': 0.0,
            'samples_scanned': 0,
            'seconds_scanned': 0.0,
            'best_access_match': None,
        }
    if best_access_result is not None:
        best_result['best_access_match'] = best_access_result.get('best_access_match')

    best_result['file_duration_seconds'] = duration
    best_result['file_samples'] = int(file_samples)
    best_result['windows_scanned'] = len(starts)
    best_result['total_samples_scanned'] = int(scanned)
    best_result['frequency_shift_hz'] = float(frequency_shift_hz)
    best_result['low_pass_hz'] = None if low_pass_hz is None else float(low_pass_hz)
    best_result['filter_taps'] = int(filter_taps)
    if decode:
        if vote_repeated_frames:
            _apply_repeated_frame_voting(decoded_aggregate, max_frames=max_decoded_frames)
        best_result['decoded'] = decoded_aggregate
    return best_result


def main() -> None:
    parser = argparse.ArgumentParser(description='Scan complex64 IQ captures for RoboMaster access codes.')
    parser.add_argument('paths', nargs='+', type=Path)
    parser.add_argument('--seconds', type=float, default=2.0)
    parser.add_argument('--window-seconds', type=float, default=2.0)
    parser.add_argument('--step-seconds', type=float, default=1.0)
    parser.add_argument('--sample-rate', type=float, default=2_000_000.0)
    parser.add_argument('--sps', default='91,91.5,92,92.5,93,93.5,94,94.5,95')
    parser.add_argument('--offset-step', type=int, default=1)
    parser.add_argument('--no-decode', action='store_true', help='Only report access-code matches; skip air payload/frame decode.')
    parser.add_argument('--decode-threshold', type=int, default=3, help='Maximum access-code Hamming distance accepted for payload extraction.')
    parser.add_argument('--max-decoded-payloads', type=int, default=2048)
    parser.add_argument('--max-decoded-frames', type=int, default=512)
    parser.add_argument('--freq-shift-hz', type=float, default=0.0, help='Mix this offset to baseband before demodulation. Positive values isolate a signal above the recorded center.')
    parser.add_argument('--low-pass-hz', type=float, default=0.0, help='Optional FIR low-pass cutoff after frequency shift. Use this to isolate one channel from a multi-signal capture.')
    parser.add_argument('--filter-taps', type=int, default=513, help='FIR tap count for --low-pass-hz. Even values are rounded up to the next odd value.')
    parser.add_argument('--aggregate-decode', dest='aggregate_decode', action='store_true', default=True, help='Merge decoded payloads/frames from all tested SPS and offsets. This is the default.')
    parser.add_argument('--no-aggregate-decode', dest='aggregate_decode', action='store_false', help='Keep only the best decode per window.')
    parser.add_argument('--fast', action='store_true', help='Use the old faster coarse scan profile: 2 s non-overlapping windows, coarse SPS/offset search, and no aggregate decode.')
    parser.add_argument('--summary', action='store_true', help='Print one compact decoded summary per file.')
    parser.add_argument('--summary-frame-limit', type=int, default=8)
    args = parser.parse_args()

    if args.fast:
        args.window_seconds = 2.0
        args.step_seconds = 2.0
        args.sps = '92,92.5,93,93.5,94'
        args.offset_step = 4
        args.aggregate_decode = False
        if args.max_decoded_payloads == 2048:
            args.max_decoded_payloads = 128
        if args.max_decoded_frames == 512:
            args.max_decoded_frames = 64

    sps_values = [float(item) for item in args.sps.split(',') if item.strip()]
    for path in args.paths:
        if args.window_seconds is not None:
            result = scan_iq_file_windows(
                path,
                window_seconds=args.window_seconds,
                step_seconds=args.step_seconds or args.window_seconds,
                sample_rate=args.sample_rate,
                sps_values=sps_values,
                offset_step=args.offset_step,
                decode=not args.no_decode,
                decode_threshold=args.decode_threshold,
                max_decoded_payloads=args.max_decoded_payloads,
                max_decoded_frames=args.max_decoded_frames,
                aggregate_decode=args.aggregate_decode,
                frequency_shift_hz=args.freq_shift_hz,
                low_pass_hz=args.low_pass_hz if args.low_pass_hz > 0.0 else None,
                filter_taps=args.filter_taps,
            )
        else:
            result = scan_iq_file(
                path,
                max_seconds=args.seconds,
                sample_rate=args.sample_rate,
                sps_values=sps_values,
                offset_step=args.offset_step,
                decode=not args.no_decode,
                decode_threshold=args.decode_threshold,
                max_decoded_payloads=args.max_decoded_payloads,
                max_decoded_frames=args.max_decoded_frames,
                aggregate_decode=args.aggregate_decode,
                frequency_shift_hz=args.freq_shift_hz,
                low_pass_hz=args.low_pass_hz if args.low_pass_hz > 0.0 else None,
                filter_taps=args.filter_taps,
            )
        if args.summary:
            result = summarize_scan_result(result, frame_limit=args.summary_frame_limit)
        print(json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
