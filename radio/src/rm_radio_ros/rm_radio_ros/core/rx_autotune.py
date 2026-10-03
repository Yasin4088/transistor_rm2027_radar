from __future__ import annotations

from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np

from .offline_iq_scan import scan_iq_file_windows, scan_iq_samples, summarize_scan_result


DEFAULT_EXPECTED_COMMANDS = {
    'broadcast': ('0x0A01', '0x0A02', '0x0A03', '0x0A04', '0x0A05'),
    'interference': ('0x0A06',),
}


def normalize_access_name(access: Any) -> str:
    return str(access or '').replace('_inverted', '').strip().lower()


def default_sps_values(sample_rate: float, *, half_steps: int = 4) -> tuple[float, ...]:
    nominal = float(sample_rate) / (1_000_000.0 / 47.0)
    return tuple(
        round(nominal + step * 0.5, 3)
        for step in range(-int(half_steps), int(half_steps) + 1)
        if nominal + step * 0.5 > 1.0
    )


def _unique(values: Iterable[float], *, step_hz: float = 1.0) -> list[float]:
    out: list[float] = []
    seen: set[int] = set()
    for value in values:
        rounded = round(float(value) / float(step_hz)) * float(step_hz)
        key = int(round(rounded))
        if key in seen:
            continue
        seen.add(key)
        out.append(float(rounded))
    return out


def low_pass_candidates(
    profile: str,
    level: int,
    base_low_pass_hz: float,
    sample_rate: float,
    *,
    mode: str = 'quick',
) -> tuple[float, ...]:
    nyquist = float(sample_rate) / 2.0
    base = float(base_low_pass_hz) if float(base_low_pass_hz) > 0 else 320_000.0
    clean_profile = str(profile).strip().lower()
    clean_mode = str(mode or 'quick').strip().lower()

    if clean_mode == 'off':
        raw = [base]
    elif clean_profile == 'broadcast':
        if int(level) == 3:
            raw = [base, 220_000.0, 260_000.0, 320_000.0]
        elif int(level) == 2:
            raw = [base, 260_000.0, 320_000.0, 380_000.0]
        else:
            raw = [base] if clean_mode == 'quick' else [base, 260_000.0, 320_000.0, 380_000.0]
    else:
        raw = [base]

    return tuple(value for value in _unique(raw, step_hz=1_000.0) if 0.0 < value < nyquist)


def estimate_peak_offsets_from_iq(
    iq: np.ndarray,
    *,
    sample_rate: float,
    fft_size: int = 8192,
    max_peaks: int = 3,
    exclude_dc_hz: float = 5_000.0,
    min_separation_hz: float = 25_000.0,
) -> tuple[float, ...]:
    data = np.asarray(iq, dtype=np.complex64)
    if data.size < 1024:
        return ()

    safe_fft = min(int(fft_size), int(data.size))
    safe_fft = 1 << int(np.floor(np.log2(max(256, safe_fft))))
    if safe_fft < 256:
        return ()

    segment = data[-safe_fft:].astype(np.complex64, copy=False)
    segment = segment - np.mean(segment)
    window = np.hanning(safe_fft).astype(np.float32)
    spectrum = np.fft.fftshift(np.fft.fft(segment * window))
    power = np.abs(spectrum)
    freqs = np.fft.fftshift(np.fft.fftfreq(safe_fft, d=1.0 / float(sample_rate)))

    order = np.argsort(power)[::-1]
    peaks: list[float] = []
    for index in order:
        freq = float(freqs[int(index)])
        if abs(freq) < float(exclude_dc_hz):
            continue
        if any(abs(freq - old) < float(min_separation_hz) for old in peaks):
            continue
        peaks.append(freq)
        if len(peaks) >= int(max_peaks):
            break
    return tuple(_unique(peaks, step_hz=1_000.0))


def estimate_peak_offsets(
    path: Path,
    *,
    sample_rate: float,
    max_seconds: float = 0.25,
    fft_size: int = 8192,
    max_peaks: int = 3,
    exclude_dc_hz: float = 5_000.0,
    min_separation_hz: float = 25_000.0,
) -> tuple[float, ...]:
    sample_count = max(1024, int(float(max_seconds) * float(sample_rate)))
    iq = np.fromfile(path, dtype=np.complex64, count=sample_count)
    return estimate_peak_offsets_from_iq(
        iq,
        sample_rate=sample_rate,
        fft_size=fft_size,
        max_peaks=max_peaks,
        exclude_dc_hz=exclude_dc_hz,
        min_separation_hz=min_separation_hz,
    )


def frequency_shift_candidates(
    profile: str,
    level: int,
    requested_shift_hz: float,
    peak_offsets_hz: Sequence[float] = (),
    *,
    mode: str = 'quick',
    max_candidates: int = 8,
) -> tuple[float, ...]:
    clean_profile = str(profile).strip().lower()
    clean_mode = str(mode or 'quick').strip().lower()
    requested = float(requested_shift_hz)
    raw: list[float] = [requested]

    if clean_mode != 'off':
        raw.append(0.0)
        for peak in peak_offsets_hz:
            raw.extend([float(peak), float(peak) - 25_000.0, float(peak) + 25_000.0])

        if clean_profile == 'broadcast':
            if clean_mode == 'full':
                raw.extend([-350_000.0, -325_000.0, -300_000.0, -275_000.0, -250_000.0, 250_000.0, 300_000.0])
            elif int(level) >= 2:
                raw.extend([-325_000.0, -300_000.0, -275_000.0])
        elif clean_mode == 'full':
            raw.extend([-25_000.0, 25_000.0])

    return tuple(_unique(raw, step_hz=1_000.0)[: max(1, int(max_candidates))])


def build_candidates(
    *,
    profile: str,
    level: int,
    sample_rate: float,
    base_low_pass_hz: float,
    requested_frequency_shift_hz: float = 0.0,
    peak_offsets_hz: Sequence[float] = (),
    mode: str = 'quick',
    max_frequency_candidates: int = 8,
    max_candidates: int = 24,
) -> list[dict[str, float]]:
    low_passes = low_pass_candidates(profile, level, base_low_pass_hz, sample_rate, mode=mode)
    shifts = frequency_shift_candidates(
        profile,
        level,
        requested_frequency_shift_hz,
        peak_offsets_hz,
        mode=mode,
        max_candidates=max_frequency_candidates,
    )
    candidates = [
        {'frequency_shift_hz': float(shift), 'low_pass_hz': float(low_pass)}
        for shift in shifts
        for low_pass in low_passes
    ]
    return candidates[: max(1, int(max_candidates))]


def score_summary(
    summary: dict[str, Any],
    *,
    expected_access: str,
    expected_cmds: Sequence[str],
) -> dict[str, Any]:
    commands = {str(item.get('cmd_hex')).upper() for item in summary.get('commands', [])}
    expected = {str(cmd).upper() for cmd in expected_cmds}
    expected_count = len(commands & expected)
    command_complete = bool(expected) and expected.issubset(commands)
    access_match = normalize_access_name(summary.get('best_access')) == normalize_access_name(expected_access)
    frame_count = int(summary.get('referee_frame_count') or 0)
    payload_count = int(summary.get('air_payload_count') or 0)
    hamming = summary.get('best_access_hamming_distance')
    try:
        hamming_value = int(hamming)
    except Exception:
        hamming_value = 999

    score = (
        (10_000_000 if command_complete else 0)
        + expected_count * 1_000_000
        + frame_count * 10_000
        + (1_000 if access_match else 0)
        + payload_count
        - hamming_value
    )
    return {
        'score': int(score),
        'access_match': bool(access_match),
        'command_complete': bool(command_complete),
        'expected_command_count': int(expected_count),
        'referee_frame_count': int(frame_count),
        'air_payload_count': int(payload_count),
        'best_access_hamming_distance': hamming if hamming is not None else None,
    }


def autotune_iq_file(
    path: Path,
    *,
    profile: str,
    level: int,
    sample_rate: float,
    base_low_pass_hz: float,
    requested_frequency_shift_hz: float = 0.0,
    expected_access: str | None = None,
    expected_cmds: Sequence[str] | None = None,
    window_seconds: float = 2.0,
    step_seconds: float = 1.0,
    sps_values: Iterable[float] | None = None,
    offset_step: int = 2,
    decode_threshold: int = 3,
    max_decoded_payloads: int = 2048,
    max_decoded_frames: int = 512,
    frame_limit: int = 12,
    filter_taps: int = 513,
    mode: str = 'quick',
    estimate_peaks: bool = True,
    max_frequency_candidates: int = 8,
    max_candidates: int = 24,
    vote_repeated_frames: bool = False,
) -> dict[str, Any]:
    clean_profile = str(profile).strip().lower()
    expected_access = expected_access or clean_profile
    expected_cmds = tuple(expected_cmds or DEFAULT_EXPECTED_COMMANDS.get(clean_profile, ()))
    sps_values = tuple(sps_values or default_sps_values(sample_rate))

    peak_offsets = estimate_peak_offsets(path, sample_rate=sample_rate) if estimate_peaks and str(mode) != 'off' else ()
    candidates = build_candidates(
        profile=clean_profile,
        level=int(level),
        sample_rate=float(sample_rate),
        base_low_pass_hz=float(base_low_pass_hz),
        requested_frequency_shift_hz=float(requested_frequency_shift_hz),
        peak_offsets_hz=peak_offsets,
        mode=mode,
        max_frequency_candidates=max_frequency_candidates,
        max_candidates=max_candidates,
    )

    candidate_reports: list[dict[str, Any]] = []
    best: dict[str, Any] | None = None
    best_key: tuple[int, int, float, float] | None = None
    errors: list[str] = []

    for index, candidate in enumerate(candidates):
        try:
            result = scan_iq_file_windows(
                path,
                window_seconds=window_seconds,
                step_seconds=step_seconds,
                sample_rate=sample_rate,
                sps_values=sps_values,
                offset_step=offset_step,
                decode=True,
                decode_threshold=decode_threshold,
                max_decoded_payloads=max_decoded_payloads,
                max_decoded_frames=max_decoded_frames,
                aggregate_decode=True,
                frequency_shift_hz=candidate['frequency_shift_hz'],
                low_pass_hz=candidate['low_pass_hz'],
                filter_taps=filter_taps,
                allowed_access_names=(expected_access,),
                vote_repeated_frames=vote_repeated_frames,
            )
            summary = summarize_scan_result(result, frame_limit=frame_limit)
            score = score_summary(summary, expected_access=expected_access, expected_cmds=expected_cmds)
            report = {
                'index': index,
                'frequency_shift_hz': candidate['frequency_shift_hz'],
                'low_pass_hz': candidate['low_pass_hz'],
                'score': score,
                'summary': summary,
            }
            candidate_reports.append(report)
            key = (
                int(score['score']),
                -abs(int(candidate['frequency_shift_hz']) - int(requested_frequency_shift_hz)),
                -abs(float(candidate['low_pass_hz']) - float(base_low_pass_hz)),
                -float(index),
            )
            if best_key is None or key > best_key:
                best_key = key
                best = {
                    'index': index,
                    'candidate': candidate,
                    'result': result,
                    'summary': summary,
                    'score': score,
                }
        except Exception as exc:  # noqa: BLE001 - keep scanning other compliant RX candidates.
            message = f'candidate {index} failed: {exc}'
            errors.append(message)
            candidate_reports.append(
                {
                    'index': index,
                    'frequency_shift_hz': candidate['frequency_shift_hz'],
                    'low_pass_hz': candidate['low_pass_hz'],
                    'error': str(exc),
                }
            )

    if best is None:
        raise RuntimeError('; '.join(errors) if errors else 'rx autotune produced no candidates')

    return {
        'enabled': str(mode).strip().lower() != 'off',
        'mode': str(mode).strip().lower(),
        'profile': clean_profile,
        'level': int(level),
        'expected_access': expected_access,
        'expected_cmds': list(expected_cmds),
        'sample_rate': float(sample_rate),
        'sps_values': list(sps_values),
        'offset_step': int(offset_step),
        'decode_threshold': int(decode_threshold),
        'peak_offsets_hz': list(peak_offsets),
        'candidate_count': len(candidates),
        'scanned_candidate_count': len(candidate_reports),
        'best_index': int(best['index']),
        'best_candidate': dict(best['candidate']),
        'best_score': dict(best['score']),
        'best_summary': dict(best['summary']),
        'candidates': candidate_reports,
        'errors': errors,
        'best_result': best['result'],
    }


def autotune_iq_samples(
    iq: np.ndarray,
    *,
    label: str = 'iq_samples',
    profile: str,
    level: int,
    sample_rate: float,
    base_low_pass_hz: float,
    requested_frequency_shift_hz: float = 0.0,
    expected_access: str | None = None,
    expected_cmds: Sequence[str] | None = None,
    start_second: float = 0.0,
    start_sample: int | None = None,
    sps_values: Iterable[float] | None = None,
    offset_step: int = 2,
    decode_threshold: int = 3,
    max_decoded_payloads: int = 128,
    max_decoded_frames: int = 32,
    frame_limit: int = 8,
    filter_taps: int = 513,
    mode: str = 'quick',
    estimate_peaks: bool = True,
    max_frequency_candidates: int = 4,
    max_candidates: int = 8,
    vote_repeated_frames: bool = False,
) -> dict[str, Any]:
    clean_profile = str(profile).strip().lower()
    expected_access = expected_access or clean_profile
    expected_cmds = tuple(expected_cmds or DEFAULT_EXPECTED_COMMANDS.get(clean_profile, ()))
    sps_values = tuple(sps_values or default_sps_values(sample_rate))
    samples = np.asarray(iq, dtype=np.complex64)

    peak_offsets = (
        estimate_peak_offsets_from_iq(samples, sample_rate=sample_rate)
        if estimate_peaks and str(mode).strip().lower() != 'off'
        else ()
    )
    candidates = build_candidates(
        profile=clean_profile,
        level=int(level),
        sample_rate=float(sample_rate),
        base_low_pass_hz=float(base_low_pass_hz),
        requested_frequency_shift_hz=float(requested_frequency_shift_hz),
        peak_offsets_hz=peak_offsets,
        mode=mode,
        max_frequency_candidates=max_frequency_candidates,
        max_candidates=max_candidates,
    )

    candidate_reports: list[dict[str, Any]] = []
    best: dict[str, Any] | None = None
    best_key: tuple[int, int, float, float] | None = None
    errors: list[str] = []

    for index, candidate in enumerate(candidates):
        try:
            result = scan_iq_samples(
                samples,
                label=label,
                sample_rate=sample_rate,
                start_second=start_second,
                sps_values=sps_values,
                offset_step=offset_step,
                decode=True,
                decode_threshold=decode_threshold,
                max_decoded_payloads=max_decoded_payloads,
                max_decoded_frames=max_decoded_frames,
                aggregate_decode=True,
                frequency_shift_hz=candidate['frequency_shift_hz'],
                low_pass_hz=candidate['low_pass_hz'],
                filter_taps=filter_taps,
                start_sample=start_sample,
                allowed_access_names=(expected_access,),
                vote_repeated_frames=vote_repeated_frames,
            )
            summary = summarize_scan_result(result, frame_limit=frame_limit)
            score = score_summary(summary, expected_access=expected_access, expected_cmds=expected_cmds)
            report = {
                'index': index,
                'frequency_shift_hz': candidate['frequency_shift_hz'],
                'low_pass_hz': candidate['low_pass_hz'],
                'score': score,
                'summary': summary,
            }
            candidate_reports.append(report)
            key = (
                int(score['score']),
                -abs(int(candidate['frequency_shift_hz']) - int(requested_frequency_shift_hz)),
                -abs(float(candidate['low_pass_hz']) - float(base_low_pass_hz)),
                -float(index),
            )
            if best_key is None or key > best_key:
                best_key = key
                best = {
                    'index': index,
                    'candidate': candidate,
                    'result': result,
                    'summary': summary,
                    'score': score,
                }
        except Exception as exc:  # noqa: BLE001 - live RX should keep trying other RX candidates.
            message = f'candidate {index} failed: {exc}'
            errors.append(message)
            candidate_reports.append(
                {
                    'index': index,
                    'frequency_shift_hz': candidate['frequency_shift_hz'],
                    'low_pass_hz': candidate['low_pass_hz'],
                    'error': str(exc),
                }
            )

    if best is None:
        raise RuntimeError('; '.join(errors) if errors else 'rx autotune produced no candidates')

    return {
        'enabled': str(mode).strip().lower() != 'off',
        'mode': str(mode).strip().lower(),
        'profile': clean_profile,
        'level': int(level),
        'expected_access': expected_access,
        'expected_cmds': list(expected_cmds),
        'sample_rate': float(sample_rate),
        'sps_values': list(sps_values),
        'offset_step': int(offset_step),
        'decode_threshold': int(decode_threshold),
        'peak_offsets_hz': list(peak_offsets),
        'candidate_count': len(candidates),
        'scanned_candidate_count': len(candidate_reports),
        'best_index': int(best['index']),
        'best_candidate': dict(best['candidate']),
        'best_score': dict(best['score']),
        'best_summary': dict(best['summary']),
        'candidates': candidate_reports,
        'errors': errors,
        'best_result': best['result'],
    }
