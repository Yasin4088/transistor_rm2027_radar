from rm_radio_ros.core.rx_autotune import (
    default_sps_values,
    estimate_peak_offsets_from_iq,
    build_candidates,
    frequency_shift_candidates,
    low_pass_candidates,
    score_summary,
)

import numpy as np


def test_default_sps_values_center_on_v2_rule_book_symbol_rate():
    values = default_sps_values(2_000_000, half_steps=2)

    assert values == (93.0, 93.5, 94.0, 94.5, 95.0)


def test_broadcast_level2_candidates_include_rx_only_frequency_offsets():
    shifts = frequency_shift_candidates(
        'broadcast',
        2,
        0.0,
        peak_offsets_hz=(-305_664.0,),
        mode='quick',
        max_candidates=8,
    )

    assert 0.0 in shifts
    assert -306_000.0 in shifts
    assert -300_000.0 in shifts


def test_broadcast_level_profiles_only_change_receive_lowpass_candidates():
    assert set(low_pass_candidates('broadcast', 1, 320_000, 2_000_000, mode='quick')) == {320_000.0}
    assert set(low_pass_candidates('broadcast', 2, 320_000, 2_000_000, mode='quick')) == {
        260_000.0,
        320_000.0,
        380_000.0,
    }
    assert set(low_pass_candidates('broadcast', 3, 320_000, 2_000_000, mode='quick')) == {
        220_000.0,
        260_000.0,
        320_000.0,
    }


def test_interference_profile_keeps_official_level_lowpass_as_base_candidate():
    assert low_pass_candidates('interference', 3, 160_000, 2_000_000, mode='quick') == (160_000.0,)


def test_score_summary_prefers_complete_command_set_over_payload_count_only():
    complete = score_summary(
        {
            'best_access': 'broadcast',
            'best_access_hamming_distance': 1,
            'air_payload_count': 4,
            'referee_frame_count': 2,
            'commands': [{'cmd_hex': '0x0A01'}, {'cmd_hex': '0x0A02'}],
        },
        expected_access='broadcast',
        expected_cmds=('0x0A01', '0x0A02'),
    )
    incomplete = score_summary(
        {
            'best_access': 'broadcast',
            'best_access_hamming_distance': 0,
            'air_payload_count': 1000,
            'referee_frame_count': 20,
            'commands': [{'cmd_hex': '0x0A01'}],
        },
        expected_access='broadcast',
        expected_cmds=('0x0A01', '0x0A02'),
    )

    assert complete['command_complete'] is True
    assert incomplete['command_complete'] is False
    assert complete['score'] > incomplete['score']


def test_build_candidates_is_bounded_for_dashboard_quick_scan():
    candidates = build_candidates(
        profile='broadcast',
        level=2,
        sample_rate=2_000_000,
        base_low_pass_hz=320_000,
        peak_offsets_hz=(-305_664.0, -250_000.0),
        mode='quick',
        max_frequency_candidates=8,
        max_candidates=10,
    )

    assert 1 <= len(candidates) <= 10
    assert all(0 < item['low_pass_hz'] < 1_000_000 for item in candidates)


def test_estimate_peak_offsets_from_live_iq_samples():
    sample_rate = 2_000_000
    tone_hz = -305_000
    samples = np.exp(1j * 2 * np.pi * tone_hz * np.arange(16384) / sample_rate).astype(np.complex64)

    peaks = estimate_peak_offsets_from_iq(samples, sample_rate=sample_rate, max_peaks=1)

    assert peaks
    assert abs(peaks[0] - tone_hz) <= 2_000
