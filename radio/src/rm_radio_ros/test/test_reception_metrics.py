import pytest

from rm_radio_ros.core.reception_metrics import ReceptionMetrics


def _record(metrics, stage, seq, timestamp):
    metrics.record({'stage': stage, 'seq': seq}, timestamp=timestamp)


def test_loss_estimate_handles_uint8_wrap_and_sequence_gap():
    metrics = ReceptionMetrics(window_sec=10.0, idle_reset_sec=2.0)
    _record(metrics, 'valid', 254, 0.0)
    _record(metrics, 'valid', 255, 0.1)
    _record(metrics, 'valid', 0, 0.2)
    _record(metrics, 'valid', 2, 0.3)

    snapshot = metrics.snapshot(now=0.3)

    assert snapshot['estimate_available'] is True
    assert snapshot['estimated_original_frames'] == 5
    assert snapshot['suspected_frames'] == 4
    assert snapshot['pre_crc_missing_frames'] == 1
    assert snapshot['end_to_end_loss_rate'] == pytest.approx(0.2)


def test_loss_estimate_counts_crc_and_command_length_rejections_only_once():
    metrics = ReceptionMetrics(window_sec=10.0)
    _record(metrics, 'valid', 10, 1.0)
    _record(metrics, 'crc16_failure', 11, 1.1)
    _record(metrics, 'command_length_failure', 12, 1.2)
    metrics.record({'stage': 'crc8_failure'}, timestamp=1.25)
    _record(metrics, 'valid', 13, 1.3)

    snapshot = metrics.snapshot(now=1.3)

    assert snapshot['estimated_original_frames'] == 4
    assert snapshot['rejected_frames'] == 2
    assert snapshot['crc_valid_frames'] == 2
    assert snapshot['counts']['crc8_failures'] == 1
    assert snapshot['rejection_rate'] == pytest.approx(0.5)
    assert snapshot['end_to_end_loss_rate'] == pytest.approx(0.5)
    assert snapshot['crc_rejected_frames'] == 2
    assert snapshot['crc_rejection_rate'] == pytest.approx(0.5)
    assert snapshot['pre_crc_loss_rate'] == 0.0
    assert snapshot['scope'] == 'primary_strict_decoder_sliding_window'
    assert snapshot['estimate_method'] == 'CRC8-protected uint8 sequence gaps'
    assert snapshot['candidate_sequence']['received_frames'] == 4
    assert snapshot['candidate_sequence']['expected_frames'] == 4
    assert snapshot['debug'] == {
        'sof_candidates': 5,
        'crc8_passed_candidates': 4,
        'length_failures': 0,
        'command_length_failures': 1,
        'crc16_attempts': 4,
        'crc8_failures': 1,
        'crc16_failures': 1,
        'valid_frames': 2,
    }


def test_loss_estimate_resets_sequence_across_idle_gap():
    metrics = ReceptionMetrics(window_sec=10.0, idle_reset_sec=2.0)
    _record(metrics, 'valid', 1, 0.0)
    _record(metrics, 'valid', 100, 3.0)

    snapshot = metrics.snapshot(now=3.0)

    assert snapshot['estimate_available'] is False
    assert snapshot['pre_crc_missing_frames'] == 0
    assert snapshot['candidate_sequence']['sessions'] == 2


def test_loss_estimate_prunes_events_outside_sliding_window():
    metrics = ReceptionMetrics(window_sec=10.0)
    _record(metrics, 'valid', 1, 0.0)
    _record(metrics, 'crc16_failure', 2, 1.0)
    _record(metrics, 'valid', 20, 20.0)
    _record(metrics, 'valid', 21, 20.1)

    snapshot = metrics.snapshot(now=20.1)

    assert snapshot['suspected_frames'] == 2
    assert snapshot['crc_valid_frames'] == 2
    assert snapshot['rejected_frames'] == 0
    assert snapshot['counts']['crc16_failures'] == 0
