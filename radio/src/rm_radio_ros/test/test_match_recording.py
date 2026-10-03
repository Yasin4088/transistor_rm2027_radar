import gzip
import json

import numpy as np
import pytest

from rm_radio_ros.core.match_recording import (
    CompactIqRecorder,
    CompressedJsonlWriter,
    iter_rmiq_chunks,
    read_rmiq_metadata,
    safe_session_id,
)


def test_compact_iq_recorder_decimates_quantizes_and_round_trips(tmp_path):
    recorder = CompactIqRecorder(
        enabled=True,
        root=tmp_path,
        role="rx1",
        input_sample_rate=2_000_000,
        output_sample_rate=1_000_000,
        segment_seconds=1.0,
        queue_chunks=16,
        min_free_gb=0,
        compression="zlib",
        compression_level=1,
    )
    source = (
        0.08
        * np.exp(1j * np.arange(20_002, dtype=np.float32) * np.float32(0.017))
    ).astype(np.complex64)

    assert recorder.start("20260729_120000_match", {"center_frequency_hz": 433_200_000})
    for block in np.array_split(source, 9):
        assert recorder.enqueue(block)
    assert recorder.stop(timeout=5.0)

    paths = sorted((tmp_path / "20260729_120000_match" / "rx1").glob("*.rmiq"))
    assert len(paths) == 1
    metadata = read_rmiq_metadata(paths[0])
    restored = np.concatenate([samples for _, samples in iter_rmiq_chunks(paths[0])])
    expected = source[::2]

    assert metadata["schema"] == "shark.radio.compact_iq.v1"
    assert metadata["output_sample_rate"] == 1_000_000
    assert restored.size == expected.size
    assert np.max(np.abs(restored - expected)) < 0.002
    status = recorder.snapshot()
    assert status["samples_written"] == expected.size
    assert status["dropped_chunks"] == 0
    assert status["segments_completed"] == 1


def test_compact_iq_recorder_rejects_non_integer_rate_ratio(tmp_path):
    with pytest.raises(ValueError, match="divide input_sample_rate"):
        CompactIqRecorder(
            enabled=True,
            root=tmp_path,
            role="rx1",
            input_sample_rate=2_000_000,
            output_sample_rate=750_000,
        )


def test_event_writer_gzip_is_valid_jsonl(tmp_path):
    writer = CompressedJsonlWriter(
        compression="gzip",
        compression_level=1,
        queue_events=8,
    )
    path = writer.start(tmp_path)
    assert writer.enqueue({"kind": "game_status", "game_progress": 4})
    assert writer.enqueue({"kind": "frame", "cmd": "0x0A01"})
    assert writer.stop(timeout=5.0)

    with gzip.open(path, "rt", encoding="utf-8") as stream:
        rows = [json.loads(line) for line in stream]
    assert [row["kind"] for row in rows] == ["game_status", "frame"]
    assert writer.snapshot()["events_written"] == 2


def test_recording_session_id_cannot_escape_root():
    assert safe_session_id("20260729_120000_match") == "20260729_120000_match"
    for invalid in ("", "../match", "match/name", "match name"):
        with pytest.raises(ValueError):
            safe_session_id(invalid)
