from __future__ import annotations

import gzip
import json
import os
import queue
import re
import shutil
import struct
import subprocess
import threading
import time
import zlib
from pathlib import Path
from typing import Any, BinaryIO, Iterator, Optional

import numpy as np


RMIQ_MAGIC = b"RMIQ1\0\0\0"
RMIQ_CHUNK_MAGIC = b"CHNK"
_UINT32 = struct.Struct("<I")
_CHUNK_HEADER = struct.Struct("<4sIQQQIfII")
_SESSION_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,95}$")


def safe_session_id(value: str) -> str:
    session_id = str(value).strip()
    if not _SESSION_ID_RE.fullmatch(session_id):
        raise ValueError(f"invalid recording session_id: {value!r}")
    return session_id


def recording_session_dir(root: Path | str, session_id: str) -> Path:
    base = Path(root).expanduser().resolve()
    return base / safe_session_id(session_id)


def free_space_bytes(path: Path | str) -> int:
    target = Path(path).expanduser()
    target.mkdir(parents=True, exist_ok=True)
    return int(shutil.disk_usage(target).free)


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}-{threading.get_ident()}")
    try:
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


class CompactIqRecorder:
    """Asynchronously write filtered IQ to a segmented, replayable CI8 container.

    Each chunk has its own float scale, so quiet filtered signals retain the full
    signed 8-bit range. Disk work and quantization never run in the GNU Radio
    callback. A full queue drops recording chunks instead of blocking reception.
    """

    def __init__(
        self,
        *,
        enabled: bool,
        root: Path | str,
        role: str,
        input_sample_rate: float,
        output_sample_rate: float,
        segment_seconds: float = 30.0,
        queue_chunks: int = 64,
        min_free_gb: float = 15.0,
        compression: str = "zlib",
        compression_level: int = 1,
    ) -> None:
        self.enabled = bool(enabled)
        self.root = Path(root).expanduser().resolve()
        self.role = str(role).strip() or "rx"
        self.input_sample_rate = float(input_sample_rate)
        self.output_sample_rate = float(output_sample_rate)
        self.segment_seconds = max(float(segment_seconds), 1.0)
        self.min_free_bytes = max(float(min_free_gb), 0.0) * 1_000_000_000
        self.compression = str(compression).strip().lower()
        if self.compression not in {"none", "zlib"}:
            raise ValueError(f"unsupported IQ compression: {compression!r}")
        self.compression_level = min(max(int(compression_level), 0), 9)
        ratio = self.input_sample_rate / self.output_sample_rate
        self.decimation = int(round(ratio))
        if (
            self.input_sample_rate <= 0
            or self.output_sample_rate <= 0
            or self.decimation < 1
            or abs(ratio - self.decimation) > 1e-6
        ):
            raise ValueError(
                "recording output_sample_rate must divide input_sample_rate by "
                f"an integer: input={self.input_sample_rate:g}, output={self.output_sample_rate:g}"
            )

        self._queue: queue.Queue[tuple[int, int, np.ndarray]] = queue.Queue(
            maxsize=max(int(queue_chunks), 1)
        )
        self._lock = threading.RLock()
        self._thread: Optional[threading.Thread] = None
        self._stop_requested = threading.Event()
        self._accepting = False
        self._session_id = ""
        self._session_dir: Optional[Path] = None
        self._initial_metadata: dict[str, Any] = {}
        self._last_error = ""
        self._started_at = 0.0
        self._stopped_at = 0.0
        self._bytes_written = 0
        self._samples_written = 0
        self._input_samples_seen = 0
        self._chunks_written = 0
        self._dropped_chunks = 0
        self._dropped_input_samples = 0
        self._segments_completed = 0

    def start(self, session_id: str, metadata: Optional[dict[str, Any]] = None) -> bool:
        if not self.enabled:
            return False
        clean_session = safe_session_id(session_id)
        with self._lock:
            if self._accepting and self._session_id == clean_session:
                return True
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("previous IQ recording writer is still active")
            session_dir = recording_session_dir(self.root, clean_session) / self.role
            session_dir.mkdir(parents=True, exist_ok=True)
            if free_space_bytes(session_dir) < self.min_free_bytes:
                raise OSError(
                    f"recording free space is below {self.min_free_bytes / 1_000_000_000:g} GB"
                )
            while True:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    break
            self._stop_requested.clear()
            self._accepting = True
            self._session_id = clean_session
            self._session_dir = session_dir
            self._initial_metadata = dict(metadata or {})
            self._last_error = ""
            self._started_at = time.time()
            self._stopped_at = 0.0
            self._bytes_written = 0
            self._samples_written = 0
            self._input_samples_seen = 0
            self._chunks_written = 0
            self._dropped_chunks = 0
            self._dropped_input_samples = 0
            self._segments_completed = 0
            self._thread = threading.Thread(
                target=self._writer_main,
                name=f"rm-iq-writer-{self.role}",
                daemon=True,
            )
            self._thread.start()
        return True

    def enqueue(self, samples: Any, *, wall_time_ns: Optional[int] = None) -> bool:
        if not self.enabled:
            return False
        data = np.asarray(samples, dtype=np.complex64)
        if data.size == 0:
            return False
        with self._lock:
            accepting = self._accepting
        if not accepting:
            return False
        item = (
            int(time.time_ns() if wall_time_ns is None else wall_time_ns),
            int(time.monotonic_ns()),
            data.copy(),
        )
        try:
            self._queue.put_nowait(item)
            return True
        except queue.Full:
            with self._lock:
                self._dropped_chunks += 1
                self._dropped_input_samples += int(data.size)
            return False

    def stop(self, reason: str = "stop", timeout: float = 8.0) -> bool:
        del reason  # Reason is recorded by the coordinator event log.
        with self._lock:
            was_active = self._accepting or bool(self._thread and self._thread.is_alive())
            self._accepting = False
            thread = self._thread
            self._stop_requested.set()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=max(float(timeout), 0.0))
            if thread.is_alive():
                with self._lock:
                    self._last_error = "IQ writer did not stop before timeout"
                return False
        with self._lock:
            self._stopped_at = time.time()
            if self._thread is thread and (thread is None or not thread.is_alive()):
                self._thread = None
        return was_active

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            thread_alive = bool(self._thread and self._thread.is_alive())
            return {
                "enabled": self.enabled,
                "active": bool(self._accepting or thread_alive),
                "session_id": self._session_id or None,
                "session_dir": str(self._session_dir) if self._session_dir else None,
                "role": self.role,
                "input_sample_rate": self.input_sample_rate,
                "output_sample_rate": self.output_sample_rate,
                "decimation": self.decimation,
                "format": "ci8_block_scaled",
                "compression": self.compression,
                "bytes_written": int(self._bytes_written),
                "samples_written": int(self._samples_written),
                "chunks_written": int(self._chunks_written),
                "dropped_chunks": int(self._dropped_chunks),
                "dropped_input_samples": int(self._dropped_input_samples),
                "segments_completed": int(self._segments_completed),
                "queue_depth": int(self._queue.qsize()),
                "queue_capacity": int(self._queue.maxsize),
                "started_at": self._started_at or None,
                "stopped_at": self._stopped_at or None,
                "last_error": self._last_error,
            }

    def _writer_main(self) -> None:
        stream: Optional[BinaryIO] = None
        part_path: Optional[Path] = None
        segment_samples = 0
        segment_index = 0
        chunk_sequence = 0
        target_segment_samples = max(
            int(round(self.segment_seconds * self.output_sample_rate)),
            1,
        )
        output_index = 0
        input_index = 0
        try:
            while not self._stop_requested.is_set() or not self._queue.empty():
                try:
                    wall_ns, monotonic_ns, data = self._queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                phase = (-input_index) % self.decimation
                decimated = data[phase:: self.decimation]
                input_index += int(data.size)
                with self._lock:
                    self._input_samples_seen = input_index
                offset = 0
                while offset < int(decimated.size):
                    if stream is None:
                        stream, part_path = self._open_segment(segment_index, output_index)
                        segment_samples = 0
                    count = min(
                        int(decimated.size) - offset,
                        target_segment_samples - segment_samples,
                    )
                    block = decimated[offset:offset + count]
                    written = self._write_chunk(
                        stream,
                        chunk_sequence,
                        wall_ns,
                        monotonic_ns,
                        output_index,
                        block,
                    )
                    with self._lock:
                        self._bytes_written += written
                        self._samples_written += count
                        self._chunks_written += 1
                    chunk_sequence += 1
                    segment_samples += count
                    output_index += count
                    offset += count
                    if segment_samples >= target_segment_samples:
                        self._finalize_segment(stream, part_path)
                        stream = None
                        part_path = None
                        segment_index += 1
                        with self._lock:
                            self._segments_completed += 1
            if stream is not None and part_path is not None:
                self._finalize_segment(stream, part_path)
                stream = None
                part_path = None
                with self._lock:
                    self._segments_completed += 1
        except Exception as exc:
            with self._lock:
                self._last_error = f"IQ recording failed: {exc}"
                self._accepting = False
        finally:
            if stream is not None:
                try:
                    stream.flush()
                    os.fsync(stream.fileno())
                    stream.close()
                except Exception:
                    pass
            with self._lock:
                self._accepting = False

    def _open_segment(self, segment_index: int, output_index: int) -> tuple[BinaryIO, Path]:
        assert self._session_dir is not None
        if free_space_bytes(self._session_dir) < self.min_free_bytes:
            raise OSError("recording stopped because reserved free-space threshold was reached")
        part_path = self._session_dir / f"channel_{segment_index:03d}.rmiq.part"
        stream = part_path.open("wb")
        metadata = {
            "schema": "shark.radio.compact_iq.v1",
            "session_id": self._session_id,
            "role": self.role,
            "segment_index": segment_index,
            "segment_output_start": output_index,
            "input_sample_rate": self.input_sample_rate,
            "output_sample_rate": self.output_sample_rate,
            "decimation": self.decimation,
            "data_type": "ci8_interleaved",
            "chunk_scale": "complex64 = int8 * scale",
            "payload_compression": self.compression,
            "created_at": time.time(),
            "initial": self._initial_metadata,
        }
        encoded = json.dumps(metadata, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        stream.write(RMIQ_MAGIC)
        stream.write(_UINT32.pack(len(encoded)))
        stream.write(encoded)
        return stream, part_path

    def _write_chunk(
        self,
        stream: BinaryIO,
        sequence: int,
        wall_ns: int,
        monotonic_ns: int,
        output_index: int,
        samples: np.ndarray,
    ) -> int:
        max_component = max(
            float(np.max(np.abs(samples.real), initial=0.0)),
            float(np.max(np.abs(samples.imag), initial=0.0)),
        )
        scale = max(max_component / 127.0, 1e-12)
        quantized = np.empty(samples.size * 2, dtype=np.int8)
        quantized[0::2] = np.clip(np.rint(samples.real / scale), -127, 127).astype(np.int8)
        quantized[1::2] = np.clip(np.rint(samples.imag / scale), -127, 127).astype(np.int8)
        raw_payload = quantized.tobytes()
        crc = zlib.crc32(raw_payload) & 0xFFFFFFFF
        payload = (
            zlib.compress(raw_payload, self.compression_level)
            if self.compression == "zlib"
            else raw_payload
        )
        header = _CHUNK_HEADER.pack(
            RMIQ_CHUNK_MAGIC,
            int(sequence),
            int(wall_ns),
            int(monotonic_ns),
            int(output_index),
            int(samples.size),
            float(scale),
            len(payload),
            crc,
        )
        stream.write(header)
        stream.write(payload)
        return len(header) + len(payload)

    @staticmethod
    def _finalize_segment(stream: BinaryIO, part_path: Path) -> None:
        stream.flush()
        os.fsync(stream.fileno())
        stream.close()
        os.replace(part_path, part_path.with_suffix(""))


class CompressedJsonlWriter:
    """Bounded asynchronous JSONL writer with zstd, gzip, or plain output."""

    def __init__(
        self,
        *,
        compression: str = "zstd",
        compression_level: int = 1,
        queue_events: int = 4096,
    ) -> None:
        self.requested_compression = str(compression).strip().lower()
        if self.requested_compression not in {"zstd", "gzip", "none"}:
            raise ValueError(f"unsupported event compression: {compression!r}")
        self.compression_level = min(max(int(compression_level), 1), 9)
        self._queue: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=max(int(queue_events), 1))
        self._lock = threading.RLock()
        self._thread: Optional[threading.Thread] = None
        self._stop_requested = threading.Event()
        self._accepting = False
        self._path: Optional[Path] = None
        self._actual_compression = self.requested_compression
        self._bytes_uncompressed = 0
        self._events_written = 0
        self._events_dropped = 0
        self._last_error = ""

    def start(self, directory: Path) -> Path:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("event writer is already active")
            directory.mkdir(parents=True, exist_ok=True)
            actual = self.requested_compression
            if actual == "zstd" and shutil.which("zstd") is None:
                actual = "gzip"
            suffix = {"zstd": ".jsonl.zst", "gzip": ".jsonl.gz", "none": ".jsonl"}[actual]
            self._path = directory / f"events{suffix}"
            self._actual_compression = actual
            self._bytes_uncompressed = 0
            self._events_written = 0
            self._events_dropped = 0
            self._last_error = ""
            while True:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    break
            self._stop_requested.clear()
            self._accepting = True
            self._thread = threading.Thread(
                target=self._writer_main,
                name="rm-match-event-writer",
                daemon=True,
            )
            self._thread.start()
            return self._path

    def enqueue(self, payload: dict[str, Any]) -> bool:
        with self._lock:
            accepting = self._accepting
        if not accepting:
            return False
        try:
            self._queue.put_nowait(dict(payload))
            return True
        except queue.Full:
            with self._lock:
                self._events_dropped += 1
            return False

    def stop(self, timeout: float = 8.0) -> bool:
        with self._lock:
            was_active = self._accepting or bool(self._thread and self._thread.is_alive())
            self._accepting = False
            thread = self._thread
            self._stop_requested.set()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=max(float(timeout), 0.0))
            if thread.is_alive():
                with self._lock:
                    self._last_error = "event writer did not stop before timeout"
                return False
        with self._lock:
            if self._thread is thread and (thread is None or not thread.is_alive()):
                self._thread = None
        return was_active

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "active": bool(self._accepting or (self._thread and self._thread.is_alive())),
                "path": str(self._path) if self._path else None,
                "compression": self._actual_compression,
                "bytes_uncompressed": int(self._bytes_uncompressed),
                "events_written": int(self._events_written),
                "events_dropped": int(self._events_dropped),
                "queue_depth": int(self._queue.qsize()),
                "queue_capacity": int(self._queue.maxsize),
                "last_error": self._last_error,
            }

    def _writer_main(self) -> None:
        assert self._path is not None
        part_path = self._path.with_name(self._path.name + ".part")
        raw_stream: Optional[BinaryIO] = None
        output: Any = None
        process: Optional[subprocess.Popen] = None
        try:
            if self._actual_compression == "zstd":
                raw_stream = part_path.open("wb")
                process = subprocess.Popen(
                    [
                        "zstd",
                        "-q",
                        f"-{self.compression_level}",
                        "-c",
                    ],
                    stdin=subprocess.PIPE,
                    stdout=raw_stream,
                    stderr=subprocess.PIPE,
                )
                if process.stdin is None:
                    raise RuntimeError("zstd stdin was not created")
                output = process.stdin
            elif self._actual_compression == "gzip":
                output = gzip.open(part_path, "wb", compresslevel=self.compression_level)
            else:
                output = part_path.open("wb")

            while not self._stop_requested.is_set() or not self._queue.empty():
                try:
                    payload = self._queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                encoded = (
                    json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
                    + b"\n"
                )
                output.write(encoded)
                with self._lock:
                    self._bytes_uncompressed += len(encoded)
                    self._events_written += 1

            output.flush()
            output.close()
            output = None
            if process is not None:
                returncode = process.wait(timeout=5.0)
                if returncode != 0:
                    stderr = (
                        process.stderr.read().decode("utf-8", errors="replace").strip()
                        if process.stderr is not None
                        else ""
                    )
                    raise RuntimeError(stderr or f"zstd exited with {returncode}")
                process = None
            if raw_stream is not None:
                raw_stream.flush()
                os.fsync(raw_stream.fileno())
                raw_stream.close()
                raw_stream = None
            os.replace(part_path, self._path)
        except Exception as exc:
            with self._lock:
                self._last_error = f"event recording failed: {exc}"
                self._accepting = False
        finally:
            if output is not None:
                try:
                    output.close()
                except Exception:
                    pass
            if process is not None:
                try:
                    process.terminate()
                    process.wait(timeout=1.0)
                except Exception:
                    pass
            if raw_stream is not None:
                try:
                    raw_stream.close()
                except Exception:
                    pass
            with self._lock:
                self._accepting = False


def read_rmiq_metadata(path: Path | str) -> dict[str, Any]:
    with Path(path).open("rb") as stream:
        magic = stream.read(len(RMIQ_MAGIC))
        if magic != RMIQ_MAGIC:
            raise ValueError(f"not an RMIQ file: {path}")
        raw_length = stream.read(_UINT32.size)
        if len(raw_length) != _UINT32.size:
            raise ValueError("truncated RMIQ metadata length")
        length = _UINT32.unpack(raw_length)[0]
        encoded = stream.read(length)
        if len(encoded) != length:
            raise ValueError("truncated RMIQ metadata")
        return json.loads(encoded.decode("utf-8"))


def iter_rmiq_chunks(path: Path | str) -> Iterator[tuple[dict[str, Any], np.ndarray]]:
    with Path(path).open("rb") as stream:
        if stream.read(len(RMIQ_MAGIC)) != RMIQ_MAGIC:
            raise ValueError(f"not an RMIQ file: {path}")
        raw_length = stream.read(_UINT32.size)
        if len(raw_length) != _UINT32.size:
            raise ValueError("truncated RMIQ metadata length")
        metadata_length = _UINT32.unpack(raw_length)[0]
        metadata = json.loads(stream.read(metadata_length).decode("utf-8"))
        compression = str(metadata.get("payload_compression", "none"))
        while True:
            raw_header = stream.read(_CHUNK_HEADER.size)
            if not raw_header:
                return
            if len(raw_header) != _CHUNK_HEADER.size:
                raise ValueError("truncated RMIQ chunk header")
            (
                magic,
                sequence,
                wall_ns,
                monotonic_ns,
                output_index,
                sample_count,
                scale,
                payload_length,
                expected_crc,
            ) = _CHUNK_HEADER.unpack(raw_header)
            if magic != RMIQ_CHUNK_MAGIC:
                raise ValueError("invalid RMIQ chunk magic")
            payload = stream.read(payload_length)
            if len(payload) != payload_length:
                raise ValueError("truncated RMIQ chunk payload")
            raw_payload = zlib.decompress(payload) if compression == "zlib" else payload
            if zlib.crc32(raw_payload) & 0xFFFFFFFF != expected_crc:
                raise ValueError("RMIQ chunk CRC mismatch")
            values = np.frombuffer(raw_payload, dtype=np.int8)
            if values.size != sample_count * 2:
                raise ValueError("RMIQ chunk sample count mismatch")
            samples = (
                values[0::2].astype(np.float32) + 1j * values[1::2].astype(np.float32)
            ) * float(scale)
            yield (
                {
                    "sequence": sequence,
                    "wall_time_ns": wall_ns,
                    "monotonic_time_ns": monotonic_ns,
                    "output_start": output_index,
                    "sample_count": sample_count,
                    "scale": float(scale),
                },
                samples.astype(np.complex64),
            )
