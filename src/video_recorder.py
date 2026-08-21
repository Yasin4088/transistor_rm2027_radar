from collections import deque
from concurrent.futures import ThreadPoolExecutor
from fractions import Fraction
import math
import threading
import time
from pathlib import Path

import cv2

try:
    import av
except ImportError:
    av = None


class _PyAVVideoWriter:
    """Small VideoWriter-compatible wrapper for FFmpeg encoders exposed by PyAV."""

    def __init__(self, path, codec, fps, size, options=None):
        if av is None:
            raise RuntimeError("PyAV 未安装")
        self._container = None
        self._stream = None
        self._open = False
        try:
            self._container = av.open(str(path), mode="w")
            rate = Fraction(str(float(fps))).limit_denominator(1001)
            self._stream = self._container.add_stream(
                codec,
                rate=rate,
                options=dict(options or {}),
            )
            self._stream.width = int(size[0])
            self._stream.height = int(size[1])
            self._stream.pix_fmt = "yuv420p"
            # Force encoder initialization here so an unavailable NVENC device
            # can fall back before the recording paths are announced.
            self._stream.codec_context.open()
            self._open = True
        except Exception:
            if self._container is not None:
                self._container.close()
            raise

    def isOpened(self):
        return self._open

    def write(self, frame):
        if not self._open:
            raise RuntimeError("录像编码器已经关闭")
        video_frame = av.VideoFrame.from_ndarray(frame, format="bgr24")
        for packet in self._stream.encode(video_frame):
            self._container.mux(packet)

    def release(self):
        if not self._open:
            return
        try:
            for packet in self._stream.encode():
                self._container.mux(packet)
        finally:
            self._container.close()
            self._open = False


class MatchVideoRecorder:
    """Write synchronized map and camera videos at a stable wall-clock frame rate."""

    def __init__(
        self,
        output_directory,
        fps=20.0,
        codec="h264_nvenc",
        max_catchup_seconds=1.0,
        map_max_width=None,
        camera_max_width=None,
    ):
        self.output_directory = Path(output_directory)
        self.fps = float(fps)
        if not math.isfinite(self.fps) or self.fps <= 0:
            raise ValueError("录像帧率必须大于 0")
        self.codec = str(codec or "h264_nvenc")
        self.max_catchup_frames = max(1, int(round(float(max_catchup_seconds) * self.fps)))
        self.map_max_width = self._normalize_max_width(map_max_width)
        self.camera_max_width = self._normalize_max_width(camera_max_width)
        self.map_writer = None
        self.camera_writer = None
        self.map_path = None
        self.camera_path = None
        self.map_size = None
        self.camera_size = None
        self.selected_codec = None
        self.selected_backend = None
        self.started_at = None
        self.frames_written = 0
        self._stream_workers = ThreadPoolExecutor(
            max_workers=2,
            thread_name_prefix="match-video-stream",
        )

    @staticmethod
    def _normalize_max_width(value):
        if value in (None, "", 0, "0"):
            return None
        value = int(value)
        if value < 2:
            raise ValueError("录像最大宽度必须至少为 2，或设为 0 保留原始尺寸")
        return value - value % 2

    @property
    def is_open(self):
        return bool(
            self.map_writer is not None
            and self.camera_writer is not None
            and self.map_writer.isOpened()
            and self.camera_writer.isOpened()
        )

    @staticmethod
    def _pyav_options(codec):
        if codec == "h264_nvenc":
            return {"preset": "p1", "tune": "ll", "rc": "vbr", "cq": "23"}
        if codec == "libx264":
            return {"preset": "ultrafast", "tune": "zerolatency", "crf": "23"}
        return {}

    def _candidate_codecs(self):
        primary = self.codec.strip()
        primary_lower = primary.lower()
        if primary_lower in ("h264_nvenc", "libx264"):
            primary_candidate = (primary_lower, "mp4", "pyav")
        else:
            extension = "avi" if primary.upper() in ("MJPG", "XVID") else "mp4"
            primary_candidate = (primary, extension, "opencv")
        candidates = [
            primary_candidate,
            ("h264_nvenc", "mp4", "pyav"),
            ("libx264", "mp4", "pyav"),
            ("mp4v", "mp4", "opencv"),
            ("MJPG", "avi", "opencv"),
        ]
        unique = []
        seen = set()
        for codec, extension, backend in candidates:
            key = (codec.lower(), extension, backend)
            if key not in seen:
                unique.append((codec, extension, backend))
                seen.add(key)
        return unique

    @staticmethod
    def _frame_size(frame):
        if frame is None or len(frame.shape) != 3 or frame.shape[2] != 3:
            raise ValueError("录像画面必须是 BGR 三通道图像")
        return int(frame.shape[1]), int(frame.shape[0])

    @classmethod
    def _target_size(cls, frame, max_width):
        width, height = cls._frame_size(frame)
        if max_width is None or width <= max_width:
            return width - width % 2, height - height % 2
        target_height = max(2, int(round(height * max_width / width / 2.0)) * 2)
        return int(max_width), target_height

    def _create_writer(self, path, codec, backend, size):
        if backend == "pyav":
            return _PyAVVideoWriter(
                path,
                codec=codec,
                fps=self.fps,
                size=size,
                options=self._pyav_options(codec),
            )
        fourcc = cv2.VideoWriter_fourcc(*codec)
        return cv2.VideoWriter(str(path), fourcc, self.fps, size)

    def _open(self, map_frame, camera_frame):
        self.output_directory.mkdir(parents=True, exist_ok=True)
        self.map_size = self._target_size(map_frame, self.map_max_width)
        self.camera_size = self._target_size(camera_frame, self.camera_max_width)

        failures = []
        for codec, extension, backend in self._candidate_codecs():
            map_path = self.output_directory / f"map.{extension}"
            camera_path = self.output_directory / f"camera.{extension}"
            map_writer = None
            camera_writer = None
            try:
                map_writer = self._create_writer(map_path, codec, backend, self.map_size)
                camera_writer = self._create_writer(camera_path, codec, backend, self.camera_size)
                if map_writer.isOpened() and camera_writer.isOpened():
                    self.map_writer = map_writer
                    self.camera_writer = camera_writer
                    self.map_path = map_path
                    self.camera_path = camera_path
                    self.selected_codec = codec
                    self.selected_backend = backend
                    return
                failures.append(f"{codec}/{backend}: 打开失败")
            except Exception as error:
                failures.append(f"{codec}/{backend}: {error}")
            for writer in (map_writer, camera_writer):
                if writer is not None:
                    try:
                        writer.release()
                    except Exception:
                        pass
            for failed_path in (map_path, camera_path):
                try:
                    failed_path.unlink()
                except FileNotFoundError:
                    pass

        raise RuntimeError(f"无法创建录像文件，已尝试编码器: {'; '.join(failures)}")

    @staticmethod
    def _normalize_frame(frame, expected_size):
        current_size = (int(frame.shape[1]), int(frame.shape[0]))
        if current_size == expected_size:
            return frame
        return cv2.resize(frame, expected_size)

    def prepare(self, map_frame, camera_frame):
        """Open and warm both encoders before live frame submission begins."""
        if not self.is_open:
            self._open(map_frame, camera_frame)

    @classmethod
    def _write_stream(cls, writer, frame, expected_size, frames_due):
        frame = cls._normalize_frame(frame, expected_size)
        for _ in range(frames_due):
            writer.write(frame)

    def write(self, map_frame, camera_frame, now=None):
        self.prepare(map_frame, camera_frame)

        now = time.monotonic() if now is None else float(now)
        if self.started_at is None:
            self.started_at = now

        elapsed_frames = (now - self.started_at) * self.fps
        expected_total = int(math.floor(elapsed_frames + 1e-6)) + 1
        frames_due = expected_total - self.frames_written
        if frames_due <= 0:
            return 0
        if frames_due > self.max_catchup_frames:
            skipped = frames_due - self.max_catchup_frames
            self.started_at += skipped / self.fps
            frames_due = self.max_catchup_frames

        futures = (
            self._stream_workers.submit(
                self._write_stream,
                self.map_writer,
                map_frame,
                self.map_size,
                frames_due,
            ),
            self._stream_workers.submit(
                self._write_stream,
                self.camera_writer,
                camera_frame,
                self.camera_size,
                frames_due,
            ),
        )
        first_error = None
        for future in futures:
            try:
                future.result()
            except Exception as error:
                if first_error is None:
                    first_error = error
        if first_error is not None:
            raise first_error
        self.frames_written += frames_due
        return frames_due

    def release(self):
        self._stream_workers.shutdown(wait=True)
        first_error = None
        for writer in (self.map_writer, self.camera_writer):
            if writer is not None:
                try:
                    writer.release()
                except Exception as error:
                    if first_error is None:
                        first_error = error
        self.map_writer = None
        self.camera_writer = None
        if first_error is not None:
            raise first_error

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.release()


class AsyncMatchVideoRecorder:
    """
    Sample and encode videos on a short bounded background queue.

    Calls arriving faster than the configured video FPS are rate-limited before
    entering the queue. A short queue absorbs normal encoder jitter without
    blocking inference. Only a genuinely full encoder queue replaces an old
    frame and increments ``dropped_frames``.
    """

    def __init__(
        self,
        output_directory,
        fps=20.0,
        codec="h264_nvenc",
        max_catchup_seconds=1.0,
        queue_size=4,
        map_max_width=None,
        camera_max_width=None,
    ):
        self._recorder = MatchVideoRecorder(
            output_directory,
            fps=fps,
            codec=codec,
            max_catchup_seconds=max_catchup_seconds,
            map_max_width=map_max_width,
            camera_max_width=camera_max_width,
        )
        self._queue_size = max(1, int(queue_size))
        self._queue = deque()
        self._condition = threading.Condition()
        self._stopping = False
        self._finished = False
        self._error = None
        self._next_submit_at = None
        self._input_frames = 0
        self._submitted_frames = 0
        self._rate_limited_frames = 0
        self._dropped_frames = 0
        self._queue_high_watermark = 0
        self._encoded_pairs = 0
        self._total_write_seconds = 0.0
        self._last_write_seconds = 0.0
        self._status = {
            "is_open": False,
            "map_path": None,
            "camera_path": None,
            "map_size": None,
            "camera_size": None,
            "frames_written": 0,
            "selected_codec": None,
            "selected_backend": None,
        }
        self._thread = threading.Thread(
            target=self._run,
            name="match-video-writer",
            daemon=True,
        )
        self._thread.start()

    def _current_recorder_status(self):
        return {
            "is_open": self._recorder.is_open,
            "map_path": self._recorder.map_path,
            "camera_path": self._recorder.camera_path,
            "map_size": self._recorder.map_size,
            "camera_size": self._recorder.camera_size,
            "frames_written": self._recorder.frames_written,
            "selected_codec": self._recorder.selected_codec,
            "selected_backend": self._recorder.selected_backend,
        }

    def prepare(self, map_frame, camera_frame):
        """Open both encoders before the live loop so startup cannot overflow the queue."""
        with self._condition:
            if self._input_frames:
                raise RuntimeError("录像开始接收画面后不能再次预热编码器")
            if self._stopping or self._error is not None:
                return False
        try:
            self._recorder.prepare(map_frame, camera_frame)
        except Exception as error:
            with self._condition:
                self._error = error
                self._stopping = True
                self._condition.notify_all()
            raise
        with self._condition:
            self._status = self._current_recorder_status()
            return True

    def submit(self, map_frame, camera_frame, now=None):
        """
        Sample and queue a frame pair without copying it or waiting for storage.

        Callers must not mutate the submitted arrays after this method returns.
        The match pipelines naturally satisfy this by allocating their next
        annotated map and camera frames on every loop iteration.
        """
        captured_at = time.monotonic() if now is None else float(now)
        with self._condition:
            if self._stopping or self._error is not None:
                return False
            self._input_frames += 1
            frame_interval = 1.0 / self._recorder.fps
            if self._next_submit_at is None:
                self._next_submit_at = captured_at + frame_interval
            elif captured_at + 1e-9 < self._next_submit_at:
                self._rate_limited_frames += 1
                return True
            else:
                elapsed_slots = int(
                    math.floor((captured_at - self._next_submit_at) / frame_interval + 1e-9)
                )
                self._next_submit_at += (elapsed_slots + 1) * frame_interval

            if len(self._queue) >= self._queue_size:
                self._queue.popleft()
                self._dropped_frames += 1
            self._queue.append((map_frame, camera_frame, captured_at))
            self._submitted_frames += 1
            self._queue_high_watermark = max(self._queue_high_watermark, len(self._queue))
            self._condition.notify()
            return True

    def _run(self):
        try:
            while True:
                with self._condition:
                    while not self._queue and not self._stopping:
                        self._condition.wait()
                    if not self._queue and self._stopping:
                        break
                    map_frame, camera_frame, captured_at = self._queue.popleft()

                write_started_at = time.perf_counter()
                self._recorder.write(map_frame, camera_frame, now=captured_at)
                write_seconds = time.perf_counter() - write_started_at
                with self._condition:
                    self._encoded_pairs += 1
                    self._total_write_seconds += write_seconds
                    self._last_write_seconds = write_seconds
                    self._status = self._current_recorder_status()
        except Exception as error:
            with self._condition:
                self._error = error
                self._queue.clear()
                self._stopping = True
        finally:
            try:
                self._recorder.release()
            except Exception as error:
                with self._condition:
                    if self._error is None:
                        self._error = error
            with self._condition:
                self._status["is_open"] = False
                self._finished = True
                self._condition.notify_all()

    def snapshot(self):
        with self._condition:
            status = dict(self._status)
            status.update(
                {
                    "error": None if self._error is None else str(self._error),
                    "input_frames": self._input_frames,
                    "submitted_frames": self._submitted_frames,
                    "rate_limited_frames": self._rate_limited_frames,
                    "dropped_frames": self._dropped_frames,
                    "queue_depth": len(self._queue),
                    "queue_capacity": self._queue_size,
                    "queue_high_watermark": self._queue_high_watermark,
                    "encoded_pairs": self._encoded_pairs,
                    "average_write_ms": (
                        self._total_write_seconds / self._encoded_pairs * 1000.0
                        if self._encoded_pairs
                        else 0.0
                    ),
                    "last_write_ms": self._last_write_seconds * 1000.0,
                    "finished": self._finished,
                }
            )
            return status

    def release(self, timeout=15.0):
        """Flush all queued frames and finalize both files."""
        with self._condition:
            self._stopping = True
            self._condition.notify_all()
        self._thread.join(timeout=None if timeout is None else max(float(timeout), 0.0))
        return not self._thread.is_alive()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.release()
