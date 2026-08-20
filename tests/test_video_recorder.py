import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

import cv2
import numpy as np

from video_recorder import AsyncMatchVideoRecorder, MatchVideoRecorder


class MatchVideoRecorderTest(unittest.TestCase):
    def test_writes_synchronized_readable_videos(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            map_frame = np.full((48, 64, 3), (20, 80, 160), dtype=np.uint8)
            camera_frame = np.full((60, 80, 3), (160, 80, 20), dtype=np.uint8)
            recorder = MatchVideoRecorder(temporary_directory, fps=10.0, codec="MJPG")

            self.assertEqual(recorder.write(map_frame, camera_frame, now=100.0), 1)
            self.assertEqual(recorder.write(map_frame, camera_frame, now=100.2), 2)
            map_path = recorder.map_path
            camera_path = recorder.camera_path
            recorder.release()

            for path, expected_size in ((map_path, (64, 48)), (camera_path, (80, 60))):
                self.assertTrue(Path(path).exists())
                self.assertGreater(Path(path).stat().st_size, 0)
                capture = cv2.VideoCapture(str(path))
                self.assertTrue(capture.isOpened())
                self.assertEqual(int(capture.get(cv2.CAP_PROP_FRAME_COUNT)), 3)
                self.assertEqual(int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)), expected_size[0])
                self.assertEqual(int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)), expected_size[1])
                capture.release()

    def test_rejects_invalid_frame_rate(self):
        with self.assertRaises(ValueError):
            MatchVideoRecorder("unused", fps=0)

    def test_scales_each_stream_to_configured_maximum_width(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            map_frame = np.full((48, 64, 3), (20, 80, 160), dtype=np.uint8)
            camera_frame = np.full((60, 80, 3), (160, 80, 20), dtype=np.uint8)
            recorder = MatchVideoRecorder(
                temporary_directory,
                fps=10.0,
                codec="MJPG",
                map_max_width=32,
                camera_max_width=40,
            )

            recorder.write(map_frame, camera_frame, now=100.0)
            map_path = recorder.map_path
            camera_path = recorder.camera_path
            recorder.release()

            for path, expected_size in ((map_path, (32, 24)), (camera_path, (40, 30))):
                capture = cv2.VideoCapture(str(path))
                self.assertTrue(capture.isOpened())
                self.assertEqual(int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)), expected_size[0])
                self.assertEqual(int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)), expected_size[1])
                capture.release()


class AsyncMatchVideoRecorderTest(unittest.TestCase):
    def test_writes_on_background_thread(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            map_frame = np.full((48, 64, 3), (20, 80, 160), dtype=np.uint8)
            camera_frame = np.full((60, 80, 3), (160, 80, 20), dtype=np.uint8)
            recorder = AsyncMatchVideoRecorder(
                temporary_directory,
                fps=10.0,
                codec="MJPG",
            )

            self.assertTrue(recorder.prepare(map_frame, camera_frame))
            prepared = recorder.snapshot()
            self.assertEqual(prepared["selected_codec"], "MJPG")
            self.assertEqual(prepared["frames_written"], 0)
            self.assertTrue(recorder.submit(map_frame, camera_frame, now=100.0))
            self.assertTrue(recorder.release())
            status = recorder.snapshot()

            self.assertIsNone(status["error"])
            self.assertTrue(status["finished"])
            self.assertEqual(status["submitted_frames"], 1)
            self.assertEqual(status["frames_written"], 1)
            for key in ("map_path", "camera_path"):
                path = Path(status[key])
                self.assertTrue(path.exists())
                self.assertGreater(path.stat().st_size, 0)

    def test_replaces_waiting_frame_instead_of_blocking_submitter(self):
        write_started = threading.Event()
        allow_write = threading.Event()

        def slow_write(recorder, map_frame, camera_frame, now=None):
            write_started.set()
            allow_write.wait(2.0)
            recorder.frames_written += 1
            return 1

        with tempfile.TemporaryDirectory() as temporary_directory:
            frame = np.zeros((8, 8, 3), dtype=np.uint8)
            with mock.patch.object(MatchVideoRecorder, "write", slow_write):
                recorder = AsyncMatchVideoRecorder(
                    temporary_directory,
                    fps=10.0,
                    queue_size=1,
                )
                self.assertTrue(recorder.submit(frame, frame, now=1.0))
                self.assertTrue(write_started.wait(1.0))
                started_at = time.monotonic()
                self.assertTrue(recorder.submit(frame, frame, now=1.1))
                self.assertTrue(recorder.submit(frame, frame, now=1.2))
                submit_elapsed = time.monotonic() - started_at
                allow_write.set()
                self.assertTrue(recorder.release())

            status = recorder.snapshot()
            self.assertLess(submit_elapsed, 0.5)
            self.assertEqual(status["submitted_frames"], 3)
            self.assertEqual(status["dropped_frames"], 1)
            self.assertEqual(status["frames_written"], 2)

    def test_rate_limits_input_before_it_can_fill_encoder_queue(self):
        def fast_write(recorder, map_frame, camera_frame, now=None):
            recorder.frames_written += 1
            return 1

        with tempfile.TemporaryDirectory() as temporary_directory:
            frame = np.zeros((8, 8, 3), dtype=np.uint8)
            with mock.patch.object(MatchVideoRecorder, "write", fast_write):
                recorder = AsyncMatchVideoRecorder(
                    temporary_directory,
                    fps=20.0,
                    queue_size=4,
                )
                for captured_at in (1.0, 1.01, 1.049, 1.05, 1.099, 1.1):
                    self.assertTrue(recorder.submit(frame, frame, now=captured_at))
                self.assertTrue(recorder.release())

            status = recorder.snapshot()
            self.assertEqual(status["input_frames"], 6)
            self.assertEqual(status["submitted_frames"], 3)
            self.assertEqual(status["rate_limited_frames"], 3)
            self.assertEqual(status["dropped_frames"], 0)
            self.assertEqual(status["frames_written"], 3)


if __name__ == "__main__":
    unittest.main()
