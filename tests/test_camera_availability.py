import unittest

from camera_availability import wait_for_initial_camera_frame


class CameraAvailabilityTest(unittest.TestCase):
    def test_missing_camera_keeps_running_without_recording_until_frame_arrives(self):
        state = {
            "frame": None,
            "error": "未发现海康相机",
            "now": 0.0,
            "sleeps": 0,
        }
        statuses = []
        logs = []
        real_frame = object()

        def sleep(_seconds):
            state["sleeps"] += 1
            state["now"] += 1.0
            if state["sleeps"] == 2:
                state["frame"] = real_frame

        result = wait_for_initial_camera_frame(
            get_frame=lambda: state["frame"],
            get_error=lambda: state["error"],
            should_stop=lambda: False,
            report_status=lambda **values: statuses.append(values),
            logger=logs.append,
            poll_interval=0.0,
            status_interval=1.0,
            log_interval=5.0,
            monotonic=lambda: state["now"],
            wall_time=lambda: 100.0 + state["now"],
            sleeper=sleep,
        )

        self.assertIs(result, real_frame)
        self.assertEqual(state["sleeps"], 2)
        self.assertGreaterEqual(len(statuses), 2)
        self.assertTrue(all(status["phase"] == "running" for status in statuses))
        self.assertTrue(all(status["camera_ready"] is False for status in statuses))
        self.assertTrue(all(status["recording"] is False for status in statuses))
        self.assertTrue(
            all(
                status["recording_suspended_reason"] == "camera_unavailable"
                for status in statuses
            )
        )
        self.assertIn("不会录像", logs[0])

    def test_stop_request_ends_camera_wait_cleanly(self):
        state = {"stop": False}
        statuses = []

        def sleep(_seconds):
            state["stop"] = True

        result = wait_for_initial_camera_frame(
            get_frame=lambda: None,
            get_error=lambda: None,
            should_stop=lambda: state["stop"],
            report_status=lambda **values: statuses.append(values),
            logger=lambda _message: None,
            poll_interval=0.0,
            monotonic=lambda: 0.0,
            wall_time=lambda: 100.0,
            sleeper=sleep,
        )

        self.assertIsNone(result)
        self.assertEqual(len(statuses), 1)
        self.assertFalse(statuses[0]["recording"])


if __name__ == "__main__":
    unittest.main()
