import tempfile
import unittest
from pathlib import Path

from output.runtime_status import (
    clear_runtime_status,
    initialize_runtime_status,
    read_runtime_status,
    update_runtime_status,
)


class RuntimeStatusTest(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.status_path = Path(self.temporary_directory.name) / "runtime" / "status.json"

    def tearDown(self):
        self.temporary_directory.cleanup()

    def test_status_lifecycle(self):
        initialized = initialize_runtime_status(
            self.status_path,
            process_id=123,
            serial_enabled=True,
            serial_port="/dev/ttyUSB0",
        )
        self.assertEqual(initialized["phase"], "starting")
        self.assertFalse(initialized["serial_open"])
        self.assertIsNone(initialized["camera_error"])
        self.assertIsNone(initialized["camera_fps"])
        self.assertEqual(initialized["processing_fps"], 0.0)
        self.assertFalse(initialized["recording"])
        self.assertIsNone(initialized["recording_suspended_reason"])

        updated = update_runtime_status(
            self.status_path,
            serial_open=True,
            last_referee_packet_at=100.0,
            last_referee_command="0x020C",
        )
        self.assertTrue(updated["serial_open"])
        self.assertEqual(read_runtime_status(self.status_path), updated)

        clear_runtime_status(self.status_path)
        self.assertEqual(read_runtime_status(self.status_path), {})

    def test_invalid_json_is_treated_as_empty_status(self):
        self.status_path.parent.mkdir(parents=True)
        self.status_path.write_text("{partial", encoding="utf-8")
        self.assertEqual(read_runtime_status(self.status_path), {})


if __name__ == "__main__":
    unittest.main()
