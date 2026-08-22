import tempfile
import unittest
from pathlib import Path

from ui.config_editor import load_config, update_config_values


SAMPLE_CONFIG = """global:
  state: 'B'  # 己方阵营
  use_serial: false  # 串口开关
  test:
    video_path: ''  # 测试视频
serial: # 裁判系统
  port: '/dev/ttyUSB0'
calibration:
  points_per_layer: 4
"""


class ConfigEditorTest(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.config_path = Path(self.temporary_directory.name) / "config.yaml"
        self.config_path.write_text(SAMPLE_CONFIG, encoding="utf-8")

    def tearDown(self):
        self.temporary_directory.cleanup()

    def test_updates_nested_values_and_preserves_comments(self):
        updated = update_config_values(
            self.config_path,
            {
                "global.state": "R",
                "global.use_serial": True,
                "global.test.video_path": "raw/match.mp4",
                "serial.port": "/dev/ttyACM0",
                "calibration.points_per_layer": 6,
            },
        )

        self.assertEqual(updated["global"]["state"], "R")
        self.assertTrue(updated["global"]["use_serial"])
        self.assertEqual(updated["serial"]["port"], "/dev/ttyACM0")
        text = self.config_path.read_text(encoding="utf-8")
        self.assertIn("state: 'R'  # 己方阵营", text)
        self.assertIn("use_serial: true  # 串口开关", text)
        self.assertIn("serial: # 裁判系统", text)
        self.assertEqual(load_config(self.config_path), updated)

    def test_missing_key_does_not_modify_file(self):
        before = self.config_path.read_text(encoding="utf-8")
        with self.assertRaises(KeyError):
            update_config_values(self.config_path, {"global.missing": True})
        self.assertEqual(self.config_path.read_text(encoding="utf-8"), before)


if __name__ == "__main__":
    unittest.main()
