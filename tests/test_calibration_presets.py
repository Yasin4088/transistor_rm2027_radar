import json
import tempfile
import unittest
from pathlib import Path

from calibration.calibration_presets import (
    CalibrationPresetError,
    delete_preset,
    get_preset,
    list_presets,
    load_presets,
    save_preset,
)


def _points(offset=0, count=4):
    return [
        [[offset + index * 10, offset + index * 5] for index in range(count)],
        [[offset + 100 + index * 10, offset + 50 + index * 5] for index in range(count)],
    ]


class CalibrationPresetsTest(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.preset_path = Path(self.temporary_directory.name) / "presets.json"

    def tearDown(self):
        self.temporary_directory.cleanup()

    def test_saves_multiple_schemes_and_filters_by_side_and_point_count(self):
        save_preset(
            "主赛场",
            "R",
            _points(1000, 4),
            path=self.preset_path,
        )
        save_preset(
            "训练场",
            "R",
            _points(1100, 6),
            path=self.preset_path,
        )
        save_preset(
            "主赛场",
            "B",
            _points(1200, 4),
            path=self.preset_path,
        )

        self.assertEqual(len(load_presets(self.preset_path)), 3)
        matching = list_presets(self.preset_path, side="R", points_per_layer=4)
        self.assertEqual([preset["name"] for preset in matching], ["主赛场"])
        self.assertEqual(matching[0]["side"], "R")
        stored_document = json.loads(self.preset_path.read_text(encoding="utf-8"))
        self.assertEqual(stored_document["version"], 2)
        self.assertTrue(
            all("image_points" not in preset for preset in stored_document["presets"])
        )

    def test_overwrite_is_explicit_and_does_not_replace_other_side(self):
        save_preset(
            "固定机位",
            "R",
            _points(1000),
            path=self.preset_path,
        )
        save_preset(
            "固定机位",
            "B",
            _points(1100),
            path=self.preset_path,
        )

        with self.assertRaises(CalibrationPresetError):
            save_preset(
                "固定机位",
                "R",
                _points(1200),
                path=self.preset_path,
            )

        save_preset(
            "固定机位",
            "R",
            _points(1200),
            path=self.preset_path,
            overwrite=True,
        )
        self.assertEqual(
            get_preset("固定机位", "R", self.preset_path)["map_points"][0][0],
            [1200.0, 1200.0],
        )
        self.assertEqual(
            get_preset("固定机位", "B", self.preset_path)["map_points"][0][0],
            [1100.0, 1100.0],
        )

    def test_rejects_incomplete_or_mismatched_layers_without_modifying_file(self):
        save_preset(
            "有效方案",
            "R",
            _points(1000),
            path=self.preset_path,
        )
        before = self.preset_path.read_bytes()
        incomplete = _points(0)
        incomplete[1][2] = None
        with self.assertRaises(CalibrationPresetError):
            save_preset(
                "无效方案",
                "R",
                incomplete,
                path=self.preset_path,
            )
        self.assertEqual(self.preset_path.read_bytes(), before)

        mismatched_layers = _points(1000)
        mismatched_layers[1].pop()
        with self.assertRaises(CalibrationPresetError):
            save_preset(
                "高度层点数不一致",
                "R",
                mismatched_layers,
                path=self.preset_path,
            )
        self.assertEqual(self.preset_path.read_bytes(), before)

    def test_delete_only_removes_selected_side_and_name(self):
        for side in ("R", "B"):
            save_preset(
                "同名方案",
                side,
                _points(1000),
                path=self.preset_path,
            )
        delete_preset("同名方案", "R", self.preset_path)
        self.assertEqual(
            [(preset["side"], preset["name"]) for preset in load_presets(self.preset_path)],
            [("B", "同名方案")],
        )

    def test_rejects_corrupt_file_instead_of_overwriting_it(self):
        corrupt_document = {"version": 1, "presets": [{"name": "broken"}]}
        self.preset_path.write_text(
            json.dumps(corrupt_document, ensure_ascii=False),
            encoding="utf-8",
        )
        before = self.preset_path.read_bytes()
        with self.assertRaises(CalibrationPresetError):
            save_preset(
                "新方案",
                "R",
                _points(1000),
                path=self.preset_path,
            )
        self.assertEqual(self.preset_path.read_bytes(), before)

    def test_reads_version_one_presets_using_map_points_only(self):
        legacy_document = {
            "version": 1,
            "presets": [
                {
                    "name": "旧方案",
                    "side": "R",
                    "points_per_layer": 4,
                    "image_points": _points(0),
                    "map_points": _points(1000),
                    "updated_at": "2026-07-30T00:00:00+00:00",
                }
            ],
        }
        self.preset_path.write_text(
            json.dumps(legacy_document, ensure_ascii=False),
            encoding="utf-8",
        )

        preset = get_preset("旧方案", "R", self.preset_path)

        self.assertNotIn("image_points", preset)
        self.assertEqual(preset["map_points"][0][0], [1000.0, 1000.0])


if __name__ == "__main__":
    unittest.main()
