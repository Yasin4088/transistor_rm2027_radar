import unittest

import numpy as np

from vehicle_color import (
    VehicleColorMemory,
    analyze_armor_light_color,
    detect_armor_light_color,
    detect_armor_light_color_with_confidence,
)


class ArmorLightColorTest(unittest.TestCase):
    @staticmethod
    def _armor_with_light_bars(color):
        armor = np.zeros((24, 48, 3), dtype=np.uint8)
        armor[3:21, 4:8] = color
        armor[3:21, 40:44] = color
        return armor

    def test_detects_clear_red_and_blue(self):
        red = self._armor_with_light_bars((0, 0, 255))
        blue = self._armor_with_light_bars((255, 0, 0))

        self.assertEqual(detect_armor_light_color(red), 'R')
        self.assertEqual(detect_armor_light_color(blue), 'B')

    def test_yellow_side_lights_are_treated_as_red(self):
        yellow = self._armor_with_light_bars((0, 255, 255))

        self.assertEqual(detect_armor_light_color(yellow), 'R')
        self.assertIsNone(detect_armor_light_color(yellow, yellow_as_red=False))

    def test_overexposed_or_balanced_color_is_ambiguous(self):
        white = np.full((8, 8, 3), 255, dtype=np.uint8)
        balanced = np.zeros((8, 8, 3), dtype=np.uint8)
        balanced[:, :4] = (0, 0, 255)
        balanced[:, 4:] = (255, 0, 0)

        self.assertIsNone(detect_armor_light_color(white))
        self.assertIsNone(detect_armor_light_color(balanced))

    def test_center_number_color_is_ignored(self):
        armor = np.zeros((10, 20, 3), dtype=np.uint8)
        armor[:, 6:14] = (0, 255, 255)

        self.assertIsNone(detect_armor_light_color(armor, min_pixels=4))

    def test_side_lights_win_over_opposite_center_number(self):
        armor = np.zeros((10, 20, 3), dtype=np.uint8)
        armor[:, :6] = (255, 0, 0)
        armor[:, 6:14] = (0, 0, 255)
        armor[:, 14:] = (255, 0, 0)

        self.assertEqual(detect_armor_light_color(armor, min_pixels=4), 'B')

        color, confidence = detect_armor_light_color_with_confidence(armor, min_pixels=4)
        self.assertEqual(color, 'B')
        self.assertGreater(confidence, 0.65)

    def test_single_light_or_wide_color_block_is_rejected(self):
        single_bar = np.zeros((24, 48, 3), dtype=np.uint8)
        single_bar[3:21, 4:8] = (0, 0, 255)
        wide_blocks = np.zeros((24, 48, 3), dtype=np.uint8)
        wide_blocks[4:20, 1:15] = (0, 255, 255)
        wide_blocks[4:20, 33:47] = (0, 255, 255)

        self.assertIsNone(detect_armor_light_color(single_bar))
        self.assertIsNone(detect_armor_light_color(wide_blocks))

    def test_single_light_bar_is_accepted_only_when_enabled(self):
        single_bar = np.zeros((24, 48, 3), dtype=np.uint8)
        single_bar[3:21, 4:8] = (0, 0, 255)

        disabled = analyze_armor_light_color(single_bar, allow_single_light_bar=False)
        enabled = analyze_armor_light_color(single_bar, allow_single_light_bar=True)
        pair = analyze_armor_light_color(self._armor_with_light_bars((0, 0, 255)))

        self.assertIsNone(disabled['color'])
        self.assertEqual(enabled['color'], 'R')
        self.assertEqual(enabled['red_bars'], 1)
        self.assertLess(enabled['confidence'], pair['confidence'])

    def test_wide_single_color_block_remains_rejected(self):
        wide_block = np.zeros((24, 48, 3), dtype=np.uint8)
        wide_block[4:20, 1:15] = (0, 255, 255)

        evidence = analyze_armor_light_color(wide_block, allow_single_light_bar=True)

        self.assertIsNone(evidence['color'])
        self.assertEqual(evidence['red_bars'], 0)

    def test_evidence_reports_a_valid_pair(self):
        evidence = analyze_armor_light_color(self._armor_with_light_bars((255, 0, 0)))

        self.assertEqual(evidence['color'], 'B')
        self.assertEqual(evidence['blue_bars'], 2)
        self.assertEqual(evidence['red_bars'], 0)

    def test_dim_light_bar_pair_is_detected(self):
        dim_red = self._armor_with_light_bars((0, 0, 65))

        self.assertIsNone(analyze_armor_light_color(dim_red, min_value=80)['color'])
        self.assertEqual(analyze_armor_light_color(dim_red, min_value=55)['color'], 'R')

    def test_tiny_symmetric_light_points_are_compact_pair_evidence(self):
        tiny = np.zeros((12, 14, 3), dtype=np.uint8)
        tiny[3:7, 1:5] = (255, 0, 0)
        tiny[3:7, 9:13] = (255, 0, 0)

        disabled = analyze_armor_light_color(tiny, allow_compact_light_pair=False)
        enabled = analyze_armor_light_color(tiny, allow_compact_light_pair=True)

        self.assertEqual(disabled['color'], 'B')
        self.assertEqual(disabled['kind'], 'pair')
        self.assertEqual(enabled['color'], 'B')
        self.assertEqual(enabled['blue_bars'], 2)
        self.assertEqual(enabled['kind'], 'compact_pair')

    def test_single_tiny_light_point_is_not_accepted_as_a_bar(self):
        tiny = np.zeros((12, 14, 3), dtype=np.uint8)
        tiny[3:7, 1:5] = (255, 0, 0)

        evidence = analyze_armor_light_color(
            tiny,
            allow_single_light_bar=True,
            allow_compact_light_pair=True,
        )

        self.assertIsNone(evidence['color'])


class VehicleColorMemoryTest(unittest.TestCase):
    def test_ambiguous_color_keeps_last_reliable_color_for_same_box(self):
        memory = VehicleColorMemory(max_center_distance=50, max_missed_frames=5, confirmation_count=1)
        first_box = (100, 100, 80, 50)
        moved_box = (105, 103, 82, 52)

        self.assertEqual(memory.resolve('R1', 'R', first_box, 1), ('R1', False))
        self.assertEqual(memory.resolve('B1', None, moved_box, 2), ('R1', True))

    def test_new_reliable_color_replaces_held_color(self):
        memory = VehicleColorMemory(
            max_center_distance=50,
            max_missed_frames=5,
            confirmation_count=1,
            switch_confirmation_count=1,
        )
        box = (100, 100, 80, 50)

        memory.resolve('R1', 'R', box, 1)
        self.assertEqual(memory.resolve('B1', 'B', box, 2), ('B1', False))
        self.assertEqual(memory.resolve('R1', None, box, 3), ('B1', True))

    def test_continuously_visible_vehicle_keeps_color_during_long_ambiguity(self):
        memory = VehicleColorMemory(max_center_distance=50, max_missed_frames=1, confirmation_count=1)
        box = (100, 100, 80, 50)

        memory.resolve('R1', 'R', box, 1)
        for frame in range(2, 20):
            self.assertEqual(memory.resolve('B1', None, box, frame), ('R1', True))

    def test_expired_track_does_not_leak_color_to_new_vehicle(self):
        memory = VehicleColorMemory(max_center_distance=50, max_missed_frames=5, confirmation_count=1)
        box = (100, 100, 80, 50)

        memory.resolve('R1', 'R', box, 1)
        self.assertEqual(memory.resolve('B1', None, box, 3), ('B1', False))

    def test_nearby_non_overlapping_box_does_not_inherit_color(self):
        memory = VehicleColorMemory(max_center_distance=120, min_box_iou=0.3, confirmation_count=1)
        original_box = (100, 100, 80, 50)
        other_box = (175, 100, 80, 50)

        memory.resolve('R1', 'R', original_box, 1)
        self.assertEqual(memory.resolve('B1', None, other_box, 2), ('B1', False))

    def test_different_robot_number_does_not_inherit_color(self):
        memory = VehicleColorMemory(max_center_distance=50, confirmation_count=1)
        box = (100, 100, 80, 50)

        memory.resolve('R1', 'R', box, 1)
        self.assertEqual(memory.resolve('B3', None, box, 2), ('B3', False))

    def test_single_opposite_color_does_not_replace_committed_color(self):
        memory = VehicleColorMemory(max_center_distance=50, confirmation_count=3)
        box = (100, 100, 80, 50)

        for frame in range(1, 4):
            memory.resolve('R1', 'R', box, frame, model_confidence=0.8, detected_confidence=0.9)

        self.assertEqual(
            memory.resolve('B1', 'B', box, 4, model_confidence=0.9, detected_confidence=0.9),
            ('R1', True),
        )

    def test_opposite_color_switches_only_after_consecutive_confirmation(self):
        memory = VehicleColorMemory(
            max_center_distance=50,
            confirmation_count=3,
            switch_confirmation_count=3,
        )
        box = (100, 100, 80, 50)

        for frame in range(1, 4):
            memory.resolve('R1', 'R', box, frame, model_confidence=0.8, detected_confidence=0.9)
        self.assertEqual(memory.resolve('B1', 'B', box, 4), ('R1', True))
        self.assertEqual(memory.resolve('B1', 'B', box, 5), ('R1', True))
        self.assertEqual(memory.resolve('B1', 'B', box, 6), ('B1', False))

    def test_interrupted_opposite_sequence_does_not_switch_color(self):
        memory = VehicleColorMemory(max_center_distance=50, confirmation_count=3)
        box = (100, 100, 80, 50)

        for frame in range(1, 4):
            memory.resolve('R1', 'R', box, frame)
        memory.resolve('B1', 'B', box, 4)
        memory.resolve('B1', None, box, 5)
        self.assertEqual(memory.resolve('B1', 'B', box, 6), ('R1', True))

    def test_new_track_uses_higher_confidence_source_until_confirmed(self):
        box = (100, 100, 80, 50)
        model_wins = VehicleColorMemory(confirmation_count=3)
        light_wins = VehicleColorMemory(confirmation_count=3)

        self.assertEqual(
            model_wins.resolve('R1', 'B', box, 1, model_confidence=0.9, detected_confidence=0.7),
            ('R1', False),
        )
        self.assertEqual(
            light_wins.resolve('R1', 'B', box, 1, model_confidence=0.6, detected_confidence=0.9),
            ('B1', False),
        )

    def test_unconfirmed_fallback_uses_current_frame_confidence(self):
        memory = VehicleColorMemory(confirmation_count=3)
        box = (100, 100, 80, 50)

        self.assertEqual(
            memory.resolve('R1', 'B', box, 1, model_confidence=0.9, detected_confidence=0.7),
            ('R1', False),
        )
        self.assertEqual(
            memory.resolve('B1', 'R', box, 2, model_confidence=0.8, detected_confidence=0.7),
            ('B1', False),
        )

    def test_model_light_disagreement_never_commits_color(self):
        memory = VehicleColorMemory(confirmation_count=3, require_model_agreement=True)
        box = (100, 100, 80, 50)

        for frame in range(1, 7):
            memory.resolve(
                'R1',
                'B',
                box,
                frame,
                model_confidence=0.7,
                detected_confidence=0.9,
            )

        self.assertIsNone(memory.tracks[0]['committed_color'])

    def test_provisional_color_reports_uncommitted_state(self):
        memory = VehicleColorMemory(confirmation_count=3)
        box = (100, 100, 80, 50)

        self.assertEqual(
            memory.resolve(
                'R1',
                'R',
                box,
                1,
                model_confidence=0.8,
                detected_confidence=0.9,
                return_state=True,
            ),
            ('R1', False, False),
        )
        memory.resolve('R1', 'R', box, 2)
        self.assertEqual(memory.resolve('R1', 'R', box, 3, return_state=True), ('R1', False, True))

    def test_single_bar_evidence_requires_more_consecutive_frames(self):
        memory = VehicleColorMemory(
            confirmation_count=3,
            single_bar_confirmation_count=5,
        )
        box = (100, 100, 80, 50)

        for frame in range(1, 5):
            result = memory.resolve(
                'R1',
                'R',
                box,
                frame,
                detected_bars=1,
                return_state=True,
            )
            self.assertFalse(result[2])

        self.assertEqual(
            memory.resolve('R1', 'R', box, 5, detected_bars=1, return_state=True),
            ('R1', False, True),
        )

    def test_single_bar_in_candidate_sequence_keeps_stricter_requirement(self):
        memory = VehicleColorMemory(
            confirmation_count=3,
            single_bar_confirmation_count=5,
        )
        box = (100, 100, 80, 50)

        memory.resolve('R1', 'R', box, 1, detected_bars=1)
        for frame in range(2, 5):
            self.assertFalse(
                memory.resolve('R1', 'R', box, frame, detected_bars=2, return_state=True)[2]
            )
        self.assertTrue(
            memory.resolve('R1', 'R', box, 5, detected_bars=2, return_state=True)[2]
        )

    def test_compact_pair_uses_stricter_confirmation_count(self):
        memory = VehicleColorMemory(
            confirmation_count=3,
            compact_pair_confirmation_count=5,
        )
        box = (100, 100, 80, 50)

        for frame in range(1, 5):
            self.assertFalse(
                memory.resolve(
                    'B2',
                    'B',
                    box,
                    frame,
                    detected_bars=2,
                    detected_kind='compact_pair',
                    return_state=True,
                )[2]
            )
        self.assertTrue(
            memory.resolve(
                'B2',
                'B',
                box,
                5,
                detected_bars=2,
                detected_kind='compact_pair',
                return_state=True,
            )[2]
        )

    def test_high_confidence_model_initializes_color_when_lamps_are_off(self):
        memory = VehicleColorMemory(
            model_only_confirmation_count=5,
            model_only_min_confidence=0.75,
        )
        box = (100, 100, 80, 50)

        for frame in range(1, 5):
            self.assertFalse(
                memory.resolve(
                    'R1',
                    None,
                    box,
                    frame,
                    model_confidence=0.86,
                    return_state=True,
                )[2]
            )
        self.assertTrue(
            memory.resolve(
                'R1',
                None,
                box,
                5,
                model_confidence=0.86,
                return_state=True,
            )[2]
        )

    def test_low_confidence_model_does_not_initialize_color(self):
        memory = VehicleColorMemory(
            model_only_confirmation_count=3,
            model_only_min_confidence=0.75,
        )
        box = (100, 100, 80, 50)

        for frame in range(1, 7):
            memory.resolve('R1', None, box, frame, model_confidence=0.7)

        self.assertIsNone(memory.tracks[0]['committed_color'])

    def test_model_only_fallback_never_replaces_committed_light_color(self):
        memory = VehicleColorMemory(
            confirmation_count=1,
            switch_confirmation_count=1,
            model_only_confirmation_count=2,
            model_only_min_confidence=0.75,
        )
        box = (100, 100, 80, 50)

        memory.resolve('R1', 'R', box, 1, model_confidence=0.9, detected_confidence=0.9)
        for frame in range(2, 7):
            memory.resolve('B1', None, box, frame, model_confidence=0.95)

        self.assertEqual(memory.tracks[0]['committed_color'], 'R')


if __name__ == '__main__':
    unittest.main()
