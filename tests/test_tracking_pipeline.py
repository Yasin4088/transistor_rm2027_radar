import unittest

import numpy as np

from detect.tracking.kalman import BoxKalman, WorldKalman
from detect.tracking.pipeline import TrackedRadarPipeline
from detect.tracking.roi_batch import make_letterboxed_roi, pack_tiles
from detect.tracking.tracker import IdentityManager, VehicleTrack, VehicleTracker
from detect.tracking.types import ArmorEvidence, CarDetection, TargetState, TrackState


class RoiBatchTests(unittest.TestCase):
    def test_letterbox_transform_round_trip(self):
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        _, transform = make_letterboxed_roi(image, 9, (100, 80, 300, 230))
        original = (130.0, 100.0, 220.0, 180.0)
        cx1, cy1, _, _ = transform.crop_box
        tile_box = (
            (original[0] - cx1) * transform.scale + transform.pad_x,
            (original[1] - cy1) * transform.scale + transform.pad_y,
            (original[2] - cx1) * transform.scale + transform.pad_x,
            (original[3] - cy1) * transform.scale + transform.pad_y,
        )
        recovered = transform.tile_box_to_image(tile_box)
        np.testing.assert_allclose(recovered, original, atol=1e-6)

    def test_static_fallback_splits_after_sixteen_tiles(self):
        image = np.zeros((400, 600, 3), dtype=np.uint8)
        pairs = [
            make_letterboxed_roi(image, index, (20, 20, 120, 100))
            for index in range(17)
        ]
        canvases, groups = pack_tiles(
            [pair[0] for pair in pairs], [pair[1] for pair in pairs]
        )
        self.assertEqual([len(group) for group in groups], [16, 1])
        self.assertEqual(canvases[0].shape, (1280, 1280, 3))


class TrackerTests(unittest.TestCase):
    def test_low_confidence_cannot_exceed_new_track_threshold(self):
        with self.assertRaises(ValueError):
            VehicleTracker(high_confidence=0.2, low_confidence=0.3)

    def test_low_confidence_second_pass_recovers_confirmed_track(self):
        tracker = VehicleTracker(confirm_hits=1)
        first = tracker.update([CarDetection((10, 10, 110, 80), 0.9)], 0.0)
        track_id = first[0].track_id
        second = tracker.update([CarDetection((14, 11, 114, 81), 0.15)], 0.1)
        self.assertEqual(second[0].track_id, track_id)
        self.assertTrue(second[0].observed)
        self.assertEqual(second[0].state, TrackState.CONFIRMED)

    def test_observed_track_exposes_current_detection_without_filter_lag(self):
        tracker = VehicleTracker(confirm_hits=1)
        tracker.update([CarDetection((10, 10, 110, 80), 0.9)], 0.0)
        current_box = (20, 12, 120, 82)
        track = tracker.update([CarDetection(current_box, 0.9)], 0.1)[0]
        self.assertEqual(track.measurement_box, current_box)
        self.assertNotEqual(track.box, current_box)

    def test_identity_assignment_is_unique_and_requires_committed_color(self):
        tracks = [
            VehicleTrack(1, BoxKalman((0, 0, 100, 80), 0), state=TrackState.CONFIRMED),
            VehicleTrack(2, BoxKalman((200, 0, 300, 80), 0), state=TrackState.CONFIRMED),
        ]
        manager = IdentityManager(["B1", "B2"], confirm_hits=2)
        manager.begin_frame([1, 2])
        manager.add_evidence(ArmorEvidence(1, "B1", 0.99, (0, 0, 1, 1)))
        self.assertEqual(manager.assign(tracks, {}), {})

        result = {}
        for _ in range(2):
            manager.begin_frame([1, 2])
            manager.add_evidence(
                ArmorEvidence(1, "B1", 0.9, (0, 0, 1, 1), color_committed=True)
            )
            manager.add_evidence(
                ArmorEvidence(2, "B2", 0.9, (0, 0, 1, 1), color_committed=True)
            )
            result = manager.assign(tracks, {1: (100, 100), 2: (300, 100)})
        self.assertEqual(result["B1"].track_id, 1)
        self.assertEqual(result["B2"].track_id, 2)

    def test_world_filter_uses_elapsed_time(self):
        kalman = WorldKalman()
        kalman.reset((100.0, 200.0), 0.0)
        kalman.x[2, 0] = 20.0
        kalman.x[3, 0] = -10.0
        predicted = kalman.predict(0.5)
        np.testing.assert_allclose(predicted, (110.0, 195.0), atol=1e-6)

    def test_confirmed_identity_correction_clears_old_slot(self):
        track = VehicleTrack(
            1, BoxKalman((0, 0, 100, 80), 0), state=TrackState.CONFIRMED
        )
        manager = IdentityManager(["B1", "B2"], confirm_hits=1, switch_hits=2)
        manager.add_evidence(
            ArmorEvidence(1, "B1", 1.0, (0, 0, 1, 1), color_committed=True)
        )
        self.assertIn("B1", manager.assign([track], {}))
        state = manager.state_for_track(1)
        state.scores = {"B1": 0.0, "B2": 5.0}
        manager.assign([track], {})
        manager.assign([track], {})
        self.assertEqual(manager.slots["B2"].track_id, 1)
        self.assertIsNone(manager.slots["B1"].track_id)


class _FakeCarDetector:
    names = ["car"]

    def __init__(self):
        self.detections = [("car", [80, 80, 160, 100], 0.9)]

    def predict(self, _image):
        return list(self.detections)


class _FakeStaticArmorDetector:
    names = ["B1"]
    supports_dynamic_batch = False

    def predict(self, _image):
        return [("B1", [110, 130, 80, 40], 0.95)]


class PipelineStateTests(unittest.TestCase):
    def test_camera_overlay_hides_unconfirmed_and_lost_tracks(self):
        car_detector = _FakeCarDetector()
        config = {
            "algorithm": {
                "vehicle_tracker": {
                    "high_confidence": 0.3,
                    "low_confidence": 0.1,
                    "confirm_hits": 2,
                },
                "identity": {"confirm_hits": 1},
                "armor_roi": {"size": 320, "fallback_canvas_size": 1280},
            },
            "vehicle_color_hold": {"enabled": False},
            "blind_zone": {"enabled": False},
        }
        pipeline = TrackedRadarPipeline(
            config,
            car_detector,
            _FakeStaticArmorDetector(),
            project_point=lambda _point: (200.0, 300.0),
            convert_map_point=lambda _point: ((400.0, 500.0), (400.0, 1000.0)),
            side="R",
        )
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        first = pipeline.process(frame, 0.0)
        self.assertFalse(first.annotated_image.any())

        confirmed = pipeline.process(frame, 0.1)
        self.assertTrue(confirmed.annotated_image.any())
        self.assertEqual(confirmed.diagnostics["visible_tracks"], 1)

        car_detector.detections = []
        lost = pipeline.process(frame, 0.2)
        self.assertFalse(lost.annotated_image.any())
        self.assertEqual(lost.diagnostics["visible_tracks"], 0)

    def test_measured_then_occlusion_then_blind_prediction(self):
        car_detector = _FakeCarDetector()
        config = {
            "algorithm": {
                "vehicle_tracker": {"confirm_hits": 1, "remove_seconds": 3.0},
                "identity": {"confirm_hits": 1, "switch_hits": 2},
                "lost_target": {"predict_seconds": 0.8},
                "armor_roi": {"size": 320, "fallback_canvas_size": 1280},
            },
            "vehicle_color_hold": {"enabled": False},
            "blind_zone": {
                "enabled": True,
                "base_time": 3.0,
                "offset_time": 0.0,
                "roles": [1, 2, 7],
                "points": {"B1": [[900, 700], [1200, 400]]},
            },
        }
        pipeline = TrackedRadarPipeline(
            config,
            car_detector,
            _FakeStaticArmorDetector(),
            project_point=lambda _point: (200.0, 300.0),
            convert_map_point=lambda _point: ((400.0, 500.0), (400.0, 1000.0)),
            side="R",
        )
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        measured = pipeline.process(frame, 0.0).targets["B1"]
        self.assertEqual(measured.state, TargetState.MEASURED)
        self.assertEqual(measured.position_cm, (400.0, 1000.0))

        car_detector.detections = []
        held = pipeline.process(frame, 0.2).targets["B1"]
        self.assertEqual(held.state, TargetState.OCCLUSION_HOLD)
        blind = pipeline.process(frame, 1.0).targets["B1"]
        self.assertEqual(blind.state, TargetState.BLIND_PREDICTION)
        self.assertIn(blind.position_cm, ((900.0, 700.0), (1200.0, 400.0)))


if __name__ == "__main__":
    unittest.main()
