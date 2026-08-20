from __future__ import annotations

import math
import re
from collections import defaultdict
from typing import Callable, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import cv2
import numpy as np

from vehicle_color import VehicleColorMemory, analyze_armor_light_color

from .roi_batch import find_transform_for_box, make_letterboxed_roi, pack_tiles
from .tracker import IdentityManager, VehicleTrack, VehicleTracker
from .types import (
    AlgorithmFrameResult,
    ArmorEvidence,
    Box,
    CarDetection,
    Point,
    TargetOutput,
    TargetState,
    TrackState,
)


ProjectPoint = Callable[[Point], Optional[Point]]
ConvertMapPoint = Callable[[Point], Tuple[Point, Point]]


def _xywh_to_box(xywh: Sequence[float]) -> Box:
    left, top, width, height = map(float, xywh)
    return left, top, left + width, top + height


def _box_to_xywh(box: Box) -> Tuple[float, float, float, float]:
    x1, y1, x2, y2 = box
    return x1, y1, x2 - x1, y2 - y1


def _class_names(names: object) -> List[str]:
    if isinstance(names, Mapping):
        values = [str(names[index]) for index in sorted(names)]
    else:
        values = [str(value) for value in names]  # type: ignore[arg-type]
    return [value for value in values if re.fullmatch(r"[RB][1-7]", value)]


def _clamp_box(box: Box, shape: Sequence[int]) -> Box:
    height, width = shape[:2]
    x1, y1, x2, y2 = box
    return (
        min(max(x1, 0.0), float(width)),
        min(max(y1, 0.0), float(height)),
        min(max(x2, 0.0), float(width)),
        min(max(y2, 0.0), float(height)),
    )


class TrackedRadarPipeline:
    """Vehicle-first radar pipeline with batched armor inference and role slots.

    The two-stage association and slot-based identity accumulation are adapted
    from HKUST Enterprize's RM2025 radar implementation (MIT, commit f12fe91),
    while retaining this project's YOLOv5 models, projection and radio schema.
    """

    def __init__(
        self,
        config: Mapping[str, object],
        car_detector: object,
        armor_detector: object,
        project_point: ProjectPoint,
        convert_map_point: ConvertMapPoint,
        side: str,
    ) -> None:
        self.config = config
        self.car_detector = car_detector
        self.armor_detector = armor_detector
        self.project_point = project_point
        self.convert_map_point = convert_map_point
        self.side = str(side).upper()
        algorithm = dict(config.get("algorithm", {}) or {})
        vehicle_cfg = dict(algorithm.get("vehicle_tracker", {}) or {})
        identity_cfg = dict(algorithm.get("identity", {}) or {})
        lost_cfg = dict(algorithm.get("lost_target", {}) or {})
        roi_cfg = dict(algorithm.get("armor_roi", {}) or {})

        self.roi_size = int(roi_cfg.get("size", 320))
        self.roi_margin = float(roi_cfg.get("margin_ratio", 0.08))
        self.tile_canvas_size = int(roi_cfg.get("fallback_canvas_size", 1280))
        self.max_armor_per_vehicle = max(1, int(roi_cfg.get("top_k", 2)))
        if bool(getattr(armor_detector, "supports_dynamic_batch", False)):
            detector_size = tuple(int(value) for value in getattr(armor_detector, "img_size", ()))
            if detector_size and detector_size != (self.roi_size, self.roi_size):
                raise ValueError(
                    "动态装甲板模型输入尺寸必须与 algorithm.armor_roi.size 一致"
                )
        elif self.tile_canvas_size % self.roi_size:
            raise ValueError(
                "algorithm.armor_roi.fallback_canvas_size 必须能被 ROI size 整除"
            )
        self.lost_predict_seconds = max(0.0, float(lost_cfg.get("predict_seconds", 0.8)))
        self.draw_tentative_tracks = bool(vehicle_cfg.get("draw_tentative", False))
        self.draw_lost_tracks = bool(vehicle_cfg.get("draw_lost", False))
        self.tracker = VehicleTracker(
            high_confidence=float(vehicle_cfg.get("high_confidence", 0.30)),
            low_confidence=float(vehicle_cfg.get("low_confidence", 0.10)),
            confirm_hits=int(vehicle_cfg.get("confirm_hits", 2)),
            remove_seconds=float(vehicle_cfg.get("remove_seconds", 3.0)),
        )
        names = _class_names(getattr(armor_detector, "names", ()))
        self.identity = IdentityManager(
            names,
            decay=float(identity_cfg.get("decay", 0.85)),
            min_confidence=float(identity_cfg.get("min_confidence", 0.60)),
            min_margin=float(identity_cfg.get("min_margin", 0.15)),
            confirm_hits=int(identity_cfg.get("confirm_hits", 3)),
            switch_hits=int(identity_cfg.get("switch_hits", 5)),
        )
        self.color_memories: Dict[int, VehicleColorMemory] = {}
        self.frame_index = 0
        self.blind_progress: Dict[str, float] = {}

    def set_blind_progress(self, progress: Mapping[str, float]) -> None:
        self.blind_progress = {
            str(name): float(value) for name, value in progress.items()
        }

    def _new_color_memory(self) -> VehicleColorMemory:
        values = dict(self.config.get("vehicle_color_hold", {}) or {})
        return VehicleColorMemory(
            max_center_distance=values.get("max_center_distance", 120.0),
            max_size_change_ratio=values.get("max_size_change_ratio", 0.5),
            min_box_iou=values.get("min_box_iou", 0.3),
            max_missed_frames=max(int(values.get("max_missed_frames", 1)), 2),
            confirmation_count=values.get("confirmation_count", 3),
            switch_confirmation_count=values.get("switch_confirmation_count", 5),
            single_bar_confirmation_count=values.get("single_bar_confirmation_count", 5),
            compact_pair_confirmation_count=values.get("compact_pair_confirmation_count", 5),
            model_only_confirmation_count=values.get("model_only_confirmation_count", 5),
            model_only_min_confidence=values.get("model_only_min_confidence", 0.75),
            model_only_max_conflicting_light_confidence=values.get(
                "model_only_max_conflicting_light_confidence", 0.4
            ),
            require_model_agreement=values.get("require_model_agreement", True),
        )

    def _light_color_kwargs(self) -> Dict[str, object]:
        values = dict(self.config.get("vehicle_color_hold", {}) or {})
        keys = (
            "min_saturation", "min_value", "min_pixels", "dominance_ratio",
            "side_strip_ratio", "yellow_as_red", "yellow_hue_min", "yellow_hue_max",
            "require_light_bar_pair", "bar_min_aspect_ratio", "bar_min_height_ratio",
            "bar_min_height_similarity", "bar_max_y_offset_ratio",
            "bar_max_angle_difference", "allow_single_light_bar",
            "single_bar_confidence_scale", "allow_compact_light_pair",
            "compact_pair_max_armor_height", "compact_pair_min_aspect_ratio",
            "compact_pair_confidence_scale",
        )
        return {key: values[key] for key in keys if key in values}

    def _detect_cars(self, frame: np.ndarray) -> List[CarDetection]:
        detections: List[CarDetection] = []
        for class_name, xywh, confidence in self.car_detector.predict(frame):
            if str(class_name).lower() == "car":
                detections.append(CarDetection(_xywh_to_box(xywh), float(confidence)))
        return detections

    def _detect_armor(
        self,
        frame: np.ndarray,
        tracks: Sequence[VehicleTrack],
    ) -> Dict[int, List[Tuple[str, Box, float]]]:
        tiles, transforms = [], []
        for track in tracks:
            if not track.observed:
                continue
            try:
                tile, transform = make_letterboxed_roi(
                    frame,
                    track.track_id,
                    track.measurement_box,
                    self.roi_size,
                    self.roi_margin,
                )
            except ValueError:
                continue
            tiles.append(tile)
            transforms.append(transform)
        grouped: Dict[int, List[Tuple[str, Box, float]]] = defaultdict(list)
        if not tiles:
            return grouped

        if bool(getattr(self.armor_detector, "supports_dynamic_batch", False)):
            results = self.armor_detector.predict_batch(tiles)
            for transform, detections in zip(transforms, results):
                for name, xywh, confidence in detections:
                    image_box = _clamp_box(
                        transform.tile_box_to_image(_xywh_to_box(xywh)), frame.shape
                    )
                    grouped[transform.track_id].append((str(name), image_box, float(confidence)))
        else:
            canvases, transform_groups = pack_tiles(
                tiles, transforms, self.roi_size, self.tile_canvas_size
            )
            for canvas, canvas_transforms in zip(canvases, transform_groups):
                for name, xywh, confidence in self.armor_detector.predict(canvas):
                    tile_box = _xywh_to_box(xywh)
                    transform = find_transform_for_box(
                        tile_box, canvas_transforms, self.roi_size
                    )
                    if transform is None:
                        continue
                    image_box = _clamp_box(transform.tile_box_to_image(tile_box), frame.shape)
                    grouped[transform.track_id].append((str(name), image_box, float(confidence)))

        for track_id in list(grouped):
            grouped[track_id] = sorted(
                grouped[track_id], key=lambda item: item[2], reverse=True
            )[:self.max_armor_per_vehicle]
        return grouped

    def _armor_evidence(
        self,
        frame: np.ndarray,
        track: VehicleTrack,
        detections: Iterable[Tuple[str, Box, float]],
    ) -> List[ArmorEvidence]:
        evidence_items: List[ArmorEvidence] = []
        hold_enabled = bool(
            dict(self.config.get("vehicle_color_hold", {}) or {}).get("enabled", True)
        )
        memory = self.color_memories.setdefault(track.track_id, self._new_color_memory())
        for model_label, box, confidence in detections:
            x1, y1, x2, y2 = map(int, box)
            crop = frame[max(y1, 0):max(y2, 0), max(x1, 0):max(x2, 0)]
            light = analyze_armor_light_color(crop, **self._light_color_kwargs())
            if hold_enabled:
                resolved, _, committed = memory.resolve(
                    model_label,
                    light.get("color"),
                    _box_to_xywh(track.measurement_box),
                    self.frame_index,
                    model_confidence=confidence,
                    detected_confidence=light.get("confidence", 0.0),
                    detected_bars=max(light.get("red_bars", 0), light.get("blue_bars", 0)),
                    detected_kind=light.get("kind"),
                    return_state=True,
                )
            else:
                resolved, committed = model_label, True
            item = ArmorEvidence(
                track_id=track.track_id,
                label=str(resolved),
                confidence=float(confidence),
                box=box,
                light_color=light.get("color"),
                light_confidence=float(light.get("confidence", 0.0)),
                light_kind=light.get("kind"),
                light_bars=max(light.get("red_bars", 0), light.get("blue_bars", 0)),
                color_committed=bool(committed),
            )
            evidence_items.append(item)
        if evidence_items:
            track.last_armor_box = max(evidence_items, key=lambda item: item.confidence).box
        return evidence_items

    def _update_measurement(self, track: VehicleTrack) -> None:
        x1, y1, x2, y2 = track.measurement_box
        projected = self.project_point(((x1 + x2) * 0.5, y1 + (y2 - y1) * 0.92))
        if projected is None:
            return
        display, referee = self.convert_map_point(projected)
        track.display_measurement = display
        track.world_measurement = referee

    def _blind_point(self, name: str, timestamp: float) -> Optional[Point]:
        blind = dict(self.config.get("blind_zone", {}) or {})
        if not blind.get("enabled", True):
            return None
        roles = {int(value) for value in blind.get("roles", [1, 2, 7])}
        match = re.fullmatch(r"([RB])(\d+)", name)
        if match is None or match.group(1) == self.side or int(match.group(2)) not in roles:
            return None
        points = list(dict(blind.get("points", {}) or {}).get(name, ()))
        if not points:
            return None
        slot = self.identity.slots[name]
        progress = float(self.blind_progress.get(name, 0.0))
        if progress < slot.blind_last_progress:
            slot.guess_index = None
            slot.guess_started_at = None
            slot.blind_last_progress = progress
        if progress > slot.blind_last_progress:
            slot.guess_started_at = timestamp
            slot.blind_last_progress = progress
        base_time = max(
            0.1,
            float(blind.get("base_time", 3.0)) + float(blind.get("offset_time", 0.0)),
        )
        if slot.guess_index is not None and slot.guess_started_at is not None:
            if timestamp - slot.guess_started_at < base_time:
                return tuple(map(float, points[slot.guess_index]))  # type: ignore[return-value]

        position = np.asarray(slot.world_filter.position, dtype=np.float64)
        velocity = np.asarray(slot.world_filter.velocity, dtype=np.float64)
        speed = float(np.linalg.norm(velocity))
        scored = []
        for index, point in enumerate(points):
            delta = np.asarray(point, dtype=np.float64) - position
            distance = float(np.linalg.norm(delta))
            direction = 0.0
            if speed > 1.0 and distance > 1.0:
                direction = float(np.dot(velocity, delta) / (speed * distance))
            score = 0.6 * math.exp(-distance / 400.0) + 0.4 * (direction + 1.0) * 0.5
            if index == slot.guess_index and len(points) > 1:
                score -= 0.25
            scored.append((score, index))
        slot.guess_index = max(scored)[1]
        slot.guess_started_at = timestamp
        slot.blind_last_progress = progress
        return tuple(map(float, points[slot.guess_index]))  # type: ignore[return-value]

    @staticmethod
    def _display_from_referee(point: Point) -> Point:
        return float(point[0]), 1500.0 - float(point[1])

    def _build_targets(
        self,
        assignments: Mapping[str, VehicleTrack],
        timestamp: float,
    ) -> Dict[str, TargetOutput]:
        targets: Dict[str, TargetOutput] = {}
        for name, slot in self.identity.slots.items():
            track = assignments.get(name)
            measured = track is not None and track.observed and track.world_measurement is not None
            if measured:
                position = slot.world_filter.update(track.world_measurement, timestamp)
                slot.last_measured_time = timestamp
                slot.last_display_xy = track.display_measurement
                slot.guess_index = None
                slot.guess_started_at = None
                slot.blind_last_progress = float(self.blind_progress.get(name, 0.0))
                state = TargetState.MEASURED
                display = track.display_measurement
            elif slot.world_filter.initialized and slot.last_measured_time is not None:
                predicted = slot.world_filter.predict(timestamp)
                age = timestamp - slot.last_measured_time
                if age <= self.lost_predict_seconds:
                    position = predicted
                    state = TargetState.OCCLUSION_HOLD
                    display = self._display_from_referee(position) if position else None
                else:
                    position = self._blind_point(name, timestamp)
                    state = TargetState.BLIND_PREDICTION if position is not None else TargetState.MISSING
                    display = self._display_from_referee(position) if position else None
            else:
                position = None
                display = None
                state = TargetState.MISSING
            targets[name] = TargetOutput(
                name=name,
                state=state,
                position_cm=position,
                display_xy=display,
                confidence=slot.confidence,
                track_id=slot.track_id,
                car_box=track.measurement_box if track is not None else None,
                armor_box=track.last_armor_box if track is not None else None,
            )
        return targets

    def _annotate(
        self,
        frame: np.ndarray,
        tracks: Sequence[VehicleTrack],
        assignments: Mapping[str, VehicleTrack],
    ) -> np.ndarray:
        output = frame.copy()
        names_by_track = {track.track_id: name for name, track in assignments.items()}
        for track in tracks:
            if track.state == TrackState.TENTATIVE and not self.draw_tentative_tracks:
                continue
            if track.state == TrackState.LOST and not self.draw_lost_tracks:
                continue
            x1, y1, x2, y2 = map(int, track.measurement_box)
            color = (30, 210, 30) if track.observed else (0, 170, 255)
            cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)
            identity = names_by_track.get(track.track_id, "UNKNOWN")
            text = (
                f"T{track.track_id} {identity} {track.state.value} "
                f"{track.confidence:.2f}"
            )
            cv2.putText(output, text, (x1, max(18, y1 - 7)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            if track.last_armor_box is not None and track.observed:
                ax1, ay1, ax2, ay2 = map(int, track.last_armor_box)
                cv2.rectangle(output, (ax1, ay1), (ax2, ay2), (255, 220, 20), 2)
        return output

    def process(self, frame: np.ndarray, timestamp: float) -> AlgorithmFrameResult:
        self.frame_index += 1
        car_detections = self._detect_cars(frame)
        tracks = self.tracker.update(car_detections, timestamp)
        for track in tracks:
            if track.observed:
                self._update_measurement(track)

        armor_by_track = self._detect_armor(frame, tracks)
        confirmed_ids = [
            track.track_id for track in tracks if track.state == TrackState.CONFIRMED
        ]
        self.identity.begin_frame(confirmed_ids)
        armor_count = 0
        for track in tracks:
            if not track.observed:
                continue
            for evidence in self._armor_evidence(
                frame, track, armor_by_track.get(track.track_id, ())
            ):
                self.identity.add_evidence(evidence)
                armor_count += 1

        world_positions = {
            track.track_id: track.world_measurement
            for track in tracks
            if track.world_measurement is not None
        }
        assignments = self.identity.assign(tracks, world_positions)
        targets = self._build_targets(assignments, timestamp)
        annotated = self._annotate(frame, tracks, assignments)
        visible_tracks = sum(
            track.state == TrackState.CONFIRMED and track.observed for track in tracks
        )
        return AlgorithmFrameResult(
            targets=targets,
            annotated_image=annotated,
            diagnostics={
                "vehicle_detections": len(car_detections),
                "active_tracks": len(tracks),
                "visible_tracks": visible_tracks,
                "armor_detections": armor_count,
                "dynamic_armor_batch": bool(
                    getattr(self.armor_detector, "supports_dynamic_batch", False)
                ),
            },
        )
