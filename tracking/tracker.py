from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment

from .kalman import BoxKalman, WorldKalman
from .types import ArmorEvidence, Box, CarDetection, Point, TrackState


def box_iou(first: Box, second: Box) -> float:
    left = max(first[0], second[0])
    top = max(first[1], second[1])
    right = min(first[2], second[2])
    bottom = min(first[3], second[3])
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    first_area = max(0.0, first[2] - first[0]) * max(0.0, first[3] - first[1])
    second_area = max(0.0, second[2] - second[0]) * max(0.0, second[3] - second[1])
    union = first_area + second_area - intersection
    return intersection / union if union > 0 else 0.0


def _center_and_size(box: Box) -> Tuple[float, float, float, float]:
    x1, y1, x2, y2 = box
    return (
        (x1 + x2) * 0.5,
        (y1 + y2) * 0.5,
        max(x2 - x1, 2.0),
        max(y2 - y1, 2.0),
    )


@dataclass
class VehicleTrack:
    track_id: int
    filter: BoxKalman
    state: TrackState = TrackState.TENTATIVE
    hits: int = 1
    misses: int = 0
    last_seen_time: float = 0.0
    last_update_time: float = 0.0
    confidence: float = 0.0
    observed: bool = True
    world_measurement: Optional[Point] = None
    display_measurement: Optional[Point] = None
    last_armor_box: Optional[Box] = None
    last_detection_box: Optional[Box] = None

    @property
    def box(self) -> Box:
        return self.filter.box

    @property
    def measurement_box(self) -> Box:
        """Use the current detector box when observed, without Kalman display lag."""
        if self.observed and self.last_detection_box is not None:
            return self.last_detection_box
        return self.box


class VehicleTracker:
    """Two-pass, globally matched vehicle tracker inspired by ByteTrack."""

    def __init__(
        self,
        high_confidence: float = 0.30,
        low_confidence: float = 0.10,
        confirm_hits: int = 2,
        remove_seconds: float = 3.0,
    ):
        self.high_confidence = float(high_confidence)
        self.low_confidence = float(low_confidence)
        if self.low_confidence > self.high_confidence:
            raise ValueError("low_confidence cannot exceed high_confidence")
        self.confirm_hits = max(1, int(confirm_hits))
        self.remove_seconds = max(float(remove_seconds), 0.1)
        self.tracks: Dict[int, VehicleTrack] = {}
        self._next_track_id = 1

    @staticmethod
    def _pair_cost(track: VehicleTrack, detection: CarDetection) -> float:
        predicted = track.box
        iou = box_iou(predicted, detection.box)
        pcx, pcy, pw, ph = _center_and_size(predicted)
        dcx, dcy, dw, dh = _center_and_size(detection.box)
        diagonal = max(math.hypot(pw, ph), 10.0)
        center_distance = math.hypot(dcx - pcx, dcy - pcy) / diagonal
        size_change = min(abs(math.log(dw / pw)) + abs(math.log(dh / ph)), 2.0) / 2.0
        # Either overlap or a plausible motion prediction is required.
        if iou < 0.01 and center_distance > 1.5:
            return float("inf")
        return 0.55 * (1.0 - iou) + 0.30 * min(center_distance, 1.0) + 0.15 * size_change

    def _associate(
        self,
        tracks: Sequence[VehicleTrack],
        detections: Sequence[CarDetection],
        max_cost: float,
    ) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
        if not tracks or not detections:
            return [], list(range(len(tracks))), list(range(len(detections)))
        costs = np.full((len(tracks), len(detections)), 1e6, dtype=np.float64)
        for row, track in enumerate(tracks):
            for column, detection in enumerate(detections):
                cost = self._pair_cost(track, detection)
                if math.isfinite(cost):
                    costs[row, column] = cost
        rows, columns = linear_sum_assignment(costs)
        matches: List[Tuple[int, int]] = []
        for row, column in zip(rows.tolist(), columns.tolist()):
            if costs[row, column] <= max_cost:
                matches.append((row, column))
        matched_rows = {row for row, _ in matches}
        matched_columns = {column for _, column in matches}
        return (
            matches,
            [index for index in range(len(tracks)) if index not in matched_rows],
            [index for index in range(len(detections)) if index not in matched_columns],
        )

    def _apply_match(self, track: VehicleTrack, detection: CarDetection, timestamp: float) -> None:
        track.last_detection_box = detection.box
        track.filter.update(detection.box)
        track.last_seen_time = float(timestamp)
        track.last_update_time = float(timestamp)
        track.confidence = float(detection.confidence)
        track.observed = True
        track.misses = 0
        track.hits += 1
        if track.state in (TrackState.LOST, TrackState.CONFIRMED):
            track.state = TrackState.CONFIRMED
        elif track.hits >= self.confirm_hits:
            track.state = TrackState.CONFIRMED

    def _create_track(self, detection: CarDetection, timestamp: float) -> None:
        track_id = self._next_track_id
        self._next_track_id += 1
        state = TrackState.CONFIRMED if self.confirm_hits <= 1 else TrackState.TENTATIVE
        self.tracks[track_id] = VehicleTrack(
            track_id=track_id,
            filter=BoxKalman(detection.box, timestamp),
            state=state,
            hits=1,
            last_seen_time=float(timestamp),
            last_update_time=float(timestamp),
            confidence=float(detection.confidence),
            last_detection_box=detection.box,
        )

    def update(self, detections: Iterable[CarDetection], timestamp: float) -> List[VehicleTrack]:
        detections = [item for item in detections if item.confidence >= self.low_confidence]
        active_tracks = [
            item for item in self.tracks.values() if item.state != TrackState.REMOVED
        ]
        for track in active_tracks:
            track.filter.predict(timestamp)
            track.last_update_time = float(timestamp)
            track.observed = False
            track.world_measurement = None
            track.display_measurement = None

        high = [item for item in detections if item.confidence >= self.high_confidence]
        low = [item for item in detections if item.confidence < self.high_confidence]
        matches, unmatched_track_indexes, unmatched_high_indexes = self._associate(
            active_tracks, high, max_cost=0.85
        )
        for track_index, detection_index in matches:
            self._apply_match(active_tracks[track_index], high[detection_index], timestamp)

        remaining_tracks = [active_tracks[index] for index in unmatched_track_indexes]
        recoverable = [
            track for track in remaining_tracks if track.state in (TrackState.CONFIRMED, TrackState.LOST)
        ]
        low_matches, _, _ = self._associate(recoverable, low, max_cost=0.70)
        recovered_ids = set()
        for track_index, detection_index in low_matches:
            track = recoverable[track_index]
            recovered_ids.add(track.track_id)
            self._apply_match(track, low[detection_index], timestamp)

        unmatched_ids = {
            track.track_id for track in remaining_tracks if track.track_id not in recovered_ids
        }
        for track_id in unmatched_ids:
            track = self.tracks[track_id]
            track.misses += 1
            if track.state == TrackState.TENTATIVE:
                track.state = TrackState.REMOVED
            elif float(timestamp) - track.last_seen_time > self.remove_seconds:
                track.state = TrackState.REMOVED
            else:
                track.state = TrackState.LOST

        for detection_index in unmatched_high_indexes:
            self._create_track(high[detection_index], timestamp)

        return [
            track for track in self.tracks.values() if track.state != TrackState.REMOVED
        ]


@dataclass
class IdentityTrackState:
    scores: Dict[str, float]
    confirmed_label: Optional[str] = None
    candidate_label: Optional[str] = None
    candidate_count: int = 0
    confidence: float = 0.0

    def probabilities(self) -> Dict[str, float]:
        total = sum(max(value, 0.0) for value in self.scores.values())
        if total <= 0:
            return {name: 0.0 for name in self.scores}
        return {name: max(value, 0.0) / total for name, value in self.scores.items()}


@dataclass
class IdentitySlot:
    name: str
    world_filter: WorldKalman = field(default_factory=WorldKalman)
    track_id: Optional[int] = None
    last_measured_time: Optional[float] = None
    last_display_xy: Optional[Point] = None
    confidence: float = 0.0
    guess_index: Optional[int] = None
    guess_started_at: Optional[float] = None
    blind_last_progress: float = 0.0

    def clear(self) -> None:
        self.world_filter = WorldKalman()
        self.track_id = None
        self.last_measured_time = None
        self.last_display_xy = None
        self.confidence = 0.0
        self.guess_index = None
        self.guess_started_at = None
        self.blind_last_progress = 0.0


class IdentityManager:
    """Globally unique role assignment with temporally accumulated evidence."""

    def __init__(
        self,
        class_names: Sequence[str],
        decay: float = 0.85,
        min_confidence: float = 0.60,
        min_margin: float = 0.15,
        confirm_hits: int = 3,
        switch_hits: int = 5,
    ):
        self.class_names = tuple(str(name) for name in class_names)
        self.decay = min(max(float(decay), 0.0), 1.0)
        self.min_confidence = float(min_confidence)
        self.min_margin = float(min_margin)
        self.confirm_hits = max(1, int(confirm_hits))
        self.switch_hits = max(self.confirm_hits, int(switch_hits))
        self.track_states: Dict[int, IdentityTrackState] = {}
        self.slots = {name: IdentitySlot(name=name) for name in self.class_names}

    def _state(self, track_id: int) -> IdentityTrackState:
        if track_id not in self.track_states:
            self.track_states[track_id] = IdentityTrackState(
                scores={name: 0.0 for name in self.class_names}
            )
        return self.track_states[track_id]

    def begin_frame(self, track_ids: Iterable[int]) -> None:
        for track_id in track_ids:
            state = self._state(track_id)
            for name in state.scores:
                state.scores[name] *= self.decay

    def add_evidence(self, evidence: ArmorEvidence) -> None:
        if evidence.label not in self.class_names or not evidence.color_committed:
            return
        state = self._state(evidence.track_id)
        state.scores[evidence.label] += min(max(float(evidence.confidence), 0.0), 1.0)

    def _assignment_cost(
        self,
        track: VehicleTrack,
        class_name: str,
        world_position: Optional[Point],
    ) -> float:
        state = self._state(track.track_id)
        probability = max(state.probabilities().get(class_name, 0.0), 1e-6)
        cost = -math.log(probability)
        slot = self.slots[class_name]
        if state.confirmed_label == class_name:
            cost -= 0.8
        if slot.track_id == track.track_id:
            cost -= 0.6
        if world_position is not None and slot.world_filter.initialized:
            distance = np.linalg.norm(
                np.asarray(world_position, dtype=np.float64)
                - np.asarray(slot.world_filter.position, dtype=np.float64)
            )
            cost += min(float(distance) / 500.0, 1.0) * 0.5
        return cost

    def assign(
        self,
        tracks: Sequence[VehicleTrack],
        world_positions: Mapping[int, Point],
    ) -> Dict[str, VehicleTrack]:
        eligible = [track for track in tracks if track.state == TrackState.CONFIRMED]
        if not eligible:
            return {}
        # One dummy column per track lets uncertain tracks remain UNKNOWN.
        costs = np.full(
            (len(eligible), len(self.class_names) + len(eligible)),
            1.4,
            dtype=np.float64,
        )
        for row, track in enumerate(eligible):
            for column, name in enumerate(self.class_names):
                costs[row, column] = self._assignment_cost(
                    track, name, world_positions.get(track.track_id)
                )
        rows, columns = linear_sum_assignment(costs)
        proposed: Dict[int, str] = {}
        for row, column in zip(rows.tolist(), columns.tolist()):
            if column < len(self.class_names):
                proposed[eligible[row].track_id] = self.class_names[column]

        result: Dict[str, VehicleTrack] = {}
        tracks_by_id = {track.track_id: track for track in eligible}
        for track_id, class_name in proposed.items():
            state = self._state(track_id)
            probabilities = state.probabilities()
            ordered = sorted(probabilities.values(), reverse=True)
            probability = probabilities.get(class_name, 0.0)
            runner_up = ordered[1] if len(ordered) > 1 else 0.0
            valid_evidence = (
                probability >= self.min_confidence
                and probability - runner_up >= self.min_margin
            )
            if state.confirmed_label == class_name:
                state.candidate_label = None
                state.candidate_count = 0
            elif valid_evidence:
                if state.candidate_label == class_name:
                    state.candidate_count += 1
                else:
                    state.candidate_label = class_name
                    state.candidate_count = 1
                required = self.confirm_hits if state.confirmed_label is None else self.switch_hits
                if state.candidate_count >= required:
                    state.confirmed_label = class_name
                    state.candidate_label = None
                    state.candidate_count = 0
            if state.confirmed_label == class_name:
                state.confidence = probability
                result[class_name] = tracks_by_id[track_id]
                # If an explicitly confirmed correction moves one physical
                # track to another role, do not keep broadcasting the old role
                # from its stale slot.
                for other_name, other_slot in self.slots.items():
                    if other_name != class_name and other_slot.track_id == track_id:
                        other_slot.clear()
                slot = self.slots[class_name]
                slot.track_id = track_id
                slot.confidence = probability
        return result

    def state_for_track(self, track_id: int) -> IdentityTrackState:
        return self._state(track_id)
