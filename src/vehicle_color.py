import re

import cv2
import numpy as np


def _find_light_bar_pair(
        color_mask,
        min_pixels,
        min_aspect_ratio,
        min_height_ratio,
        min_height_similarity,
        max_y_offset_ratio,
        max_angle_difference,
        side_region_ratio,
        allow_single_light_bar=False,
        single_bar_confidence_scale=0.55,
        allow_compact_light_pair=True,
        compact_pair_max_armor_height=16,
        compact_pair_min_aspect_ratio=1.0,
        compact_pair_confidence_scale=0.7):
    height, width = color_mask.shape
    side_ratio = min(max(float(side_region_ratio), 0.1), 0.49)
    side_only_mask = np.zeros_like(color_mask)
    left_end = max(1, int(np.ceil(width * side_ratio)))
    right_start = min(width - 1, int(np.floor(width * (1.0 - side_ratio))))
    side_only_mask[:, :left_end] = color_mask[:, :left_end]
    side_only_mask[:, right_start:] = color_mask[:, right_start:]
    vertical_kernel = np.ones((3, 1), dtype=np.uint8)
    contour_mask = cv2.morphologyEx(side_only_mask, cv2.MORPH_CLOSE, vertical_kernel)
    contours, _ = cv2.findContours(contour_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    left_bars = []
    right_bars = []
    per_bar_pixels = max(1, int(np.ceil(min_pixels / 2.0)))

    for contour in contours:
        component_mask = np.zeros_like(color_mask)
        cv2.drawContours(component_mask, [contour], -1, 255, thickness=-1)
        pixels = int(np.count_nonzero((component_mask > 0) & (side_only_mask > 0)))
        if pixels < per_bar_pixels:
            continue

        (center_x, center_y), (rect_width, rect_height), rect_angle = cv2.minAreaRect(contour)
        long_side = max(float(rect_width), float(rect_height))
        short_side = max(min(float(rect_width), float(rect_height)), 1.0)
        if long_side < height * float(min_height_ratio):
            continue
        aspect_ratio = long_side / short_side
        standard_bar = aspect_ratio >= float(min_aspect_ratio)
        compact_bar = (
            bool(allow_compact_light_pair)
            and height <= max(1, int(compact_pair_max_armor_height))
            and aspect_ratio >= max(1.0, float(compact_pair_min_aspect_ratio))
        )
        if not standard_bar and not compact_bar:
            continue

        long_axis_angle = float(rect_angle) if rect_width >= rect_height else float(rect_angle) + 90.0
        long_axis_angle %= 180.0
        bar = {
            'contour': contour,
            'center': (float(center_x), float(center_y)),
            'length': long_side,
            'angle': long_axis_angle,
            'pixels': pixels,
            'single_eligible': standard_bar,
        }
        if center_x <= width * side_ratio:
            left_bars.append(bar)
        elif center_x >= width * (1.0 - side_ratio):
            right_bars.append(bar)

    best_pair = None
    best_score = -1.0
    for left_bar in left_bars:
        for right_bar in right_bars:
            length_max = max(left_bar['length'], right_bar['length'])
            length_min = min(left_bar['length'], right_bar['length'])
            height_similarity = length_min / length_max if length_max > 0 else 0.0
            if height_similarity < float(min_height_similarity):
                continue
            y_offset_ratio = abs(left_bar['center'][1] - right_bar['center'][1]) / max(length_max, 1.0)
            if y_offset_ratio > float(max_y_offset_ratio):
                continue
            compact_pair_candidate = (
                (bool(allow_compact_light_pair) and height <= max(1, int(compact_pair_max_armor_height)))
                or not (left_bar['single_eligible'] and right_bar['single_eligible'])
            )
            angle_difference = abs(left_bar['angle'] - right_bar['angle'])
            angle_difference = min(angle_difference, 180.0 - angle_difference)
            if not compact_pair_candidate and angle_difference > float(max_angle_difference):
                continue
            angle_score = (
                1.0
                if compact_pair_candidate
                else max(0.0, 1.0 - angle_difference / max(float(max_angle_difference), 1.0))
            )
            score = height_similarity * max(0.0, 1.0 - y_offset_ratio) * (0.5 + 0.5 * angle_score)
            if score > best_score:
                best_score = score
                best_pair = (left_bar, right_bar)

    if best_pair is None:
        if not allow_single_light_bar:
            return None
        compact_size = (
            bool(allow_compact_light_pair)
            and height <= max(1, int(compact_pair_max_armor_height))
        )
        candidates = [
            bar for bar in left_bars + right_bars
            if bar['single_eligible'] and not compact_size
        ]
        if not candidates:
            return None
        best_bar = max(candidates, key=lambda bar: (bar['pixels'], bar['length']))
        confidence_scale = min(max(float(single_bar_confidence_scale), 0.0), 1.0)
        return {
            'pixels': best_bar['pixels'],
            'geometry_score': confidence_scale,
            'bars': 1,
        }

    pair_mask = np.zeros_like(color_mask)
    for bar in best_pair:
        cv2.drawContours(pair_mask, [bar['contour']], -1, 255, thickness=-1)
    pixels = int(np.count_nonzero((pair_mask > 0) & (side_only_mask > 0)))
    compact_pair = (
        (bool(allow_compact_light_pair) and height <= max(1, int(compact_pair_max_armor_height)))
        or not all(bar['single_eligible'] for bar in best_pair)
    )
    if compact_pair:
        best_score *= min(max(float(compact_pair_confidence_scale), 0.0), 1.0)
    return {
        'pixels': pixels,
        'geometry_score': best_score,
        'bars': 2,
        'kind': 'compact_pair' if compact_pair else 'pair',
    }


def analyze_armor_light_color(
        armor_image,
        min_saturation=70,
        min_value=55,
        min_pixels=4,
        dominance_ratio=1.5,
        side_strip_ratio=0.28,
        yellow_as_red=True,
        yellow_hue_min=16,
        yellow_hue_max=40,
        require_light_bar_pair=True,
        bar_min_aspect_ratio=1.3,
        bar_min_height_ratio=0.12,
        bar_min_height_similarity=0.4,
        bar_max_y_offset_ratio=0.8,
        bar_max_angle_difference=35.0,
        allow_single_light_bar=False,
        single_bar_confidence_scale=0.55,
        allow_compact_light_pair=True,
        compact_pair_max_armor_height=16,
        compact_pair_min_aspect_ratio=1.0,
        compact_pair_confidence_scale=0.7):
    """Return color evidence from valid paired or explicitly allowed single light bars."""
    empty = {
        'color': None,
        'confidence': 0.0,
        'red_pixels': 0,
        'blue_pixels': 0,
        'red_bars': 0,
        'blue_bars': 0,
        'kind': None,
    }
    if armor_image is None or armor_image.size == 0:
        return empty

    hsv = cv2.cvtColor(armor_image, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    visible = (saturation >= int(min_saturation)) & (value >= int(min_value))
    red_hue = (hue <= 15) | (hue >= 165)
    if yellow_as_red:
        yellow_min = min(max(int(yellow_hue_min), 0), 179)
        yellow_max = min(max(int(yellow_hue_max), 0), 179)
        if yellow_min > yellow_max:
            yellow_min, yellow_max = yellow_max, yellow_min
        red_hue |= (hue >= yellow_min) & (hue <= yellow_max)
    red_mask = np.where(visible & red_hue, 255, 0).astype(np.uint8)
    blue_mask = np.where(visible & (hue >= 90) & (hue <= 140), 255, 0).astype(np.uint8)

    required = max(1, int(min_pixels))
    red_pair = None
    blue_pair = None
    pair_args = (
        required,
        bar_min_aspect_ratio,
        bar_min_height_ratio,
        bar_min_height_similarity,
        bar_max_y_offset_ratio,
        bar_max_angle_difference,
        side_strip_ratio,
        allow_single_light_bar,
        single_bar_confidence_scale,
        allow_compact_light_pair,
        compact_pair_max_armor_height,
        compact_pair_min_aspect_ratio,
        compact_pair_confidence_scale,
    )
    if require_light_bar_pair:
        red_pair = _find_light_bar_pair(red_mask, *pair_args)
        blue_pair = _find_light_bar_pair(blue_mask, *pair_args)
        red_count = red_pair['pixels'] if red_pair else 0
        blue_count = blue_pair['pixels'] if blue_pair else 0
        red_geometry = red_pair['geometry_score'] if red_pair else 0.0
        blue_geometry = blue_pair['geometry_score'] if blue_pair else 0.0
        red_bars = red_pair['bars'] if red_pair else 0
        blue_bars = blue_pair['bars'] if blue_pair else 0
    else:
        red_count = int(np.count_nonzero(red_mask))
        blue_count = int(np.count_nonzero(blue_mask))
        red_geometry = blue_geometry = 1.0
        red_bars = blue_bars = 0

    evidence = dict(empty)
    evidence.update({
        'red_pixels': red_count,
        'blue_pixels': blue_count,
        'red_bars': red_bars,
        'blue_bars': blue_bars,
    })
    ratio = max(1.0, float(dominance_ratio))
    if red_count < required and blue_count < required:
        return evidence
    if red_count >= required and red_count >= blue_count * ratio:
        color, dominant, geometry_score, selected = 'R', red_count, red_geometry, red_pair
    elif blue_count >= required and blue_count >= red_count * ratio:
        color, dominant, geometry_score, selected = 'B', blue_count, blue_geometry, blue_pair
    else:
        return evidence

    total = red_count + blue_count
    purity = dominant / total if total > 0 else 0.0
    support = min(1.0, dominant / float(required * 3))
    confidence = purity * (0.5 + 0.5 * support) * geometry_score
    if require_light_bar_pair:
        kind = selected.get('kind', 'single') if selected else None
    else:
        kind = 'pixels'
    evidence.update({'color': color, 'confidence': confidence, 'kind': kind})
    return evidence


def detect_armor_light_color_with_confidence(armor_image, **kwargs):
    evidence = analyze_armor_light_color(armor_image, **kwargs)
    return evidence['color'], evidence['confidence']


def detect_armor_light_color(
        armor_image, **kwargs):
    color, _ = detect_armor_light_color_with_confidence(armor_image, **kwargs)
    return color


class VehicleColorMemory:
    """Remember the latest reliable camp color for spatially matched vehicles."""

    def __init__(self,
                 max_center_distance=120.0,
                 max_size_change_ratio=0.5,
                 min_box_iou=0.3,
                 max_missed_frames=1,
                 confirmation_count=3,
                 switch_confirmation_count=5,
                 single_bar_confirmation_count=5,
                 compact_pair_confirmation_count=5,
                 model_only_confirmation_count=5,
                 model_only_min_confidence=0.75,
                 model_only_max_conflicting_light_confidence=0.4,
                 require_model_agreement=True):
        self.max_center_distance = float(max_center_distance)
        self.max_size_change_ratio = float(max_size_change_ratio)
        self.min_box_iou = min(max(float(min_box_iou), 0.0), 1.0)
        self.max_missed_frames = max(0, int(max_missed_frames))
        self.confirmation_count = max(1, int(confirmation_count))
        self.switch_confirmation_count = max(1, int(switch_confirmation_count))
        self.single_bar_confirmation_count = max(1, int(single_bar_confirmation_count))
        self.compact_pair_confirmation_count = max(1, int(compact_pair_confirmation_count))
        self.model_only_confirmation_count = max(1, int(model_only_confirmation_count))
        self.model_only_min_confidence = self._normalized_confidence(model_only_min_confidence)
        self.model_only_max_conflicting_light_confidence = self._normalized_confidence(
            model_only_max_conflicting_light_confidence
        )
        self.require_model_agreement = bool(require_model_agreement)
        self.tracks = []

    @staticmethod
    def _box_signature(car_box):
        left, top, width, height = car_box
        return {
            'center': (float(left + width * 0.5), float(top + height * 0.5)),
            'size': (float(width), float(height)),
            'box': (float(left), float(top), float(left + width), float(top + height)),
        }

    @staticmethod
    def _box_iou(first_box, second_box):
        left = max(first_box[0], second_box[0])
        top = max(first_box[1], second_box[1])
        right = min(first_box[2], second_box[2])
        bottom = min(first_box[3], second_box[3])
        intersection = max(0.0, right - left) * max(0.0, bottom - top)
        first_area = max(0.0, first_box[2] - first_box[0]) * max(0.0, first_box[3] - first_box[1])
        second_area = max(0.0, second_box[2] - second_box[0]) * max(0.0, second_box[3] - second_box[1])
        union = first_area + second_area - intersection
        return intersection / union if union > 0 else 0.0

    @staticmethod
    def _label_number(label):
        match = re.search(r'(\d+)$', str(label))
        return match.group(1) if match else None

    @staticmethod
    def _label_color(label):
        color = str(label).upper()[:1]
        return color if color in ('R', 'B') else None

    def _find_track(self, signature, number, current_frame):
        self.tracks[:] = [
            track for track in self.tracks
            if current_frame - track['last_seen_frame'] <= self.max_missed_frames
        ]
        max_dist2 = self.max_center_distance * self.max_center_distance
        best_track = None
        best_iou = -1.0
        best_dist2 = max_dist2
        for track in self.tracks:
            if track.get('updated_frame') == current_frame:
                continue
            if track['number'] != number:
                continue
            # Color can only follow a continuously tracked original box. A
            # disappeared track is never eligible for a later vehicle.
            if current_frame - track['last_seen_frame'] != 1:
                continue
            old_width, old_height = track['size']
            width, height = signature['size']
            if old_width <= 1 or old_height <= 1:
                continue
            if abs(width - old_width) / old_width > self.max_size_change_ratio:
                continue
            if abs(height - old_height) / old_height > self.max_size_change_ratio:
                continue
            dx = signature['center'][0] - track['center'][0]
            dy = signature['center'][1] - track['center'][1]
            dist2 = dx * dx + dy * dy
            if dist2 > max_dist2:
                continue
            iou = self._box_iou(signature['box'], track['box'])
            if iou < self.min_box_iou:
                continue
            if best_track is None or iou > best_iou or (iou == best_iou and dist2 < best_dist2):
                best_track = track
                best_iou = iou
                best_dist2 = dist2
        return best_track

    @staticmethod
    def _normalized_confidence(value):
        return min(max(float(value or 0.0), 0.0), 1.0)

    def _update_fallback(self, track, color, confidence):
        if color not in ('R', 'B'):
            return
        confidence = self._normalized_confidence(confidence)
        if confidence > track['fallback_confidence']:
            track['fallback_color'] = color
            track['fallback_confidence'] = confidence

    def resolve(self,
                model_label,
                detected_color,
                car_box,
                current_frame,
                model_confidence=0.0,
                detected_confidence=0.0,
                detected_bars=2,
                detected_kind=None,
                return_state=False):
        """Return (resolved label, used_hold) for the current vehicle."""
        label = str(model_label)
        number = self._label_number(label)
        if number is None:
            return (label, False, False) if return_state else (label, False)

        signature = self._box_signature(car_box)
        track = self._find_track(signature, number, current_frame)
        raw_light_color = str(detected_color).upper() if detected_color is not None else None
        if raw_light_color not in ('R', 'B'):
            raw_light_color = None
        model_color = self._label_color(label)
        if track is None:
            track = {
                'number': number,
                'committed_color': None,
                'candidate_color': None,
                'candidate_count': 0,
                'candidate_required_count': 0,
                'fallback_color': model_color,
                'fallback_confidence': self._normalized_confidence(model_confidence),
            }
            self.tracks.append(track)

        light_confidence = self._normalized_confidence(detected_confidence)
        model_confidence = self._normalized_confidence(model_confidence)
        evidence_kind = str(detected_kind or '').lower()
        if not evidence_kind:
            evidence_kind = 'single' if int(detected_bars or 0) == 1 else 'pair'

        reliable_color = raw_light_color
        if (
                self.require_model_agreement
                and reliable_color is not None
                and model_color is not None
                and reliable_color != model_color):
            reliable_color = None

        # When lamps are off, a stable high-confidence model result is the
        # only available camp evidence. It may initialize a track, but never
        # replaces a previously committed lamp color.
        weak_or_missing_light = (
            raw_light_color is None
            or light_confidence <= self.model_only_max_conflicting_light_confidence
        )
        if (
                reliable_color is None
                and track['committed_color'] is None
                and model_color is not None
                and model_confidence >= self.model_only_min_confidence
                and weak_or_missing_light):
            reliable_color = model_color
            evidence_kind = 'model_only'

        if track['committed_color'] is None:
            track['fallback_color'] = model_color
            track['fallback_confidence'] = self._normalized_confidence(model_confidence)
            self._update_fallback(track, raw_light_color, detected_confidence)

        if reliable_color is None:
            track['candidate_color'] = None
            track['candidate_count'] = 0
            track['candidate_required_count'] = 0
        elif track['committed_color'] == reliable_color:
            track['candidate_color'] = None
            track['candidate_count'] = 0
            track['candidate_required_count'] = 0
        else:
            if evidence_kind == 'model_only':
                evidence_required_count = self.model_only_confirmation_count
            elif evidence_kind == 'compact_pair':
                evidence_required_count = self.compact_pair_confirmation_count
            elif int(detected_bars or 0) == 1:
                evidence_required_count = self.single_bar_confirmation_count
            else:
                evidence_required_count = self.confirmation_count
            required_count = (
                evidence_required_count
                if track['committed_color'] is None
                else max(self.switch_confirmation_count, evidence_required_count)
            )
            if track['candidate_color'] == reliable_color:
                track['candidate_count'] += 1
                track['candidate_required_count'] = max(
                    track.get('candidate_required_count', 0),
                    required_count,
                )
            else:
                track['candidate_color'] = reliable_color
                track['candidate_count'] = 1
                track['candidate_required_count'] = required_count
            if track['candidate_count'] >= track['candidate_required_count']:
                track['committed_color'] = reliable_color
                track['candidate_color'] = None
                track['candidate_count'] = 0
                track['candidate_required_count'] = 0

        if track['committed_color'] is not None:
            resolved_color = track['committed_color']
            used_hold = raw_light_color != resolved_color
        else:
            resolved_color = track['fallback_color'] or model_color or reliable_color
            used_hold = False

        if track is not None:
            track.update({
                'center': signature['center'],
                'size': signature['size'],
                'box': signature['box'],
                'last_seen_frame': int(current_frame),
                'updated_frame': int(current_frame),
            })

        if resolved_color is None:
            result = (label, False)
        else:
            result = (f'{resolved_color}{number}', used_hold)
        if return_state:
            return result[0], result[1], track['committed_color'] is not None
        return result
