from __future__ import annotations

import struct
from copy import deepcopy
from typing import Mapping, Optional


ROBOT_ROLE_IDS = ('1', '2', '3', '4', '6', '7')
ROBOT_NAMES = tuple(f'{side}{role}' for side in ('R', 'B') for role in ROBOT_ROLE_IDS)
VISION_STATES = {'measured', 'occlusion_hold', 'blind_prediction', 'missing'}
RADIO_WIRE_NAMES = (
    'opponent_hero',
    'opponent_engineer',
    'opponent_infantry_3',
    'opponent_infantry_4',
    'opponent_aerial',
    'opponent_sentry',
)
VISION_TELEMETRY_SCHEMAS = {
    'transistor.radar.telemetry.v1',
    # Retain upstream replay compatibility for previously recorded sessions.
    'shark.radar.telemetry.v1',
}


def normalize_side(value: object) -> Optional[str]:
    clean = str(value or '').strip().lower()
    if clean in ('r', 'red'):
        return 'red'
    if clean in ('b', 'blue'):
        return 'blue'
    return None


def side_prefix(side: str) -> str:
    clean = normalize_side(side)
    if clean is None:
        raise ValueError(f'unknown side: {side!r}')
    return 'R' if clean == 'red' else 'B'


def opponent_prefix(side: str) -> str:
    return 'B' if side_prefix(side) == 'R' else 'R'


def valid_position(position: object) -> bool:
    if not isinstance(position, Mapping):
        return False
    try:
        x = int(position.get('x', 0))
        y = int(position.get('y', 0))
    except (TypeError, ValueError):
        return False
    return (x != 0 or y != 0) and 0 <= x <= 2800 and 0 <= y <= 1500


def clean_position(position: object) -> dict[str, int]:
    if not valid_position(position):
        return {'x': 0, 'y': 0}
    assert isinstance(position, Mapping)
    return {'x': int(position['x']), 'y': int(position['y'])}


def build_0305_payload(positions: Mapping[str, object], own_side: str) -> bytes | None:
    """Build map_robot_data_t: six opponents followed by six allies."""
    ally = side_prefix(own_side)
    opponent = 'B' if ally == 'R' else 'R'
    values: list[int] = []
    has_nonzero = False
    for prefix in (opponent, ally):
        for role in ROBOT_ROLE_IDS:
            position = clean_position(positions.get(f'{prefix}{role}'))
            x, y = position['x'], position['y']
            has_nonzero = has_nonzero or x != 0 or y != 0
            values.extend((x, y))
    return struct.pack('<24H', *values) if has_nonzero else None


class RadarFusionState:
    """Deterministic cache/fusion logic shared by the ROS node and tests."""

    def __init__(
        self,
        own_side: str,
        *,
        auto_sync_side: bool = True,
        vision_timeout_sec: float = 0.5,
        radio_timeout_sec: float = 1.0,
        radio_ghost_sec: float = 2.0,
    ):
        clean_side = normalize_side(own_side)
        if clean_side is None:
            raise ValueError(f'own_side must be red or blue, got {own_side!r}')
        self.configured_side = clean_side
        self.own_side = clean_side
        self.side_source = 'configured'
        self.auto_sync_side = bool(auto_sync_side)
        self.detected_side: Optional[str] = None
        self.vision_timeout_sec = float(vision_timeout_sec)
        self.radio_timeout_sec = float(radio_timeout_sec)
        self.radio_ghost_sec = float(radio_ghost_sec)
        self.vision_received_at: Optional[float] = None
        self.radio_received_at: Optional[float] = None
        self.vision_source_time: Optional[float] = None
        self.vision_side: Optional[str] = None
        self.vision: dict[str, dict] = {}
        self.radio: dict[str, dict] = {}
        self.vision_meta: dict = {}
        self.strategy: dict[str, dict] = {}

    def _set_own_side(self, side: object, source: str) -> bool:
        clean_side = normalize_side(side)
        if clean_side is None:
            return False
        changed = clean_side != self.own_side
        self.own_side = clean_side
        self.side_source = str(source)
        if changed:
            # Radio packets are assigned opponent names using own_side. Never
            # retain a packet that was decoded under the previous orientation.
            self.radio = {}
            self.radio_received_at = None
        return changed

    def set_referee_side(self, detected_side: object, effective_side: object = None) -> bool:
        """Track the raw referee observation and hot-sync the effective side."""
        self.detected_side = normalize_side(detected_side)
        sync_side = normalize_side(effective_side) or self.detected_side
        if self.auto_sync_side and sync_side is not None:
            return self._set_own_side(sync_side, 'referee')
        if self.detected_side is None and self.side_source == 'referee':
            # Keep the last known-good side while allowing vision telemetry to
            # become the fallback source on its next update.
            self.side_source = 'last_known'
        return False

    def set_detected_side(self, side: object) -> bool:
        """Backward-compatible wrapper for callers that only have detection."""
        return self.set_referee_side(side, side)

    def update_vision(self, message: Mapping[str, object], received_at: float) -> None:
        if message.get('schema') not in VISION_TELEMETRY_SCHEMAS:
            raise ValueError('unsupported vision telemetry schema')
        message_side = normalize_side(message.get('side'))
        if message_side is None:
            raise ValueError('vision telemetry side must be red or blue')
        if self.auto_sync_side and self.detected_side is None:
            self._set_own_side(message_side, 'vision')
        robots = message.get('robots')
        if not isinstance(robots, Mapping):
            raise ValueError('vision telemetry robots must be an object')

        clean_robots: dict[str, dict] = {}
        for name in ROBOT_NAMES:
            raw = robots.get(name)
            raw = raw if isinstance(raw, Mapping) else {}
            state = str(raw.get('state', 'missing'))
            if state not in VISION_STATES:
                state = 'missing'
            position = clean_position(raw.get('position_cm'))
            valid = bool(raw.get('valid', False)) and valid_position(position)
            if not valid:
                position = {'x': 0, 'y': 0}
            confidence = raw.get('confidence')
            try:
                confidence = max(0.0, min(float(confidence), 1.0)) if confidence is not None else None
            except (TypeError, ValueError):
                confidence = None
            clean_robots[name] = {
                'position_cm': position,
                'valid': valid,
                'state': state,
                'confidence': confidence,
                'observed_at': raw.get('observed_at'),
            }

        self.vision = clean_robots
        self.vision_received_at = float(received_at)
        self.vision_side = message_side
        try:
            self.vision_source_time = float(message.get('source_time', 0.0))
        except (TypeError, ValueError):
            self.vision_source_time = None
        self.vision_meta = deepcopy(message.get('vision')) if isinstance(message.get('vision'), Mapping) else {}

    def update_radio_bridge(self, message: Mapping[str, object], received_at: float) -> bool:
        output_type = message.get('type')
        if output_type != 'RadarInfoToClient':
            if isinstance(output_type, str):
                self.strategy[output_type] = deepcopy(dict(message))
            return False
        payload = message.get('payload')
        if not isinstance(payload, Mapping):
            raise ValueError('RadarInfoToClient payload must be an object')
        robots = payload.get('RadarSingleRobotInfo')
        if not isinstance(robots, list):
            raise ValueError('RadarSingleRobotInfo must be an array')

        prefix = opponent_prefix(self.own_side)
        clean_radio: dict[str, dict] = {}
        for index, role in enumerate(ROBOT_ROLE_IDS):
            raw = robots[index] if index < len(robots) and isinstance(robots[index], Mapping) else {}
            position = clean_position({
                'x': raw.get('target_pos_x', 0),
                'y': raw.get('target_pos_y', 0),
            })
            clean_radio[f'{prefix}{role}'] = {
                'position_cm': position,
                'valid': valid_position(position),
            }
        self.radio = clean_radio
        self.radio_received_at = float(received_at)
        return True

    @staticmethod
    def _age(now: float, received_at: Optional[float]) -> Optional[float]:
        if received_at is None:
            return None
        return max(0.0, float(now) - float(received_at))

    def snapshot(self, now: float) -> dict:
        vision_age = self._age(now, self.vision_received_at)
        radio_age = self._age(now, self.radio_received_at)
        vision_online = vision_age is not None and vision_age <= self.vision_timeout_sec
        radio_fresh = radio_age is not None and radio_age <= self.radio_timeout_sec
        radio_ghost = (
            radio_age is not None
            and self.radio_timeout_sec < radio_age <= self.radio_timeout_sec + self.radio_ghost_sec
        )
        mismatch = self.detected_side is not None and self.detected_side != self.own_side
        robots: dict[str, dict] = {}
        fused_positions: dict[str, dict] = {}
        source_counts = {'vision': 0, 'radio': 0, 'missing': 0}
        own_prefix = side_prefix(self.own_side)

        for name in ROBOT_NAMES:
            visual = deepcopy(self.vision.get(name) or {
                'position_cm': {'x': 0, 'y': 0},
                'valid': False,
                'state': 'missing',
                'confidence': None,
                'observed_at': None,
            })
            visual['age_sec'] = vision_age
            visual['fresh'] = bool(vision_online and visual.get('valid'))
            radio = deepcopy(self.radio.get(name) or {
                'position_cm': {'x': 0, 'y': 0},
                'valid': False,
            })
            radio['age_sec'] = radio_age
            radio['fresh'] = bool(radio_fresh and radio.get('valid'))
            radio['expired'] = bool(radio.get('valid') and radio_age is not None and radio_age > self.radio_timeout_sec)
            radio['ghost_visible'] = bool(radio.get('valid') and (radio_fresh or radio_ghost))

            is_ally = name.startswith(own_prefix)
            if not is_ally and radio['fresh']:
                final_position = clean_position(radio['position_cm'])
                source = 'radio'
            elif visual['fresh']:
                final_position = clean_position(visual['position_cm'])
                source = 'vision'
            else:
                final_position = {'x': 0, 'y': 0}
                source = 'missing'
            source_counts[source] += 1
            fused_positions[name] = final_position
            robots[name] = {
                'visual': visual,
                'radio': radio,
                'final': {
                    'position_cm': final_position,
                    'valid': valid_position(final_position),
                    'source': source,
                },
            }

        payload = build_0305_payload(fused_positions, self.own_side)
        send_allowed = payload is not None and not mismatch
        if mismatch:
            block_reason = 'configured_side_mismatch'
        elif payload is None:
            block_reason = 'all_positions_zero_or_stale'
        else:
            block_reason = ''
        return {
            'schema': 'shark.radar.fusion.v1',
            'side': self.own_side,
            'configured_side': self.configured_side,
            'side_source': self.side_source,
            'auto_sync_side': self.auto_sync_side,
            'detected_side': self.detected_side,
            'configured_side_mismatch': self.configured_side != self.own_side,
            'side_mismatch': mismatch,
            'vision': {
                **deepcopy(self.vision_meta),
                'source_time': self.vision_source_time,
                'reported_side': self.vision_side,
                'age_sec': vision_age,
                'online': vision_online,
                'timeout_sec': self.vision_timeout_sec,
            },
            'radio': {
                'age_sec': radio_age,
                'fresh': radio_fresh,
                'expired': radio_age is not None and radio_age > self.radio_timeout_sec,
                'ghost_visible': radio_ghost,
                'timeout_sec': self.radio_timeout_sec,
                'ghost_sec': self.radio_ghost_sec,
            },
            'robots': robots,
            'source_counts': source_counts,
            'strategy': deepcopy(self.strategy),
            'send_allowed': send_allowed,
            'send_block_reason': block_reason,
            'payload_0305_hex': payload.hex() if send_allowed and payload is not None else '',
        }
