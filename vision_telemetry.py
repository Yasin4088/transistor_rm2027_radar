from __future__ import annotations

import time


ROBOT_ROLE_IDS = ('1', '2', '3', '4', '6', '7')


def classify_legacy_target_names(
    measured_names: set,
    hold_cache_names: set,
) -> tuple[set, set]:
    """Return (valid, occlusion) without misclassifying fresh measurements."""
    measured = set(measured_names)
    occlusion = set(hold_cache_names) - measured
    return measured | occlusion, occlusion


def build_vision_telemetry(
    *,
    side: str,
    send_map: dict,
    valid_names: set,
    guess_list: dict,
    occlusion_names: set,
    camera_ready: bool,
    fps: float,
    inference_ms: float,
    filter_type: str,
    camera_fps: float | None = None,
    source_time: float | None = None,
) -> dict:
    robots = {}
    now = float(source_time if source_time is not None else time.time())
    for prefix in ('R', 'B'):
        for role in ROBOT_ROLE_IDS:
            name = f'{prefix}{role}'
            raw = send_map.get(name, (0, 0))
            try:
                x, y = int(raw[0]), int(raw[1])
            except (TypeError, ValueError, IndexError):
                x, y = 0, 0
            valid = (x != 0 or y != 0) and 0 <= x <= 2800 and 0 <= y <= 1500
            if not valid:
                x, y = 0, 0
                state = 'missing'
            elif name in occlusion_names:
                state = 'occlusion_hold'
            elif bool(guess_list.get(name, False)):
                state = 'blind_prediction'
            else:
                state = 'measured'
            robots[name] = {
                'position_cm': {'x': x, 'y': y},
                'valid': valid,
                'state': state,
                'confidence': 1.0 if name in valid_names and state == 'measured' else None,
                'observed_at': now if valid else None,
            }
    processing_fps = round(max(float(fps), 0.0), 2)
    measured_camera_fps = (
        None
        if camera_fps is None
        else round(max(float(camera_fps), 0.0), 2)
    )
    return {
        'schema': 'shark.radar.telemetry.v1',
        'source_time': now,
        'side': 'red' if str(side).upper() == 'R' else 'blue',
        'vision': {
            'camera_ready': bool(camera_ready),
            'model_ready': True,
            # Keep fps as a compatibility alias for existing radio consumers.
            'fps': processing_fps,
            'processing_fps': processing_fps,
            'camera_fps': measured_camera_fps,
            'inference_ms': round(max(float(inference_ms), 0.0), 2),
            'filter_type': str(filter_type),
        },
        'robots': robots,
    }
