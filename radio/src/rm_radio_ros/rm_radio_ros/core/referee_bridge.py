from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Iterable, List, Optional

from .rm_protocol import RADAR_WIRE_ROBOT_NAMES, RefereeFrame, parse_radar_frame


RADAR_INFO_TO_CLIENT_ORDER = [
    'opponent_hero',
    'opponent_engineer',
    'opponent_infantry_3',
    'opponent_infantry_4',
    'opponent_aerial',
    'opponent_sentry',
    'ally_hero',
    'ally_engineer',
    'ally_infantry_3',
    'ally_infantry_4',
    'ally_aerial',
    'ally_sentry',
]


@dataclass(frozen=True)
class RadarSingleRobotInfo:
    target_pos_x: int
    target_pos_y: int
    is_high_light: int = 0

    def to_dict(self) -> dict:
        return {
            'target_pos_x': self.target_pos_x,
            'target_pos_y': self.target_pos_y,
            'is_high_light': self.is_high_light,
        }


@dataclass(frozen=True)
class RadarInfoToClient:
    robots: List[RadarSingleRobotInfo]

    def to_dict(self) -> dict:
        return {
            'topic': 'RadarInfoToClient',
            'RadarSingleRobotInfo': [robot.to_dict() for robot in self.robots],
        }

    def to_json(self) -> str:
        import json

        return json.dumps(self.to_dict(), ensure_ascii=False)


@dataclass(frozen=True)
class RadarCommand0121:
    radar_cmd: int
    password_cmd: int
    password: str

    def to_bytes(self) -> bytes:
        password_bytes = self.password.encode('ascii', errors='strict')
        if len(password_bytes) != 6:
            raise ValueError('radar password must be exactly 6 ASCII bytes')
        return bytes([self.radar_cmd & 0xFF, self.password_cmd & 0xFF]) + password_bytes

    def to_dict(self) -> dict:
        return {
            'cmd_id': '0x0121',
            'name': 'radar_cmd_t',
            'radar_cmd': self.radar_cmd,
            'password_cmd': self.password_cmd,
            'password': self.password,
            'payload_hex': self.to_bytes().hex(),
        }


def _bridge_payload(frame: RefereeFrame, output_type: str, payload_key: str) -> Optional[dict]:
    parsed = parse_radar_frame(frame)
    payload = parsed.get(payload_key)
    if payload is None:
        return None
    return {
        'type': output_type,
        'cmd_hex': f'0x{frame.cmd_id:04X}',
        'seq': frame.seq,
        'payload': payload,
    }


def radar_info_from_position_frame(frame: RefereeFrame) -> Optional[RadarInfoToClient]:
    if frame.cmd_id != 0x0A01:
        return None
    parsed = parse_radar_frame(frame)
    positions = parsed.get('official_positions_cm') or parsed.get('positions_cm')
    if not positions:
        return None
    zeros = {'x': 0, 'y': 0}
    full_positions = {name: positions.get(name, zeros) for name in RADAR_WIRE_ROBOT_NAMES}
    for name in RADAR_INFO_TO_CLIENT_ORDER:
        full_positions.setdefault(name, zeros)
    robots = [
        RadarSingleRobotInfo(
            target_pos_x=int(full_positions[name]['x']),
            target_pos_y=int(full_positions[name]['y']),
            is_high_light=0,
        )
        for name in RADAR_INFO_TO_CLIENT_ORDER
    ]
    return RadarInfoToClient(robots=robots)


def radar_command_from_password_frame(frame: RefereeFrame) -> Optional[RadarCommand0121]:
    if frame.cmd_id != 0x0A06:
        return None
    parsed = parse_radar_frame(frame)
    password = parsed.get('password')
    if not password or not parsed.get('password_is_alnum_ascii', False):
        return None
    return RadarCommand0121(radar_cmd=0, password_cmd=2, password=password)


class RefereeBridge:
    def __init__(self):
        self.last_radar_info: Optional[RadarInfoToClient] = None
        self.last_radar_command: Optional[RadarCommand0121] = None
        self.last_broadcast_status: dict[str, dict] = {}
        self.last_referee_status: dict[str, dict] = {}

    def process_frames(self, frames: Iterable[RefereeFrame]) -> List[dict]:
        outputs = []
        timestamp = time.time()
        for frame in frames:
            radar_info = radar_info_from_position_frame(frame)
            if radar_info is not None:
                self.last_radar_info = radar_info
                payload = {
                    'type': 'RadarInfoToClient',
                    'timestamp': timestamp,
                    'payload': radar_info.to_dict(),
                }
                outputs.append(payload)

            status_specs = {
                0x0A02: ('RadarEnemyHp', 'official_hp'),
                0x0A03: ('RadarEnemyBulletAllowance', 'official_bullet_allowance'),
                0x0A04: ('RadarEnemyMacroStatus', 'occupation'),
                0x0A05: ('RadarEnemyBuffStatus', 'buff_status'),
            }
            if frame.cmd_id in status_specs:
                output_type, payload_key = status_specs[frame.cmd_id]
                status = _bridge_payload(frame, output_type, payload_key)
                if status is not None:
                    parsed = parse_radar_frame(frame)
                    if frame.cmd_id == 0x0A04:
                        status['payload'] = {
                            'remaining_coins': parsed.get('remaining_coins'),
                            'total_coins': parsed.get('total_coins'),
                            'occupation_bits': parsed.get('occupation_bits'),
                            'occupation': parsed.get('occupation'),
                        }
                    elif frame.cmd_id == 0x0A05:
                        status['payload'] = {
                            'buff_status': parsed.get('buff_status'),
                            'sentry_mode': parsed.get('sentry_mode'),
                            'sentry_mode_name': parsed.get('sentry_mode_name'),
                            'robot_main_status': parsed.get('robot_main_status'),
                            'robot_main_status_names': parsed.get('robot_main_status_names'),
                        }
                    self.last_broadcast_status[output_type] = status
                    status['timestamp'] = timestamp
                    outputs.append(status)

            referee_status_specs = {
                0x0001: ('GameStatus', 'game_status'),
                0x0003: ('GameRobotHP', 'game_robot_hp'),
                0x0105: ('DartStatus', 'dart_status'),
                0x020C: ('RadarMarkProgress', 'radar_mark_progress'),
                0x020E: ('RadarDecisionSync', 'radar_decision_sync'),
            }
            if frame.cmd_id in referee_status_specs:
                output_type, payload_key = referee_status_specs[frame.cmd_id]
                status = _bridge_payload(frame, output_type, payload_key)
                if status is not None:
                    self.last_referee_status[output_type] = status
                    status['timestamp'] = timestamp
                    outputs.append(status)

            radar_command = radar_command_from_password_frame(frame)
            if radar_command is not None:
                self.last_radar_command = radar_command
                outputs.append({
                    'type': 'RadarCommand0121',
                    'timestamp': timestamp,
                    'payload': radar_command.to_dict(),
                })
        return outputs
