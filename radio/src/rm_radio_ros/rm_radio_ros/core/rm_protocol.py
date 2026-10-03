from __future__ import annotations

import struct
from collections import deque
from dataclasses import dataclass
from typing import Deque, Iterable, Iterator, List, Optional


RADAR_DATA_LENGTHS = {
    0x0001: 11,
    0x0003: 20,
    0x0105: 3,
    0x0201: 17,
    0x020C: 2,
    0x020E: 1,
    0x0A01: 24,
    0x0A02: 12,
    0x0A03: 10,
    0x0A04: 8,
    0x0A05: 41,
    0x0A06: 6,
}


RADAR_AIR_DATA_LENGTHS = {
    cmd_id: length
    for cmd_id, length in RADAR_DATA_LENGTHS.items()
    if 0x0A01 <= cmd_id <= 0x0A06
}


RADAR_COMMAND_NAMES = {
    0x0001: 'game_status',
    0x0003: 'game_robot_hp',
    0x0105: 'dart_status',
    0x0201: 'robot_status',
    0x020C: 'radar_mark_progress',
    0x020E: 'radar_decision_sync',
    0x0A01: 'enemy_position',
    0x0A02: 'enemy_hp',
    0x0A03: 'enemy_bullet_allowance',
    0x0A04: 'enemy_macro_status',
    0x0A05: 'enemy_buff_status',
    0x0A06: 'interference_password',
}


AIR_LENGTH_FIELD = b'\x00\x0F\x00\x0F'


RADAR_WIRE_ROBOT_NAMES = [
    'opponent_hero',
    'opponent_engineer',
    'opponent_infantry_3',
    'opponent_infantry_4',
    'opponent_aerial',
    'opponent_sentry',
]


RADAR_LEGACY_ROBOT_NAMES = ['hero', 'engineer', 'standard_3', 'standard_4', 'aerial', 'sentry']


RADAR_BUFF_ROBOT_SPECS = [
    ('opponent_hero', 0),
    ('opponent_engineer', 7),
    ('opponent_infantry_3', 14),
    ('opponent_infantry_4', 21),
    ('opponent_sentry', 28),
]


RADAR_MAIN_STATUS_ROBOT_NAMES = [
    'opponent_hero',
    'opponent_engineer',
    'opponent_infantry_3',
    'opponent_infantry_4',
    'opponent_sentry',
]


RADAR_MAIN_STATUS_NAMES = {
    0: 'alive',
    1: 'destroyed',
    2: 'invincible_not_weakened',
    3: 'invincible_weakened',
}


RADAR_SENTRY_MODE_NAMES = {
    1: 'offensive',
    2: 'defensive',
    3: 'mobile',
    4: 'enhanced_offensive',
    5: 'enhanced_defensive',
    6: 'enhanced_mobile',
}


ACCESS_CODES = {
    'broadcast': bytes([0x2F, 0x6F, 0x4C, 0x74, 0xB9, 0x14, 0x49, 0x2E]),
    'interference': bytes([0x16, 0xE8, 0xD3, 0x77, 0x15, 0x1C, 0x71, 0x2D]),
}


GAME_PROGRESS_NAMES = {
    0: 'not_started',
    1: 'preparation',
    2: 'self_check_15s',
    3: 'countdown_5s',
    4: 'running',
    5: 'settlement',
}


ROBOT_ID_TO_SIDE = {
    **{robot_id: 'red' for robot_id in range(1, 12)},
    **{robot_id: 'blue' for robot_id in range(101, 112)},
}


def u8_list(values: Iterable[int]) -> List[int]:
    return [int(value) & 0xFF for value in values]


def crc8_rm(data: Iterable[int]) -> int:
    crc = 0xFF
    for byte in u8_list(data):
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                crc = ((crc >> 1) ^ 0x8C) & 0xFF
            else:
                crc = (crc >> 1) & 0xFF
    return crc & 0xFF


def crc16_rm(data: Iterable[int]) -> int:
    crc = 0xFFFF
    for byte in u8_list(data):
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc & 0xFFFF


def le_u16(data: List[int], offset: int) -> int:
    return data[offset] | (data[offset + 1] << 8)


def le_u32(data: List[int], offset: int) -> int:
    return (
        data[offset]
        | (data[offset + 1] << 8)
        | (data[offset + 2] << 16)
        | (data[offset + 3] << 24)
    )


def le_u64(data: List[int], offset: int) -> int:
    value = 0
    for index in range(8):
        value |= data[offset + index] << (8 * index)
    return value


def put_le_u16(value: int) -> List[int]:
    clean = int(value) & 0xFFFF
    return [clean & 0xFF, (clean >> 8) & 0xFF]


def put_le_u32(value: int) -> List[int]:
    clean = int(value) & 0xFFFFFFFF
    return [
        clean & 0xFF,
        (clean >> 8) & 0xFF,
        (clean >> 16) & 0xFF,
        (clean >> 24) & 0xFF,
    ]


def _bits(value: int, start: int, width: int = 1) -> int:
    return (int(value) >> int(start)) & ((1 << int(width)) - 1)


def _strict_u8(name: str, value: int) -> int:
    clean = int(value)
    if not 0 <= clean <= 0xFF:
        raise ValueError(f'{name} must be in uint8 range 0..255, got {value!r}')
    return clean


def bytes_to_bits(data: Iterable[int]) -> List[int]:
    bits: List[int] = []
    for byte in u8_list(data):
        bits.extend((byte >> shift) & 0x01 for shift in range(7, -1, -1))
    return bits


def bits_to_bytes(bits: Iterable[int]) -> bytes:
    clean_bits = [1 if int(bit) else 0 for bit in bits]
    if len(clean_bits) % 8:
        raise ValueError(f'bit count must be a multiple of 8, got {len(clean_bits)}')
    out = []
    for offset in range(0, len(clean_bits), 8):
        value = 0
        for bit in clean_bits[offset : offset + 8]:
            value = (value << 1) | bit
        out.append(value)
    return bytes(out)


@dataclass(frozen=True)
class RefereeFrame:
    cmd_id: int
    data: bytes
    seq: int
    raw: bytes

    @property
    def command_name(self) -> str:
        return RADAR_COMMAND_NAMES.get(self.cmd_id, f'cmd_0x{self.cmd_id:04X}')

    def to_dict(self) -> dict:
        parsed = parse_radar_frame(self)
        return {
            'cmd_id': self.cmd_id,
            'cmd_hex': f'0x{self.cmd_id:04X}',
            'command': self.command_name,
            'seq': self.seq,
            'data_length': len(self.data),
            'data_hex': self.data.hex(),
            'raw_hex': self.raw.hex(),
            'crc_valid': True,
            'parsed': parsed,
        }


class RefereeFrameAssembler:
    """Self-synchronizing assembler for RoboMaster referee serial frames."""

    def __init__(self, max_buffer_size: int = 4096, allowed_cmd_lengths: Optional[dict[int, int]] = None):
        self._buffer: List[int] = []
        self.max_buffer_size = int(max_buffer_size)
        self.allowed_cmd_lengths = dict(allowed_cmd_lengths) if allowed_cmd_lengths is not None else None
        self._diagnostic_events: Deque[dict] = deque(maxlen=512)
        self._diagnostic_counts = self._new_diagnostic_counts()

    @staticmethod
    def _new_diagnostic_counts() -> dict[str, int]:
        return {
            'crc8_failures': 0,
            'length_failures': 0,
            'crc16_attempts': 0,
            'crc16_failures': 0,
            'crc16_valid_frames': 0,
            'command_length_failures': 0,
            'valid_frames': 0,
        }

    def _record_diagnostic(self, stage: str, **detail: int) -> None:
        event = {'stage': str(stage)}
        event.update({key: int(value) for key, value in detail.items()})
        self._diagnostic_events.append(event)

    def diagnostics(self) -> dict[str, int]:
        return {
            **self._diagnostic_counts,
            'buffered_bytes': len(self._buffer),
            'pending_events': len(self._diagnostic_events),
        }

    def pop_diagnostic_events(self) -> List[dict]:
        events = list(self._diagnostic_events)
        self._diagnostic_events.clear()
        return events

    def clear(self) -> None:
        self._buffer.clear()
        self._diagnostic_events.clear()
        self._diagnostic_counts = self._new_diagnostic_counts()

    def push_air_payload(self, payload: Iterable[int]) -> List[RefereeFrame]:
        data = u8_list(payload)
        if len(data) != 15:
            raise ValueError(f'air payload must be 15 bytes, got {len(data)}')
        return self.push_bytes(data)

    def push_bytes(self, data: Iterable[int]) -> List[RefereeFrame]:
        self._buffer.extend(u8_list(data))
        if len(self._buffer) > self.max_buffer_size:
            self._buffer = self._buffer[-self.max_buffer_size :]
        return list(self._drain())

    def _drain(self) -> Iterator[RefereeFrame]:
        while True:
            sof_index = self._find_sof()
            if sof_index is None:
                self._buffer.clear()
                return
            if sof_index:
                del self._buffer[:sof_index]
            if len(self._buffer) < 5:
                return

            header = self._buffer[:5]
            if crc8_rm(header[:4]) != header[4]:
                self._diagnostic_counts['crc8_failures'] += 1
                self._record_diagnostic('crc8_failure')
                del self._buffer[0]
                continue

            data_length = le_u16(header, 1)
            frame_length = 5 + 2 + data_length + 2
            if frame_length < 9:
                self._diagnostic_counts['length_failures'] += 1
                self._record_diagnostic('length_failure', seq=header[3], data_length=data_length)
                del self._buffer[0]
                continue
            if len(self._buffer) < frame_length:
                return

            raw = self._buffer[:frame_length]
            seq = raw[3]
            self._diagnostic_counts['crc16_attempts'] += 1
            expected_crc = le_u16(raw, frame_length - 2)
            if crc16_rm(raw[:-2]) != expected_crc:
                self._diagnostic_counts['crc16_failures'] += 1
                self._record_diagnostic('crc16_failure', seq=seq, data_length=data_length)
                del self._buffer[0]
                continue

            cmd_id = le_u16(raw, 5)
            data = bytes(raw[7 : 7 + data_length])
            self._diagnostic_counts['crc16_valid_frames'] += 1
            del self._buffer[:frame_length]
            if self.allowed_cmd_lengths is not None:
                expected_length = self.allowed_cmd_lengths.get(cmd_id)
                if expected_length != data_length:
                    self._diagnostic_counts['command_length_failures'] += 1
                    self._record_diagnostic(
                        'command_length_failure',
                        seq=seq,
                        cmd_id=cmd_id,
                        data_length=data_length,
                        expected_length=-1 if expected_length is None else expected_length,
                    )
                    continue
            self._diagnostic_counts['valid_frames'] += 1
            self._record_diagnostic('valid', seq=seq, cmd_id=cmd_id, data_length=data_length)
            yield RefereeFrame(cmd_id=cmd_id, data=data, seq=seq, raw=bytes(raw))

    def _find_sof(self) -> Optional[int]:
        for index, value in enumerate(self._buffer):
            if value == 0xA5:
                return index
        return None


class AccessCodeAirPacketExtractor:
    """Extract 15-byte air payloads from demodulated 0/1 bits."""

    def __init__(
        self,
        access_codes: Optional[Iterable[Iterable[int]]] = None,
        max_bits: int = 8192,
        max_access_hamming: int = 3,
        max_length_hamming: int = 0,
        allow_inverted: bool = True,
    ):
        source_codes = access_codes if access_codes is not None else ACCESS_CODES.values()
        self.access_patterns = [bytes_to_bits(code) for code in source_codes]
        self.access_variants = []
        for pattern in self.access_patterns:
            self.access_variants.append((pattern, pattern, False))
            if bool(allow_inverted):
                self.access_variants.append(([1 - bit for bit in pattern], pattern, True))
        self.length_pattern = bytes_to_bits(AIR_LENGTH_FIELD)
        self.packet_bits = 8 * (8 + 4 + 15)
        self.length_bits = 8 * 4
        self.payload_bits = 8 * 15
        self.max_bits = int(max_bits)
        self.max_access_hamming = int(max_access_hamming)
        self.max_length_hamming = int(max_length_hamming)
        self.allow_inverted = bool(allow_inverted)
        self._bits: List[int] = []

    def clear(self) -> None:
        self._bits.clear()

    def push_bits(self, bits: Iterable[int]) -> List[bytes]:
        self._bits.extend(1 if int(bit) else 0 for bit in bits)
        if len(self._bits) > self.max_bits:
            self._bits = self._bits[-self.max_bits :]
        return list(self._drain())

    def push_bit_bytes(self, bit_bytes: Iterable[int]) -> List[bytes]:
        return self.push_bits(1 if int(value) else 0 for value in bit_bytes)

    def _drain(self) -> Iterator[bytes]:
        while True:
            match = self._find_access()
            if match is None:
                keep = max(self.packet_bits - 1, max((len(pattern) for pattern in self.access_patterns), default=0) - 1)
                self._bits = self._bits[-keep:] if keep > 0 else []
                return

            start, pattern, inverted, _distance = match
            if start:
                del self._bits[:start]
            if len(self._bits) < self.packet_bits:
                return

            length_start = len(pattern)
            length_end = length_start + self.length_bits
            length_bits = self._normalized_bits(self._bits[length_start:length_end], inverted)
            if self._hamming(length_bits, self.length_pattern) > self.max_length_hamming:
                del self._bits[0]
                continue

            payload_start = length_end
            payload_end = payload_start + self.payload_bits
            payload = bits_to_bytes(self._normalized_bits(self._bits[payload_start:payload_end], inverted))
            del self._bits[:payload_end]
            yield payload

    def _find_access(self):
        best_start = None
        best_match = None
        for candidate_pattern, pattern, inverted in self.access_variants:
            found = self._find_pattern(candidate_pattern, self.max_access_hamming)
            if found is None:
                continue
            start, distance = found
            if (
                best_match is None
                or start < best_start
                or (start == best_start and distance < best_match[3])
            ):
                best_start = start
                best_match = (start, pattern, inverted, distance)
        return best_match

    def _find_pattern(self, pattern: List[int], max_hamming: int = 0):
        if not pattern or len(self._bits) < len(pattern):
            return None
        last_start = len(self._bits) - len(pattern)
        limit = int(max_hamming)
        pattern_length = len(pattern)
        for start in range(last_start + 1):
            distance = 0
            for offset in range(pattern_length):
                if int(self._bits[start + offset]) != int(pattern[offset]):
                    distance += 1
                    if distance > limit:
                        break
            if distance <= limit:
                return start, distance
        return None

    @staticmethod
    def _hamming(bits: Iterable[int], pattern: Iterable[int]) -> int:
        return sum(1 for bit, expected in zip(bits, pattern) if int(bit) != int(expected))

    @staticmethod
    def _normalized_bits(bits: Iterable[int], inverted: bool) -> List[int]:
        if not inverted:
            return [1 if int(bit) else 0 for bit in bits]
        return [0 if int(bit) else 1 for bit in bits]


def build_referee_frame(cmd_id: int, data: Iterable[int], seq: int = 1) -> bytes:
    payload = u8_list(data)
    header = [
        0xA5,
        len(payload) & 0xFF,
        (len(payload) >> 8) & 0xFF,
        seq & 0xFF,
    ]
    frame = header + [crc8_rm(header), cmd_id & 0xFF, (cmd_id >> 8) & 0xFF] + payload
    crc = crc16_rm(frame)
    return bytes(frame + [crc & 0xFF, (crc >> 8) & 0xFF])


def build_robot_interaction_data(data_cmd_id: int, sender_id: int, receiver_id: int, user_data: Iterable[int]) -> bytes:
    payload = u8_list(user_data)
    if len(payload) > 112:
        raise ValueError(f'robot interaction user_data must be <=112 bytes, got {len(payload)}')
    data = put_le_u16(data_cmd_id) + put_le_u16(sender_id) + put_le_u16(receiver_id) + payload
    return bytes(data)


def build_robot_interaction_frame(
    data_cmd_id: int,
    sender_id: int,
    receiver_id: int,
    user_data: Iterable[int],
    seq: int = 1,
) -> bytes:
    return build_referee_frame(
        0x0301,
        build_robot_interaction_data(data_cmd_id, sender_id, receiver_id, user_data),
        seq=seq,
    )


def parse_robot_interaction_data(data: Iterable[int]) -> dict:
    values = u8_list(data)
    if len(values) < 6:
        return {'valid_robot_interaction_length': False}
    data_cmd_id = le_u16(values, 0)
    user_data = values[6:]
    return {
        'valid_robot_interaction_length': len(values) <= 118 and len(user_data) <= 112,
        'data_cmd_id': data_cmd_id,
        'data_cmd_hex': f'0x{data_cmd_id:04X}',
        'sender_id': le_u16(values, 2),
        'receiver_id': le_u16(values, 4),
        'user_data_length': len(user_data),
        'user_data_hex': bytes(user_data).hex(),
    }


def build_radar_cmd_payload(radar_cmd: int = 0, password_cmd: int = 0, password: str | bytes | None = None) -> bytes:
    radar_cmd_u8 = _strict_u8('radar_cmd', radar_cmd)
    password_cmd_u8 = _strict_u8('password_cmd', password_cmd)
    if password is None and password_cmd_u8 == 0:
        return bytes([radar_cmd_u8, 0]) + bytes(6)
    if password_cmd_u8 not in (1, 2):
        raise ValueError('password_cmd must be 1 (update own password) or 2 (verify opponent password)')
    if password is None:
        raise ValueError('password is required when password_cmd is nonzero')
    if isinstance(password, str):
        password_bytes = password.encode('ascii', errors='strict')
    else:
        password_bytes = bytes(password)
    if len(password_bytes) != 6:
        raise ValueError('radar password must be exactly 6 ASCII bytes')
    if any(not (48 <= byte <= 57 or 65 <= byte <= 90 or 97 <= byte <= 122) for byte in password_bytes):
        raise ValueError('radar password bytes must be ASCII letters or digits')
    return bytes([radar_cmd_u8, password_cmd_u8]) + password_bytes


def parse_radar_cmd_payload(data: Iterable[int]) -> dict:
    values = u8_list(data)
    result = {'valid_radar_cmd_length': len(values) == 8}
    if not values:
        return result
    result['radar_cmd'] = values[0]
    if len(values) != 8:
        return result
    result['password_cmd'] = values[1]
    if values[1] == 0:
        return result
    password_bytes = bytes(values[2:8])
    result.update({
        'password': password_bytes.decode('ascii', errors='replace') if len(password_bytes) == 6 else '',
        'password_is_alnum_ascii': len(password_bytes) == 6
        and all((48 <= byte <= 57 or 65 <= byte <= 90 or 97 <= byte <= 122) for byte in password_bytes),
    })
    return result


def parse_radar_decision_sync(data: Iterable[int]) -> dict:
    values = u8_list(data)
    result = {'valid_radar_info_length': len(values) == 1}
    if not values:
        return result
    radar_info = values[0]
    result.update({
        'radar_info': radar_info,
        'double_vulnerability_count': _bits(radar_info, 0, 2),
        'is_double_vulnerability': bool(_bits(radar_info, 2)),
        'own_encryption_level': _bits(radar_info, 3, 2),
        'opponent_interference_difficulty': _bits(radar_info, 3, 2),
        'can_change_password': bool(_bits(radar_info, 5)),
        'reserved': _bits(radar_info, 6, 2),
    })
    return result


def parse_game_status(data: Iterable[int]) -> dict:
    values = u8_list(data)
    result = {'valid_game_status_length': len(values) == 11}
    if len(values) < 3:
        return result
    packed = values[0]
    game_progress = _bits(packed, 4, 4)
    result.update({
        'game_type': _bits(packed, 0, 4),
        'game_progress': game_progress,
        'game_progress_name': GAME_PROGRESS_NAMES.get(game_progress, f'unknown_{game_progress}'),
        'stage_remain_time': le_u16(values, 1),
        'match_running': game_progress == 4,
    })
    if len(values) >= 11:
        result['sync_timestamp'] = le_u64(values, 3)
    return result


def parse_game_robot_hp(data: Iterable[int]) -> dict:
    values = u8_list(data)
    result = {'valid_game_robot_hp_length': len(values) == 20}
    if len(values) < 20:
        return result
    result.update({
        'ally_hero': le_u16(values, 0),
        'ally_engineer': le_u16(values, 2),
        'ally_infantry_3': le_u16(values, 4),
        'ally_infantry_4': le_u16(values, 6),
        'damage_difference': struct.unpack_from('<h', bytes(values), 8)[0],
        'ally_sentry': le_u16(values, 10),
        'ally_outpost': le_u16(values, 12),
        'ally_base': le_u16(values, 14),
        'opponent_outpost': le_u16(values, 16),
        'opponent_base': le_u16(values, 18),
    })
    return result


def parse_dart_status(data: Iterable[int]) -> dict:
    values = u8_list(data)
    result = {'valid_dart_status_length': len(values) == 3}
    if len(values) < 3:
        return result
    packed_status = le_u16(values, 1)
    result.update({
        'dart_remaining_time': values[0],
        'recent_hit_target': _bits(packed_status, 0, 3),
        'accumulated_hit_count': _bits(packed_status, 3, 3),
        'selected_target': _bits(packed_status, 6, 3),
        'reserved': _bits(packed_status, 9, 7),
    })
    return result


def side_from_robot_id(robot_id: int) -> Optional[str]:
    return ROBOT_ID_TO_SIDE.get(int(robot_id))


def radar_sender_id_for_side(side: str) -> int:
    clean_side = str(side).strip().lower()
    if clean_side == 'red':
        return 9
    if clean_side == 'blue':
        return 109
    raise ValueError(f'unknown radio side: {side!r}')


def parse_robot_status(data: Iterable[int]) -> dict:
    values = u8_list(data)
    result = {'valid_robot_status_length': len(values) == 17}
    if not values:
        return result
    robot_id = values[0]
    result.update({
        'robot_id': robot_id,
        'radio_side': side_from_robot_id(robot_id),
    })
    if len(values) >= 17:
        power_management = values[16]
        result.update({
            'robot_level': values[1],
            'current_hp': le_u16(values, 2),
            'maximum_hp': le_u16(values, 4),
            'shooter_barrel_cooling_value': le_u16(values, 6),
            'shooter_barrel_heat_limit': le_u16(values, 8),
            'chassis_power_limit': le_u16(values, 10),
            'bullet_speed_limit': struct.unpack_from('<f', bytes(values), 12)[0],
            'power_management_gimbal_output': bool(_bits(power_management, 0)),
            'power_management_chassis_output': bool(_bits(power_management, 1)),
            'power_management_shooter_output': bool(_bits(power_management, 2)),
            'power_management_reserved': _bits(power_management, 3, 5),
        })
    return result


def parse_radar_mark_progress(data: Iterable[int]) -> dict:
    values = u8_list(data)
    result = {'valid_radar_mark_progress_length': len(values) == 2}
    if len(values) < 2:
        return result
    progress_bits = le_u16(values, 0)
    result.update({
        'progress_bits': progress_bits,
        'enemy': {
            'opponent_hero': bool(_bits(progress_bits, 0)),
            'opponent_engineer': bool(_bits(progress_bits, 1)),
            'opponent_infantry_3': bool(_bits(progress_bits, 2)),
            'opponent_infantry_4': bool(_bits(progress_bits, 3)),
            'opponent_aerial': bool(_bits(progress_bits, 4)),
            'opponent_sentry': bool(_bits(progress_bits, 5)),
        },
        'ally': {
            'ally_hero': bool(_bits(progress_bits, 6)),
            'ally_engineer': bool(_bits(progress_bits, 7)),
            'ally_infantry_3': bool(_bits(progress_bits, 8)),
            'ally_infantry_4': bool(_bits(progress_bits, 9)),
            'ally_aerial': bool(_bits(progress_bits, 10)),
            'ally_sentry': bool(_bits(progress_bits, 11)),
        },
        'aerial_countermeasure': {
            'opponent_aerial_aimed_by_ally_radar': bool(_bits(progress_bits, 12)),
            'opponent_aerial_countered': bool(_bits(progress_bits, 13)),
            'ally_aerial_aimed_by_opponent_radar': bool(_bits(progress_bits, 14)),
            'ally_aerial_countered': bool(_bits(progress_bits, 15)),
        },
    })
    return result


def parse_radar_frame(frame: RefereeFrame) -> dict:
    data = list(frame.data)
    expected_length = RADAR_DATA_LENGTHS.get(frame.cmd_id)
    result = {
        'valid_radar_length': expected_length is None or expected_length == len(data),
    }

    if frame.cmd_id == 0x0001:
        result['game_status'] = parse_game_status(data)
    elif frame.cmd_id == 0x0003:
        result['game_robot_hp'] = parse_game_robot_hp(data)
    elif frame.cmd_id == 0x0105:
        result['dart_status'] = parse_dart_status(data)
    elif frame.cmd_id == 0x0201:
        result['robot_status'] = parse_robot_status(data)
    elif frame.cmd_id == 0x020C:
        result['radar_mark_progress'] = parse_radar_mark_progress(data)
    elif frame.cmd_id == 0x0A01 and len(data) == 24:
        official_positions = {
            name: {'x': le_u16(data, index * 4), 'y': le_u16(data, index * 4 + 2)}
            for index, name in enumerate(RADAR_WIRE_ROBOT_NAMES)
        }
        legacy_positions = {
            legacy: official_positions[official]
            for legacy, official in zip(RADAR_LEGACY_ROBOT_NAMES, RADAR_WIRE_ROBOT_NAMES)
        }
        result['positions_cm'] = legacy_positions
        result['official_positions_cm'] = official_positions
    elif frame.cmd_id == 0x0A02 and len(data) == 12:
        names = [
            'opponent_hero',
            'opponent_engineer',
            'opponent_infantry_3',
            'opponent_infantry_4',
            'reserved',
            'opponent_sentry',
        ]
        hp = {
            name: le_u16(data, index * 2)
            for index, name in enumerate(names)
        }
        legacy_hp = {
            legacy: hp[official]
            for legacy, official in zip(
                ['hero', 'engineer', 'standard_3', 'standard_4', 'reserved', 'sentry'],
                names,
            )
        }
        result['hp'] = legacy_hp
        result['official_hp'] = hp
    elif frame.cmd_id == 0x0A03 and len(data) == 10:
        names = [
            'opponent_hero',
            'opponent_infantry_3',
            'opponent_infantry_4',
            'opponent_aerial',
            'opponent_sentry',
        ]
        bullet_allowance = {
            name: le_u16(data, index * 2)
            for index, name in enumerate(names)
        }
        legacy_bullet_allowance = {
            legacy: bullet_allowance[official]
            for legacy, official in zip(['hero', 'standard_3', 'standard_4', 'aerial', 'sentry'], names)
        }
        result['bullet_allowance'] = legacy_bullet_allowance
        result['official_bullet_allowance'] = bullet_allowance
    elif frame.cmd_id == 0x0A04 and len(data) == 8:
        occupation_bits = le_u32(data, 4)
        result['remaining_coins'] = le_u16(data, 0)
        result['total_coins'] = le_u16(data, 2)
        result['occupation_bits'] = occupation_bits
        result['occupation'] = {
            'opponent_supply_zone_occupied': bool(_bits(occupation_bits, 0)),
            'opponent_central_highland_status': _bits(occupation_bits, 1, 2),
            'opponent_trapezoid_highland_occupied': bool(_bits(occupation_bits, 3)),
            'opponent_fortress_buff_zone_status': _bits(occupation_bits, 4, 2),
            'opponent_outpost_buff_zone_status': _bits(occupation_bits, 6, 2),
            'opponent_base_buff_zone_occupied': bool(_bits(occupation_bits, 8)),
            'opponent_near_flyover_before_tunnel_rfid': bool(_bits(occupation_bits, 9)),
            'opponent_near_flyover_after_tunnel_rfid': bool(_bits(occupation_bits, 10)),
            'ally_near_flyover_before_tunnel_rfid': bool(_bits(occupation_bits, 11)),
            'ally_near_flyover_after_tunnel_rfid': bool(_bits(occupation_bits, 12)),
            'opponent_terrain_crossing_highland_rfid': bool(_bits(occupation_bits, 13)),
            'opponent_terrain_crossing_flyover_rfid': bool(_bits(occupation_bits, 14)),
            'opponent_terrain_crossing_road_rfid': bool(_bits(occupation_bits, 15)),
        }
    elif frame.cmd_id == 0x0A05 and len(data) == 41:
        result['raw_buff_bytes'] = data
        result['buff_status'] = {
            name: {
                'hp_recovery_percent': data[offset],
                'shooting_heat_cooling': le_u16(data, offset + 1),
                'defense_percent': data[offset + 3],
                'negative_defense_percent': data[offset + 4],
                'attack_percent': le_u16(data, offset + 5),
            }
            for name, offset in RADAR_BUFF_ROBOT_SPECS
        }
        result['sentry_mode'] = data[35]
        result['sentry_mode_name'] = RADAR_SENTRY_MODE_NAMES.get(
            data[35],
            f'unknown_{data[35]}',
        )
        result['robot_main_status'] = {
            name: data[36 + index]
            for index, name in enumerate(RADAR_MAIN_STATUS_ROBOT_NAMES)
        }
        result['robot_main_status_names'] = {
            name: RADAR_MAIN_STATUS_NAMES.get(value, f'unknown_{value}')
            for name, value in result['robot_main_status'].items()
        }
    elif frame.cmd_id == 0x0A06 and len(data) == 6:
        try:
            password = bytes(data).decode('ascii', errors='strict')
            password_is_alnum_ascii = password.isalnum()
        except UnicodeDecodeError:
            password = bytes(data).decode('ascii', errors='replace')
            password_is_alnum_ascii = False
        result['password'] = password
        result['password_is_alnum_ascii'] = password_is_alnum_ascii
    elif frame.cmd_id == 0x020E:
        result['radar_decision_sync'] = parse_radar_decision_sync(data)
    elif frame.cmd_id == 0x0301:
        interaction = parse_robot_interaction_data(data)
        result['robot_interaction'] = interaction
        if interaction.get('data_cmd_id') == 0x0121:
            result['radar_cmd'] = parse_radar_cmd_payload(data[6:])

    return result
