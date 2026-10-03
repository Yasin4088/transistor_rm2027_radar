"""RMUC 2026 radar wireless payload constraints used by Lab TX.

The wire protocol defines the integer widths, while the competition rulebook
defines the smaller ranges that can occur in a match.  Keep the constraints in
one dependency-free module so the dashboard, virtual-link builder and GNU
Radio runtime all generate the same legal values.
"""

from __future__ import annotations

import random
from typing import Any, Iterable, Sequence


POSITION_X_MAX_CM = 2800
POSITION_Y_MAX_CM = 1500

ROBOT_KEYS = (
    "opponent_hero",
    "opponent_engineer",
    "opponent_infantry_3",
    "opponent_infantry_4",
    "opponent_aerial",
    "opponent_sentry",
)
HP_KEYS = (
    "opponent_hero",
    "opponent_engineer",
    "opponent_infantry_3",
    "opponent_infantry_4",
    "reserved",
    "opponent_sentry",
)
BULLET_KEYS = (
    "opponent_hero",
    "opponent_infantry_3",
    "opponent_infantry_4",
    "opponent_aerial",
    "opponent_sentry",
)
BUFF_ROBOT_KEYS = (
    "opponent_hero",
    "opponent_engineer",
    "opponent_infantry_3",
    "opponent_infantry_4",
    "opponent_sentry",
)

# Rulebook V2.0.1: maximum in-match HP for the corresponding robot type.
HP_MAX = {
    "opponent_hero": 600,
    "opponent_engineer": 250,
    "opponent_infantry_3": 400,
    "opponent_infantry_4": 400,
    "reserved": 0,
    "opponent_sentry": 400,
}

# Theoretical per-robot maxima from the ammunition acquisition rules.  The
# infantry values include the fortress's up-to-500 reserve as specified by
# 0x0A03.  The sentry value includes 300 initial, 1000 team exchange and six
# 100-round supply-zone grants.
BULLET_MAX = {
    "opponent_hero": 100,
    "opponent_infantry_3": 1500,
    "opponent_infantry_4": 1500,
    "opponent_aerial": 750,
    "opponent_sentry": 1900,
}

# Current buff fields.  Cooling is the direct increase (not the final cooling
# rate): the level-10 hero can gain at most +200/s from a 5x energy mechanism;
# infantry/sentry can gain at most +120/s.  Engineering has no launcher.
BUFF_FIELD_MAX = {
    "hp_recovery_percent": {key: 25 for key in BUFF_ROBOT_KEYS},
    "shooting_heat_cooling": {
        "opponent_hero": 200,
        "opponent_engineer": 0,
        "opponent_infantry_3": 120,
        "opponent_infantry_4": 120,
        "opponent_sentry": 120,
    },
    "defense_percent": {key: 99 for key in BUFF_ROBOT_KEYS},
    "negative_defense_percent": {key: 100 for key in BUFF_ROBOT_KEYS},
    "attack_percent": {key: 300 for key in BUFF_ROBOT_KEYS},
}

RADAR_COMMAND_DATA_LENGTHS = {
    0x0A01: 24,
    0x0A02: 12,
    0x0A03: 10,
    0x0A04: 8,
    0x0A05: 41,
    0x0A06: 6,
}
RADAR_BROADCAST_COMMANDS = ("0x0A01", "0x0A02", "0x0A03", "0x0A04", "0x0A05")


def clamp_int(value: Any, low: int, high: int, default: int = 0) -> int:
    try:
        parsed = int(value, 0) if isinstance(value, str) else int(value)
    except (TypeError, ValueError):
        parsed = int(default)
    return min(max(parsed, int(low)), int(high))


def normalize_occupation_bits(value: Any) -> int:
    """Clear reserved bits and normalize multi-bit occupation enumerations."""
    bits = clamp_int(value, 0, 0xFFFFFFFF) & 0xFFFF
    central = min((bits >> 1) & 0x03, 2)  # 0/1/2; value 3 is undefined.
    fortress = (bits >> 4) & 0x03         # 0/1/2/3 are all defined.
    outpost = min((bits >> 6) & 0x03, 2)  # 0/1/2; value 3 is undefined.
    boolean_bits = bits & ((1 << 0) | (1 << 3) | 0xFF00)
    return boolean_bits | (central << 1) | (fortress << 4) | (outpost << 6)


def random_occupation_bits(rng: Any = random) -> int:
    bits = rng.randint(0, 1)
    bits |= rng.randint(0, 2) << 1
    bits |= rng.randint(0, 1) << 3
    bits |= rng.randint(0, 3) << 4
    bits |= rng.randint(0, 2) << 6
    for bit in range(8, 16):
        bits |= rng.randint(0, 1) << bit
    return bits


def command_id_value(cmd_id: int | Sequence[int]) -> int:
    if isinstance(cmd_id, int):
        return int(cmd_id) & 0xFFFF
    values = [int(item) & 0xFF for item in cmd_id]
    if len(values) != 2:
        raise ValueError("cmd_id must contain exactly 2 bytes")
    # The flowgraph API represents 0x0A01 as [0x0A, 0x01].
    return (values[0] << 8) | values[1]


def _put_u16(value: int) -> list[int]:
    return [value & 0xFF, (value >> 8) & 0xFF]


def _put_u32(value: int) -> list[int]:
    return [value & 0xFF, (value >> 8) & 0xFF, (value >> 16) & 0xFF, (value >> 24) & 0xFF]


def random_payload_for_command(
    cmd_id: int | Sequence[int],
    size: int,
    rng: Any = random,
) -> list[int]:
    """Generate one semantically valid payload for a radar wireless command."""
    command = command_id_value(cmd_id)
    expected_size = RADAR_COMMAND_DATA_LENGTHS.get(command)
    if expected_size is not None and int(size) != expected_size:
        raise ValueError(
            f"0x{command:04X} payload must be {expected_size} bytes, got {int(size)}"
        )

    out: list[int] = []
    if command == 0x0A01:
        for _key in ROBOT_KEYS:
            out.extend(_put_u16(rng.randint(0, POSITION_X_MAX_CM)))
            out.extend(_put_u16(rng.randint(0, POSITION_Y_MAX_CM)))
    elif command == 0x0A02:
        for key in HP_KEYS:
            out.extend(_put_u16(rng.randint(0, HP_MAX[key])))
    elif command == 0x0A03:
        for key in BULLET_KEYS:
            out.extend(_put_u16(rng.randint(0, BULLET_MAX[key])))
    elif command == 0x0A04:
        total = rng.randint(0, 0xFFFF)
        remaining = rng.randint(0, total)
        out.extend(_put_u16(remaining))
        out.extend(_put_u16(total))
        out.extend(_put_u32(random_occupation_bits(rng)))
    elif command == 0x0A05:
        for key in BUFF_ROBOT_KEYS:
            out.append(rng.randint(0, BUFF_FIELD_MAX["hp_recovery_percent"][key]))
            out.extend(_put_u16(rng.randint(0, BUFF_FIELD_MAX["shooting_heat_cooling"][key])))
            out.append(rng.randint(0, BUFF_FIELD_MAX["defense_percent"][key]))
            out.append(rng.randint(0, BUFF_FIELD_MAX["negative_defense_percent"][key]))
            out.extend(_put_u16(rng.randint(0, BUFF_FIELD_MAX["attack_percent"][key])))
        out.append(rng.randint(1, 6))
        out.extend(rng.randint(0, 3) for _key in BUFF_ROBOT_KEYS)
    else:
        return [rng.getrandbits(8) for _ in range(max(0, int(size)))]

    if len(out) != int(size):
        raise AssertionError(f"0x{command:04X} generated {len(out)} bytes, expected {int(size)}")
    return out


def valid_password_bytes(values: Iterable[int]) -> bool:
    data = bytes(int(value) & 0xFF for value in values)
    return len(data) == 6 and all(
        ord("0") <= value <= ord("9")
        or ord("A") <= value <= ord("Z")
        or ord("a") <= value <= ord("z")
        for value in data
    )
