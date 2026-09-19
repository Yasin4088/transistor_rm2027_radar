"""Minimal team-defined radar message for opponent invincibility states."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence


INVINCIBLE_TARGETS_DATA_CMD_ID = 0x0234

INVINCIBLE_MAIN_STATUSES = frozenset((2, 3))
VALID_MAIN_STATUSES = frozenset((0, 1, 2, 3))

WIRE_STATUS_ROBOT_NUMBERS = (
    ("opponent_hero", 1),
    ("opponent_engineer", 2),
    ("opponent_infantry_3", 3),
    ("opponent_infantry_4", 4),
    ("opponent_sentry", 7),
)
# Hero, all infantry, aerial and sentry receive the same five target states.
RECIPIENT_ROBOT_NUMBERS = (1, 3, 4, 5, 6, 7)


def _clean_side(side: object) -> str:
    value = str(side).strip().lower()
    if value not in ("red", "blue"):
        raise ValueError(f"radio side must be red or blue, got {side!r}")
    return value


def robot_id_for_side(robot_number: int, side: object) -> int:
    clean_side = _clean_side(side)
    number = int(robot_number)
    if not 1 <= number <= 9:
        raise ValueError(f"robot number must be in 1..9, got {number}")
    return number if clean_side == "red" else number + 100


def invincible_target_receivers(own_side: object) -> tuple[int, ...]:
    return tuple(robot_id_for_side(number, own_side) for number in RECIPIENT_ROBOT_NUMBERS)


def fallback_invincible_target_statuses(own_side: object) -> tuple[tuple[int, bool], ...]:
    """Return all five opponent targets as not invincible."""

    opponent_side = "blue" if _clean_side(own_side) == "red" else "red"
    return tuple(
        (robot_id_for_side(number, opponent_side), False)
        for _name, number in WIRE_STATUS_ROBOT_NUMBERS
    )


def invincible_target_statuses(
    robot_main_status: Mapping[str, object],
    own_side: object,
) -> tuple[tuple[int, bool], ...]:
    """Map a complete 0x0A05 snapshot to five ``(robot_id, bool)`` pairs."""

    opponent_side = "blue" if _clean_side(own_side) == "red" else "red"
    result = []
    for name, number in WIRE_STATUS_ROBOT_NUMBERS:
        if name not in robot_main_status:
            raise ValueError(f"0x0A05 robot_main_status is missing {name}")
        try:
            value = int(robot_main_status[name])
        except (TypeError, ValueError) as exc:
            raise ValueError(f"invalid 0x0A05 status for {name}: {robot_main_status[name]!r}") from exc
        if value not in VALID_MAIN_STATUSES:
            raise ValueError(f"0x0A05 status for {name} must be in 0..3, got {value}")
        result.append((robot_id_for_side(number, opponent_side), value in INVINCIBLE_MAIN_STATUSES))
    return tuple(result)


def invincible_target_ids(
    robot_main_status: Mapping[str, object],
    own_side: object,
) -> tuple[int, ...]:
    """Compatibility helper returning only IDs whose state is invincible."""

    return tuple(
        robot_id
        for robot_id, invincible in invincible_target_statuses(robot_main_status, own_side)
        if invincible
    )


@dataclass(frozen=True)
class InvincibleTargetsMessage:
    """One-byte payload: five fixed-order invincibility bits."""

    invincible_mask: int

    def to_bytes(self) -> bytes:
        mask = int(self.invincible_mask)
        if not 0 <= mask <= 0x1F:
            raise ValueError(f"invincible mask must use only bits 0..4, got 0x{mask:02X}")
        return bytes((mask,))

    @classmethod
    def from_target_statuses(
        cls,
        target_statuses: Sequence[tuple[int, bool]],
    ) -> "InvincibleTargetsMessage":
        entries = tuple(target_statuses)
        if len(entries) != len(WIRE_STATUS_ROBOT_NUMBERS):
            raise ValueError(f"exactly {len(WIRE_STATUS_ROBOT_NUMBERS)} target states are required")
        return cls(
            invincible_mask=sum(
                (1 << index) if bool(invincible) else 0
                for index, (_robot_id, invincible) in enumerate(entries)
            ),
        )

    @classmethod
    def from_bytes(cls, payload: bytes | bytearray | Sequence[int]) -> "InvincibleTargetsMessage":
        data = bytes(payload)
        if len(data) != 1:
            raise ValueError(f"invincible-target payload must be 1 byte, got {len(data)}")
        message = cls(invincible_mask=data[0])
        message.to_bytes()
        return message

    @property
    def invincible_states(self) -> tuple[bool, ...]:
        return tuple(bool(self.invincible_mask & (1 << index)) for index in range(5))

    def to_dict(self) -> dict:
        return {
            "schema": "shark.radar.invincible_target_states.v1",
            "data_cmd_id": INVINCIBLE_TARGETS_DATA_CMD_ID,
            "invincible_mask": int(self.invincible_mask),
            "states": {
                name: self.invincible_states[index]
                for index, (name, _robot_number) in enumerate(WIRE_STATUS_ROBOT_NUMBERS)
            },
        }
