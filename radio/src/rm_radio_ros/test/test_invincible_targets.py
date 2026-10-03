import pytest

from rm_radio_ros.core.invincible_targets import (
    INVINCIBLE_TARGETS_DATA_CMD_ID,
    InvincibleTargetsMessage,
    fallback_invincible_target_statuses,
    invincible_target_ids,
    invincible_target_receivers,
    invincible_target_statuses,
)


VALID_STATUS = {
    "opponent_hero": 0,
    "opponent_engineer": 1,
    "opponent_infantry_3": 2,
    "opponent_infantry_4": 3,
    "opponent_sentry": 0,
}


def test_recipient_ids_cover_hero_all_infantry_aerial_and_sentry():
    assert invincible_target_receivers("red") == (1, 3, 4, 5, 6, 7)
    assert invincible_target_receivers("blue") == (101, 103, 104, 105, 106, 107)


def test_statuses_carry_only_opponent_id_and_current_invincible_bool():
    assert invincible_target_statuses(VALID_STATUS, "red") == (
        (101, False),
        (102, False),
        (103, True),
        (104, True),
        (107, False),
    )
    assert invincible_target_ids(VALID_STATUS, "blue") == (3, 4)


def test_invincible_status_snapshot_must_be_complete_and_valid():
    partial = dict(VALID_STATUS)
    partial.pop("opponent_sentry")
    with pytest.raises(ValueError, match="missing opponent_sentry"):
        invincible_target_statuses(partial, "red")

    invalid = dict(VALID_STATUS, opponent_sentry=4)
    with pytest.raises(ValueError, match="must be in 0..3"):
        invincible_target_statuses(invalid, "red")


def test_payload_is_one_fixed_order_bitmask_byte_and_round_trips():
    statuses = invincible_target_statuses(VALID_STATUS, "red")
    message = InvincibleTargetsMessage.from_target_statuses(statuses)

    payload = message.to_bytes()
    decoded = InvincibleTargetsMessage.from_bytes(payload)

    assert INVINCIBLE_TARGETS_DATA_CMD_ID == 0x0234
    assert payload == bytes((0b01100,))
    assert decoded == message
    assert decoded.invincible_states == (False, False, True, True, False)


def test_fallback_is_all_five_targets_not_invincible():
    statuses = fallback_invincible_target_statuses("red")
    payload = InvincibleTargetsMessage.from_target_statuses(statuses).to_bytes()

    assert statuses == (
        (101, False),
        (102, False),
        (103, False),
        (104, False),
        (107, False),
    )
    assert payload == b"\x00"


def test_payload_rejects_bad_length_or_reserved_bits():
    with pytest.raises(ValueError, match="1 byte"):
        InvincibleTargetsMessage.from_bytes(bytes(2))
    with pytest.raises(ValueError, match="bits 0..4"):
        InvincibleTargetsMessage.from_bytes(b"\x20")
