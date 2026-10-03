import pytest

from rm_radio_ros.nodes.auto_password_node import DEFAULT_PASSWORD_ALPHABET, generate_password


def test_generate_password_uses_protocol_safe_alnum_alphabet():
    password = generate_password()

    assert len(password) == 6
    assert password.isascii()
    assert password.isalnum()
    assert not any(ch in '0O1Il' for ch in password)


def test_generate_password_rejects_invalid_alphabet():
    with pytest.raises(ValueError, match='16 unique'):
        generate_password(alphabet='ABC')

    with pytest.raises(ValueError, match='ASCII letters or digits'):
        generate_password(alphabet=DEFAULT_PASSWORD_ALPHABET + '_')

    with pytest.raises(ValueError, match='exactly 6'):
        generate_password(length=8)
