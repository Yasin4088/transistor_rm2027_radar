import os
import re
import tempfile
from pathlib import Path

import yaml


_KEY_PATTERN = re.compile(r"^(?P<indent>[ ]*)(?P<key>[A-Za-z_][A-Za-z0-9_-]*):(?P<rest>.*)$")


def load_config(config_path):
    with open(config_path, "r", encoding="utf-8") as config_file:
        loaded = yaml.safe_load(config_file)
    if not isinstance(loaded, dict):
        raise ValueError(f"配置文件不是有效的 YAML 映射: {config_path}")
    return loaded


def _split_inline_comment(text):
    quote = None
    escaped = False
    for index, character in enumerate(text):
        if escaped:
            escaped = False
            continue
        if character == "\\" and quote == '"':
            escaped = True
            continue
        if character in ("'", '"'):
            if quote is None:
                quote = character
            elif quote == character:
                quote = None
            continue
        if character == "#" and quote is None and (index == 0 or text[index - 1].isspace()):
            return text[:index], text[index:]
    return text, ""


def _format_scalar(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, str):
        return "'" + value.replace("'", "''") + "'"
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return str(value)
    if isinstance(value, list):
        return "[" + ", ".join(_format_scalar(item) for item in value) + "]"
    raise TypeError(f"不支持写入配置的值类型: {type(value).__name__}")


def _get_nested(mapping, path):
    current = mapping
    for key in path:
        if not isinstance(current, dict) or key not in current:
            raise KeyError(".".join(path))
        current = current[key]
    return current


def update_config_values(config_path, updates):
    """Update selected YAML scalar values while preserving layout and comments."""
    path = Path(config_path)
    normalized_updates = {tuple(key.split(".")): value for key, value in updates.items()}
    original_mode = path.stat().st_mode
    with path.open("r", encoding="utf-8", newline="") as config_file:
        lines = config_file.readlines()

    parents = []
    found = set()
    updated_lines = []

    for line in lines:
        newline = "\r\n" if line.endswith("\r\n") else "\n" if line.endswith("\n") else ""
        body = line[:-len(newline)] if newline else line
        match = _KEY_PATTERN.match(body)
        if match is None:
            updated_lines.append(line)
            continue

        indent = len(match.group("indent"))
        key = match.group("key")
        rest = match.group("rest")
        while parents and parents[-1][0] >= indent:
            parents.pop()
        current_path = tuple(item[1] for item in parents) + (key,)
        value_text, comment = _split_inline_comment(rest)

        if current_path in normalized_updates:
            formatted = _format_scalar(normalized_updates[current_path])
            replacement = f"{match.group('indent')}{key}: {formatted}"
            if comment:
                replacement += "  " + comment.lstrip()
            updated_lines.append(replacement + newline)
            found.add(current_path)
        else:
            updated_lines.append(line)

        if not value_text.strip():
            parents.append((indent, key))

    missing = set(normalized_updates) - found
    if missing:
        missing_text = ", ".join(".".join(item) for item in sorted(missing))
        raise KeyError(f"config.yaml 缺少配置项: {missing_text}")

    updated_text = "".join(updated_lines)
    parsed = yaml.safe_load(updated_text)
    for item_path, expected in normalized_updates.items():
        actual = _get_nested(parsed, item_path)
        if actual != expected:
            raise ValueError(
                f"配置校验失败 {'.'.join(item_path)}: expected={expected!r}, actual={actual!r}"
            )

    file_descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(file_descriptor, "w", encoding="utf-8", newline="") as temporary_file:
            temporary_file.write(updated_text)
            temporary_file.flush()
            os.fsync(temporary_file.fileno())
        os.chmod(temporary_name, original_mode)
        os.replace(temporary_name, path)
    finally:
        if os.path.exists(temporary_name):
            os.remove(temporary_name)

    return parsed
