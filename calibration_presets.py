import json
import math
import os
import tempfile
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path


PRESET_FILE_VERSION = 2
PRESET_ENVIRONMENT_VARIABLE = "TRANSISTOR_CALIBRATION_PRESET"
DEFAULT_PRESET_PATH = Path(__file__).resolve().parent / "calibration_presets.json"


class CalibrationPresetError(ValueError):
    """Raised when a calibration preset file or preset is invalid."""


def _validate_name(name):
    normalized = str(name).strip()
    if not normalized:
        raise CalibrationPresetError("预制方案名称不能为空")
    if len(normalized) > 64:
        raise CalibrationPresetError("预制方案名称不能超过 64 个字符")
    if any(ord(character) < 32 for character in normalized):
        raise CalibrationPresetError("预制方案名称不能包含控制字符")
    return normalized


def _validate_side(side):
    normalized = str(side).upper()
    if normalized not in ("R", "B"):
        raise CalibrationPresetError(f"无效的己方阵营: {side}")
    return normalized


def _normalize_points(points, field_name):
    try:
        layers = list(points)
    except TypeError as error:
        raise CalibrationPresetError(f"{field_name} 必须包含两个高度层") from error
    if len(layers) != 2:
        raise CalibrationPresetError(f"{field_name} 必须包含两个高度层")

    normalized_layers = []
    expected_count = None
    for layer_index, layer in enumerate(layers):
        try:
            layer_points = list(layer)
        except TypeError as error:
            raise CalibrationPresetError(
                f"{field_name} 高度 {layer_index} 不是有效点集"
            ) from error
        if expected_count is None:
            expected_count = len(layer_points)
        elif len(layer_points) != expected_count:
            raise CalibrationPresetError(f"{field_name} 两个高度层的点数必须一致")

        normalized_layer = []
        for point_index, point in enumerate(layer_points):
            if point is None:
                raise CalibrationPresetError(
                    f"{field_name} 高度 {layer_index} 的点 {point_index} 尚未设置"
                )
            try:
                coordinates = list(point)
            except TypeError as error:
                raise CalibrationPresetError(
                    f"{field_name} 高度 {layer_index} 的点 {point_index} 格式错误"
                ) from error
            if len(coordinates) != 2:
                raise CalibrationPresetError(
                    f"{field_name} 高度 {layer_index} 的点 {point_index} 必须为二维坐标"
                )
            try:
                x, y = (float(value) for value in coordinates)
            except (TypeError, ValueError) as error:
                raise CalibrationPresetError(
                    f"{field_name} 高度 {layer_index} 的点 {point_index} 坐标无效"
                ) from error
            if not math.isfinite(x) or not math.isfinite(y):
                raise CalibrationPresetError(
                    f"{field_name} 高度 {layer_index} 的点 {point_index} 坐标必须为有限数值"
                )
            normalized_layer.append([x, y])
        normalized_layers.append(normalized_layer)

    if expected_count is None or expected_count < 4:
        raise CalibrationPresetError("每个高度层至少需要 4 个标定点")
    return normalized_layers


def make_preset(name, side, map_points, updated_at=None):
    normalized_map_points = _normalize_points(map_points, "地图点")
    points_per_layer = len(normalized_map_points[0])
    return {
        "name": _validate_name(name),
        "side": _validate_side(side),
        "points_per_layer": points_per_layer,
        "map_points": normalized_map_points,
        "updated_at": updated_at
        or datetime.now(timezone.utc).isoformat(timespec="seconds"),
    }


def _normalize_preset(raw_preset):
    if not isinstance(raw_preset, dict):
        raise CalibrationPresetError("预制方案条目必须为映射")
    preset = make_preset(
        raw_preset.get("name", ""),
        raw_preset.get("side", ""),
        raw_preset.get("map_points"),
        updated_at=str(raw_preset.get("updated_at") or ""),
    )
    declared_count = raw_preset.get("points_per_layer")
    if declared_count != preset["points_per_layer"]:
        raise CalibrationPresetError(
            f"预制方案“{preset['name']}”记录的点数与实际点数不一致"
        )
    return preset


def load_presets(path=DEFAULT_PRESET_PATH):
    preset_path = Path(path)
    if not preset_path.exists():
        return []
    try:
        with preset_path.open("r", encoding="utf-8") as preset_file:
            document = json.load(preset_file)
    except (OSError, json.JSONDecodeError) as error:
        raise CalibrationPresetError(f"无法读取预制方案文件 {preset_path}: {error}") from error

    if not isinstance(document, dict):
        raise CalibrationPresetError("预制方案文件顶层必须为映射")
    file_version = document.get("version")
    if file_version not in (1, PRESET_FILE_VERSION):
        raise CalibrationPresetError(
            f"不支持的预制方案文件版本: {file_version!r}"
        )
    raw_presets = document.get("presets")
    if not isinstance(raw_presets, list):
        raise CalibrationPresetError("预制方案文件缺少 presets 列表")

    presets = []
    keys = set()
    for raw_preset in raw_presets:
        preset = _normalize_preset(raw_preset)
        key = (preset["side"], preset["name"])
        if key in keys:
            raise CalibrationPresetError(
                f"己方 {preset['side']} 存在重复方案名“{preset['name']}”"
            )
        keys.add(key)
        presets.append(preset)
    return presets


def list_presets(path=DEFAULT_PRESET_PATH, side=None, points_per_layer=None):
    normalized_side = _validate_side(side) if side is not None else None
    presets = load_presets(path)
    if normalized_side is not None:
        presets = [preset for preset in presets if preset["side"] == normalized_side]
    if points_per_layer is not None:
        expected_count = int(points_per_layer)
        presets = [
            preset for preset in presets
            if preset["points_per_layer"] == expected_count
        ]
    return sorted(
        (deepcopy(preset) for preset in presets),
        key=lambda preset: preset["name"].casefold(),
    )


def get_preset(name, side, path=DEFAULT_PRESET_PATH):
    normalized_name = _validate_name(name)
    normalized_side = _validate_side(side)
    for preset in load_presets(path):
        if preset["side"] == normalized_side and preset["name"] == normalized_name:
            return deepcopy(preset)
    raise CalibrationPresetError(
        f"未找到己方 {normalized_side} 的预制方案“{normalized_name}”"
    )


def _write_presets(path, presets):
    preset_path = Path(path)
    preset_path.parent.mkdir(parents=True, exist_ok=True)
    document = {
        "version": PRESET_FILE_VERSION,
        "presets": presets,
    }
    existing_mode = preset_path.stat().st_mode if preset_path.exists() else None
    file_descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{preset_path.name}.",
        suffix=".tmp",
        dir=str(preset_path.parent),
    )
    try:
        with os.fdopen(file_descriptor, "w", encoding="utf-8") as temporary_file:
            json.dump(document, temporary_file, ensure_ascii=False, indent=2)
            temporary_file.write("\n")
            temporary_file.flush()
            os.fsync(temporary_file.fileno())
        if existing_mode is not None:
            os.chmod(temporary_name, existing_mode)
        os.replace(temporary_name, preset_path)
    finally:
        if os.path.exists(temporary_name):
            os.remove(temporary_name)


def save_preset(
    name,
    side,
    map_points,
    path=DEFAULT_PRESET_PATH,
    overwrite=False,
):
    preset = make_preset(name, side, map_points)
    presets = load_presets(path)
    matching_index = next(
        (
            index for index, current in enumerate(presets)
            if current["side"] == preset["side"] and current["name"] == preset["name"]
        ),
        None,
    )
    if matching_index is not None and not overwrite:
        raise CalibrationPresetError(
            f"己方 {preset['side']} 已存在预制方案“{preset['name']}”"
        )
    if matching_index is None:
        presets.append(preset)
    else:
        presets[matching_index] = preset
    presets.sort(key=lambda item: (item["side"], item["name"].casefold()))
    _write_presets(path, presets)
    return deepcopy(preset)


def delete_preset(name, side, path=DEFAULT_PRESET_PATH):
    normalized_name = _validate_name(name)
    normalized_side = _validate_side(side)
    presets = load_presets(path)
    remaining = [
        preset for preset in presets
        if not (
            preset["side"] == normalized_side
            and preset["name"] == normalized_name
        )
    ]
    if len(remaining) == len(presets):
        raise CalibrationPresetError(
            f"未找到己方 {normalized_side} 的预制方案“{normalized_name}”"
        )
    _write_presets(path, remaining)
