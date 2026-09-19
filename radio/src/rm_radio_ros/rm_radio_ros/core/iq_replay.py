from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

from .radio_config import BROADCAST_FREQUENCIES, interference_setters_for_side_level


IQ_SAMPLE_BYTES = 8
IQ_REPLAY_EXCLUDED_SUFFIXES = {
    ".jpg",
    ".jpeg",
    ".png",
    ".json",
    ".md",
    ".txt",
}


@dataclass(frozen=True)
class IqReplayItem:
    path: Path
    profile: str
    radio_side: str
    center_frequency: int
    bandwidth_hz: int
    sample_rate: int
    size_bytes: int
    duration_seconds: float

    def to_dict(self) -> dict:
        return {
            "path": str(self.path),
            "profile": self.profile,
            "radio_side": self.radio_side,
            "center_frequency": self.center_frequency,
            "bandwidth_hz": self.bandwidth_hz,
            "sample_rate": self.sample_rate,
            "size_bytes": self.size_bytes,
            "duration_seconds": self.duration_seconds,
        }


def split_iq_replay_path_text(raw: str) -> list[str]:
    return [part.strip() for part in re.split(r"[\n,;]+", str(raw or "")) if part.strip()]


def resolve_iq_replay_items(
    raw_paths: str | Iterable[str],
    workspace_root: Path,
    default_radio_side: str = "red",
    default_tx_type: str = "broadcast",
    default_interference_level: int = 1,
    sample_rate: int = 2_000_000,
) -> list[IqReplayItem]:
    tokens = list(raw_paths) if not isinstance(raw_paths, str) else split_iq_replay_path_text(raw_paths)
    files: list[Path] = []
    for token in tokens:
        path = Path(token).expanduser()
        if not path.is_absolute():
            path = workspace_root / path
        path = path.resolve()
        if path.is_dir():
            files.extend(_candidate_iq_files(path))
        elif path.is_file():
            files.append(path)
        else:
            raise ValueError(f"TX IQ 录波路径不存在：{path}")

    items = [
        _item_from_file(path, default_radio_side, default_tx_type, default_interference_level, sample_rate)
        for path in files
    ]
    if not items:
        raise ValueError("TX IQ 录波路径未匹配到 complex64 IQ 文件")
    return items


def _candidate_iq_files(directory: Path) -> list[Path]:
    return sorted(path for path in directory.iterdir() if _looks_like_iq_file(path))


def _looks_like_iq_file(path: Path) -> bool:
    if not path.is_file():
        return False
    suffix = path.suffix.lower()
    if suffix in IQ_REPLAY_EXCLUDED_SUFFIXES:
        return False
    if suffix == ".fc32":
        return path.stat().st_size > 0 and path.stat().st_size % IQ_SAMPLE_BYTES == 0
    name = path.name.lower()
    if name.startswith("rx_") or name.startswith("disturb_"):
        return path.stat().st_size > 0 and path.stat().st_size % IQ_SAMPLE_BYTES == 0
    return suffix == "" and path.stat().st_size > 0 and path.stat().st_size % IQ_SAMPLE_BYTES == 0


def _item_from_file(
    path: Path,
    default_radio_side: str,
    default_tx_type: str,
    default_interference_level: int,
    sample_rate: int,
) -> IqReplayItem:
    side = _infer_side(path.name, default_radio_side)
    profile = _infer_profile(path.name, default_tx_type)
    if profile == "interference":
        level = _infer_level(path.name, default_interference_level)
        setters = interference_setters_for_side_level(side, level)
        center_frequency = int(setters["center_f"])
        bandwidth_hz = int(setters["BW_ganrao"])
    else:
        profile = "broadcast"
        center_frequency = int(BROADCAST_FREQUENCIES[side])
        bandwidth_hz = 540_000

    size_bytes = path.stat().st_size
    duration_seconds = size_bytes / IQ_SAMPLE_BYTES / float(sample_rate)
    return IqReplayItem(
        path=path,
        profile=profile,
        radio_side=side,
        center_frequency=center_frequency,
        bandwidth_hz=bandwidth_hz,
        sample_rate=int(sample_rate),
        size_bytes=size_bytes,
        duration_seconds=duration_seconds,
    )


def _infer_side(name: str, default_radio_side: str) -> str:
    lower = name.lower()
    if "blue" in lower:
        return "blue"
    if "red" in lower:
        return "red"
    return default_radio_side if default_radio_side in BROADCAST_FREQUENCIES else "red"


def _infer_profile(name: str, default_tx_type: str) -> str:
    lower = name.lower()
    if any(token in lower for token in ("ganrao", "disturb", "interference")):
        return "interference"
    if lower.startswith("rx_"):
        return "broadcast"
    if any(token in lower for token in ("broadcast", "xinxibo", "information")):
        return "broadcast"
    return "interference" if default_tx_type == "interference" else "broadcast"


def _infer_level(name: str, default_interference_level: int) -> int:
    lower = name.lower()
    match = re.search(r"(?:ganrao|level)[_\-\s]*([123])", lower)
    if match:
        return int(match.group(1))
    return int(default_interference_level) if int(default_interference_level) in (1, 2, 3) else 1
