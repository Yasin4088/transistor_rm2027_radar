import json
import os
import tempfile
import threading
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]  # src/ — 保留 .runtime 状态目录位置（本模块已移入 src/output/）
STATUS_PATH = PROJECT_ROOT / ".runtime" / "radar_status.json"
_STATUS_LOCK = threading.Lock()


def read_runtime_status(status_path=STATUS_PATH):
    path = Path(status_path)
    try:
        with path.open("r", encoding="utf-8") as status_file:
            payload = json.load(status_file)
        return payload if isinstance(payload, dict) else {}
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return {}


def _write_runtime_status(payload, status_path):
    path = Path(status_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    file_descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(file_descriptor, "w", encoding="utf-8") as temporary_file:
            json.dump(payload, temporary_file, ensure_ascii=False, sort_keys=True)
            temporary_file.flush()
            os.fsync(temporary_file.fileno())
        os.replace(temporary_name, path)
    finally:
        if os.path.exists(temporary_name):
            os.remove(temporary_name)


def initialize_runtime_status(status_path=STATUS_PATH, **values):
    payload = {
        "phase": "starting",
        "serial_enabled": False,
        "serial_open": False,
        "camera_ready": False,
        "camera_error": None,
        "camera_fps": None,
        "processing_fps": 0.0,
        "fps": 0.0,
        "recording_requested": False,
        "recording": False,
        "recording_suspended_reason": None,
        "last_referee_packet_at": None,
        "last_referee_command": None,
    }
    payload.update(values)
    with _STATUS_LOCK:
        _write_runtime_status(payload, status_path)
    return payload


def update_runtime_status(status_path=STATUS_PATH, **values):
    with _STATUS_LOCK:
        payload = read_runtime_status(status_path)
        payload.update(values)
        _write_runtime_status(payload, status_path)
    return payload


def clear_runtime_status(status_path=STATUS_PATH):
    path = Path(status_path)
    with _STATUS_LOCK:
        try:
            path.unlink()
        except FileNotFoundError:
            pass
