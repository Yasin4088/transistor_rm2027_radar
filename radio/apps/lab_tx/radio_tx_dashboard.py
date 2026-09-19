#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import random
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
import traceback
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import unquote, urlparse


RADIO_TX_DIR = Path(__file__).resolve().parent
WS_ROOT = RADIO_TX_DIR.parent.parent
WEB_ROOT = RADIO_TX_DIR / "web"
GENERATED_ROOT = RADIO_TX_DIR / "generated"
LOG_ROOT = Path(os.environ.get("RM_RADIO_DASHBOARD_LOG_ROOT") or os.environ.get("RM_RADIO_LOG_ROOT") or "/tmp/rm_radio_runtime_logs").expanduser()

PACKAGE_SRC = WS_ROOT / "src" / "rm_radio_ros"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from rm_radio_ros.core.radio_config import (
    BROADCAST_FREQUENCIES,
    GFSK_TX_FLOWGRAPH_SAMPLE_RATE,
    interference_setters_for_side_level,
)
from rm_radio_ros.core.radar_wireless_constraints import (
    BUFF_FIELD_MAX,
    BULLET_MAX,
    HP_MAX,
    POSITION_X_MAX_CM,
    POSITION_Y_MAX_CM,
    RADAR_BROADCAST_COMMANDS,
    clamp_int as _rule_clamp_int,
    normalize_occupation_bits,
    random_occupation_bits,
)
from rm_radio_ros.core.rm_protocol import ACCESS_CODES, bytes_to_bits, bits_to_bytes
from rm_radio_ros.core.virtual_link_test import (
    _buff_payload,
    _build_broadcast_air_packets,
    _build_interference_air_packets,
    _bullet_payload,
    _default_broadcast_config,
    _deep_merge,
    _hp_payload,
    _macro_payload,
    _position_payload,
    _stream_decode,
)


AIR_HEADER_HEX = "000f000f"
BROADCAST_BW_HZ = 540_000
OFFICIAL_POWER_DBM = {"broadcast": -60, "interference": -10}
DEFAULT_TX_ATTENUATION_DB = {
    "broadcast": 61,
    "interference": 11,
}
DEFAULT_TX_SAMPLE_RATE = GFSK_TX_FLOWGRAPH_SAMPLE_RATE
AD9361_NO_FIR_MIN_SAMPLE_RATE_HZ = 2_083_333
IIO_CHECK_LOCK = threading.RLock()
FOUR_SDR_ROLE_LABELS = {
    "broadcast_tx": "信息波 TX",
    "interference_tx": "干扰波 TX",
}
ROBOT_KEYS = [
    "opponent_hero",
    "opponent_engineer",
    "opponent_infantry_3",
    "opponent_infantry_4",
    "opponent_aerial",
    "opponent_sentry",
]


def _ensure_dir(path: Path) -> Path:
    path = path.resolve()
    current = Path(path.anchor) if path.is_absolute() else Path(".")
    parts = path.parts[1:] if path.is_absolute() else path.parts
    for part in parts:
        current = current / part
        for _attempt in range(3):
            if current.exists():
                if current.is_dir():
                    break
                backup = current.with_name(f"{current.name}.blocked_{time.strftime('%Y%m%d_%H%M%S')}")
                current.replace(backup)
                print(f"已将阻塞目录创建的文件移到 {backup}", file=sys.stderr, flush=True)
            try:
                current.mkdir()
                break
            except FileExistsError:
                if current.is_dir():
                    break
                if _attempt == 2:
                    raise
                time.sleep(0.05)
    return path


def _port_owner_hint(port: int) -> str:
    checks = [
        ["bash", "-lc", f"ss -ltnp 2>/dev/null | grep -E ':{int(port)}[[:space:]]' || true"],
        ["netstat", "-ano"],
    ]
    for cmd in checks:
        try:
            completed = subprocess.run(cmd, text=True, capture_output=True, timeout=2)
        except Exception:
            continue
        output = (completed.stdout or "").strip()
        if not output:
            continue
        if cmd[0] == "netstat":
            lines = [line for line in output.splitlines() if f":{port} " in line or f":{port}\t" in line]
            output = "\n".join(lines).strip()
        if output:
            return output
    return ""


def _radio_process_env(base: dict[str, str] | None = None) -> dict[str, str]:
    env = dict(base or os.environ)
    ros_log_dir = env.get("ROS_LOG_DIR", "")
    ros_home = env.get("ROS_HOME", "")

    def bad_ros_dir(value: str) -> bool:
        normalized = value.rstrip("/\\").replace("\\", "/")
        return normalized in {"", str(LOG_ROOT).replace("\\", "/")} or normalized.endswith("/rm_radio_ws/log")

    if bad_ros_dir(ros_log_dir):
        env["ROS_LOG_DIR"] = "/tmp/ros_log"
    if bad_ros_dir(ros_home):
        env["ROS_HOME"] = "/tmp/ros_home"
    return env
HP_KEYS = [
    "opponent_hero",
    "opponent_engineer",
    "opponent_infantry_3",
    "opponent_infantry_4",
    "reserved",
    "opponent_sentry",
]
BULLET_KEYS = [
    "opponent_hero",
    "opponent_infantry_3",
    "opponent_infantry_4",
    "opponent_aerial",
    "opponent_sentry",
]
BUFF_KEYS = [
    "opponent_hero",
    "opponent_engineer",
    "opponent_infantry_3",
    "opponent_infantry_4",
    "opponent_sentry",
]
SIDE_LABELS = {"red": "红方", "blue": "蓝方"}
WAVE_LABELS = {"broadcast": "信息波", "interference": "干扰波"}
ROBOT_LABELS = {
    "opponent_hero": "英雄",
    "opponent_engineer": "工程",
    "opponent_infantry_3": "步兵3",
    "opponent_infantry_4": "步兵4",
    "opponent_aerial": "空中",
    "opponent_sentry": "哨兵",
    "reserved": "保留",
}
GROUP_LABELS = {"hp": "血量", "bullets": "发弹量"}


def _json_dumps(payload: Any) -> bytes:
    return json.dumps(payload, ensure_ascii=False, indent=2).encode("utf-8")


def _compact_json(payload: Any) -> str:
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))


def _ros_string_override(name: str, value: Any) -> str:
    """Keep URI-like ROS 2 string overrides valid YAML, including ``usb:``."""
    return f"{name}:={json.dumps(str(value), ensure_ascii=False)}"


def _generated_tx_ros_args(
    plan: dict[str, Any],
    uri: str,
    *,
    node_name: str | None = None,
) -> tuple[list[str], str]:
    """Build explicit ROS inputs without changing the generated TX flowgraph."""
    setters_json = _compact_json(plan["tx_setters"])
    flowgraph_dir = WS_ROOT / "src/rm_radio_ros/flowgraphs/ganraoyuan"
    args = ["--ros-args"]
    if node_name:
        args.extend(["-r", f"__node:={node_name}"])
    args.extend(
        [
            "-p", "flowgraph_name:=ganraoyuan",
            "-p", f"flowgraph_dir:={flowgraph_dir}",
            "-p", "rx_only:=false",
            "-p", "qt_platform:=offscreen",
            "-p", "disable_gui_sinks:=true",
            "-p", _ros_string_override("interference_tx_uri", uri),
            "-p", f"sample_rate:={float(plan['modulation']['sample_rate'])}",
            "-p", _ros_string_override("setters_json", setters_json),
            "-p", f"radio_side:={plan['side']}",
            "-p", f"interference_level:={plan['level']}",
        ]
    )
    return args, setters_json


def _now_token() -> str:
    return time.strftime("%Y%m%d_%H%M%S")


def _bool_env(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def _as_dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _as_list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


def _int_value(value: Any, default: int = 0, low: int = 0, high: int = 0xFFFF) -> int:
    try:
        parsed = int(str(value), 0)
    except Exception:
        parsed = default
    return max(low, min(high, parsed))


def _float_value(value: Any, default: float = 0.0, low: float | None = None, high: float | None = None) -> float:
    try:
        parsed = float(value)
    except Exception:
        parsed = default
    if low is not None:
        parsed = max(float(low), parsed)
    if high is not None:
        parsed = min(float(high), parsed)
    return parsed


def _side_label(side: str) -> str:
    return SIDE_LABELS.get(str(side), str(side))


def _wave_label(wave: str) -> str:
    return WAVE_LABELS.get(str(wave), str(wave))


def _side(state: dict[str, Any]) -> str:
    side = str(state.get("side") or os.environ.get("RM_RADIO_SIDE", "blue")).strip().lower()
    if side not in {"red", "blue"}:
        raise ValueError(f"阵营必须是 red 或 blue，当前为 {side!r}")
    return side


def _wave(state: dict[str, Any]) -> str:
    wave = str(state.get("wave") or "broadcast").strip().lower()
    aliases = {"information": "broadcast", "info": "broadcast", "interference": "interference"}
    wave = aliases.get(wave, wave)
    if wave not in {"broadcast", "interference"}:
        raise ValueError(f"信号类型必须是 broadcast 或 interference，当前为 {wave!r}")
    return wave


def _level(state: dict[str, Any]) -> int:
    level = _int_value(state.get("level", os.environ.get("INTERFERENCE_LEVEL", 1)), default=1, low=1, high=3)
    if level not in (1, 2, 3):
        raise ValueError(f"干扰等级必须是 1、2 或 3，当前为 {level!r}")
    return level


def _password(state: dict[str, Any]) -> str:
    password = str(state.get("password") or os.environ.get("INTERFERENCE_PASSWORD", "R1L001")).strip()
    if not re.fullmatch(r"[A-Za-z0-9]{6}", password):
        raise ValueError("干扰波密钥必须是 6 位 ASCII 字母或数字")
    return password


def _broadcast_config(state: dict[str, Any]) -> dict[str, Any]:
    incoming = _as_dict(state.get("broadcast"))
    cfg = _deep_merge(_default_broadcast_config(), incoming)
    enabled = cfg.get("enabled_cmds")
    if not isinstance(enabled, list) or not enabled:
        enabled = list(RADAR_BROADCAST_COMMANDS)
    enabled_set = {str(item).strip().upper() for item in enabled}
    cfg["enabled_cmds"] = [
        command for command in RADAR_BROADCAST_COMMANDS
        if command.upper() in enabled_set
    ] or list(RADAR_BROADCAST_COMMANDS)
    cfg["auto_change_data"] = cfg.get("auto_change_data") is True

    robots = _as_dict(cfg.get("robots"))
    cfg["robots"] = {
        key: {
            "x": _rule_clamp_int(_as_dict(robots.get(key)).get("x"), 0, POSITION_X_MAX_CM),
            "y": _rule_clamp_int(_as_dict(robots.get(key)).get("y"), 0, POSITION_Y_MAX_CM),
        }
        for key in ROBOT_KEYS
    }

    hp = _as_dict(cfg.get("hp"))
    cfg["hp"] = {
        key: _rule_clamp_int(hp.get(key), 0, HP_MAX[key])
        for key in HP_KEYS
    }
    bullets = _as_dict(cfg.get("bullets"))
    cfg["bullets"] = {
        key: _rule_clamp_int(bullets.get(key), 0, BULLET_MAX[key])
        for key in BULLET_KEYS
    }

    macro = _as_dict(cfg.get("macro"))
    total_coins = _rule_clamp_int(macro.get("total_coins"), 0, 0xFFFF)
    cfg["macro"] = {
        "remaining_coins": _rule_clamp_int(macro.get("remaining_coins"), 0, total_coins),
        "total_coins": total_coins,
        "occupation_bits": normalize_occupation_bits(macro.get("occupation_bits")),
    }

    buffs = _as_dict(cfg.get("buffs"))
    cfg["buffs"] = {
        key: {
            field: _rule_clamp_int(
                _as_dict(buffs.get(key)).get(field),
                0,
                limits[key],
            )
            for field, limits in BUFF_FIELD_MAX.items()
        }
        for key in BUFF_KEYS
    }
    cfg["sentry_mode"] = _rule_clamp_int(cfg.get("sentry_mode"), 1, 6, 2)
    main_status = _as_dict(cfg.get("robot_main_status"))
    cfg["robot_main_status"] = {
        key: _rule_clamp_int(main_status.get(key), 0, 3)
        for key in BUFF_KEYS
    }
    # Do not let the legacy raw-byte escape hatch bypass the structured
    # protocol constraints in Lab TX.
    cfg.pop("buff_raw", None)
    return cfg


def _broadcast_command_cycle(config: dict[str, Any]) -> list[dict[str, Any]]:
    enabled = {str(item).upper() for item in _as_list(config.get("enabled_cmds"))}
    auto_change_data = config.get("auto_change_data") is True
    specs = [
        ("0X0A01", [0x0A, 0x01], _position_payload(config)),
        ("0X0A02", [0x0A, 0x02], _hp_payload(config)),
        ("0X0A03", [0x0A, 0x03], _bullet_payload(config)),
        ("0X0A04", [0x0A, 0x04], _macro_payload(config)),
        ("0X0A05", [0x0A, 0x05], _buff_payload(config)),
    ]
    command_cycle = []
    for key, cmd_id, payload in specs:
        if key not in enabled:
            continue
        if auto_change_data:
            command_cycle.append(
                {
                    "cmd_id": cmd_id,
                    "payload_size": len(payload),
                    "randomize_payload": True,
                }
            )
        else:
            command_cycle.append({"cmd_id": cmd_id, "payload_data": list(payload)})
    return command_cycle


def _apply_bit_noise(data: bytes, bit_error_rate: float, seed: int) -> tuple[bytes, int]:
    rate = max(0.0, min(float(bit_error_rate), 0.5))
    if rate <= 0.0:
        return data, 0
    rng = random.Random(int(seed))
    bits = bytes_to_bits(data)
    changed = 0
    for index, bit in enumerate(bits):
        if rng.random() < rate:
            bits[index] = 1 - int(bit)
            changed += 1
    return bits_to_bytes(bits), changed


def _frame_summaries(frames: list[Any]) -> list[dict[str, Any]]:
    out = []
    for frame in frames:
        data = frame.to_dict()
        out.append(
            {
                "cmd_hex": data.get("cmd_hex"),
                "command": data.get("command"),
                "seq": data.get("seq"),
                "data_length": data.get("data_length"),
                "data_hex": data.get("data_hex"),
                "raw_hex": data.get("raw_hex"),
                "parsed": data.get("parsed"),
            }
        )
    return out


def _decode_packets(access_name: str, packets: list[bytes], state: dict[str, Any]) -> dict[str, Any]:
    virtual = _as_dict(state.get("virtual"))
    ber = _float_value(virtual.get("bit_error_rate", 0.0), default=0.0, low=0.0, high=0.5)
    seed = _int_value(virtual.get("random_seed", 2026), default=2026, low=0, high=2**31 - 1)
    threshold = _int_value(virtual.get("max_access_hamming", 3), default=3, low=0, high=16)
    result = _stream_decode(
        access_name,
        packets,
        bit_error_rate=ber,
        random_seed=seed,
        max_access_hamming=threshold,
    )
    return {
        "air_payload_count": result.air_payload_count,
        "referee_frame_count": len(result.frames),
        "bit_count": result.bit_count,
        "bit_error_count": result.bit_error_count,
        "frames": _frame_summaries(result.frames),
        "bridge_outputs": result.bridge_outputs,
    }


def _rf_plan(side: str, wave: str, level: int) -> dict[str, Any]:
    if wave == "broadcast":
        return {
            "center_hz": BROADCAST_FREQUENCIES[side],
            "bandwidth_hz": BROADCAST_BW_HZ,
            "official_power_dbm": OFFICIAL_POWER_DBM["broadcast"],
            "access_name": "broadcast",
            "access_hex": ACCESS_CODES["broadcast"].hex(" ").upper(),
        }
    setters = interference_setters_for_side_level(side, level)
    return {
        "center_hz": setters["center_f"],
        "bandwidth_hz": setters["BW_ganrao"],
        "official_power_dbm": OFFICIAL_POWER_DBM["interference"],
        "access_name": "interference",
        "access_hex": ACCESS_CODES["interference"].hex(" ").upper(),
    }


def build_plan(state: dict[str, Any]) -> dict[str, Any]:
    side = _side(state)
    wave = _wave(state)
    level = _level(state)
    password = _password(state)
    rf = _rf_plan(side, wave, level)
    if wave == "broadcast":
        broadcast = _broadcast_config(state)
        packets = _build_broadcast_air_packets(broadcast)
        command_cycle = _broadcast_command_cycle(broadcast)
    else:
        broadcast = None
        packets = _build_interference_air_packets(level, password=password)
        command_cycle = []

    clean_decode = _stream_decode(rf["access_name"], packets, max_access_hamming=3)
    loopback = _decode_packets(rf["access_name"], packets, state)
    # The 2026 waveform is fixed at 1 MS/s and SPS=47. Lab TX emulates the
    # official source, so an arbitrary experimental rate must not leak into
    # an actual transmission.
    sample_rate = DEFAULT_TX_SAMPLE_RATE
    sps = 47
    return {
        "side": side,
        "wave": wave,
        "level": level,
        "password": password,
        "rf": rf,
        "air": {
            "access_hex": rf["access_hex"],
            "header_hex": AIR_HEADER_HEX.upper(),
            "payload_bytes": 15,
            "packet_bytes": 27,
            "packet_count": len(packets),
            "stream_bytes_per_second": 1400 if wave == "broadcast" else 1350,
            "clean_packet_hex_samples": [packet.hex(" ").upper() for packet in packets[:8]],
        },
        "modulation": {
            "type": "2-GFSK",
            "bt": 0.35,
            "sample_rate": sample_rate,
            "sps": sps,
            "official_equivalent_sample_rate": 1_000_000,
            "official_equivalent_sps": 47,
        },
        "clean_decode": {
            "air_payload_count": clean_decode.air_payload_count,
            "referee_frame_count": len(clean_decode.frames),
            "frames": _frame_summaries(clean_decode.frames),
            "bridge_outputs": clean_decode.bridge_outputs,
        },
        "loopback": loopback,
        "tx_setters": build_tx_setters(state, command_cycle=command_cycle),
        "broadcast_config": broadcast,
    }


def build_tx_setters(state: dict[str, Any], command_cycle: list[dict[str, Any]] | None = None) -> dict[str, Any]:
    side = _side(state)
    wave = _wave(state)
    level = _level(state)
    tx = _as_dict(state.get("tx"))
    default_attenuation = DEFAULT_TX_ATTENUATION_DB[wave]
    attenuation = _float_value(
        tx.get("attenuation_db", os.environ.get("INTERFERENCE_TX_ATTEND_GR", default_attenuation)),
        default=default_attenuation,
        low=0.0,
        high=89.75,
    )
    rf = _rf_plan(side, wave, level)
    base = {
        "center_f": rf["center_hz"],
        "BW_ganrao": rf["bandwidth_hz"],
        "attend_gr": attenuation,
        "access": list(ACCESS_CODES[rf["access_name"]]),
        "Period": 100.0,
    }
    if wave == "broadcast":
        if command_cycle is None:
            command_cycle = _broadcast_command_cycle(_broadcast_config(state))
        base["command_cycle"] = command_cycle
        return base
    password_bytes = list(_password(state).encode("ascii"))
    base.update({"cmd_id": [0x0A, 0x06], "payload_data": password_bytes})
    return base


def _default_four_sdr() -> dict[str, Any]:
    shared_tx_sample_rate = DEFAULT_TX_SAMPLE_RATE
    return {
        "broadcast_tx": {
            "uri": os.environ.get("BROADCAST_TX_URI", "ip:192.168.2.1"),
            "rf_port": os.environ.get("BROADCAST_TX_RF_PORT", os.environ.get("TX_RF_PORT", "A")),
            "attenuation_db": _float_value(
                os.environ.get("BROADCAST_TX_ATTENUATION_DB", DEFAULT_TX_ATTENUATION_DB["broadcast"]),
                DEFAULT_TX_ATTENUATION_DB["broadcast"],
                0.0,
                89.75,
            ),
            "sample_rate": shared_tx_sample_rate,
        },
        "interference_tx": {
            "uri": os.environ.get("INTERFERENCE_TX_URI", ""),
            "rf_port": os.environ.get("INTERFERENCE_TX_RF_PORT", os.environ.get("TX_RF_PORT", "A")),
            "attenuation_db": _float_value(
                os.environ.get("INTERFERENCE_TX_ATTEND_GR", DEFAULT_TX_ATTENUATION_DB["interference"]),
                DEFAULT_TX_ATTENUATION_DB["interference"],
                0.0,
                89.75,
            ),
            "sample_rate": shared_tx_sample_rate,
        },
    }


def _four_sdr_config(state: dict[str, Any]) -> dict[str, Any]:
    config = _default_four_sdr()
    incoming = _as_dict(state.get("four_sdr"))
    for role, defaults in list(config.items()):
        item = dict(defaults)
        item.update(_as_dict(incoming.get(role)))
        default_attenuation = DEFAULT_TX_ATTENUATION_DB["broadcast" if role == "broadcast_tx" else "interference"]
        item["attenuation_db"] = _float_value(item.get("attenuation_db", default_attenuation), default_attenuation, 0.0, 89.75)
        item["sample_rate"] = DEFAULT_TX_SAMPLE_RATE
        config[role] = item
    return config


def _tx_state_for_role(state: dict[str, Any], role: str, wave: str) -> dict[str, Any]:
    config = _four_sdr_config(state)
    device = _as_dict(config.get(role))
    tx = dict(_as_dict(state.get("tx")))
    default_attenuation = DEFAULT_TX_ATTENUATION_DB[wave]
    tx.update(
        {
            "uri": str(device.get("uri") or tx.get("uri") or "").strip(),
            "rf_port": str(device.get("rf_port") or tx.get("rf_port") or "A").strip(),
            "attenuation_db": _float_value(
                device.get("attenuation_db", tx.get("attenuation_db", default_attenuation)),
                default_attenuation,
                0.0,
                89.75,
            ),
            "sample_rate": DEFAULT_TX_SAMPLE_RATE,
        }
    )
    out = dict(state)
    out["wave"] = wave
    out["tx"] = tx
    return out


def _four_sdr_topology_checks(state: dict[str, Any]) -> tuple[list[str], list[str]]:
    config = _four_sdr_config(state)
    errors: list[str] = []
    warnings: list[str] = []
    broadcast_uri = str(config["broadcast_tx"].get("uri") or "").strip()
    interference_uri = str(config["interference_tx"].get("uri") or "").strip()
    if not broadcast_uri:
        errors.append("信息波 TX URI 为空")
    if not interference_uri:
        warnings.append("干扰波 TX 当前未连接，等待单独接入后识别")
    if broadcast_uri and interference_uri and broadcast_uri == interference_uri:
        errors.append("信息波 TX 与干扰波 TX 必须使用两台不同 SDR")
    tx_attenuations = []
    if broadcast_uri:
        tx_attenuations.append(
            _float_value(
                config["broadcast_tx"].get("attenuation_db", DEFAULT_TX_ATTENUATION_DB["broadcast"]),
                DEFAULT_TX_ATTENUATION_DB["broadcast"],
                0.0,
                89.75,
            )
        )
    if interference_uri:
        tx_attenuations.append(
            _float_value(
                config["interference_tx"].get("attenuation_db", DEFAULT_TX_ATTENUATION_DB["interference"]),
                DEFAULT_TX_ATTENUATION_DB["interference"],
                0.0,
                89.75,
            )
        )
    if any(value < 30 for value in tx_attenuations):
        warnings.append("已配置的 TX 中存在衰减低于 30 dB 的通道，请确认台架射频安全")
    return errors, warnings


def _four_sdr_availability_errors(check_result: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    for role in FOUR_SDR_ROLE_LABELS:
        item = _as_dict(check_result.get(role))
        if item.get("available") is True:
            continue
        label = item.get("role_label") or FOUR_SDR_ROLE_LABELS.get(role, role)
        uri = item.get("uri") or item.get("reported_uri") or ""
        message = str(item.get("message") or "不可用").strip()
        errors.append(f"{label}({uri}) 不可用：{message}")
    topology = _as_dict(check_result.get("topology"))
    errors.extend(str(item) for item in _as_list(topology.get("errors")) if str(item).strip())
    return errors


def build_dual_plans(state: dict[str, Any]) -> dict[str, Any]:
    broadcast_state = _tx_state_for_role(state, "broadcast_tx", "broadcast")
    interference_state = _tx_state_for_role(state, "interference_tx", "interference")
    return {
        "broadcast": build_plan(broadcast_state),
        "interference": build_plan(interference_state),
        "four_sdr": _four_sdr_config(state),
    }


def _field_checks(state: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    broadcast = _as_dict(state.get("broadcast"))

    def parsed_int(label: str, value: Any) -> int | None:
        try:
            return int(str(value), 0)
        except Exception:
            errors.append(f"{label} 必须是整数")
            return None

    enabled = broadcast.get("enabled_cmds")
    if isinstance(enabled, list):
        unknown = [
            str(item) for item in enabled
            if str(item).strip().upper() not in {cmd.upper() for cmd in RADAR_BROADCAST_COMMANDS}
        ]
        if unknown:
            errors.append(f"信息波只允许 0x0A01～0x0A05，发现：{', '.join(unknown)}")

    robots = _as_dict(broadcast.get("robots"))
    for robot in ROBOT_KEYS:
        item = _as_dict(robots.get(robot))
        for axis, maximum in (("x", POSITION_X_MAX_CM), ("y", POSITION_Y_MAX_CM)):
            value = item.get(axis, 0)
            label = f"{ROBOT_LABELS.get(robot, robot)} {axis.upper()}"
            parsed = parsed_int(label, value)
            if parsed is None:
                continue
            if not 0 <= parsed <= maximum:
                errors.append(f"{label} 必须在 0..{maximum} 范围内")

    for group_name, keys, maxima in (
        ("hp", HP_KEYS, HP_MAX),
        ("bullets", BULLET_KEYS, BULLET_MAX),
    ):
        group = _as_dict(broadcast.get(group_name))
        for key in keys:
            value = group.get(key, 0)
            label = f"{GROUP_LABELS.get(group_name, group_name)} {ROBOT_LABELS.get(key, key)}"
            parsed = parsed_int(label, value)
            if parsed is None:
                continue
            maximum = maxima[key]
            if not 0 <= parsed <= maximum:
                errors.append(f"{label} 必须在 0..{maximum} 范围内")

    macro = _as_dict(broadcast.get("macro"))
    total = parsed_int("累计金币", macro.get("total_coins", 0))
    remaining = parsed_int("剩余金币", macro.get("remaining_coins", 0))
    if total is not None and not 0 <= total <= 0xFFFF:
        errors.append("累计金币必须在 0..65535 范围内")
    if remaining is not None:
        remaining_max = total if total is not None and 0 <= total <= 0xFFFF else 0xFFFF
        if not 0 <= remaining <= remaining_max:
            errors.append(f"剩余金币必须在 0..{remaining_max} 范围内，且不能超过累计金币")
    occupation = parsed_int("场地状态位图", macro.get("occupation_bits", 0))
    if occupation is not None and occupation != normalize_occupation_bits(occupation):
        errors.append("场地状态位图含保留位或未定义枚举，只允许协议定义的 bit 0..15 状态")

    buffs = _as_dict(broadcast.get("buffs"))
    buff_labels = {
        "hp_recovery_percent": "回血增益",
        "shooting_heat_cooling": "热量冷却增益",
        "defense_percent": "防御增益",
        "negative_defense_percent": "易伤增益",
        "attack_percent": "攻击增益",
    }
    for key in BUFF_KEYS:
        item = _as_dict(buffs.get(key))
        for field, maxima in BUFF_FIELD_MAX.items():
            label = f"{ROBOT_LABELS.get(key, key)} {buff_labels[field]}"
            parsed = parsed_int(label, item.get(field, 0))
            if parsed is not None and not 0 <= parsed <= maxima[key]:
                errors.append(f"{label} 必须在 0..{maxima[key]} 范围内")

    sentry_mode = parsed_int("哨兵姿态", broadcast.get("sentry_mode", 2))
    if sentry_mode is not None and not 1 <= sentry_mode <= 6:
        errors.append("哨兵姿态必须在 1..6 范围内")
    main_status = _as_dict(broadcast.get("robot_main_status"))
    for key in BUFF_KEYS:
        label = f"{ROBOT_LABELS.get(key, key)}主要状态"
        status = parsed_int(label, main_status.get(key, 0))
        if status is not None and not 0 <= status <= 3:
            errors.append(f"{label}必须在 0..3 范围内")
    return errors


def validate_state(state: dict[str, Any], lab_tx_effective_confirm: bool | None = None) -> dict[str, Any]:
    errors = _field_checks(state)
    warnings: list[str] = []
    ok: list[str] = []
    plan: dict[str, Any] | None = None
    dual_plan: dict[str, Any] | None = None
    topology_errors, topology_warnings = _four_sdr_topology_checks(state)
    errors.extend(topology_errors)
    warnings.extend(topology_warnings)
    try:
        plan = build_plan(state)
    except Exception as exc:
        errors.append(str(exc))
    interference_uri = str(_four_sdr_config(state)["interference_tx"].get("uri") or "").strip()
    if interference_uri:
        try:
            dual_plan = build_dual_plans(state)
        except Exception as exc:
            errors.append(f"双路 TX 计划失败：{exc}")

    if plan is not None:
        frames = plan["clean_decode"]["frames"]
        expected = ["0x0A06"] if plan["wave"] == "interference" else list(_as_list(plan["broadcast_config"].get("enabled_cmds")))
        actual = [str(frame.get("cmd_hex")) for frame in frames]
        expected_upper = [cmd.upper().replace("X", "x") for cmd in expected]
        if [cmd.upper().replace("X", "x") for cmd in actual] != expected_upper:
            errors.append(f"解码命令序列不匹配：期望 {expected}，实际 {actual}")
        else:
            ok.append("0xA5 裁判帧 CRC8/CRC16 校验通过")
        if all(len(bytes.fromhex(sample.replace(" ", ""))) == 27 for sample in plan["air"]["clean_packet_hex_samples"]):
            ok.append("空口包结构为 8B Access Code + 00 0F 00 0F + 15B payload")
        ok.append(f"{_side_label(plan['side'])}{_wave_label(plan['wave'])}频点/带宽符合 RM2026 表")
        if plan["modulation"]["sample_rate"] == 1_000_000 and plan["modulation"]["sps"] == 47:
            ok.append("TX 使用官方 1 MS/s、SPS=47、BT=0.35")
        warnings.append("绝对 RF 功率不能只靠 SDR 衰减判断；dBm 合规需要功率计或频谱仪校准")
        warnings.append("LabTX 仅限实验室使用；比赛启动链路必须保持 RX-only")

    tx = _as_dict(state.get("tx"))
    attenuation = _float_value(tx.get("attenuation_db", 60), 60.0, 0.0, 89.75)
    if attenuation < 30:
        warnings.append("TX 衰减低于 30 dB，台架测试风险较高")
    if str(state.get("mode", "virtual")) == "actual_tx":
        effective_confirm = _bool_env("RM_RADIO_LAB_TX_ANTENNA_CONFIRM", False)
        if lab_tx_effective_confirm is not None:
            effective_confirm = bool(lab_tx_effective_confirm)
        if not effective_confirm:
            warnings.append("实际发射需要先在页面建立本次实验室安全确认，或启动前设置 RM_RADIO_LAB_TX_ANTENNA_CONFIRM=true")
        if not bool(tx.get("lab_confirm")):
            warnings.append("实际发射还需要勾选页面里的实验室安全确认")

    if dual_plan is not None:
        ok.append("双 SDR 发射拓扑已生成：Z103 信息波 TX、Pluto 干扰波 TX")
    else:
        ok.append("当前为单信息波 TX 模式；干扰波 TX 等待接入识别")

    return {
        "ok": not errors,
        "errors": errors,
        "warnings": warnings,
        "passes": ok,
        "plan": plan,
        "dual_plan": dual_plan,
    }


def generate_files(state: dict[str, Any]) -> dict[str, Any]:
    plan = build_plan(state)
    access_name = plan["rf"]["access_name"]
    if access_name == "broadcast":
        packets = _build_broadcast_air_packets(plan["broadcast_config"])
    else:
        packets = _build_interference_air_packets(plan["level"], password=plan["password"])
    clean_bytes = b"".join(packets)
    virtual = _as_dict(state.get("virtual"))
    ber = _float_value(virtual.get("bit_error_rate", 0.0), default=0.0, low=0.0, high=0.5)
    seed = _int_value(virtual.get("random_seed", 2026), default=2026, low=0, high=2**31 - 1)
    noisy_bytes, changed_bits = _apply_bit_noise(clean_bytes, ber, seed)

    out_dir = _ensure_dir(GENERATED_ROOT / f"{_now_token()}_{plan['side']}_{plan['wave']}")
    files = {
        "config": out_dir / "tx_config.json",
        "plan": out_dir / "plan.json",
        "clean_bin": out_dir / "air_packets_clean.bin",
        "clean_hex": out_dir / "air_packets_clean.hex",
        "noisy_bin": out_dir / "air_packets_noisy.bin",
        "noisy_hex": out_dir / "air_packets_noisy.hex",
        "summary": out_dir / "summary.md",
    }
    files["config"].write_bytes(_json_dumps(state))
    files["plan"].write_bytes(_json_dumps(plan))
    files["clean_bin"].write_bytes(clean_bytes)
    files["clean_hex"].write_text("\n".join(packet.hex(" ").upper() for packet in packets) + "\n", encoding="utf-8")
    files["noisy_bin"].write_bytes(noisy_bytes)
    files["noisy_hex"].write_text(noisy_bytes.hex(" ").upper() + "\n", encoding="utf-8")
    files["summary"].write_text(
        "\n".join(
            [
                "# LabTX 虚拟文件生成",
                "",
                f"- 阵营：`{_side_label(plan['side'])}`",
                f"- 信号：`{_wave_label(plan['wave'])}`",
                f"- 中心频率 Hz：`{plan['rf']['center_hz']}`",
                f"- 带宽 Hz：`{plan['rf']['bandwidth_hz']}`",
                f"- 数据流速率 byte/s：`{plan['air']['stream_bytes_per_second']}`",
                f"- 空口包数量：`{plan['air']['packet_count']}`",
                f"- 解码帧数量：`{plan['clean_decode']['referee_frame_count']}`",
                f"- 比特噪声率：`{ber}`",
                f"- 翻转 bit 数：`{changed_bits}`",
                "",
            ]
        ),
        encoding="utf-8",
    )
    return {
        "directory": str(out_dir),
        "changed_bits": changed_bits,
        "files": {name: str(path) for name, path in files.items()},
        "plan": plan,
    }


def check_sdr_uri(uri: str, timeout_sec: float = 6.0) -> dict[str, Any]:
    clean_uri = str(uri or "").strip()
    if not clean_uri:
        return {"uri": clean_uri, "available": False, "message": "URI 为空"}
    with IIO_CHECK_LOCK:
        try:
            completed = subprocess.run(
                ["iio_info", "-u", clean_uri],
                cwd=str(WS_ROOT),
                text=True,
                capture_output=True,
                timeout=max(1.0, float(timeout_sec)),
            )
        except FileNotFoundError:
            return {"uri": clean_uri, "available": None, "message": "未找到 iio_info，请安装 libiio-utils"}
        except subprocess.TimeoutExpired:
            return {"uri": clean_uri, "available": False, "message": f"iio_info 超时：{clean_uri}"}

    output = "\n".join(part for part in (completed.stdout.strip(), completed.stderr.strip()) if part)
    summary = output.splitlines()[:24]
    if completed.returncode == 0:
        backend = "unknown"
        for line in output.splitlines():
            if "IIO context created with" in line:
                backend = line.strip()
                break
        return {
            "uri": clean_uri,
            "available": True,
            "message": backend,
            "summary": summary,
        }
    hint = ""
    if clean_uri.startswith("usb:"):
        hint = "；若是 Pluto/Z103，请确认 usbipd 已 attach 到 WSL；仅连接一台 USB IIO 设备时可使用稳定 URI usb:"
    elif clean_uri.startswith("ip:"):
        hint = "；请确认设备 IP、网段和 iiod 服务"
    return {
        "uri": clean_uri,
        "available": False,
        "message": (output or f"iio_info 返回 {completed.returncode}") + hint,
        "summary": summary,
    }


def _busy_sdr_result(uri: str) -> dict[str, Any]:
    return {
        "uri": str(uri or "").strip(),
        "available": None,
        "busy": True,
        "message": "TX 正在使用该 SDR，Dashboard 已跳过 IIO 探测",
        "summary": [],
    }


def _summary_value(item: dict[str, Any], key: str) -> str:
    prefix = f"{key}:"
    for line in _as_list(item.get("summary")):
        text = str(line).strip()
        if text.startswith(prefix):
            return text.split(":", 1)[1].strip()
    return ""


def _device_family(item: dict[str, Any]) -> str:
    model = _summary_value(item, "hw_model")
    if "ANTSDR" in model:
        return "e310"
    if "PlutoSDR" in model:
        return "z103"
    return "unknown"


def _annotate_sdr_role(role: str, item: dict[str, Any]) -> dict[str, Any]:
    out = dict(item)
    out["role"] = role
    out["role_label"] = FOUR_SDR_ROLE_LABELS.get(role, role)
    out["hw_model"] = _summary_value(out, "hw_model")
    out["hw_serial"] = _summary_value(out, "hw_serial")
    out["reported_uri"] = _summary_value(out, "uri") or str(out.get("uri") or "")
    family = _device_family(out)
    out["device_family"] = family
    if out.get("available") is not True:
        out["role_match"] = None
        return out
    expected = "e310" if role.endswith("_rx") else "z103"
    out["expected_family"] = expected
    out["role_match"] = family in {expected, "unknown"}
    if family != "unknown" and family != expected:
        out["available"] = False
        out["message"] = f"{out.get('message', '')}；角色设备类型不匹配：期望 {expected.upper()}，实际 {family.upper()}".strip("；")
    return out


def _tx_log_diagnostics(log_path: Path | None, max_bytes: int = 10_000_000) -> dict[str, Any]:
    """Extract gr-iio DAC-underflow markers without opening the live SDR."""
    result: dict[str, Any] = {
        "sample_supply_mode": "scheduler_clocked_continuous_bits",
        "underflow_available": False,
        "underflow_count": 0,
        "underflow_detected": False,
        "log_truncated": False,
    }
    if log_path is None:
        return result
    path = Path(log_path)
    result["log"] = str(path)
    try:
        stat = path.stat()
        size = int(stat.st_size)
        start = max(0, size - max(1, int(max_bytes)))
        with path.open("rb") as handle:
            if start:
                handle.seek(start)
                # Discard the potentially partial first line so a normal word
                # beginning with U cannot be mistaken for a marker run.
                handle.readline()
            data = handle.read()
        # gr-iio writes one literal "U" per DAC underflow, without a newline.
        # Its markers therefore appear as a run at the start of an output line,
        # sometimes immediately before another library log message.
        runs = re.findall(rb"(?m)^U+", data)
        count = sum(len(run) for run in runs)
        result.update(
            {
                "underflow_available": True,
                "underflow_count": int(count),
                "underflow_detected": bool(count),
                "log_bytes": size,
                "log_mtime": float(stat.st_mtime),
                "log_truncated": bool(start),
            }
        )
    except OSError as exc:
        result["error"] = str(exc)
    return result


def _update_underflow_observation(
    diagnostics: dict[str, Any],
    observation: dict[str, Any],
    *,
    running: bool,
    now_monotonic: float | None = None,
    stable_after_sec: float = 6.0,
) -> dict[str, Any]:
    """Classify a cumulative gr-iio U count as active, stable, or historical.

    gr-iio emits bare ``U`` bytes without timestamps.  A cumulative non-zero
    count alone therefore cannot distinguish a few startup underflows from a
    stream that is still starving.  Dashboard polling gives us that missing
    time dimension: remember when the observed count last increased and only
    show a live fault while it is still changing (or has changed recently).
    """
    result = dict(diagnostics)
    now = time.monotonic() if now_monotonic is None else float(now_monotonic)
    available = bool(result.get("underflow_available"))
    count = max(0, int(result.get("underflow_count") or 0))
    log_identity = str(result.get("log") or "")
    previous_log = str(observation.get("log") or "")
    previous_count = max(0, int(observation.get("count") or 0))

    reset = log_identity != previous_log
    if reset:
        delta = count
        last_change = now if count else None
    elif count != previous_count:
        delta = max(0, count - previous_count)
        last_change = now if count else None
    else:
        delta = 0
        last_change = observation.get("last_change_monotonic")

    observation.update(
        {
            "log": log_identity,
            "count": count,
            "last_change_monotonic": last_change,
        }
    )

    age = None if last_change is None else max(0.0, now - float(last_change))
    stable_after = max(0.1, float(stable_after_sec))
    if not available:
        state = "unavailable"
    elif count == 0:
        state = "clean"
    elif not running:
        state = "historical"
    elif age is not None and age >= stable_after:
        state = "stable"
    else:
        state = "active"

    result.update(
        {
            "underflow_state": state,
            "underflow_count_delta": int(delta),
            "underflow_active": state == "active",
            "underflow_stable": state == "stable",
            "underflow_last_change_age_sec": age,
            "underflow_stable_after_sec": stable_after,
        }
    )
    return result


class RadioTxHttpServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, server_address: tuple[str, int], handler_cls: type[SimpleHTTPRequestHandler]):
        super().__init__(server_address, handler_cls)
        self.tx_lock = threading.RLock()
        self.tx_process: subprocess.Popen | None = None
        self.tx_log_path: Path | None = None
        self.tx_last_returncode: int | None = None
        self.dual_tx_process: subprocess.Popen | None = None
        self.dual_tx_log_path: Path | None = None
        self.dual_tx_last_returncode: int | None = None
        # 独立按路发射：broadcast / interference 各一个进程槽。
        self.wave_processes: dict[str, subprocess.Popen | None] = {"broadcast": None, "interference": None}
        self.wave_log_paths: dict[str, Path | None] = {"broadcast": None, "interference": None}
        self.wave_last_state: dict[str, dict[str, Any] | None] = {"broadcast": None, "interference": None}
        self.wave_last_returncode: dict[str, int | None] = {"broadcast": None, "interference": None}
        self.wave_underflow_observations: dict[str, dict[str, Any]] = {
            "broadcast": {},
            "interference": {},
        }
        self.lab_tx_session_confirm = _bool_env("RM_RADIO_LAB_TX_ANTENNA_CONFIRM", False)
        self.last_tx_state: dict[str, Any] | None = None
        self.last_dual_tx_state: dict[str, Any] | None = None
        self.tx_hardware_cache: dict[str, Any] = {"uris": [], "all_muted": None, "results": [], "cached": True}
        self.tx_hardware_cache_time = 0.0

    def lab_tx_effective_confirm(self) -> bool:
        return _bool_env("RM_RADIO_LAB_TX_ANTENNA_CONFIRM", False) or bool(self.lab_tx_session_confirm)

    def tx_hardware_uris(self) -> list[str]:
        return (
            _tx_mute_uris_from_state(self.last_dual_tx_state or self.last_tx_state or default_state())
            + _tx_mute_uris_from_recent_logs()
        )

    def _busy_tx_targets(self) -> tuple[set[str], set[str]]:
        """Return TX-owned URIs/roles without opening any IIO context."""
        with self.tx_lock:
            busy: set[str] = set()
            busy_roles: set[str] = set()

            def add_uri(value: Any) -> None:
                clean = str(value or "").strip()
                if clean:
                    busy.add(clean)

            def role_uri(role: str, state: dict[str, Any] | None) -> str:
                config = _four_sdr_config(state or default_state())
                return str(_as_dict(config.get(role)).get("uri") or "").strip()

            for item in _running_managed_tx(self):
                kind = str(item.get("kind") or "")
                if kind == "wave":
                    wave = str(item.get("wave") or "")
                    role = "broadcast_tx" if wave == "broadcast" else "interference_tx"
                    busy_roles.add(role)
                    add_uri(item.get("uri"))
                    add_uri(role_uri(role, self.wave_last_state.get(wave)))
                elif kind == "generic":
                    wave = str(_as_dict(self.last_tx_state).get("wave") or "").strip().lower()
                    if wave in {"broadcast", "interference"}:
                        role = "broadcast_tx" if wave == "broadcast" else "interference_tx"
                        busy_roles.add(role)
                        add_uri(role_uri(role, self.last_tx_state))
                    else:
                        busy_roles.update({"broadcast_tx", "interference_tx"})
                    generic_uri = str(
                        _as_dict(_as_dict(self.last_tx_state).get("tx")).get("uri") or ""
                    ).strip()
                    add_uri(generic_uri)
                    if not generic_uri:
                        add_uri(role_uri("broadcast_tx", self.last_tx_state))
                        add_uri(role_uri("interference_tx", self.last_tx_state))
                elif kind == "dual":
                    busy_roles.update({"broadcast_tx", "interference_tx"})
                    add_uri(role_uri("broadcast_tx", self.last_dual_tx_state))
                    add_uri(role_uri("interference_tx", self.last_dual_tx_state))

            fallback_state = self.last_dual_tx_state or self.last_tx_state or default_state()
            for item in _detect_tx_processes():
                explicit_uri = str(item.get("uri") or "").strip()
                role = str(item.get("role") or "unknown")
                if role in {"broadcast_tx", "interference_tx"}:
                    busy_roles.add(role)
                    add_uri(explicit_uri)
                    # Also protect the configured alias for this role (for
                    # example usb: and ip: reaching the same physical Pluto).
                    add_uri(role_uri(role, fallback_state))
                else:
                    add_uri(explicit_uri)
                    busy_roles.update({"broadcast_tx", "interference_tx"})
                    # An unqualified or dual TX process may own either device.
                    # Be conservative: skipping an idle probe is safer than
                    # creating a second context on a transmitting SDR.
                    add_uri(role_uri("broadcast_tx", fallback_state))
                    add_uri(role_uri("interference_tx", fallback_state))
            return busy, busy_roles

    def busy_tx_uris(self) -> set[str]:
        return self._busy_tx_targets()[0]

    def check_sdr_uri_safely(self, uri: str, *, role: str = "") -> dict[str, Any]:
        clean_uri = str(uri or "").strip()
        with self.tx_lock:
            busy_uris, busy_roles = self._busy_tx_targets()
            if clean_uri and (clean_uri in busy_uris or str(role) in busy_roles):
                return _busy_sdr_result(clean_uri)
            return check_sdr_uri(clean_uri)

    def check_four_sdr_safely(self, state: dict[str, Any]) -> dict[str, Any]:
        with self.tx_lock:
            busy_uris, busy_roles = self._busy_tx_targets()
            return check_four_sdr(
                state,
                busy_tx_uris=busy_uris,
                busy_tx_roles=busy_roles,
            )

    def cached_tx_hardware_status(self) -> dict[str, Any]:
        status = dict(self.tx_hardware_cache)
        status["cached"] = True
        status["cache_age_sec"] = max(0.0, time.time() - float(self.tx_hardware_cache_time)) if self.tx_hardware_cache_time else None
        return status

    def refresh_tx_hardware_status(self) -> dict[str, Any]:
        with self.tx_lock:
            uris = sorted({str(uri).strip() for uri in self.tx_hardware_uris() if str(uri).strip()})
            busy_uris = self.busy_tx_uris()
            idle_uris = [uri for uri in uris if uri not in busy_uris]
            status = tx_hardware_status(idle_uris)
            results = list(_as_list(status.get("results")))
            results.extend(
                {
                    **_busy_sdr_result(uri),
                    "ok": None,
                    "muted": None,
                }
                for uri in uris
                if uri in busy_uris
            )
            status.update(
                {
                    "uris": uris,
                    "busy_uris": sorted(uri for uri in uris if uri in busy_uris),
                    "results": sorted(results, key=lambda item: str(_as_dict(item).get("uri") or "")),
                    "all_muted": None if any(uri in busy_uris for uri in uris) else status.get("all_muted"),
                    "cached": False,
                    "cache_age_sec": 0.0,
                }
            )
            self.tx_hardware_cache = dict(status)
            self.tx_hardware_cache_time = time.time()
            return status

    def tx_status(self) -> dict[str, Any]:
        with self.tx_lock:
            proc = self.tx_process
            returncode = self.tx_last_returncode
            stale = False
            if proc is not None and proc.poll() is not None:
                returncode = proc.returncode
                self.tx_last_returncode = returncode
                stale = True
                proc = None
                self.tx_process = None
            external = _detect_tx_processes()
            external_running = proc is None and bool(external)
            return {
                "running": proc is not None,
                "pid": proc.pid if proc is not None else (",".join(str(item["pid"]) for item in external) if external_running else None),
                "returncode": returncode,
                "stale": stale,
                "external": external_running,
                "external_processes": external if external_running else [],
                "log": str(self.tx_log_path) if self.tx_log_path else None,
            }

    def dual_tx_status(self) -> dict[str, Any]:
        with self.tx_lock:
            proc = self.dual_tx_process
            returncode = self.dual_tx_last_returncode
            stale = False
            if proc is not None and proc.poll() is not None:
                returncode = proc.returncode
                self.dual_tx_last_returncode = returncode
                stale = True
                proc = None
                self.dual_tx_process = None
            external = _detect_tx_processes()
            status = {
                "running": proc is not None,
                "pid": proc.pid if proc is not None else None,
                "returncode": returncode,
                "stale": stale,
                "external": False,
                "external_processes": [],
                "log": str(self.dual_tx_log_path) if self.dual_tx_log_path else None,
            }
            if self.dual_tx_log_path:
                log_dir = self.dual_tx_log_path.parent
                status["log_dir"] = str(log_dir)
                status["children"] = _read_dual_tx_pid_status(log_dir)
                if status["children"]:
                    child_running = any(_as_dict(item).get("running") for item in status["children"].values())
                    status["child_running"] = child_running
                    status["running"] = bool(status["running"] or child_running)
                    if child_running and status["pid"] is None:
                        status["pid"] = ",".join(str(_as_dict(item).get("pid")) for item in status["children"].values() if _as_dict(item).get("running"))
            if not status["running"] and external:
                status["running"] = True
                status["external"] = True
                status["external_processes"] = external
                status["pid"] = ",".join(str(item["pid"]) for item in external)
            elif status["running"]:
                status["external_processes"] = external
            return status

    def wave_status(self, wave: str) -> dict[str, Any]:
        with self.tx_lock:
            proc = self.wave_processes.get(wave)
            returncode = self.wave_last_returncode.get(wave)
            stale = False
            if proc is not None and proc.poll() is not None:
                returncode = proc.returncode
                self.wave_last_returncode[wave] = returncode
                stale = True
                proc = None
                self.wave_processes[wave] = None
            role = "broadcast_tx" if wave == "broadcast" else "interference_tx"
            external = [item for item in _detect_tx_processes() if _as_dict(item).get("role") == role]
            external_running = proc is None and bool(external)
            running = proc is not None or external_running
            log_path = self.wave_log_paths.get(wave)
            diagnostics = _update_underflow_observation(
                _tx_log_diagnostics(log_path),
                self.wave_underflow_observations.setdefault(wave, {}),
                running=running,
                stable_after_sec=_float_value(
                    os.environ.get("RM_RADIO_TX_UNDERFLOW_ACTIVE_SEC", 6.0),
                    6.0,
                    0.1,
                    60.0,
                ),
            )
            status = {
                "wave": wave,
                "running": running,
                "managed": proc is not None,
                "pid": proc.pid if proc is not None else (",".join(str(item["pid"]) for item in external) if external_running else None),
                "returncode": returncode,
                "stale": stale,
                "external": external_running,
                "log": str(log_path) if log_path else None,
                "diagnostics": diagnostics,
            }
            if wave == "broadcast":
                last_state = self.wave_last_state.get(wave)
                status["auto_change_data"] = bool(
                    last_state
                    and _broadcast_config(last_state).get("auto_change_data") is True
                )
            return status


def _tx_uri_from_process_command(command: str) -> str:
    text = str(command or "")
    patterns = (
        r"(?:interference_tx_uri|RM_RADIO_INTERFERENCE_TX_URI|INTERFERENCE_TX_URI|BROADCAST_TX_URI)(?::=|=)['\"]?([^\s'\";]+)",
        r"(?:^|\s)-u\s+['\"]?([^\s'\";]+)",
    )
    for pattern in patterns:
        match = re.search(pattern, text)
        if match:
            return match.group(1).strip()
    return ""


def _is_tx_process_command(command: str) -> bool:
    """Match actual TX executables, not shells that merely mention TX text."""
    try:
        tokens = shlex.split(str(command or ""))
    except ValueError:
        return False
    if not tokens:
        return False
    executable = Path(tokens[0]).name
    token_basenames = {Path(token).name for token in tokens[1:] if token and not token.startswith("-")}
    if executable in {"bash", "sh", "dash", "zsh"}:
        return "start_dual_tx.sh" in token_basenames
    if executable in {"iio_writedev", "rm_iq_replay_tx_node", "ganraoyuan_node"}:
        return True
    text = " ".join(tokens)
    if executable == "ros2":
        return any(name in text for name in ("ganraoyuan.launch.py", "rm_iq_replay_tx_node", "ganraoyuan_node"))
    if executable.startswith("python"):
        return (
            "from rm_radio_ros.nodes.ganraoyuan_node import main; main()" in text
            or "ganraoyuan_node.py" in token_basenames
            or "rm_iq_replay_tx_node" in token_basenames
            or "rm_radio_cw_test" in tokens
        )
    return False


def _detect_tx_processes() -> list[dict[str, Any]]:
    needles = (
        "start_dual_tx.sh",
        "rm_broadcast_tx_node",
        "rm_interference_tx_node",
        "ganraoyuan_node",
        "flowgraph_name:=ganraoyuan",
        "iio_writedev",
        "iq_replay_tx_node",
        "rm_radio_cw_test",
    )
    try:
        completed = subprocess.run(
            ["ps", "-eo", "pid=,ppid=,pgid=,stat=,cmd="],
            text=True,
            capture_output=True,
            timeout=3,
        )
    except Exception:
        return []
    if completed.returncode != 0:
        return []
    current_pid = os.getpid()
    out: list[dict[str, Any]] = []
    for line in completed.stdout.splitlines():
        parts = line.strip().split(None, 4)
        if len(parts) < 5:
            continue
        try:
            pid = int(parts[0])
            ppid = int(parts[1])
            pgid = int(parts[2])
        except ValueError:
            continue
        stat = parts[3]
        cmd = parts[4]
        if pid == current_pid or "radio_tx_dashboard.py" in cmd or stat.startswith("Z"):
            continue
        if not _is_tx_process_command(cmd):
            continue
        if not any(needle in cmd for needle in needles):
            continue
        role = "unknown"
        if "rm_broadcast_tx_node" in cmd:
            role = "broadcast_tx"
        elif "rm_interference_tx_node" in cmd or "iq_replay_tx_node" in cmd or "rm_radio_cw_test" in cmd:
            role = "interference_tx"
        elif "start_dual_tx.sh" in cmd:
            role = "dual_tx_launcher"
        elif "ganraoyuan_node" in cmd or "flowgraph_name:=ganraoyuan" in cmd:
            role = "generic_tx"
        out.append(
            {
                "pid": pid,
                "ppid": ppid,
                "pgid": pgid,
                "stat": stat,
                "role": role,
                "uri": _tx_uri_from_process_command(cmd),
                "cmd": cmd[:500],
            }
        )
    return out


def _process_stat(pid: int) -> str | None:
    try:
        completed = subprocess.run(
            ["ps", "-p", str(int(pid)), "-o", "stat="],
            text=True,
            capture_output=True,
            timeout=1,
        )
    except Exception:
        return None
    if completed.returncode != 0:
        return None
    text = completed.stdout.strip()
    return text.split()[0] if text else None


def _process_running(pid: int) -> bool:
    if pid <= 0:
        return False
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    stat = _process_stat(pid)
    if stat and stat.startswith("Z"):
        return False
    return True


def _wait_for_pids_exit(pids: list[int], timeout_sec: float) -> list[int]:
    deadline = time.time() + max(0.0, float(timeout_sec))
    alive = [pid for pid in sorted(set(int(item) for item in pids if int(item) > 0)) if _process_running(pid)]
    while alive and time.time() < deadline:
        time.sleep(0.1)
        alive = [pid for pid in alive if _process_running(pid)]
    return alive


def _signal_tx_processes(pids: list[int], sig: signal.Signals) -> list[int]:
    current_pid = os.getpid()
    current_pgid = os.getpgrp() if hasattr(os, "getpgrp") else None
    signaled: list[int] = []
    used_pgids: set[int] = set()
    for pid in sorted(set(int(item) for item in pids if int(item) > 0)):
        if pid == current_pid or not _process_running(pid):
            continue
        if os.name != "nt":
            try:
                pgid = os.getpgid(pid)
            except ProcessLookupError:
                continue
            except Exception:
                pgid = None
            if pgid and pgid > 0 and pgid != current_pgid:
                if pgid not in used_pgids:
                    try:
                        os.killpg(pgid, sig)
                        used_pgids.add(pgid)
                        signaled.append(pid)
                    except ProcessLookupError:
                        pass
                    except PermissionError:
                        pass
                continue
        try:
            os.kill(pid, sig)
            signaled.append(pid)
        except ProcessLookupError:
            pass
        except PermissionError:
            pass
    return signaled


def _terminate_tx_processes(pids: list[int], *, term_timeout: float = 5.0, kill_timeout: float = 2.0) -> dict[str, Any]:
    clean_pids = sorted(set(int(item) for item in pids if int(item) > 0))
    if not clean_pids:
        return {"requested_pids": [], "sigterm": [], "alive_after_sigterm": [], "sigkill": [], "alive_after_sigkill": []}
    sigterm = _signal_tx_processes(clean_pids, signal.SIGTERM)
    alive_after_term = _wait_for_pids_exit(clean_pids, term_timeout)
    sigkill: list[int] = []
    alive_after_kill: list[int] = alive_after_term
    if alive_after_term:
        sigkill = _signal_tx_processes(alive_after_term, signal.SIGKILL)
        alive_after_kill = _wait_for_pids_exit(alive_after_term, kill_timeout)
    # libusb may need a short beat after the owning process exits before iio_info
    # can claim the same USB interface again.
    time.sleep(0.4)
    return {
        "requested_pids": clean_pids,
        "sigterm": sigterm,
        "alive_after_sigterm": alive_after_term,
        "sigkill": sigkill,
        "alive_after_sigkill": alive_after_kill,
    }


def _read_dual_tx_pid_status(log_dir: Path) -> dict[str, Any]:
    pid_file = log_dir / "dual_tx.pids"
    if not pid_file.exists():
        return {}
    out: dict[str, Any] = {}
    for line in pid_file.read_text(encoding="utf-8", errors="replace").splitlines():
        if "=" not in line:
            continue
        key, raw_pid = line.split("=", 1)
        try:
            pid = int(raw_pid.strip())
        except ValueError:
            continue
        role = key.removesuffix("_pid")
        out[role] = {
            "pid": pid,
            "running": _process_running(pid),
            "stat": _process_stat(pid),
            "log": str(log_dir / f"{role}.log"),
        }
    return out


def _dual_tx_child_pids(log_path: Path | None) -> list[int]:
    if not log_path:
        return []
    children = _read_dual_tx_pid_status(log_path.parent)
    out: list[int] = []
    for item in children.values():
        pid = _as_dict(item).get("pid")
        if pid is not None:
            try:
                out.append(int(pid))
            except (TypeError, ValueError):
                pass
    return out


def _external_tx_pids() -> list[int]:
    out: list[int] = []
    for item in _detect_tx_processes():
        try:
            out.append(int(item["pid"]))
        except (KeyError, TypeError, ValueError):
            pass
    return out


def _running_managed_tx(server: RadioTxHttpServer) -> list[dict[str, Any]]:
    running: list[dict[str, Any]] = []
    slots = (
        ("generic", "tx_process", "tx_last_returncode"),
        ("dual", "dual_tx_process", "dual_tx_last_returncode"),
    )
    for kind, process_attr, returncode_attr in slots:
        proc = getattr(server, process_attr, None)
        if proc is None:
            continue
        try:
            returncode = proc.poll()
        except Exception:
            returncode = None
        if returncode is None:
            running.append({"kind": kind, "pid": getattr(proc, "pid", None)})
            continue
        setattr(server, process_attr, None)
        setattr(server, returncode_attr, returncode)

    for wave in ("broadcast", "interference"):
        proc = _as_dict(getattr(server, "wave_processes", {})).get(wave)
        if proc is None:
            continue
        try:
            returncode = proc.poll()
        except Exception:
            returncode = None
        if returncode is None:
            running.append(
                {
                    "kind": "wave",
                    "wave": wave,
                    "role": "broadcast_tx" if wave == "broadcast" else "interference_tx",
                    "pid": getattr(proc, "pid", None),
                    "uri": _wave_uri_from_state(_as_dict(getattr(server, "wave_last_state", {})).get(wave), wave),
                }
            )
            continue
        getattr(server, "wave_processes", {})[wave] = None
        getattr(server, "wave_last_returncode", {})[wave] = returncode
    return running


def _wave_uri_from_state(state: dict[str, Any] | None, wave: str) -> str:
    if not state:
        return ""
    role = "broadcast_tx" if wave == "broadcast" else "interference_tx"
    return str(_as_dict(_four_sdr_config(state).get(role)).get("uri") or "").strip()


def _tx_conflict_description(item: dict[str, Any]) -> str:
    kind = str(item.get("kind") or "external")
    role = str(item.get("role") or kind)
    pid = item.get("pid")
    uri = str(item.get("uri") or "").strip()
    detail = f"{role} PID={pid}" if pid is not None else role
    return f"{detail} URI={uri}" if uri else detail


def _assert_tx_start_allowed(
    server: RadioTxHttpServer,
    *,
    mode: str,
    wave: str | None = None,
    uri: str = "",
) -> None:
    managed = _running_managed_tx(server)
    external = [dict(item, kind="external") for item in _detect_tx_processes()]
    if mode in {"generic", "dual"}:
        conflicts = managed + external
        if conflicts:
            summary = "；".join(_tx_conflict_description(item) for item in conflicts)
            raise RuntimeError(f"{mode} TX 与任何现有 TX 互斥，拒绝启动：{summary}")
        return

    if mode != "wave" or wave not in {"broadcast", "interference"}:
        raise ValueError(f"未知 TX 启动模式：{mode!r}/{wave!r}")
    uri = str(uri or "").strip()
    target_role = "broadcast_tx" if wave == "broadcast" else "interference_tx"
    other_wave = "interference" if wave == "broadcast" else "broadcast"
    conflicts: list[dict[str, Any]] = []
    for item in managed:
        kind = str(item.get("kind") or "")
        if kind in {"generic", "dual"}:
            conflicts.append(item)
            continue
        if str(item.get("wave") or "") == wave:
            conflicts.append(item)
            continue
        if str(item.get("wave") or "") == other_wave:
            other_uri = str(item.get("uri") or "").strip()
            if not other_uri or not uri or other_uri == uri:
                conflicts.append(item)

    for item in external:
        role = str(item.get("role") or "unknown")
        other_uri = str(item.get("uri") or "").strip()
        if role in {"unknown", "generic_tx", "dual_tx_launcher"}:
            conflicts.append(item)
        elif role == target_role:
            conflicts.append(item)
        elif role in {"broadcast_tx", "interference_tx"} and (not other_uri or not uri or other_uri == uri):
            conflicts.append(item)
        else:
            # A known opposite role may run concurrently only when its URI is
            # observable and distinct from this wave's URI.
            continue
    if conflicts:
        summary = "；".join(_tx_conflict_description(item) for item in conflicts)
        raise RuntimeError(f"{wave} TX 启动互斥检查拒绝：{summary}")


def _terminate_tx_processes_with_rescan(
    pids: list[int],
    *,
    term_timeout: float = 5.0,
    kill_timeout: float = 2.0,
    rescan_timeout: float = 2.0,
    roles: set[str] | None = None,
) -> dict[str, Any]:
    cleanup = _terminate_tx_processes(pids, term_timeout=term_timeout, kill_timeout=kill_timeout)
    detected = _detect_tx_processes()
    residual = [item for item in detected if roles is None or str(_as_dict(item).get("role") or "") in roles]
    residual_pids = sorted(
        {
            int(item["pid"])
            for item in residual
            if _as_dict(item).get("pid") is not None and _process_running(int(_as_dict(item)["pid"]))
        }
    )
    cleanup["rescan_scope_roles"] = sorted(roles) if roles is not None else None
    cleanup["rescan_processes"] = residual
    cleanup["rescan_pids"] = residual_pids
    cleanup["rescan_sigkill"] = []
    cleanup["alive_after_rescan"] = []
    if residual_pids:
        cleanup["rescan_sigkill"] = _signal_tx_processes(residual_pids, signal.SIGKILL)
        cleanup["alive_after_rescan"] = _wait_for_pids_exit(residual_pids, rescan_timeout)
        time.sleep(0.4)
        cleanup["rescan_processes_after"] = [
            item
            for item in _detect_tx_processes()
            if roles is None or str(_as_dict(item).get("role") or "") in roles
        ]
    return cleanup


def _tx_preinit_sample_rate_mode(sample_rate: int) -> str:
    """Choose strict IIO setup unless GNU Radio must install a low-rate FIR."""
    return "defer" if int(sample_rate) < AD9361_NO_FIR_MIN_SAMPLE_RATE_HZ else "strict"


def _preinit_tx(plan: dict[str, Any], state: dict[str, Any]) -> dict[str, Any]:
    if not _bool_env("RM_RADIO_IIO_PREINIT", True):
        return {"skipped": True}
    tx = _as_dict(state.get("tx"))
    uri = str(tx.get("uri") or os.environ.get("INTERFERENCE_TX_URI", "")).strip()
    rf_port = str(tx.get("rf_port") or os.environ.get("TX_RF_PORT", "A")).strip()
    attenuation = _float_value(
        tx.get("attenuation_db", os.environ.get("INTERFERENCE_TX_ATTEND_GR", DEFAULT_TX_ATTENUATION_DB[plan["wave"]])),
        DEFAULT_TX_ATTENUATION_DB[plan["wave"]],
        0.0,
        89.75,
    )
    sample_rate = _int_value(tx.get("sample_rate", DEFAULT_TX_SAMPLE_RATE), DEFAULT_TX_SAMPLE_RATE, 1, 20_000_000)
    sample_rate_mode = _tx_preinit_sample_rate_mode(sample_rate)
    script = (
        f"source {shlex.quote(str(WS_ROOT / 'apps/common/iio_init.sh'))}; "
        f"init_ad9361_tx {shlex.quote(uri)} {int(plan['rf']['center_hz'])} "
        f"{int(plan['rf']['bandwidth_hz'])} {shlex.quote(str(attenuation))} "
        f"{shlex.quote(rf_port)} {int(sample_rate)} {shlex.quote(sample_rate_mode)}"
    )
    completed = subprocess.run(["bash", "-lc", script], cwd=str(WS_ROOT), text=True, capture_output=True, timeout=20)
    if completed.returncode != 0:
        raise RuntimeError((completed.stderr or completed.stdout or "IIO TX 预初始化失败").strip())
    return {
        "已跳过": False,
        "采样率配置模式": sample_rate_mode,
        "请求采样率": sample_rate,
        "标准输出": completed.stdout,
        "错误输出": completed.stderr,
    }


def _preinit_role_tx(plan: dict[str, Any], device: dict[str, Any]) -> dict[str, Any]:
    if not _bool_env("RM_RADIO_IIO_PREINIT", True):
        return {"skipped": True}
    uri = str(device.get("uri") or "").strip()
    if not uri:
        raise ValueError("TX URI 为空")
    rf_port = str(device.get("rf_port") or os.environ.get("TX_RF_PORT", "A")).strip()
    attenuation = _float_value(
        device.get("attenuation_db", DEFAULT_TX_ATTENUATION_DB[plan["wave"]]),
        DEFAULT_TX_ATTENUATION_DB[plan["wave"]],
        0.0,
        89.75,
    )
    sample_rate = _int_value(device.get("sample_rate", DEFAULT_TX_SAMPLE_RATE), DEFAULT_TX_SAMPLE_RATE, 1, 20_000_000)
    sample_rate_mode = _tx_preinit_sample_rate_mode(sample_rate)
    script = (
        f"source {shlex.quote(str(WS_ROOT / 'apps/common/iio_init.sh'))}; "
        f"init_ad9361_tx {shlex.quote(uri)} {int(plan['rf']['center_hz'])} "
        f"{int(plan['rf']['bandwidth_hz'])} {shlex.quote(str(attenuation))} "
        f"{shlex.quote(rf_port)} {int(sample_rate)} {shlex.quote(sample_rate_mode)}"
    )
    completed = subprocess.run(["bash", "-lc", script], cwd=str(WS_ROOT), text=True, capture_output=True, timeout=20)
    if completed.returncode != 0:
        raise RuntimeError((completed.stderr or completed.stdout or "IIO TX 预初始化失败").strip())
    return {
        "已跳过": False,
        "采样率配置模式": sample_rate_mode,
        "请求采样率": sample_rate,
        "标准输出": completed.stdout,
        "错误输出": completed.stderr,
    }


def _tx_mute_uris_from_state(state: dict[str, Any] | None = None) -> list[str]:
    uris: list[str] = []
    if state:
        tx = _as_dict(state.get("tx"))
        if tx.get("uri"):
            uris.append(str(tx.get("uri")))
        config = _four_sdr_config(state)
        for role in ("broadcast_tx", "interference_tx"):
            uri = _as_dict(config.get(role)).get("uri")
            if uri:
                uris.append(str(uri))
    for name in ("BROADCAST_TX_URI", "INTERFERENCE_TX_URI", "RM_RADIO_INTERFERENCE_TX_URI", "TX_URI"):
        value = os.environ.get(name)
        if value:
            uris.append(value)
    return sorted({uri.strip() for uri in uris if uri and uri.strip()})


def _tx_mute_uris_from_recent_logs() -> list[str]:
    uris: list[str] = []
    try:
        log_dirs = sorted(LOG_ROOT.glob("radio_tx_dashboard_dual_tx_*"), reverse=True)[:3]
    except Exception:
        log_dirs = []
    for log_dir in log_dirs:
        log_path = log_dir / "dashboard_dual_tx.log"
        if not log_path.exists():
            continue
        try:
            text = log_path.read_text(encoding="utf-8", errors="replace")
        except Exception:
            continue
        for key in ("BROADCAST_TX_URI", "INTERFERENCE_TX_URI"):
            match = re.search(rf'"{key}"\s*:\s*"([^"]+)"', text)
            if match:
                uris.append(match.group(1))
    try:
        log_paths = sorted(LOG_ROOT.glob("radio_tx_dashboard_tx_*.log"), reverse=True)[:3]
    except Exception:
        log_paths = []
    for log_path in log_paths:
        try:
            text = log_path.read_text(encoding="utf-8", errors="replace")
        except Exception:
            continue
        for match in re.finditer(r"(?:interference_tx_uri|RM_RADIO_INTERFERENCE_TX_URI):?=([^\s'\"]+)", text):
            uris.append(match.group(1))
    return sorted({uri.strip() for uri in uris if uri and uri.strip()})


def mute_tx_hardware(uris: list[str], attenuation_db: float = 89.75) -> dict[str, Any]:
    clean_uris = sorted({str(uri).strip() for uri in uris if str(uri).strip()})
    if not clean_uris:
        return {"uris": [], "results": []}
    iio_init = WS_ROOT / "apps" / "common" / "iio_init.sh"
    if not iio_init.exists():
        raise FileNotFoundError(str(iio_init))
    results: list[dict[str, Any]] = []
    for uri in clean_uris:
        script = (
            f"source {shlex.quote(str(iio_init))}; "
            f"mute_ad9361_tx {shlex.quote(uri)} {float(attenuation_db)}"
        )
        try:
            completed = subprocess.run(
                ["bash", "-lc", script],
                cwd=str(WS_ROOT),
                text=True,
                capture_output=True,
                timeout=8,
            )
        except Exception as exc:
            results.append({"uri": uri, "ok": False, "error": str(exc)})
            continue
        results.append(
            {
                "uri": uri,
                "ok": completed.returncode == 0,
                "returncode": int(completed.returncode),
                "stdout": completed.stdout.strip().splitlines(),
                "stderr": completed.stderr.strip().splitlines(),
            }
        )
    return {"uris": clean_uris, "results": results}


def tx_hardware_status(uris: list[str]) -> dict[str, Any]:
    clean_uris = sorted({str(uri).strip() for uri in uris if str(uri).strip()})
    results: list[dict[str, Any]] = []
    for uri in clean_uris:
        script = "\n".join(
            [
                "set +e",
                f"uri={shlex.quote(uri)}",
                "gain=$(iio_attr -u \"$uri\" -o -c ad9361-phy voltage0 hardwaregain 2>/dev/null)",
                "powerdown=$(iio_attr -u \"$uri\" -c ad9361-phy altvoltage1 powerdown 2>/dev/null)",
                "ensm=$(iio_attr -u \"$uri\" -d ad9361-phy ensm_mode 2>/dev/null)",
                "rc=0",
                "[ -n \"$gain\" ] || rc=1",
                "printf '{\"hardwaregain\":\"%s\",\"tx_lo_powerdown\":\"%s\",\"ensm_mode\":\"%s\",\"rc\":%s}\\n' \"$gain\" \"$powerdown\" \"$ensm\" \"$rc\"",
                "exit $rc",
            ]
        )
        try:
            completed = subprocess.run(
                ["bash", "-lc", script],
                cwd=str(WS_ROOT),
                text=True,
                capture_output=True,
                timeout=2.0 if uri.startswith("usb:") else 3.0,
            )
        except Exception as exc:
            results.append({"uri": uri, "ok": False, "error": str(exc)})
            continue
        item: dict[str, Any]
        try:
            item = json.loads(completed.stdout.strip().splitlines()[-1])
        except Exception:
            item = {}
        gain_text = str(item.get("hardwaregain") or "")
        powerdown_text = str(item.get("tx_lo_powerdown") or "")
        ensm_text = str(item.get("ensm_mode") or "")
        muted = powerdown_text.strip() == "1" and ("-89.750000" in gain_text or "-89.75" in gain_text)
        results.append(
            {
                "uri": uri,
                "ok": completed.returncode == 0,
                "muted": muted if completed.returncode == 0 else None,
                "hardwaregain": gain_text,
                "tx_lo_powerdown": powerdown_text,
                "ensm_mode": ensm_text,
                "returncode": int(completed.returncode),
                "stderr": completed.stderr.strip().splitlines(),
            }
        )
    return {"uris": clean_uris, "all_muted": bool(results) and all(item.get("muted") is True for item in results), "results": results}


class TxSafetyError(RuntimeError):
    def __init__(self, message: str, safety: dict[str, Any]):
        super().__init__(message)
        self.safety = safety


def _mute_and_verify_tx_hardware(uris: list[str]) -> dict[str, Any]:
    clean_uris = sorted({str(uri).strip() for uri in uris if str(uri).strip()})
    if not clean_uris:
        return {
            "ok": True,
            "required": False,
            "uris": [],
            "mute": {"uris": [], "results": []},
            "verification": {"uris": [], "all_muted": True, "results": []},
        }
    try:
        mute_result = mute_tx_hardware(clean_uris)
    except Exception as exc:
        mute_result = {"uris": clean_uris, "results": [], "error": str(exc)}
    try:
        verification = tx_hardware_status(clean_uris)
    except Exception as exc:
        verification = {"uris": clean_uris, "all_muted": False, "results": [], "error": str(exc)}
    mute_items = _as_list(_as_dict(mute_result).get("results"))
    mute_ok = len(mute_items) == len(clean_uris) and all(_as_dict(item).get("ok") is True for item in mute_items)
    verification_ok = _as_dict(verification).get("all_muted") is True
    return {
        "ok": bool(mute_ok and verification_ok),
        "required": True,
        "uris": clean_uris,
        "mute": mute_result,
        "verification": verification,
    }


def _verify_managed_process_start(proc: subprocess.Popen, label: str) -> None:
    settle_sec = _float_value(os.environ.get("RM_RADIO_TX_START_SETTLE_SEC", 0.15), 0.15, 0.0, 5.0)
    if settle_sec:
        time.sleep(settle_sec)
    returncode = proc.poll()
    if returncode is not None:
        raise RuntimeError(f"{label} 启动后立即退出，退出码={returncode}")


def _wait_for_tx_node_started(
    node_name: str,
    expected_sample_rate: int,
    expected_sps: int,
    *,
    timeout: float | None = None,
) -> dict[str, Any]:
    """Wait for the node's durable status instead of treating a live PID as ready."""
    timeout_sec = timeout
    if timeout_sec is None:
        timeout_sec = _float_value(os.environ.get("RM_RADIO_TX_STATUS_VERIFY_SEC", 8.0), 8.0, 1.0, 20.0)
    status_topic = f"/{node_name}/status"
    env = _radio_process_env()
    env.update(
        {
            "RM_RADIO_TX_VERIFY_TOPIC": status_topic,
            "RM_RADIO_TX_VERIFY_SAMPLE_RATE": str(int(expected_sample_rate)),
            "RM_RADIO_TX_VERIFY_SPS": str(int(expected_sps)),
            "RM_RADIO_TX_VERIFY_TIMEOUT": str(float(timeout_sec)),
            "PYTHONPATH": f"{PACKAGE_SRC}:{env.get('PYTHONPATH', '')}",
        }
    )
    py = "\n".join(
        [
            "import json, os, time",
            "import rclpy",
            "from rclpy.node import Node",
            "from rclpy.qos import QoSProfile, DurabilityPolicy",
            "from std_msgs.msg import String",
            "topic = os.environ['RM_RADIO_TX_VERIFY_TOPIC']",
            "expected_rate = int(os.environ['RM_RADIO_TX_VERIFY_SAMPLE_RATE'])",
            "expected_sps = int(os.environ['RM_RADIO_TX_VERIFY_SPS'])",
            "timeout = float(os.environ['RM_RADIO_TX_VERIFY_TIMEOUT'])",
            "latest = {}",
            "def status_cb(msg):",
            "    global latest",
            "    try: latest = json.loads(msg.data)",
            "    except Exception: pass",
            "rclpy.init()",
            "node = Node('rm_labtx_start_verify_' + str(os.getpid()))",
            "qos = QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL)",
            "sub = node.create_subscription(String, topic, status_cb, qos)",
            "confirmed = False",
            "deadline = time.time() + timeout",
            "while time.time() < deadline:",
            "    rclpy.spin_once(node, timeout_sec=0.08)",
            "    tx = latest.get('tx_stream', {}) if isinstance(latest, dict) else {}",
            "    try: rate_ok = abs(float(tx.get('configured_sample_rate_hz')) - expected_rate) <= 2.0",
            "    except (TypeError, ValueError): rate_ok = False",
            "    try: sps_ok = int(tx.get('samples_per_symbol')) == expected_sps",
            "    except (TypeError, ValueError): sps_ok = False",
            "    confirmed = (latest.get('loaded') is True and latest.get('started') is True and not latest.get('last_error') and tx.get('mode') == 'scheduler_clocked_continuous_bits' and tx.get('iio_filter_mode') == 'Auto' and rate_ok and sps_ok)",
            "    if confirmed: break",
            "    if latest.get('last_error') and latest.get('loaded') is not True: break",
            "tx = latest.get('tx_stream', {}) if isinstance(latest, dict) else {}",
            "summary = {'loaded': latest.get('loaded'), 'started': latest.get('started'), 'last_error': latest.get('last_error'), 'tx_stream': tx}",
            "reason = '' if confirmed else ('未收到节点状态' if not latest else (latest.get('last_error') or '节点状态或 TX 流参数未达到预期'))",
            "result = {'confirmed': confirmed, 'topic': topic, 'expected_sample_rate_hz': expected_rate, 'expected_sps': expected_sps, 'reason': reason, 'status': summary}",
            "node.destroy_node(); rclpy.shutdown()",
            "print(json.dumps(result, ensure_ascii=False))",
            "raise SystemExit(0 if confirmed else 2)",
        ]
    )
    script = "\n".join(
        [
            "set -eo pipefail",
            "source /opt/ros/humble/setup.bash",
            f"cd {shlex.quote(str(WS_ROOT))}",
            f"export PYTHONPATH={shlex.quote(str(PACKAGE_SRC))}:${{PYTHONPATH:-}}",
            f"exec python3 -c {shlex.quote(py)}",
        ]
    )
    try:
        completed = subprocess.run(
            ["bash", "-lc", script],
            cwd=str(WS_ROOT),
            env=env,
            text=True,
            capture_output=True,
            timeout=float(timeout_sec) + 2.0,
        )
    except subprocess.TimeoutExpired as exc:
        return {
            "ok": False,
            "topic": status_topic,
            "returncode": None,
            "confirmation": {},
            "stderr": [f"启动状态确认进程超时：{exc}"],
        }
    confirmation: dict[str, Any] = {}
    for line in reversed(completed.stdout.strip().splitlines()):
        try:
            parsed = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(parsed, dict) and "confirmed" in parsed:
            confirmation = parsed
            break
    return {
        "ok": completed.returncode == 0 and confirmation.get("confirmed") is True,
        "topic": status_topic,
        "returncode": completed.returncode,
        "confirmation": confirmation,
        "stdout": completed.stdout.strip().splitlines()[-6:],
        "stderr": completed.stderr.strip().splitlines()[-6:],
    }


def _require_tx_node_started(
    node_name: str,
    expected_sample_rate: int,
    expected_sps: int,
) -> dict[str, Any]:
    confirmation = _wait_for_tx_node_started(node_name, expected_sample_rate, expected_sps)
    if not confirmation.get("ok"):
        detail = confirmation.get("confirmation") or confirmation.get("stderr") or "未收到节点启动状态"
        raise RuntimeError(f"TX 节点 {node_name} 未确认完成 GNU Radio/IIO 启动：{detail}")
    return confirmation


def _verify_dual_managed_process_start(proc: subprocess.Popen, log_path: Path) -> None:
    timeout_sec = _float_value(os.environ.get("RM_RADIO_DUAL_TX_START_VERIFY_SEC", 2.0), 2.0, 0.1, 10.0)
    deadline = time.time() + timeout_sec
    children: dict[str, Any] = {}
    while time.time() < deadline:
        returncode = proc.poll()
        if returncode is not None:
            raise RuntimeError(f"双路 TX 托管进程启动后立即退出，退出码={returncode}")
        children = _read_dual_tx_pid_status(log_path.parent)
        if all(_as_dict(children.get(role)).get("running") is True for role in ("broadcast_tx", "interference_tx")):
            return
        time.sleep(0.05)
    child_summary = {
        role: {
            "pid": _as_dict(children.get(role)).get("pid"),
            "running": _as_dict(children.get(role)).get("running"),
        }
        for role in ("broadcast_tx", "interference_tx")
    }
    raise RuntimeError(f"双路 TX 子进程未在 {timeout_sec:.2f}s 内全部进入托管运行状态：{child_summary}")


def _tx_start_safety_error(
    label: str,
    error: BaseException,
    touched_uris: list[str],
    cleanup: dict[str, Any] | None = None,
) -> TxSafetyError:
    hardware_mute = _mute_and_verify_tx_hardware(touched_uris)
    if hardware_mute.get("ok") is True:
        outcome = "所有已触碰 TX URI 均已静音并通过回读校验"
    else:
        outcome = "TX 静音或回读校验失败，请立即人工断开发射链路"
    safety = {
        "stage_error": f"{type(error).__name__}: {error}",
        "touched_uris": sorted({str(uri).strip() for uri in touched_uris if str(uri).strip()}),
        "process_cleanup": cleanup or _terminate_tx_processes([]),
        "hardware_mute": hardware_mute,
    }
    return TxSafetyError(f"{label} 启动失败：{error}；{outcome}", safety)


def _cleanup_failed_tx_start(proc: subprocess.Popen | None, extra_pids: list[int] | None = None) -> dict[str, Any]:
    pids = list(extra_pids or [])
    if proc is not None and getattr(proc, "pid", None) is not None:
        pids.append(int(proc.pid))
    try:
        return _terminate_tx_processes(pids, term_timeout=2.0, kill_timeout=1.0)
    except Exception as exc:
        return {"requested_pids": sorted(set(pids)), "cleanup_error": str(exc)}


def start_tx(server: RadioTxHttpServer, state: dict[str, Any]) -> dict[str, Any]:
    tx = _as_dict(state.get("tx"))
    if not server.lab_tx_effective_confirm():
        raise PermissionError("本次 dashboard 会话尚未建立实验室安全确认")
    if not bool(tx.get("lab_confirm")):
        raise PermissionError("页面里的实验室安全确认未勾选")
    with server.tx_lock:
        plan = build_plan(state)
        uri = str(tx.get("uri") or os.environ.get("INTERFERENCE_TX_URI", "")).strip()
        if not uri:
            raise ValueError("TX URI 为空")
        _assert_tx_start_allowed(server, mode="generic", uri=uri)
        touched_uris: list[str] = []
        proc: subprocess.Popen | None = None
        log_file: Any = None
        try:
            touched_uris.append(uri)
            preinit = _preinit_tx(plan, state)
            _ensure_dir(LOG_ROOT)
            log_path = LOG_ROOT / f"radio_tx_dashboard_tx_{_now_token()}.log"
            env = _radio_process_env()
            env["RM_RADIO_SIDE"] = plan["side"]
            env["INTERFERENCE_LEVEL"] = str(plan["level"])
            env["RM_RADIO_INTERFERENCE_LEVEL"] = str(plan["level"])
            env["INTERFERENCE_TX_URI"] = uri
            env["RM_RADIO_INTERFERENCE_TX_URI"] = uri
            env["RM_RADIO_LAB_TX_ANTENNA_CONFIRM"] = "true"
            env["RM_RADIO_TX_PREFILL_PACKETS"] = env.get("RM_RADIO_TX_PREFILL_PACKETS", "0")
            env["PYTHONPATH"] = f"{PACKAGE_SRC}:{env.get('PYTHONPATH', '')}"
            ros_args, setters_json = _generated_tx_ros_args(plan, uri)
            env["RM_RADIO_SETTERS_JSON"] = setters_json
            env["PYTHONUNBUFFERED"] = "1"
            quoted_ros_args = " ".join(shlex.quote(item) for item in ros_args)
            script = "\n".join(
                [
                    "set -eo pipefail",
                    "source /opt/ros/humble/setup.bash",
                    f"cd {shlex.quote(str(WS_ROOT))}",
                    f"export PYTHONPATH={shlex.quote(str(PACKAGE_SRC))}:${{PYTHONPATH:-}}",
                    f"export RM_RADIO_INTERFERENCE_TX_URI={shlex.quote(uri)}",
                    f"export INTERFERENCE_TX_URI={shlex.quote(uri)}",
                    f"export RM_RADIO_SETTERS_JSON={shlex.quote(setters_json)}",
                    f"export RM_RADIO_TX_PREFILL_PACKETS={shlex.quote(env['RM_RADIO_TX_PREFILL_PACKETS'])}",
                    "export RM_RADIO_RX_ONLY=0",
                    "export RM_RADIO_LAB_TX_ANTENNA_CONFIRM=true",
                    f"exec stdbuf -o0 -e0 python3 -c 'from rm_radio_ros.nodes.ganraoyuan_node import main; main()' {quoted_ros_args}",
                ]
            )
            args = ["bash", "-lc", script]
            log_file = log_path.open("a", encoding="utf-8")
            log_file.write("launcher=source-module\n")
            log_file.write(f"ros_args={ros_args}\n")
            log_file.write(f"script={script}\n")
            log_file.flush()
            proc = subprocess.Popen(
                args,
                cwd=str(WS_ROOT),
                env=env,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=(os.name != "nt"),
            )
            _verify_managed_process_start(proc, "generic TX")
            startup_confirmation = _require_tx_node_started(
                "rm_ganraoyuan_node",
                int(plan["modulation"]["sample_rate"]),
                int(plan["modulation"]["sps"]),
            )
            server.tx_process = proc
            server.tx_log_path = log_path
            server.last_tx_state = state
            return {
                "已启动": True,
                "PID": proc.pid,
                "日志": str(log_path),
                "预初始化": preinit,
                "启动确认": startup_confirmation,
                "plan": plan,
            }
        except Exception as exc:
            if log_file is not None and not log_file.closed:
                log_file.close()
            cleanup = _cleanup_failed_tx_start(proc)
            if server.tx_process is proc:
                server.tx_process = None
            raise _tx_start_safety_error("generic TX", exc, touched_uris, cleanup) from exc
        finally:
            if log_file is not None and not log_file.closed:
                log_file.close()


def stop_tx(server: RadioTxHttpServer) -> dict[str, Any]:
    with server.tx_lock:
        proc = server.tx_process
        mute_state = server.last_tx_state
        mute_uris = sorted(set(_tx_mute_uris_from_state(mute_state) + _tx_mute_uris_from_recent_logs()))
        pids: list[int] = []
        if proc is not None:
            pids.append(proc.pid)
        pids.extend(_external_tx_pids())
        if not pids:
            server.tx_process = None
            hardware_mute = _mute_and_verify_tx_hardware(mute_uris)
            return {"已停止": False, "消息": "TX 进程未运行", "cleanup": _terminate_tx_processes([]), "hardware_mute": hardware_mute}
        cleanup = _terminate_tx_processes_with_rescan(pids)
        if proc is not None:
            try:
                proc.wait(timeout=0.2)
            except Exception:
                pass
            server.tx_last_returncode = proc.returncode
        server.tx_process = None
        hardware_mute = _mute_and_verify_tx_hardware(mute_uris)
        return {
            "已停止": True,
            "退出码": server.tx_last_returncode,
            "日志": str(server.tx_log_path) if server.tx_log_path else None,
            "cleanup": cleanup,
            "hardware_mute": hardware_mute,
        }


# ---------------------------------------------------------------------------
# 按路独立发射：信息波 / 干扰波 各自一个 ganraoyuan 节点进程，可单独开关。
# ---------------------------------------------------------------------------
_WAVE_ROLE = {"broadcast": "broadcast_tx", "interference": "interference_tx"}
_WAVE_NODE = {"broadcast": "rm_broadcast_tx_node", "interference": "rm_interference_tx_node"}
_WAVE_LABEL = {"broadcast": "信息波", "interference": "干扰波"}


def _normalize_wave(wave: Any) -> str:
    text = str(wave or "").strip().lower()
    if text in ("broadcast", "info", "information", "xinxibo"):
        return "broadcast"
    if text in ("interference", "ganrao", "ganraobo"):
        return "interference"
    raise ValueError(f"未知发射波类型：{wave!r}")


def _wave_plan(state: dict[str, Any], wave: str) -> dict[str, Any]:
    role = _WAVE_ROLE[wave]
    wave_state = _tx_state_for_role(state, role, wave)
    wave_state["wave"] = wave
    return build_plan(wave_state)


def start_wave(server: RadioTxHttpServer, state: dict[str, Any], wave: str) -> dict[str, Any]:
    wave = _normalize_wave(wave)
    tx = _as_dict(state.get("tx"))
    if not server.lab_tx_effective_confirm():
        raise PermissionError("本次 dashboard 会话尚未建立实验室安全确认")
    if not bool(tx.get("lab_confirm")):
        raise PermissionError("页面里的实验室安全确认未勾选")
    label = _WAVE_LABEL[wave]
    with server.tx_lock:
        plan = _wave_plan(state, wave)
        device = _as_dict(_four_sdr_config(state).get(_WAVE_ROLE[wave]))
        uri = str(device.get("uri") or "").strip()
        if not uri:
            if wave == "interference":
                raise ValueError("干扰波 TX 当前未连接，需单独接入并识别后才能发射")
            raise ValueError(f"{label} TX URI 为空")
        _assert_tx_start_allowed(server, mode="wave", wave=wave, uri=uri)
        touched_uris: list[str] = []
        proc: subprocess.Popen | None = None
        log_file: Any = None
        try:
            touched_uris.append(uri)
            preinit = _preinit_role_tx(plan, device)
            _ensure_dir(LOG_ROOT)
            log_path = LOG_ROOT / f"radio_tx_dashboard_{wave}_{_now_token()}.log"
            ros_args, setters_json = _generated_tx_ros_args(
                plan,
                uri,
                node_name=_WAVE_NODE[wave],
            )
            env = _radio_process_env()
            env.update(
                {
                    "RM_RADIO_SIDE": plan["side"],
                    "INTERFERENCE_LEVEL": str(plan["level"]),
                    "RM_RADIO_INTERFERENCE_LEVEL": str(plan["level"]),
                    "INTERFERENCE_TX_URI": uri,
                    "RM_RADIO_INTERFERENCE_TX_URI": uri,
                    "RM_RADIO_SAMPLE_RATE": str(plan["modulation"]["sample_rate"]),
                    "RM_RADIO_LAB_TX_ANTENNA_CONFIRM": "true",
                    "RM_RADIO_TX_PREFILL_PACKETS": env.get("RM_RADIO_TX_PREFILL_PACKETS", "0"),
                    "RM_RADIO_SETTERS_JSON": setters_json,
                    "PYTHONUNBUFFERED": "1",
                    "PYTHONPATH": f"{PACKAGE_SRC}:{env.get('PYTHONPATH', '')}",
                }
            )
            quoted_ros_args = " ".join(shlex.quote(item) for item in ros_args)
            script = "\n".join(
                [
                    "set -eo pipefail",
                    "source /opt/ros/humble/setup.bash",
                    f"cd {shlex.quote(str(WS_ROOT))}",
                    f"export PYTHONPATH={shlex.quote(str(PACKAGE_SRC))}:${{PYTHONPATH:-}}",
                    f"export RM_RADIO_INTERFERENCE_TX_URI={shlex.quote(uri)}",
                    f"export INTERFERENCE_TX_URI={shlex.quote(uri)}",
                    f"export RM_RADIO_SETTERS_JSON={shlex.quote(setters_json)}",
                    f"export RM_RADIO_TX_PREFILL_PACKETS={shlex.quote(env['RM_RADIO_TX_PREFILL_PACKETS'])}",
                    "export RM_RADIO_RX_ONLY=0",
                    "export RM_RADIO_LAB_TX_ANTENNA_CONFIRM=true",
                    f"exec stdbuf -o0 -e0 python3 -c 'from rm_radio_ros.nodes.ganraoyuan_node import main; main()' {quoted_ros_args}",
                ]
            )
            log_file = log_path.open("a", encoding="utf-8")
            log_file.write(f"launcher=wave:{wave}\nsetters={setters_json}\n")
            log_file.flush()
            proc = subprocess.Popen(
                ["bash", "-lc", script],
                cwd=str(WS_ROOT),
                env=env,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=(os.name != "nt"),
            )
            _verify_managed_process_start(proc, f"{label} TX")
            startup_confirmation = _require_tx_node_started(
                _WAVE_NODE[wave],
                int(plan["modulation"]["sample_rate"]),
                int(plan["modulation"]["sps"]),
            )
            server.wave_processes[wave] = proc
            server.wave_log_paths[wave] = log_path
            server.wave_last_state[wave] = state
            return {
                "已启动": True,
                "波": label,
                "PID": proc.pid,
                "日志": str(log_path),
                "预初始化": preinit,
                "启动确认": startup_confirmation,
                "plan": plan,
            }
        except Exception as exc:
            if log_file is not None and not log_file.closed:
                log_file.close()
            cleanup = _cleanup_failed_tx_start(proc)
            if server.wave_processes.get(wave) is proc:
                server.wave_processes[wave] = None
            raise _tx_start_safety_error(f"{label} TX", exc, touched_uris, cleanup) from exc
        finally:
            if log_file is not None and not log_file.closed:
                log_file.close()


def stop_wave(server: RadioTxHttpServer, wave: str) -> dict[str, Any]:
    wave = _normalize_wave(wave)
    role = _WAVE_ROLE[wave]
    label = _WAVE_LABEL[wave]
    with server.tx_lock:
        proc = server.wave_processes.get(wave)
        detected = [item for item in _detect_tx_processes() if _as_dict(item).get("role") == role]
        mute_uris = [
            _wave_uri_from_state(server.wave_last_state.get(wave), wave),
            *(str(_as_dict(item).get("uri") or "").strip() for item in detected),
        ]
        mute_uris = sorted({uri for uri in mute_uris if uri})
        pids: list[int] = []
        if proc is not None:
            pids.append(proc.pid)
        pids.extend(int(item["pid"]) for item in detected)
        pids = sorted(set(pids))
        if not pids:
            server.wave_processes[wave] = None
            return {
                "已停止": False,
                "波": label,
                "消息": f"{label} TX 进程未运行",
                "hardware_mute": _mute_and_verify_tx_hardware(mute_uris),
            }
        cleanup = _terminate_tx_processes_with_rescan(pids, roles={role})
        if proc is not None:
            try:
                proc.wait(timeout=0.2)
            except Exception:
                pass
            server.wave_last_returncode[wave] = proc.returncode
        server.wave_processes[wave] = None
        return {
            "已停止": True,
            "波": label,
            "退出码": server.wave_last_returncode.get(wave),
            "日志": str(server.wave_log_paths.get(wave)) if server.wave_log_paths.get(wave) else None,
            "cleanup": cleanup,
            "hardware_mute": _mute_and_verify_tx_hardware(mute_uris),
        }


def _publish_setters_once(node_name: str, setters: dict[str, Any], timeout: float = 6.0) -> dict[str, Any]:
    """向运行中的 ganraoyuan 节点 ~/setters_json 发布一次新 setters（热重配）。"""
    setters_json = _compact_json(setters)
    topic = f"/{node_name}/setters_json"
    env = _radio_process_env()
    env["RM_RADIO_ONESHOT_SETTERS_JSON"] = setters_json
    env["RM_RADIO_ONESHOT_TOPIC"] = topic
    env["PYTHONPATH"] = f"{PACKAGE_SRC}:{env.get('PYTHONPATH', '')}"
    py = "\n".join(
        [
            "import json, os, time",
            "import rclpy",
            "from rclpy.node import Node",
            "from rclpy.qos import QoSProfile, DurabilityPolicy",
            "from std_msgs.msg import String",
            "topic = os.environ['RM_RADIO_ONESHOT_TOPIC']",
            "status_topic = topic.rsplit('/', 1)[0] + '/status'",
            "data = os.environ['RM_RADIO_ONESHOT_SETTERS_JSON']",
            "expected = json.loads(data)",
            "latest = {}",
            "def status_cb(msg):",
            "    global latest",
            "    try: latest = json.loads(msg.data)",
            "    except Exception: pass",
            "def same(actual, wanted):",
            "    if isinstance(wanted, bool): return actual is wanted",
            "    if isinstance(wanted, (int, float)) and isinstance(actual, (int, float)):",
            "        return abs(float(actual) - float(wanted)) <= 1e-6 * max(1.0, abs(float(wanted)))",
            "    if isinstance(wanted, list):",
            "        return isinstance(actual, list) and len(actual) == len(wanted) and all(same(a, b) for a, b in zip(actual, wanted))",
            "    if isinstance(wanted, dict):",
            "        return isinstance(actual, dict) and all(k in actual and same(actual[k], v) for k, v in wanted.items())",
            "    return actual == wanted",
            "rclpy.init()",
            "node = Node('rm_labtx_setter_' + str(os.getpid()))",
            "qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)",
            "status_qos = QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL)",
            "pub = node.create_publisher(String, topic, qos)",
            "sub = node.create_subscription(String, status_topic, status_cb, status_qos)",
            "discover_end = time.time() + 0.8",
            "while time.time() < discover_end and not latest: rclpy.spin_once(node, timeout_sec=0.05)",
            "baseline = int(latest.get('setters_apply', {}).get('count', -1))",
            "msg = String(); msg.data = data",
            "published_at = time.time(); pub.publish(msg)",
            "confirmed = False; confirmation = {}",
            "end = time.time() + 3.5",
            "while time.time() < end:",
            "    rclpy.spin_once(node, timeout_sec=0.08)",
            "    applied = latest.get('last_setters', {})",
            "    meta = latest.get('setters_apply', {})",
            "    count = int(meta.get('count', -1)); applied_at = float(meta.get('timestamp', 0.0))",
            "    values_match = all(k in applied and same(applied[k], v) for k, v in expected.items())",
            "    is_new = (baseline >= 0 and count > baseline) or (baseline < 0 and applied_at >= published_at)",
            "    if values_match and is_new:",
            "        confirmed = True",
            "        confirmation = {'count': count, 'timestamp': applied_at, 'last_setters': applied}",
            "        break",
            "result = {'confirmed': confirmed, 'topic': topic, 'status_topic': status_topic, 'baseline_count': baseline, 'confirmation': confirmation}",
            "node.destroy_node(); rclpy.shutdown()",
            "print(json.dumps(result, ensure_ascii=False))",
            "raise SystemExit(0 if confirmed else 2)",
        ]
    )
    script = "\n".join(
        [
            "set -eo pipefail",
            "source /opt/ros/humble/setup.bash",
            f"cd {shlex.quote(str(WS_ROOT))}",
            f"export PYTHONPATH={shlex.quote(str(PACKAGE_SRC))}:${{PYTHONPATH:-}}",
            f"exec python3 -c {shlex.quote(py)}",
        ]
    )
    completed = subprocess.run(["bash", "-lc", script], cwd=str(WS_ROOT), env=env, text=True, capture_output=True, timeout=timeout)
    confirmation: dict[str, Any] = {}
    for line in reversed(completed.stdout.strip().splitlines()):
        try:
            parsed = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(parsed, dict) and "confirmed" in parsed:
            confirmation = parsed
            break
    return {
        "ok": completed.returncode == 0 and confirmation.get("confirmed") is True,
        "topic": topic,
        "setters": setters,
        "confirmation": confirmation,
        "returncode": completed.returncode,
        "stdout": completed.stdout.strip().splitlines()[-6:],
        "stderr": completed.stderr.strip().splitlines()[-6:],
    }


def reconfigure_wave(server: RadioTxHttpServer, state: dict[str, Any], wave: str, changes: dict[str, Any]) -> dict[str, Any]:
    """热重配运行中的某路发射：应用新等级/密钥/数据对应的 setters。"""
    wave = _normalize_wave(wave)
    label = _WAVE_LABEL[wave]
    status = server.wave_status(wave)
    if not status.get("running"):
        raise RuntimeError(f"{label} TX 未在发射，无法热重配")
    plan = _wave_plan(state, wave)
    setters = dict(plan["tx_setters"])
    live_keys = ("center_f", "BW_ganrao", "access", "attend_gr", "payload_data", "cmd_id", "command_cycle", "Period")
    payload = {key: setters[key] for key in live_keys if key in setters}
    if wave == "interference":
        payload["interference_level"] = plan["level"]
    result = _publish_setters_once(_WAVE_NODE[wave], payload)
    if not result.get("ok"):
        detail = result.get("confirmation") or result.get("stderr") or "未收到节点应用回执"
        raise RuntimeError(f"{label} TX 热重配未确认：{detail}")
    result["波"] = label
    result["level"] = plan["level"]
    server.wave_last_state[wave] = state
    return result


def _random_password() -> str:
    alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789"
    return "".join(random.choice(alphabet) for _ in range(6))


def randomize_broadcast_config(state: dict[str, Any]) -> dict[str, Any]:
    """随机化信息波 0x0A01~0x0A05 内置数据，返回新的 broadcast 配置。"""
    config = _broadcast_config(state)
    enabled = _as_list(config.get("enabled_cmds")) or list(RADAR_BROADCAST_COMMANDS)
    robots = {
        key: {
            "x": random.randint(0, POSITION_X_MAX_CM),
            "y": random.randint(0, POSITION_Y_MAX_CM),
        }
        for key in ROBOT_KEYS
    }
    hp = {key: random.randint(0, HP_MAX[key]) for key in HP_KEYS}
    bullets = {key: random.randint(0, BULLET_MAX[key]) for key in BULLET_KEYS}
    buffs = {
        key: {
            field: random.randint(0, maxima[key])
            for field, maxima in BUFF_FIELD_MAX.items()
        }
        for key in BUFF_KEYS
    }
    total_coins = random.randint(0, 0xFFFF)
    randomized = {
        "enabled_cmds": enabled,
        "auto_change_data": config.get("auto_change_data") is True,
        "robots": robots,
        "hp": hp,
        "bullets": bullets,
        "macro": {
            "remaining_coins": random.randint(0, total_coins),
            "total_coins": total_coins,
            "occupation_bits": random_occupation_bits(random),
        },
        "buffs": buffs,
        "sentry_mode": random.randint(1, 6),
        "robot_main_status": {
            key: random.randint(0, 3)
            for key in BUFF_KEYS
        },
    }
    return _broadcast_config({"broadcast": randomized})


def _dual_tx_env(state: dict[str, Any], dual_plan: dict[str, Any], log_dir: Path) -> dict[str, str]:
    config = _as_dict(dual_plan.get("four_sdr"))
    broadcast_tx = _as_dict(config.get("broadcast_tx"))
    interference_tx = _as_dict(config.get("interference_tx"))
    env = _radio_process_env()
    env.update(
        {
            "RM_RADIO_WS": str(WS_ROOT),
            "LOG_DIR": str(log_dir),
            "RM_RADIO_SIDE": _side(state),
            "INTERFERENCE_LEVEL": str(_level(state)),
            "RM_RADIO_INTERFERENCE_LEVEL": str(_level(state)),
            "INTERFERENCE_PASSWORD": _password(state),
            "BROADCAST_TX_URI": str(broadcast_tx.get("uri") or ""),
            "BROADCAST_TX_ATTENUATION_DB": str(
                _float_value(
                    broadcast_tx.get("attenuation_db", DEFAULT_TX_ATTENUATION_DB["broadcast"]),
                    DEFAULT_TX_ATTENUATION_DB["broadcast"],
                    0.0,
                    89.75,
                )
            ),
            "BROADCAST_TX_RF_PORT": str(broadcast_tx.get("rf_port") or os.environ.get("TX_RF_PORT", "A")),
            "INTERFERENCE_TX_URI": str(interference_tx.get("uri") or ""),
            "INTERFERENCE_TX_ATTEND_GR": str(
                _float_value(
                    interference_tx.get("attenuation_db", DEFAULT_TX_ATTENUATION_DB["interference"]),
                    DEFAULT_TX_ATTENUATION_DB["interference"],
                    0.0,
                    89.75,
                )
            ),
            "INTERFERENCE_TX_RF_PORT": str(interference_tx.get("rf_port") or os.environ.get("TX_RF_PORT", "A")),
            "TX_RF_PORT": str(interference_tx.get("rf_port") or broadcast_tx.get("rf_port") or os.environ.get("TX_RF_PORT", "A")),
            "RM_RADIO_SAMPLE_RATE": str(_int_value(interference_tx.get("sample_rate", DEFAULT_TX_SAMPLE_RATE), DEFAULT_TX_SAMPLE_RATE, 1, 20_000_000)),
            "RM_RADIO_TX_PREFILL_PACKETS": env.get("RM_RADIO_TX_PREFILL_PACKETS", "0"),
            "RM_RADIO_LAB_TX_ANTENNA_CONFIRM": "true",
            "RM_RADIO_IIO_PREINIT": "false",
            # 纯 TX：关闭发射端 RX 回采自检，双路发射不再依赖任何 RX 设备。
            "BROADCAST_IIO_CAPTURE_DECODER_ENABLED": "false",
            "BROADCAST_IIO_CAPTURE_DECODER_ONLY": "false",
            "RM_RADIO_SETTERS_JSON": _compact_json(dual_plan["interference"]["tx_setters"]),
            "BROADCAST_SETTERS_JSON": _compact_json(dual_plan["broadcast"]["tx_setters"]),
            "INTERFERENCE_SETTERS_JSON": _compact_json(dual_plan["interference"]["tx_setters"]),
            "PYTHONPATH": f"{PACKAGE_SRC}:{env.get('PYTHONPATH', '')}",
        }
    )
    return env


def start_dual_tx(server: RadioTxHttpServer, state: dict[str, Any]) -> dict[str, Any]:
    tx = _as_dict(state.get("tx"))
    if not server.lab_tx_effective_confirm():
        raise PermissionError("本次 dashboard 会话尚未建立实验室安全确认")
    if not bool(tx.get("lab_confirm")):
        raise PermissionError("页面里的实验室安全确认未勾选")
    interference_uri = str(_four_sdr_config(state)["interference_tx"].get("uri") or "").strip()
    if not interference_uri:
        raise ValueError("干扰波 TX 当前未连接，不能启动双路 TX")
    topology_errors, _ = _four_sdr_topology_checks(state)
    if topology_errors:
        raise ValueError("；".join(topology_errors))
    with server.tx_lock:
        _assert_tx_start_allowed(server, mode="dual")
        availability = check_four_sdr(state)
        availability_errors = _four_sdr_availability_errors(availability)
        if availability_errors:
            raise RuntimeError("四路 SDR 自检失败，未启动双路 TX：" + "；".join(availability_errors))
        dual_plan = build_dual_plans(state)
        config = _as_dict(dual_plan.get("four_sdr"))
        devices = {
            "broadcast_tx": _as_dict(config.get("broadcast_tx")),
            "interference_tx": _as_dict(config.get("interference_tx")),
        }
        uris = {
            role: str(device.get("uri") or "").strip()
            for role, device in devices.items()
        }
        if any(not uri for uri in uris.values()):
            raise ValueError(f"双路 TX URI 为空：{uris}")
        touched_uris: list[str] = []
        proc: subprocess.Popen | None = None
        log_file: Any = None
        log_path: Path | None = None
        try:
            touched_uris.append(uris["broadcast_tx"])
            broadcast_preinit = _preinit_role_tx(dual_plan["broadcast"], devices["broadcast_tx"])
            touched_uris.append(uris["interference_tx"])
            interference_preinit = _preinit_role_tx(dual_plan["interference"], devices["interference_tx"])
            preinit = {
                "broadcast_tx": broadcast_preinit,
                "interference_tx": interference_preinit,
            }
            _ensure_dir(LOG_ROOT)
            log_dir = _ensure_dir(LOG_ROOT / f"radio_tx_dashboard_dual_tx_{_now_token()}")
            env = _dual_tx_env(state, dual_plan, log_dir)
            log_path = log_dir / "dashboard_dual_tx.log"
            script_path = RADIO_TX_DIR / "start_dual_tx.sh"
            log_file = log_path.open("a", encoding="utf-8")
            log_file.write("launcher=start_dual_tx.sh\n")
            log_file.write(f"script={script_path}\n")
            log_file.write(f"env_summary={json.dumps({k: env[k] for k in ['RM_RADIO_SIDE', 'INTERFERENCE_LEVEL', 'BROADCAST_TX_URI', 'BROADCAST_TX_RF_PORT', 'INTERFERENCE_TX_URI', 'INTERFERENCE_TX_RF_PORT', 'RM_RADIO_SAMPLE_RATE']}, ensure_ascii=False)}\n")
            log_file.flush()
            proc = subprocess.Popen(
                ["bash", str(script_path)],
                cwd=str(WS_ROOT),
                env=env,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=(os.name != "nt"),
            )
            _verify_dual_managed_process_start(proc, log_path)
            startup_confirmation = {
                "broadcast_tx": _require_tx_node_started(
                    _WAVE_NODE["broadcast"],
                    int(dual_plan["broadcast"]["modulation"]["sample_rate"]),
                    int(dual_plan["broadcast"]["modulation"]["sps"]),
                ),
                "interference_tx": _require_tx_node_started(
                    _WAVE_NODE["interference"],
                    int(dual_plan["interference"]["modulation"]["sample_rate"]),
                    int(dual_plan["interference"]["modulation"]["sps"]),
                ),
            }
            server.dual_tx_process = proc
            server.dual_tx_log_path = log_path
            server.last_dual_tx_state = state
            return {
                "已启动": True,
                "PID": proc.pid,
                "日志": str(log_path),
                "日志目录": str(log_dir),
                "预初始化": preinit,
                "启动确认": startup_confirmation,
                "dual_plan": dual_plan,
            }
        except Exception as exc:
            if log_file is not None and not log_file.closed:
                log_file.close()
            child_pids = _dual_tx_child_pids(log_path) if log_path is not None else []
            cleanup = _cleanup_failed_tx_start(proc, child_pids)
            if server.dual_tx_process is proc:
                server.dual_tx_process = None
            raise _tx_start_safety_error("双路 TX", exc, touched_uris, cleanup) from exc
        finally:
            if log_file is not None and not log_file.closed:
                log_file.close()


def stop_dual_tx(server: RadioTxHttpServer) -> dict[str, Any]:
    with server.tx_lock:
        proc = server.dual_tx_process
        mute_state = server.last_dual_tx_state or server.last_tx_state
        mute_uris = sorted(set(_tx_mute_uris_from_state(mute_state) + _tx_mute_uris_from_recent_logs()))
        pids: list[int] = []
        if proc is not None:
            pids.append(proc.pid)
        pids.extend(_dual_tx_child_pids(server.dual_tx_log_path))
        pids.extend(_external_tx_pids())
        if not pids:
            server.dual_tx_process = None
            hardware_mute = _mute_and_verify_tx_hardware(mute_uris)
            return {"已停止": False, "消息": "双路 TX 进程未运行", "cleanup": _terminate_tx_processes([]), "hardware_mute": hardware_mute}
        cleanup = _terminate_tx_processes_with_rescan(pids, term_timeout=8.0, kill_timeout=3.0)
        if proc is not None:
            try:
                proc.wait(timeout=0.2)
            except Exception:
                pass
            server.dual_tx_last_returncode = proc.returncode
        server.dual_tx_process = None
        hardware_mute = _mute_and_verify_tx_hardware(mute_uris)
        return {
            "已停止": True,
            "退出码": server.dual_tx_last_returncode,
            "日志": str(server.dual_tx_log_path) if server.dual_tx_log_path else None,
            "cleanup": cleanup,
            "hardware_mute": hardware_mute,
        }


def check_four_sdr(
    state: dict[str, Any],
    *,
    busy_tx_uris: set[str] | None = None,
    busy_tx_roles: set[str] | None = None,
) -> dict[str, Any]:
    config = _four_sdr_config(state)
    busy = {str(uri).strip() for uri in (busy_tx_uris or set()) if str(uri).strip()}
    busy_roles = {str(role).strip() for role in (busy_tx_roles or set()) if str(role).strip()}
    result: dict[str, Any] = {}
    for role in FOUR_SDR_ROLE_LABELS:
        uri = str(_as_dict(config.get(role)).get("uri") or "").strip()
        if role == "interference_tx" and not uri:
            result[role] = _annotate_sdr_role(
                role,
                {"uri": "", "available": None, "message": "当前未连接，等待识别", "summary": []},
            )
        elif uri in busy or role in busy_roles:
            result[role] = _annotate_sdr_role(role, _busy_sdr_result(uri))
        else:
            result[role] = _annotate_sdr_role(role, check_sdr_uri(uri))
    topology_errors, topology_warnings = _four_sdr_topology_checks(state)
    role_errors = [
        f"{_as_dict(result.get(role)).get('role_label', role)} 类型不匹配"
        for role in FOUR_SDR_ROLE_LABELS
        if _as_dict(result.get(role)).get("available") is False and _as_dict(result.get(role)).get("role_match") is False
    ]
    topology_errors.extend(role_errors)
    broadcast_serial = str(_as_dict(result.get("broadcast_tx")).get("hw_serial") or "").strip()
    interference_serial = str(_as_dict(result.get("interference_tx")).get("hw_serial") or "").strip()
    if broadcast_serial and interference_serial and broadcast_serial == interference_serial:
        topology_errors.append(
            f"信息波 TX 与干扰波 TX 报告相同硬件序列号 {broadcast_serial}，实际是同一台 SDR"
        )
    interference_installed = bool(str(_as_dict(config.get("interference_tx")).get("uri") or "").strip())
    required_roles = ["broadcast_tx"] + (["interference_tx"] if interference_installed else [])
    result["topology"] = {
        "ok": all(_as_dict(result.get(role)).get("available") is True for role in required_roles) and not topology_errors,
        "mode": "dual" if interference_installed else "single_broadcast",
        "errors": topology_errors,
        "warnings": topology_warnings,
    }
    return result


def default_state() -> dict[str, Any]:
    four_sdr = _default_four_sdr()
    return {
        "side": os.environ.get("RM_RADIO_SIDE", "blue").strip().lower(),
        "wave": "broadcast",
        "level": _int_value(os.environ.get("INTERFERENCE_LEVEL", 1), 1, 1, 3),
        "password": os.environ.get("INTERFERENCE_PASSWORD", "R1L001"),
        "mode": "virtual",
        "virtual": {"bit_error_rate": 0.0, "random_seed": 2026, "max_access_hamming": 3},
        "tx": {
            "uri": os.environ.get("BROADCAST_TX_URI", "ip:192.168.2.1"),
            "rf_port": os.environ.get("TX_RF_PORT", "A"),
            "attenuation_db": _float_value(
                os.environ.get("BROADCAST_TX_ATTENUATION_DB", 61),
                61,
                0.0,
                89.75,
            ),
            "sample_rate": DEFAULT_TX_SAMPLE_RATE,
            "lab_confirm": False,
        },
        "four_sdr": four_sdr,
        "broadcast": _default_broadcast_config(),
    }


class RadioTxHandler(SimpleHTTPRequestHandler):
    server: RadioTxHttpServer

    def __init__(self, *args: Any, **kwargs: Any):
        super().__init__(*args, directory=str(WEB_ROOT), **kwargs)

    def log_message(self, fmt: str, *args: Any) -> None:
        try:
            sys.stderr.write("[%s] %s\n" % (time.strftime("%H:%M:%S"), fmt % args))
        except (BrokenPipeError, ConnectionResetError):
            return

    def end_headers(self) -> None:
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def _send_json(self, payload: Any, status: HTTPStatus = HTTPStatus.OK) -> None:
        body = _json_dumps(payload)
        try:
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            return

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0") or "0")
        if length > 8 * 1024 * 1024:
            raise ValueError("请求体过大")
        raw = self.rfile.read(length) if length else b"{}"
        data = json.loads(raw.decode("utf-8"))
        if not isinstance(data, dict):
            raise ValueError("请求 JSON 必须是对象")
        return data

    def do_GET(self) -> None:
        path = unquote(urlparse(self.path).path)
        if path == "/api/state":
            self._send_json(
                {
                    "state": default_state(),
                    "server": {
                        "workspace": str(WS_ROOT),
                        "radio_tx_dir": str(RADIO_TX_DIR),
                        "lab_tx_env_confirm": _bool_env("RM_RADIO_LAB_TX_ANTENNA_CONFIRM", False),
                        "lab_tx_session_confirm": bool(self.server.lab_tx_session_confirm),
                        "lab_tx_effective_confirm": self.server.lab_tx_effective_confirm(),
                        "generated_root": str(GENERATED_ROOT),
                        "tx": self.server.tx_status(),
                        "dual_tx": self.server.dual_tx_status(),
                        "waves": {
                            "broadcast": self.server.wave_status("broadcast"),
                            "interference": self.server.wave_status("interference"),
                        },
                        "tx_hardware": self.server.cached_tx_hardware_status(),
                    },
                }
            )
            return
        if path == "/api/tx-status":
            self._send_json({"tx": self.server.tx_status(), "dual_tx": self.server.dual_tx_status()})
            return
        if path == "/api/check-sdr":
            state = default_state()
            self._send_json({"tx": self.server.check_sdr_uri_safely(
                _as_dict(state.get("tx")).get("uri", ""),
                role="broadcast_tx",
            )})
            return
        if path == "/api/check-four-sdr":
            self._send_json(self.server.check_four_sdr_safely(default_state()))
            return
        if path == "/api/tx-hardware-status":
            self._send_json(self.server.refresh_tx_hardware_status())
            return
        if path == "/":
            self.path = "/index.html"
        super().do_GET()

    def do_POST(self) -> None:
        path = unquote(urlparse(self.path).path)
        try:
            data = self._read_json()
            state = _as_dict(data.get("state", data))
            if path == "/api/validate":
                self._send_json(validate_state(state, self.server.lab_tx_effective_confirm()))
            elif path == "/api/check-sdr":
                tx = _as_dict(state.get("tx"))
                wave = str(state.get("wave") or "broadcast").strip().lower()
                role = "interference_tx" if wave == "interference" else "broadcast_tx"
                self._send_json({"tx": self.server.check_sdr_uri_safely(
                    str(tx.get("uri") or ""),
                    role=role,
                )})
            elif path == "/api/check-four-sdr":
                self._send_json(self.server.check_four_sdr_safely(state))
            elif path == "/api/tx-hardware-status":
                self._send_json(self.server.refresh_tx_hardware_status())
            elif path == "/api/lab-confirm":
                self.server.lab_tx_session_confirm = bool(data.get("confirmed"))
                self._send_json(
                    {
                        "lab_tx_env_confirm": _bool_env("RM_RADIO_LAB_TX_ANTENNA_CONFIRM", False),
                        "lab_tx_session_confirm": bool(self.server.lab_tx_session_confirm),
                        "lab_tx_effective_confirm": self.server.lab_tx_effective_confirm(),
                    }
                )
            elif path == "/api/generate":
                validation = validate_state(state, self.server.lab_tx_effective_confirm())
                if not validation["ok"]:
                    self._send_json(validation, HTTPStatus.BAD_REQUEST)
                    return
                self._send_json(generate_files(state))
            elif path == "/api/tx-wave-start":
                self._send_json(start_wave(self.server, state, data.get("wave")))
            elif path == "/api/tx-wave-stop":
                self._send_json(stop_wave(self.server, data.get("wave")))
            elif path == "/api/tx-wave-reconfigure":
                self._send_json(reconfigure_wave(self.server, state, data.get("wave"), _as_dict(data.get("changes"))))
            elif path == "/api/randomize-broadcast":
                self._send_json({"broadcast": randomize_broadcast_config(state)})
            elif path == "/api/randomize-password":
                self._send_json({"password": _random_password()})
            elif path == "/api/start-tx":
                self._send_json(start_tx(self.server, state))
            elif path == "/api/stop-tx":
                self._send_json(stop_tx(self.server))
            elif path == "/api/start-dual-tx":
                self._send_json(start_dual_tx(self.server, state))
            elif path == "/api/stop-dual-tx":
                self._send_json(stop_dual_tx(self.server))
            else:
                self._send_json({"error": f"未知接口：{path}"}, HTTPStatus.NOT_FOUND)
        except PermissionError as exc:
            self._send_json({"error": str(exc)}, HTTPStatus.FORBIDDEN)
        except TxSafetyError as exc:
            traceback.print_exc(file=sys.stderr)
            self._send_json({"error": str(exc), "safety": exc.safety}, HTTPStatus.BAD_REQUEST)
        except Exception as exc:
            traceback.print_exc(file=sys.stderr)
            self._send_json({"error": str(exc), "traceback": traceback.format_exc()}, HTTPStatus.BAD_REQUEST)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="LabTX lab dashboard")
    parser.add_argument("--host", default=os.environ.get("RADIO_TX_DASHBOARD_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("RADIO_TX_DASHBOARD_PORT", "8780")))
    args = parser.parse_args(argv)

    if not WEB_ROOT.exists():
        raise FileNotFoundError(f"前端目录不存在：{WEB_ROOT}")
    _ensure_dir(GENERATED_ROOT)
    try:
        server = RadioTxHttpServer((args.host, args.port), RadioTxHandler)
    except OSError as exc:
        if exc.errno in {98, 10048}:
            hint = _port_owner_hint(args.port)
            detail = f"\n占用信息：\n{hint}" if hint else ""
            raise SystemExit(f"前端端口 {args.host}:{args.port} 已被占用，请先停止旧 dashboard 或换端口。{detail}") from exc
        raise
    print(f"LabTX dashboard serving http://{args.host}:{args.port}/", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        for label, cleanup in (
            ("broadcast wave", lambda: stop_wave(server, "broadcast")),
            ("interference wave", lambda: stop_wave(server, "interference")),
            ("dual TX", lambda: stop_dual_tx(server)),
            ("generic TX", lambda: stop_tx(server)),
        ):
            try:
                cleanup()
            except Exception as exc:
                print(f"LabTX shutdown warning ({label}): {exc}", file=sys.stderr)
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
