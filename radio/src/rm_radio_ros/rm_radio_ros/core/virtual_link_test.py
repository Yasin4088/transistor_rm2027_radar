from __future__ import annotations

import argparse
import json
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

from .radio_config import BROADCAST_FREQUENCIES, interference_setters_for_side_level
from .radar_wireless_constraints import (
    BUFF_FIELD_MAX,
    BULLET_MAX,
    HP_MAX,
    POSITION_X_MAX_CM,
    POSITION_Y_MAX_CM,
    clamp_int,
    normalize_occupation_bits,
)
from .referee_bridge import RefereeBridge
from .rm_protocol import (
    ACCESS_CODES,
    RADAR_AIR_DATA_LENGTHS,
    AccessCodeAirPacketExtractor,
    RefereeFrame,
    RefereeFrameAssembler,
    build_referee_frame,
    bytes_to_bits,
    put_le_u16,
    put_le_u32,
)


AIR_HEADER = b"\x00\x0F\x00\x0F"
AIR_PAYLOAD_SIZE = 15
INTERFERENCE_PASSWORDS = {
    1: b"R1L001",
    2: b"R2L002",
    3: b"R3L003",
}
ROBOT_KEYS = [
    "opponent_hero",
    "opponent_engineer",
    "opponent_infantry_3",
    "opponent_infantry_4",
    "opponent_aerial",
    "opponent_sentry",
]
BUFF_ROBOT_KEYS = [
    "opponent_hero",
    "opponent_engineer",
    "opponent_infantry_3",
    "opponent_infantry_4",
    "opponent_sentry",
]


@dataclass
class DecodeResult:
    air_payload_count: int
    frames: list[RefereeFrame]
    bridge_outputs: list[dict]
    bit_count: int
    bit_error_count: int


def _chunks(data: Sequence[int], size: int = AIR_PAYLOAD_SIZE) -> list[bytes]:
    out = []
    for offset in range(0, len(data), size):
        chunk = bytes(data[offset : offset + size])
        if len(chunk) < size:
            chunk += b"\x00" * (size - len(chunk))
        out.append(chunk)
    return out


def _air_packet(access_name: str, payload: bytes) -> bytes:
    if len(payload) != AIR_PAYLOAD_SIZE:
        raise ValueError(f"air payload must be 15 bytes, got {len(payload)}")
    return ACCESS_CODES[access_name] + AIR_HEADER + payload


def _int_value(value: Any, default: int = 0, low: int = 0, high: int = 0xFFFF) -> int:
    try:
        parsed = int(value)
    except Exception:
        parsed = default
    return min(max(parsed, low), high)


def _u8(value: Any, default: int = 0) -> int:
    return _int_value(value, default=default, low=0, high=0xFF)


def _apply_bit_noise(bits: list[int], bit_error_rate: float, seed: int) -> tuple[list[int], int]:
    rate = max(0.0, min(float(bit_error_rate), 0.5))
    if rate <= 0.0:
        return bits, 0
    noisy = list(bits)
    rng = random.Random(int(seed))
    changed = 0
    for index, bit in enumerate(noisy):
        if rng.random() < rate:
            noisy[index] = 1 - int(bit)
            changed += 1
    return noisy, changed


def _stream_decode(
    access_name: str,
    packets: Iterable[bytes],
    *,
    bit_error_rate: float = 0.0,
    random_seed: int = 2026,
    max_access_hamming: int = 3,
) -> DecodeResult:
    bit_stream = []
    for packet in packets:
        bit_stream.extend(bytes_to_bits(packet))
    bit_stream, bit_error_count = _apply_bit_noise(bit_stream, bit_error_rate, random_seed)

    extractor = AccessCodeAirPacketExtractor(
        access_codes=[ACCESS_CODES[access_name]],
        max_access_hamming=max(0, int(max_access_hamming)),
        max_length_hamming=0,
        allow_inverted=False,
    )
    assembler = RefereeFrameAssembler(allowed_cmd_lengths=RADAR_AIR_DATA_LENGTHS)
    frames: list[RefereeFrame] = []
    air_payload_count = 0

    # Feed uneven chunks to exercise streaming buffering instead of one-shot parsing.
    for offset in range(0, len(bit_stream), 57):
        payloads = extractor.push_bit_bytes(bit_stream[offset : offset + 57])
        air_payload_count += len(payloads)
        for payload in payloads:
            frames.extend(assembler.push_air_payload(payload))

    bridge_outputs = RefereeBridge().process_frames(frames)
    return DecodeResult(
        air_payload_count=air_payload_count,
        frames=frames,
        bridge_outputs=bridge_outputs,
        bit_count=len(bit_stream),
        bit_error_count=bit_error_count,
    )


def _default_broadcast_config() -> dict[str, Any]:
    return {
        "enabled_cmds": ["0x0A01", "0x0A02", "0x0A03", "0x0A04", "0x0A05"],
        "robots": {
            "opponent_hero": {"x": 400, "y": 300},
            "opponent_engineer": {"x": 800, "y": 500},
            "opponent_infantry_3": {"x": 1200, "y": 700},
            "opponent_infantry_4": {"x": 1600, "y": 900},
            "opponent_aerial": {"x": 2000, "y": 1100},
            "opponent_sentry": {"x": 2400, "y": 1300},
        },
        "hp": {
            "opponent_hero": 500,
            "opponent_engineer": 250,
            "opponent_infantry_3": 400,
            "opponent_infantry_4": 390,
            "reserved": 0,
            "opponent_sentry": 400,
        },
        "bullets": {
            "opponent_hero": 45,
            "opponent_infantry_3": 120,
            "opponent_infantry_4": 118,
            "opponent_aerial": 0,
            "opponent_sentry": 300,
        },
        "macro": {
            "remaining_coins": 123,
            "total_coins": 456,
            "occupation_bits": (1 << 0) | (2 << 1) | (1 << 3) | (2 << 4) | (1 << 6) | (1 << 13),
        },
        "buffs": {},
        "sentry_mode": 2,
        "robot_main_status": {
            name: 0
            for name in BUFF_ROBOT_KEYS
        },
    }


def _position_payload(config: dict[str, Any]) -> bytes:
    robots = config.get("robots") if isinstance(config.get("robots"), dict) else {}
    out: list[int] = []
    for name in ROBOT_KEYS:
        robot = robots.get(name) if isinstance(robots.get(name), dict) else {}
        out.extend(put_le_u16(clamp_int(robot.get("x"), 0, POSITION_X_MAX_CM)))
        out.extend(put_le_u16(clamp_int(robot.get("y"), 0, POSITION_Y_MAX_CM)))
    return bytes(out)


def _hp_payload(config: dict[str, Any]) -> bytes:
    hp = config.get("hp") if isinstance(config.get("hp"), dict) else {}
    defaults = _default_broadcast_config()["hp"]
    return b"".join(
        bytes(put_le_u16(clamp_int(hp.get(key, defaults[key]), 0, HP_MAX[key])))
        for key in ("opponent_hero", "opponent_engineer", "opponent_infantry_3", "opponent_infantry_4", "reserved", "opponent_sentry")
    )


def _bullet_payload(config: dict[str, Any]) -> bytes:
    bullets = config.get("bullets") if isinstance(config.get("bullets"), dict) else {}
    defaults = _default_broadcast_config()["bullets"]
    return b"".join(
        bytes(put_le_u16(clamp_int(bullets.get(key, defaults[key]), 0, BULLET_MAX[key])))
        for key in ("opponent_hero", "opponent_infantry_3", "opponent_infantry_4", "opponent_aerial", "opponent_sentry")
    )


def _macro_payload(config: dict[str, Any]) -> bytes:
    macro = config.get("macro") if isinstance(config.get("macro"), dict) else {}
    total = clamp_int(macro.get("total_coins"), 0, 0xFFFF, 300)
    remaining = clamp_int(macro.get("remaining_coins"), 0, total, 120)
    return bytes(
        put_le_u16(remaining)
        + put_le_u16(total)
        + put_le_u32(normalize_occupation_bits(macro.get("occupation_bits")))
    )


def _buff_payload(config: dict[str, Any]) -> bytes:
    raw = config.get("buff_raw")
    if isinstance(raw, list) and len(raw) in (36, 41):
        payload = bytes(_u8(item) for item in raw)
        # Parse legacy raw configurations and pass them through the same
        # rulebook constraints instead of allowing arbitrary bytes to bypass
        # the structured editor. V1.3.1 omitted the five main-status bytes.
        parsed_buffs: dict[str, dict[str, int]] = {}
        for index, name in enumerate(BUFF_ROBOT_KEYS):
            offset = index * 7
            parsed_buffs[name] = {
                "hp_recovery_percent": payload[offset],
                "shooting_heat_cooling": payload[offset + 1] | (payload[offset + 2] << 8),
                "defense_percent": payload[offset + 3],
                "negative_defense_percent": payload[offset + 4],
                "attack_percent": payload[offset + 5] | (payload[offset + 6] << 8),
            }
        parsed = {
            "buffs": parsed_buffs,
            "sentry_mode": payload[35],
            "robot_main_status": {
                name: payload[36 + index] if len(payload) == 41 else 0
                for index, name in enumerate(BUFF_ROBOT_KEYS)
            },
        }
        return _buff_payload(parsed)
    buffs = config.get("buffs") if isinstance(config.get("buffs"), dict) else {}
    main_status = (
        config.get("robot_main_status")
        if isinstance(config.get("robot_main_status"), dict)
        else {}
    )
    out: list[int] = []
    for name in BUFF_ROBOT_KEYS:
        item = buffs.get(name) if isinstance(buffs.get(name), dict) else {}
        out.append(clamp_int(item.get("hp_recovery_percent"), 0, BUFF_FIELD_MAX["hp_recovery_percent"][name]))
        out.extend(put_le_u16(clamp_int(item.get("shooting_heat_cooling"), 0, BUFF_FIELD_MAX["shooting_heat_cooling"][name])))
        out.append(clamp_int(item.get("defense_percent"), 0, BUFF_FIELD_MAX["defense_percent"][name]))
        out.append(clamp_int(item.get("negative_defense_percent"), 0, BUFF_FIELD_MAX["negative_defense_percent"][name]))
        out.extend(put_le_u16(clamp_int(item.get("attack_percent"), 0, BUFF_FIELD_MAX["attack_percent"][name])))
    out.append(clamp_int(config.get("sentry_mode"), 1, 6, 2))
    out.extend(clamp_int(main_status.get(name), 0, 3) for name in BUFF_ROBOT_KEYS)
    return bytes(out)


def _build_broadcast_air_packets(config: dict[str, Any] | None = None) -> list[bytes]:
    cfg = _default_broadcast_config()
    if isinstance(config, dict):
        cfg = _deep_merge(cfg, config)
    enabled = cfg.get("enabled_cmds")
    if not isinstance(enabled, list) or not enabled:
        enabled = ["0x0A01", "0x0A02", "0x0A03", "0x0A04", "0x0A05"]
    enabled_set = {str(item).upper() for item in enabled}

    specs = [
        ("0X0A01", 0x0A01, _position_payload(cfg), 1),
        ("0X0A02", 0x0A02, _hp_payload(cfg), 2),
        ("0X0A03", 0x0A03, _bullet_payload(cfg), 3),
        ("0X0A04", 0x0A04, _macro_payload(cfg), 4),
        ("0X0A05", 0x0A05, _buff_payload(cfg), 5),
    ]
    frames = [
        build_referee_frame(cmd_id, payload, seq=seq)
        for key, cmd_id, payload, seq in specs
        if key in enabled_set
    ]
    payloads = _chunks(b"".join(frames))
    return [_air_packet("broadcast", payload) for payload in payloads]


def _deep_merge(base: dict[str, Any], update: dict[str, Any]) -> dict[str, Any]:
    out = dict(base)
    for key, value in update.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_merge(out[key], value)
        else:
            out[key] = value
    return out


def _build_interference_air_packets(level: int, password: bytes | str | None = None) -> list[bytes]:
    if password is None:
        password = INTERFERENCE_PASSWORDS[int(level)]
    if isinstance(password, str):
        password = password.encode("ascii")
    if len(password) != 6:
        raise ValueError("interference password must be exactly 6 bytes")
    frame = build_referee_frame(0x0A06, password, seq=int(level))
    payloads = _chunks(frame)
    rng = random.Random(0x202600 + int(level))
    while len(payloads) < 9:
        payloads.append(bytes(rng.getrandbits(8) for _ in range(AIR_PAYLOAD_SIZE)))
    return [_air_packet("interference", payload) for payload in payloads]


def _frame_summary(frame: RefereeFrame) -> dict:
    data = frame.to_dict()
    return {
        "cmd_hex": data["cmd_hex"],
        "command": data["command"],
        "seq": data["seq"],
        "data_length": data["data_length"],
        "parsed": data["parsed"],
    }


def run_virtual_link(
    tx_side: str = "red",
    rx_side: str = "blue",
    *,
    broadcast_config: dict[str, Any] | None = None,
    interference_password: str | None = None,
    interference_level: int | None = None,
    bit_error_rate: float = 0.0,
    random_seed: int = 2026,
    max_access_hamming: int = 3,
) -> dict:
    if tx_side not in ("red", "blue"):
        raise ValueError("tx_side must be red or blue")
    if rx_side not in ("red", "blue"):
        raise ValueError("rx_side must be red or blue")

    broadcast_packets = _build_broadcast_air_packets(broadcast_config)
    broadcast_result = _stream_decode(
        "broadcast",
        broadcast_packets,
        bit_error_rate=bit_error_rate,
        random_seed=random_seed,
        max_access_hamming=max_access_hamming,
    )

    interference_results = {}
    levels = [int(interference_level)] if interference_level in (1, 2, 3) else [1, 2, 3]
    for level in levels:
        password = interference_password if interference_password is not None else None
        packets = _build_interference_air_packets(level, password=password)
        result = _stream_decode(
            "interference",
            packets,
            bit_error_rate=bit_error_rate,
            random_seed=int(random_seed) + level,
            max_access_hamming=max_access_hamming,
        )
        interference_results[level] = {
            "tx_settings": interference_setters_for_side_level(tx_side, level),
            "air_packet_count": len(packets),
            "air_payload_count": result.air_payload_count,
            "bit_count": result.bit_count,
            "bit_error_count": result.bit_error_count,
            "frames": [_frame_summary(frame) for frame in result.frames],
            "bridge_outputs": result.bridge_outputs,
        }

    return {
        "scenario": {
            "tx_side": tx_side,
            "rx_side": rx_side,
            "scope": "software air-packet loopback with optional demod-bit noise; no SDR, RF, AGC, or field-strength validation",
            "bit_error_rate": float(bit_error_rate),
            "random_seed": int(random_seed),
            "max_access_hamming": int(max_access_hamming),
        },
        "frequency_plan": {
            "broadcast_hz": BROADCAST_FREQUENCIES[tx_side],
            "interference": {
                level: interference_setters_for_side_level(tx_side, level)
                for level in (1, 2, 3)
            },
        },
        "broadcast": {
            "air_packet_count": len(broadcast_packets),
            "air_payload_count": broadcast_result.air_payload_count,
            "bit_count": broadcast_result.bit_count,
            "bit_error_count": broadcast_result.bit_error_count,
            "frames": [_frame_summary(frame) for frame in broadcast_result.frames],
            "bridge_outputs": broadcast_result.bridge_outputs,
        },
        "interference": interference_results,
    }


def _markdown_report(result: dict) -> str:
    broadcast_frames = result["broadcast"]["frames"]
    interference = result["interference"]
    ok_broadcast = [frame["cmd_hex"] for frame in broadcast_frames] == [
        "0x0A01",
        "0x0A02",
        "0x0A03",
        "0x0A04",
        "0x0A05",
    ]
    ok_interference = all(
        len(data["frames"]) == 1
        and data["frames"][0]["cmd_hex"] == "0x0A06"
        and data["bridge_outputs"]
        and data["bridge_outputs"][0]["type"] == "RadarCommand0121"
        for data in interference.values()
    )
    verdict = "通过" if ok_broadcast and ok_interference else "失败"

    lines = [
        "# 虚拟红方发射蓝方解析全链路测试",
        "",
        f"结论：**{verdict}**。",
        "",
        "## 测试范围",
        "",
        "- 发送侧：虚拟红方信息波、红方 1/2/3 级干扰波。",
        "- 接收侧：蓝方视角解析空口包，并输出裁判帧与 bridge 结果。",
        "- 链路层级：`Access Code -> 0x000F000F Header -> 15 字节 payload -> 0xA5 裁判帧 -> CRC -> 协议解析 -> referee_bridge`。",
        "- 不包含：SDR 硬件、GFSK 调制/解调误码、AGC、场强和天线链路。",
        "",
        "## 频点表校验",
        "",
        f"- 红方信息波频点：`{result['frequency_plan']['broadcast_hz']} Hz`。",
    ]
    for level, settings in result["frequency_plan"]["interference"].items():
        lines.append(
            f"- 红方 {level} 级干扰波：`center_f={settings['center_f']} Hz`, `BW_ganrao={settings['BW_ganrao']} Hz`。"
        )

    lines.extend([
        "",
        "## 信息波结果",
        "",
        f"- 空口包数：`{result['broadcast']['air_packet_count']}`。",
        f"- 提取 payload 数：`{result['broadcast']['air_payload_count']}`。",
        f"- CRC 通过裁判帧：`{len(broadcast_frames)}`。",
        "",
        "| 顺序 | cmd | command | 长度 | 关键解析 |",
        "| ---: | --- | --- | ---: | --- |",
    ])
    for index, frame in enumerate(broadcast_frames, start=1):
        parsed = frame["parsed"]
        if frame["cmd_hex"] == "0x0A01":
            detail = f"hero={parsed['official_positions_cm']['opponent_hero']}"
        elif frame["cmd_hex"] == "0x0A02":
            detail = f"hero_hp={parsed['official_hp']['opponent_hero']}, sentry_hp={parsed['official_hp']['opponent_sentry']}"
        elif frame["cmd_hex"] == "0x0A03":
            detail = f"hero_bullets={parsed['official_bullet_allowance']['opponent_hero']}"
        elif frame["cmd_hex"] == "0x0A04":
            detail = f"coins={parsed['remaining_coins']}/{parsed['total_coins']}"
        elif frame["cmd_hex"] == "0x0A05":
            detail = (
                f"sentry_mode={parsed['sentry_mode']}, "
                f"hero_status={parsed['robot_main_status']['opponent_hero']}"
            )
        else:
            detail = "-"
        lines.append(
            f"| {index} | `{frame['cmd_hex']}` | `{frame['command']}` | {frame['data_length']} | {detail} |"
        )

    bridge_types = [item["type"] for item in result["broadcast"]["bridge_outputs"]]
    lines.extend([
        "",
        f"- 信息波 bridge 输出：`{bridge_types}`。",
        "",
        "## 干扰波结果",
        "",
        "| 等级 | 频点 Hz | 带宽 Hz | 空口包数 | payload 数 | 裁判帧 | 密钥 | bridge |",
        "| ---: | ---: | ---: | ---: | ---: | --- | --- | --- |",
    ])
    for level, data in interference.items():
        settings = data["tx_settings"]
        frame = data["frames"][0] if data["frames"] else {}
        parsed = frame.get("parsed", {})
        bridge = data["bridge_outputs"][0]["type"] if data["bridge_outputs"] else "-"
        lines.append(
            f"| {level} | {settings['center_f']} | {settings['BW_ganrao']} | {data['air_packet_count']} | "
            f"{data['air_payload_count']} | `{frame.get('cmd_hex', '-')}` | `{parsed.get('password', '-')}` | `{bridge}` |"
        )

    lines.extend([
        "",
        "## 判定",
        "",
        "- 红方信息波单轮 140 字节裁判帧数据被蓝方解析链路提取，并还原 `0x0A01~0x0A05`。",
        "- 红方 1/2/3 级干扰波均按 9 个空口包/100ms 的虚拟结构输入，每级均解析出 `0x0A06` 并生成 `RadarCommand0121`。",
        "- 因本测试不经过 SDR/RF，不能证明实际场强、天线距离、AGC 或现场抗干扰裕量，只证明当前软件空口结构与协议解析链路可行。",
        "",
        "## 原始摘要",
        "",
        "```json",
        json.dumps(result, ensure_ascii=False, indent=2),
        "```",
        "",
    ])
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tx-side", default="red", choices=["red"])
    parser.add_argument("--rx-side", default="blue", choices=["blue"])
    parser.add_argument(
        "--output",
        default="src/rm_radio_ros/test_results/虚拟红方发射蓝方解析全链路测试.md",
    )
    args = parser.parse_args(argv)

    result = run_virtual_link(tx_side=args.tx_side, rx_side=args.rx_side)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(_markdown_report(result), encoding="utf-8")
    print(json.dumps({
        "output": str(output),
        "broadcast_frames": [frame["cmd_hex"] for frame in result["broadcast"]["frames"]],
        "interference_passwords": {
            str(level): data["frames"][0]["parsed"]["password"]
            for level, data in result["interference"].items()
        },
    }, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
