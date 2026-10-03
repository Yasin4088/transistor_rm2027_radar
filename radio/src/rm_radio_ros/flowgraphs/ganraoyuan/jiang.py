#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Packet builder and scheduler-clocked continuous air-bit source.

The archived project only kept ``Hier_Block/untitled.grc`` and Python 3.12
``__pycache__`` files.  On the WSL GNU Radio runtime used here (Python 3.10),
those cache files cannot be imported and ``grcc`` cannot regenerate this hier
block.  This source file provides the same public API that ``RM.py`` expects:

    jiang(Access_Code=..., Period=..., com_id=..., payload_size=...)

and message output ports named ``access``, ``header`` and ``payload``.  The
runtime TX path now consumes the uint8 stream output instead: every scheduler
request is satisfied, complete 27-byte air packets are emitted at the official
payload cadence, and the remaining packet interval is represented explicitly
by alternating idle bits.  This keeps the GFSK phase continuous and prevents a
non-cyclic IIO sink from using DAC underflow as an implicit packet gap.
"""

from __future__ import annotations

import random
import threading
from collections import deque
from typing import Deque, Iterable, List, Sequence, Tuple

import numpy as np
from gnuradio import gr
import pmt

from rm_radio_ros.core.radar_wireless_constraints import random_payload_for_command


_DEFAULT_PASSWORD = [ord(ch) for ch in "ABC123"]
_BROADCAST_ACCESS_CODE = [0x2F, 0x6F, 0x4C, 0x74, 0xB9, 0x14, 0x49, 0x2E]
_INTERFERENCE_ACCESS_CODE = [0x16, 0xE8, 0xD3, 0x77, 0x15, 0x1C, 0x71, 0x2D]
_AIR_PAYLOAD_SIZE = 15
_BROADCAST_BYTES_PER_SECOND = 1400.0
_INTERFERENCE_BYTES_PER_SECOND = 1350.0
_OFFICIAL_SYMBOL_RATE = 1_000_000.0 / 47.0
_AIR_PACKET_BYTES = 8 + 4 + _AIR_PAYLOAD_SIZE
_AIR_PACKET_BITS = _AIR_PACKET_BYTES * 8


def _u8_list(values: Iterable[int]) -> List[int]:
    return [int(v) & 0xFF for v in values]


def _crc8_rm(data: Iterable[int]) -> int:
    """CRC8 used by the RM frame header, matching the official lookup table."""
    crc = 0xFF
    for byte in _u8_list(data):
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                crc = ((crc >> 1) ^ 0x8C) & 0xFF
            else:
                crc = (crc >> 1) & 0xFF
    return crc & 0xFF


def _crc16_rm(data: Iterable[int]) -> int:
    """CRC16 used by RM frames, appended low byte first.

    Equivalent to the RoboMaster reference implementation with init 0xFFFF
    and the reflected 0x8408 polynomial/table.
    """
    crc = 0xFFFF
    for byte in _u8_list(data):
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc & 0xFFFF


def _pdu_u8vector(data: Iterable[int]):
    data = _u8_list(data)
    return pmt.cons(pmt.PMT_NIL, pmt.init_u8vector(len(data), data))


class jiang(gr.sync_block):
    """Continuous air-bit source retaining the missing GRC block's API."""

    def __init__(
        self,
        Access_Code=None,
        Period=10,
        com_id=None,
        payload_size=6,
        command_cycle=None,
        symbol_rate=_OFFICIAL_SYMBOL_RATE,
    ):
        gr.sync_block.__init__(
            self,
            name="jiang_continuous_air_bits",
            in_sig=None,
            out_sig=[np.uint8],
        )

        self._port_access = pmt.intern("access")
        self._port_header = pmt.intern("header")
        self._port_payload = pmt.intern("payload")
        self.message_port_register_out(self._port_access)
        self.message_port_register_out(self._port_header)
        self.message_port_register_out(self._port_payload)

        self._lock = threading.RLock()

        self.Access_Code = _u8_list(
            Access_Code
            if Access_Code is not None
            else _BROADCAST_ACCESS_CODE
        )
        self.Period = float(Period)
        self.com_id = _u8_list(com_id if com_id is not None else [0x0A, 0x01])
        self.payload_size = int(payload_size)
        self.SOF = 0xA5
        self.seq = 0x01
        self.command_cycle = self._normalize_command_cycle(command_cycle)
        self._payload_data = self._new_payload_data_for_command(self.com_id, self.payload_size)
        self._packet_queue: Deque[List[int]] = deque()
        self._broadcast_byte_queue: Deque[int] = deque()
        self._interference_byte_queue: Deque[int] = deque()
        self._interference_phase_prefilled = False
        self._symbol_rate = max(float(symbol_rate), 1.0)
        self._active_packet_bits: Deque[int] = deque()
        self._packet_start_symbols: Deque[int] = deque(maxlen=32)
        self._symbol_index = 0
        self._next_packet_symbol = 0.0
        self._idle_bit = 0
        self._packet_count = 0
        self._packet_symbol_count = 0
        self._idle_symbol_count = 0

    def _clear_payload_queues_locked(self) -> None:
        self._packet_queue.clear()
        self._broadcast_byte_queue.clear()
        self._interference_byte_queue.clear()
        self._interference_phase_prefilled = False

    def _reset_stream_locked(self, *, reset_counters: bool = False) -> None:
        self._active_packet_bits.clear()
        self._idle_bit = 0
        if reset_counters:
            self._symbol_index = 0
            self._packet_count = 0
            self._packet_symbol_count = 0
            self._idle_symbol_count = 0
            self._packet_start_symbols.clear()
        self._next_packet_symbol = float(self._symbol_index)

    def _target_chunks_per_period(
        self,
        frame_chunk_count: int = 1,
        bytes_per_second: float = _INTERFERENCE_BYTES_PER_SECOND,
    ) -> int:
        target_chunks = int(
            round(
                float(bytes_per_second)
                * max(self.Period, 1.0)
                / 1000.0
                / _AIR_PAYLOAD_SIZE
            )
        )
        return max(int(frame_chunk_count), target_chunks)

    def _target_bytes_per_period(self, bytes_per_second: float, minimum: int = 0) -> int:
        target_bytes = int(
            round(float(bytes_per_second) * max(self.Period, 1.0) / 1000.0)
        )
        return max(int(minimum), target_bytes)

    @staticmethod
    def _new_payload_data(size: int) -> List[int]:
        # Deterministic mock data keeps dry-runs and packet checks reproducible.
        return [0] * max(0, int(size))

    @staticmethod
    def _new_payload_data_for_command(cmd_id: Sequence[int], size: int) -> List[int]:
        if _u8_list(cmd_id) == [0x0A, 0x06] and int(size) == 6:
            return list(_DEFAULT_PASSWORD)
        return jiang._new_payload_data(size)

    @staticmethod
    def _normalize_command_cycle(command_cycle) -> List[Tuple[List[int], int, List[int] | None, bool]]:
        if not command_cycle:
            return []
        normalized: List[Tuple[List[int], int, List[int] | None, bool]] = []
        for item in command_cycle:
            payload_data = None
            randomize_payload = False
            if isinstance(item, dict):
                cmd = item.get("cmd_id", item.get("com_id"))
                size = int(item.get("payload_size", item.get("data_length", 0)))
                randomize_payload = item.get("randomize_payload") is True
                if "payload_data" in item:
                    payload_data = _u8_list(item["payload_data"])
                    size = len(payload_data)
            else:
                if len(item) == 4:
                    cmd, size, payload_data, randomize_payload = item
                    payload_data = None if payload_data is None else _u8_list(payload_data)
                    size = int(size) if payload_data is None else len(payload_data)
                    randomize_payload = bool(randomize_payload)
                elif len(item) == 3:
                    cmd, size, payload_data = item
                    payload_data = _u8_list(payload_data)
                    size = len(payload_data)
                else:
                    cmd, size = item
                    size = int(size)
            normalized.append((_u8_list(cmd), size, payload_data, randomize_payload))
        return normalized

    @staticmethod
    def _random_payload_data(size: int) -> List[int]:
        return [random.getrandbits(8) for _ in range(max(0, int(size)))]

    @staticmethod
    def _random_payload_data_for_command(cmd_id: Sequence[int], size: int) -> List[int]:
        # Lab TX automatic mode must randomize fields, not raw bytes.  This
        # preserves every 0x0A01~0x0A05 range, enum and reserved bit defined by
        # the RMUC 2026 protocol/rulebook while still changing every period.
        return random_payload_for_command(cmd_id, size, random)

    @staticmethod
    def _cmd_id_le(cmd_id: Sequence[int]) -> List[int]:
        cmd = _u8_list(cmd_id)
        if len(cmd) != 2:
            raise ValueError("cmd_id must contain exactly 2 bytes")
        # RM.py historically stores command ids as [0x0A, 0x01] for 0x0A01.
        # The referee serial protocol itself is little-endian on the wire.
        return [cmd[1], cmd[0]]

    def _build_frame(self, cmd_id: Sequence[int], payload_data: Sequence[int]) -> List[int]:
        data = _u8_list(payload_data)
        data_length = len(data)
        seq = self.seq & 0xFF
        self.seq = (self.seq + 1) & 0xFF
        header = [
            self.SOF,
            data_length & 0xFF,
            (data_length >> 8) & 0xFF,
            seq,
        ]
        frame = header + [_crc8_rm(header)] + self._cmd_id_le(cmd_id) + data
        crc16 = _crc16_rm(frame)
        return frame + [crc16 & 0xFF, (crc16 >> 8) & 0xFF]

    @staticmethod
    def _chunks(data: Sequence[int], size: int = 15) -> List[List[int]]:
        chunks = []
        for offset in range(0, len(data), size):
            chunk = list(data[offset : offset + size])
            if len(chunk) < size:
                chunk.extend([0] * (size - len(chunk)))
            chunks.append(chunk)
        return chunks

    def _current_frames(self) -> List[int]:
        frames: List[int] = []
        if self.command_cycle:
            for cmd_id, size, payload_data, randomize_payload in self.command_cycle:
                if randomize_payload:
                    payload = self._random_payload_data_for_command(cmd_id, size)
                elif payload_data is not None:
                    payload = payload_data
                else:
                    payload = self._new_payload_data_for_command(cmd_id, size)
                frames.extend(self._build_frame(cmd_id, payload))
            return frames
        return self._build_frame(
            self.com_id,
            self._payload_data[: self.payload_size],
        )

    def _append_broadcast_period(self) -> None:
        frames = self._current_frames()
        period_bytes = self._target_bytes_per_period(
            _BROADCAST_BYTES_PER_SECOND,
            minimum=len(frames),
        )
        self._broadcast_byte_queue.extend(frames)
        self._broadcast_byte_queue.extend(
            random.getrandbits(8)
            for _ in range(period_bytes - len(frames))
        )

    def _next_broadcast_payload(self) -> List[int]:
        # V2.0.0 carries 140 bytes every 100 ms. Because 140 is not divisible
        # by the 15-byte air payload, retain the remaining 5/10 bytes and let
        # subsequent referee-frame cycles cross air-packet boundaries.
        while len(self._broadcast_byte_queue) < _AIR_PAYLOAD_SIZE:
            self._append_broadcast_period()
        return [
            self._broadcast_byte_queue.popleft()
            for _ in range(_AIR_PAYLOAD_SIZE)
        ]

    def _messages(self):
        access = self.Access_Code
        header = [0x00, 0x0F, 0x00, 0x0F]
        if self._is_broadcast_stream():
            return access, header, self._next_broadcast_payload()

        if not self._packet_queue:
            frames = self._current_frames()
            period_bytes = self._target_chunks_per_period(max(1, (len(frames) + 14) // 15)) * 15
            if len(frames) < period_bytes:
                frames.extend(random.getrandbits(8) for _ in range(period_bytes - len(frames)))
            self._packet_queue.extend(self._chunks(frames, 15))

        payload = self._packet_queue.popleft()
        return access, header, payload

    def _interference_chunks_per_period(self, frame_chunk_count: int = 1) -> int:
        return self._target_chunks_per_period(frame_chunk_count)

    def _append_interference_period(self) -> None:
        frame = self._build_frame(self.com_id, self._payload_data[: self.payload_size])
        frame_chunk_count = max(1, (len(frame) + 14) // 15)
        period_bytes = self._interference_chunks_per_period(frame_chunk_count) * 15

        if not self._interference_phase_prefilled:
            # The official stream is sliced after bytes are pushed, so the
            # referee frame boundary is independent from the 15-byte air packet.
            phase_offset = random.randrange(0, 15)
            self._interference_byte_queue.extend(random.getrandbits(8) for _ in range(phase_offset))
            self._interference_phase_prefilled = True

        self._interference_byte_queue.extend(frame)
        fill_count = max(0, period_bytes - len(frame))
        self._interference_byte_queue.extend(random.getrandbits(8) for _ in range(fill_count))

    def _next_interference_payload(self) -> List[int]:
        while len(self._interference_byte_queue) < _AIR_PAYLOAD_SIZE:
            self._append_interference_period()
        return [
            self._interference_byte_queue.popleft()
            for _ in range(_AIR_PAYLOAD_SIZE)
        ]

    def _is_broadcast_stream(self) -> bool:
        return self.Access_Code == _BROADCAST_ACCESS_CODE

    def _is_interference_password_stream(self) -> bool:
        return (
            not self.command_cycle
            and self.com_id == [0x0A, 0x06]
            and self.payload_size == 6
            and self.Access_Code == _INTERFERENCE_ACCESS_CODE
        )

    def _publish_once(self):
        with self._lock:
            access = self.Access_Code
            header = [0x00, 0x0F, 0x00, 0x0F]
            if self._is_interference_password_stream():
                payload = self._next_interference_payload()
            else:
                access, header, payload = self._messages()
        self.message_port_pub(self._port_access, _pdu_u8vector(access))
        self.message_port_pub(self._port_header, _pdu_u8vector(header))
        self.message_port_pub(self._port_payload, _pdu_u8vector(payload))

    def _publish_interval_seconds(self) -> float:
        if self._is_broadcast_stream():
            interval = _AIR_PAYLOAD_SIZE / _BROADCAST_BYTES_PER_SECOND
        elif self._is_interference_password_stream():
            interval = _AIR_PAYLOAD_SIZE / _INTERFERENCE_BYTES_PER_SECOND
        else:
            interval = max(float(self.Period) / 1000.0, 0.001)
            interval = interval / self._target_chunks_per_period()
        return max(interval, 0.001)

    def _packet_interval_symbols_locked(self) -> float:
        requested = self._symbol_rate * self._publish_interval_seconds()
        # A malformed runtime period must never make the source fall behind or
        # drop bits from an air packet.  Official information/interference
        # intervals are both safely longer than the 216-symbol packet.
        return max(float(_AIR_PACKET_BITS), float(requested))

    @staticmethod
    def _bytes_to_msb_bits(data: Sequence[int]) -> List[int]:
        return [
            (int(byte) >> shift) & 0x01
            for byte in data
            for shift in range(7, -1, -1)
        ]

    def _next_air_packet_bits_locked(self) -> List[int]:
        access = list(self.Access_Code)
        header = [0x00, 0x0F, 0x00, 0x0F]
        if self._is_interference_password_stream():
            payload = self._next_interference_payload()
        else:
            access, header, payload = self._messages()
        packet = access + header + payload
        if len(packet) != _AIR_PACKET_BYTES:
            raise RuntimeError(
                f"air packet must contain {_AIR_PACKET_BYTES} bytes, got {len(packet)}"
            )
        return self._bytes_to_msb_bits(packet)

    def work(self, _input_items, output_items):
        output = output_items[0]
        with self._lock:
            for index in range(len(output)):
                if not self._active_packet_bits:
                    scheduled_start = int(round(self._next_packet_symbol))
                    if self._symbol_index >= scheduled_start:
                        self._packet_start_symbols.append(self._symbol_index)
                        self._active_packet_bits.extend(self._next_air_packet_bits_locked())
                        self._next_packet_symbol += self._packet_interval_symbols_locked()
                        self._packet_count += 1

                if self._active_packet_bits:
                    bit = self._active_packet_bits.popleft()
                    self._packet_symbol_count += 1
                else:
                    # Alternating idle symbols preserve constant envelope and
                    # avoid the broadband transient caused by gating IQ to 0.
                    self._idle_bit ^= 0x01
                    bit = self._idle_bit
                    self._idle_symbol_count += 1

                output[index] = bit
                self._symbol_index += 1
        return len(output)

    def get_stream_diagnostics(self):
        with self._lock:
            starts = list(self._packet_start_symbols)
            intervals = [b - a for a, b in zip(starts[:-1], starts[1:])]
            interval_symbols = self._packet_interval_symbols_locked()
            return {
                "mode": "scheduler_clocked_continuous_bits",
                "constant_envelope": True,
                "idle_pattern": "alternating_01",
                "symbol_rate": float(self._symbol_rate),
                "packet_interval_symbols": float(interval_symbols),
                "packet_rate_hz": float(self._symbol_rate / interval_symbols),
                "packet_bits": _AIR_PACKET_BITS,
                "mean_idle_bits_per_packet": float(interval_symbols - _AIR_PACKET_BITS),
                "symbols_emitted": int(self._symbol_index),
                "packet_symbols_emitted": int(self._packet_symbol_count),
                "idle_symbols_emitted": int(self._idle_symbol_count),
                "packets_started": int(self._packet_count),
                "recent_packet_intervals": intervals[-12:],
            }

    def get_symbol_rate(self):
        with self._lock:
            return float(self._symbol_rate)

    def set_symbol_rate(self, symbol_rate):
        with self._lock:
            self._symbol_rate = max(float(symbol_rate), 1.0)
            self._reset_stream_locked()

    def start(self):
        with self._lock:
            self._clear_payload_queues_locked()
            self._reset_stream_locked(reset_counters=True)
        return True

    def stop(self):
        return True

    def set_Access_Code(self, Access_Code):
        with self._lock:
            self.Access_Code = _u8_list(Access_Code)
            self._clear_payload_queues_locked()
            self._reset_stream_locked()

    def get_Access_Code(self):
        with self._lock:
            return list(self.Access_Code)

    def set_Period(self, Period):
        with self._lock:
            self.Period = float(Period)
            self._clear_payload_queues_locked()
            self._reset_stream_locked()

    def get_Period(self):
        with self._lock:
            return self.Period

    def set_com_id(self, com_id):
        with self._lock:
            self.com_id = _u8_list(com_id)
            self._payload_data = self._new_payload_data_for_command(self.com_id, self.payload_size)
            self.command_cycle = []
            self._clear_payload_queues_locked()
            self._reset_stream_locked()

    def get_com_id(self):
        with self._lock:
            return list(self.com_id)

    def set_payload_size(self, payload_size):
        with self._lock:
            self.payload_size = int(payload_size)
            self._payload_data = self._new_payload_data_for_command(self.com_id, self.payload_size)
            self.command_cycle = []
            self._clear_payload_queues_locked()
            self._reset_stream_locked()

    def get_payload_size(self):
        with self._lock:
            return self.payload_size

    def set_payload_data(self, payload_data):
        with self._lock:
            data = _u8_list(payload_data)
            self.payload_size = len(data)
            self._payload_data = data
            self.command_cycle = []
            self._clear_payload_queues_locked()
            self._reset_stream_locked()

    def get_payload_data(self):
        with self._lock:
            return list(self._payload_data)

    def set_command_cycle(self, command_cycle):
        with self._lock:
            self.command_cycle = self._normalize_command_cycle(command_cycle)
            self._clear_payload_queues_locked()
            self._reset_stream_locked()

    def get_command_cycle(self):
        with self._lock:
            return [
                {
                    "cmd_id": list(cmd_id),
                    "payload_size": size,
                    "payload_data": None if payload_data is None else list(payload_data),
                    "randomize_payload": randomize_payload,
                }
                for cmd_id, size, payload_data, randomize_payload in self.command_cycle
            ]
