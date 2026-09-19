from __future__ import annotations

import threading
import time
from collections import Counter, deque
from typing import Any, Callable, Deque, Iterable


_SEQUENCE_STAGES = {'crc16_failure', 'command_length_failure', 'valid'}


class ReceptionMetrics:
    """Sliding, read-only diagnostics derived from parser decisions.

    This observer never feeds data back into decoding. Sequence gaps are an
    estimate: only frames that reached the CRC16 decision expose a usable seq.
    """

    def __init__(
        self,
        window_sec: float = 10.0,
        idle_reset_sec: float = 2.0,
        max_events: int = 4096,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.window_sec = max(0.1, float(window_sec))
        self.idle_reset_sec = max(0.0, float(idle_reset_sec))
        self._clock = clock
        self._events: Deque[dict[str, Any]] = deque(maxlen=max(32, int(max_events)))
        self._lock = threading.RLock()

    def clear(self) -> None:
        with self._lock:
            self._events.clear()

    def record(self, event: dict[str, Any], *, timestamp: float | None = None) -> None:
        stage = str(event.get('stage') or '').strip()
        if not stage:
            return
        item = dict(event)
        item['stage'] = stage
        item['timestamp'] = float(self._clock() if timestamp is None else timestamp)
        if 'seq' in item:
            try:
                item['seq'] = int(item['seq']) & 0xFF
            except (TypeError, ValueError):
                item.pop('seq', None)
        with self._lock:
            self._events.append(item)

    def record_many(self, events: Iterable[dict[str, Any]]) -> None:
        timestamp = self._clock()
        for event in events:
            self.record(event, timestamp=timestamp)

    def _prune_unlocked(self, now: float) -> None:
        cutoff = float(now) - self.window_sec
        while self._events and float(self._events[0]['timestamp']) < cutoff:
            self._events.popleft()

    def _sequence_summary(self, events: list[dict[str, Any]]) -> dict[str, Any]:
        observed = 0
        estimated_original = 0
        missing = 0
        duplicates = 0
        out_of_order = 0
        transitions = 0
        sessions = 0
        last_seq: int | None = None
        last_time: float | None = None
        for event in events:
            if 'seq' not in event:
                continue
            seq = int(event['seq']) & 0xFF
            timestamp = float(event['timestamp'])
            if last_seq is None or (
                last_time is not None and timestamp - last_time > self.idle_reset_sec
            ):
                sessions += 1
                observed += 1
                estimated_original += 1
                last_seq = seq
                last_time = timestamp
                continue
            delta = (seq - last_seq) & 0xFF
            if delta == 0:
                duplicates += 1
                last_time = timestamp
                continue
            if delta <= 127:
                transitions += 1
                observed += 1
                estimated_original += delta
                missing += delta - 1
                last_seq = seq
                last_time = timestamp
                continue
            out_of_order += 1
        loss_rate = (float(missing) / float(estimated_original)) if estimated_original > 0 else None
        return {
            # Stable diagnostic schema used by the match Dashboard.
            'received_frames': sum(1 for event in events if 'seq' in event),
            'in_order_frames': observed,
            'expected_frames': estimated_original,
            'lost_frames': missing,
            'loss_rate': loss_rate,
            'duplicate_frames': duplicates,
            'out_of_order_frames': out_of_order,
            'sequence_resets': max(0, sessions - 1),
            'last_seq': last_seq,
            'reset_after_sec': self.idle_reset_sec,
            # Additional explicit names retained for programmatic consumers.
            'observed_in_order': observed,
            'estimated_original': estimated_original,
            'missing': missing,
            'duplicates': duplicates,
            'out_of_order': out_of_order,
            'transitions': transitions,
            'sessions': sessions,
        }

    def snapshot(self, *, now: float | None = None) -> dict[str, Any]:
        current = float(self._clock() if now is None else now)
        with self._lock:
            self._prune_unlocked(current)
            events = [dict(event) for event in self._events]

        counts = Counter(str(event['stage']) for event in events)
        candidates = [event for event in events if event['stage'] in _SEQUENCE_STAGES]
        valid_events = [event for event in events if event['stage'] == 'valid']
        candidate_sequence = self._sequence_summary(candidates)
        valid_sequence = self._sequence_summary(valid_events)
        suspected_frames = len(candidates)
        valid_frames = len(valid_events)
        rejected_frames = counts['crc16_failure'] + counts['command_length_failure']
        missing_frames = candidate_sequence['missing']
        original_frames = candidate_sequence['estimated_original']
        total_lost = missing_frames + rejected_frames
        estimate_available = candidate_sequence['transitions'] > 0

        def ratio(numerator: int, denominator: int) -> float | None:
            if denominator <= 0:
                return None
            return max(0.0, min(1.0, float(numerator) / float(denominator)))

        pre_crc_rate = ratio(missing_frames, original_frames)
        rejection_rate = ratio(rejected_frames, suspected_frames)
        end_to_end_rate = ratio(total_lost, original_frames)
        debug = {
            'sof_candidates': (
                counts['crc8_failure']
                + counts['length_failure']
                + suspected_frames
            ),
            'crc8_passed_candidates': counts['length_failure'] + suspected_frames,
            'length_failures': counts['length_failure'],
            'command_length_failures': counts['command_length_failure'],
            'crc16_attempts': suspected_frames,
            'crc8_failures': counts['crc8_failure'],
            'crc16_failures': counts['crc16_failure'],
            'valid_frames': counts['valid'],
        }
        return {
            'scope': 'primary_strict_decoder_sliding_window',
            'window_sec': self.window_sec,
            'window_start_monotonic': current - self.window_sec,
            'window_end_monotonic': current,
            'estimate_method': 'CRC8-protected uint8 sequence gaps',
            'idle_reset_sec': self.idle_reset_sec,
            'estimate_available': estimate_available,
            'estimated_original_frames': original_frames,
            'suspected_frames': suspected_frames,
            'crc_valid_frames': valid_frames,
            'pre_crc_missing_frames': missing_frames,
            'pre_crc_loss_rate': pre_crc_rate,
            'pre_crc_missing_rate': pre_crc_rate,
            'crc_rejected_frames': rejected_frames,
            'rejected_frames': rejected_frames,
            'crc_rejection_rate': rejection_rate,
            'rejection_rate': rejection_rate,
            'total_lost_frames': total_lost,
            'end_to_end_loss_rate': end_to_end_rate,
            'candidate_sequence': candidate_sequence,
            'valid_sequence': valid_sequence,
            'debug': debug,
            'counts': {
                'crc8_failures': counts['crc8_failure'],
                'length_failures': counts['length_failure'],
                'crc16_failures': counts['crc16_failure'],
                'command_length_failures': counts['command_length_failure'],
                'valid_frames': counts['valid'],
            },
        }
