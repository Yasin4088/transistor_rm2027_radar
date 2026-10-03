from __future__ import annotations

import json
import os
import shutil
import socket
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ..core.match_recording import (
    CompressedJsonlWriter,
    atomic_write_json,
    free_space_bytes,
    recording_session_dir,
    safe_session_id,
)


DEFAULT_EVENT_TOPICS = ",".join(
    (
        "/rm_gfsk_node/status",
        "/rm_gfsk_node/frames",
        "/rm_gfsk_node/referee_bridge",
        "/rm_gfsk_node/setters_json",
        "/rm_gfsk_interference_node/status",
        "/rm_gfsk_interference_node/frames",
        "/rm_gfsk_interference_node/referee_bridge",
        "/rm_gfsk_interference_node/setters_json",
        "/rm_referee_serial_node/status",
        "/rm_referee_serial_node/rx_frames",
        "/rm_referee_serial_node/tx_frames",
        "/rm_referee_serial_node/raw_rx",
        "/rm_referee_serial_node/referee_bridge",
        "/rm_radar_integration/status",
        "/rm_radar_algorithm/telemetry",
        "/rm_radar_algorithm/radar_cmd",
    )
)


class MatchRecorderNode(Node):
    """Coordinate match-triggered IQ writers and a compressed event journal."""

    def __init__(self, *, context=None, parameter_overrides=None) -> None:
        super().__init__(
            "rm_match_recorder",
            context=context,
            parameter_overrides=parameter_overrides,
        )
        self.declare_parameter("enabled", True)
        self.declare_parameter("auto_start", True)
        self.declare_parameter(
            "record_root", "~/.local/share/shark-radio/match-records"
        )
        self.declare_parameter(
            "iq_record_root", "~/.local/share/shark-radio/match-records"
        )
        self.declare_parameter("match_run_id", "")
        self.declare_parameter("config_path", "")
        self.declare_parameter("game_status_topic", "/rm_referee_serial_node/game_status")
        self.declare_parameter("control_topic", "/rm_match_recorder/control")
        self.declare_parameter("start_confirm_frames", 2)
        self.declare_parameter("stop_confirm_frames", 2)
        self.declare_parameter("stop_game_progress", 5)
        self.declare_parameter("post_roll_sec", 10.0)
        self.declare_parameter("max_duration_sec", 900.0)
        self.declare_parameter("finalize_timeout_sec", 5.0)
        self.declare_parameter("min_free_gb", 0.1)
        self.declare_parameter("event_topics", DEFAULT_EVENT_TOPICS)
        self.declare_parameter("event_compression", "zstd")
        self.declare_parameter("event_compression_level", 1)
        self.declare_parameter("event_queue_size", 4096)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.record_root = Path(
            str(self.get_parameter("record_root").value)
        ).expanduser().resolve()
        self.iq_record_root = Path(
            str(self.get_parameter("iq_record_root").value)
        ).expanduser().resolve()
        raw_match_run_id = str(self.get_parameter("match_run_id").value).strip()
        if raw_match_run_id:
            safe_session_id(raw_match_run_id)
            safe_session_id(f"{raw_match_run_id}_radio")
        self.match_run_id = raw_match_run_id
        self.min_free_bytes = (
            max(float(self.get_parameter("min_free_gb").value), 0.0)
            * 1_000_000_000
        )
        self._lock = threading.RLock()
        self._state = "armed" if self.enabled else "disabled"
        self._last_error = ""
        self._session_id = ""
        self._session_dir: Optional[Path] = None
        self._session_started_at = 0.0
        self._session_stopped_at = 0.0
        self._start_confirm_count = 0
        self._stop_confirm_count = 0
        self._last_game_status: dict[str, Any] = {}
        self._pending_stop_deadline = 0.0
        self._pending_stop_reason = ""
        self._finalize_deadline = 0.0
        self._seen_active_roles: set[str] = set()
        self._stopped_roles: set[str] = set()
        self._event_writer = CompressedJsonlWriter(
            compression=str(self.get_parameter("event_compression").value),
            compression_level=int(self.get_parameter("event_compression_level").value),
            queue_events=int(self.get_parameter("event_queue_size").value),
        )

        control_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.control_pub = self.create_publisher(
            String,
            str(self.get_parameter("control_topic").value),
            control_qos,
        )
        self.status_pub = self.create_publisher(String, "~/status", status_qos)
        game_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String,
            str(self.get_parameter("game_status_topic").value),
            self._handle_game_status,
            game_qos,
        )
        event_topics = self._topic_list(str(self.get_parameter("event_topics").value))
        for topic in event_topics:
            self.create_subscription(String, topic, self._event_callback(topic), 20)

        self.create_service(Trigger, "~/start", self._handle_manual_start)
        self.create_service(Trigger, "~/stop", self._handle_manual_stop)
        self.create_timer(0.2, self._state_tick)
        self.create_timer(1.0, self._publish_status)

        if self.enabled:
            try:
                self.record_root.mkdir(parents=True, exist_ok=True)
            except Exception as exc:
                self._state = "error"
                self._last_error = f"cannot create recording root: {exc}"
        self._publish_status()
        self.get_logger().info(
            f"match recorder {self._state}: root={self.record_root}, auto_start={self.auto_start}"
        )

    @staticmethod
    def _topic_list(raw: str) -> list[str]:
        topics = [item.strip() for item in str(raw).replace(";", ",").split(",")]
        return list(dict.fromkeys(item for item in topics if item))

    @staticmethod
    def _json_message(msg: String) -> Any:
        try:
            return json.loads(msg.data)
        except Exception:
            return {"raw": msg.data}

    def _event_callback(self, topic: str):
        def callback(msg: String) -> None:
            data = self._json_message(msg)
            with self._lock:
                if topic == "/rm_gfsk_node/status":
                    self._observe_iq_status_unlocked("rx1", data)
                elif topic == "/rm_gfsk_interference_node/status":
                    self._observe_iq_status_unlocked("rx2", data)
                if self._session_id and self._state in {
                    "recording",
                    "post_roll",
                    "finalizing",
                }:
                    self._enqueue_event_unlocked("topic", {"topic": topic, "data": data})

        return callback

    def _observe_iq_status_unlocked(self, role: str, data: Any) -> None:
        if not isinstance(data, dict) or not self._session_id:
            return
        recording = data.get("recording")
        if not isinstance(recording, dict):
            return
        recording_error = str(recording.get("last_error") or "").strip()
        if recording_error and self._state in {"recording", "post_roll", "finalizing"}:
            self._last_error = f"{role} IQ recorder: {recording_error}"
        try:
            dropped_chunks = int(recording.get("dropped_chunks", 0))
        except (TypeError, ValueError):
            dropped_chunks = 0
        if dropped_chunks > 0 and self._state in {"recording", "post_roll", "finalizing"}:
            self._last_error = f"{role} IQ recorder dropped {dropped_chunks} chunk(s)"
        session_id = str(recording.get("session_id") or "")
        active = bool(recording.get("active", False))
        if session_id == self._session_id and active:
            self._seen_active_roles.add(role)
            self._stopped_roles.discard(role)
        elif (
            self._state == "finalizing"
            and role in self._seen_active_roles
            and session_id == self._session_id
            and not active
        ):
            self._stopped_roles.add(role)

    def _handle_game_status(self, msg: String) -> None:
        data = self._json_message(msg)
        if not isinstance(data, dict):
            return
        try:
            progress = int(data.get("game_progress"))
        except (TypeError, ValueError):
            return
        with self._lock:
            self._last_game_status = dict(data)
            if self._session_id and self._state in {"recording", "post_roll", "finalizing"}:
                self._enqueue_event_unlocked(
                    "game_status",
                    {"topic": str(self.get_parameter("game_status_topic").value), "data": data},
                )
            if not self.enabled or not self.auto_start or self._state in {"error", "disabled"}:
                return
            if progress == 4:
                self._start_confirm_count += 1
                self._stop_confirm_count = 0
                if self._state == "post_roll":
                    self._state = "recording"
                    self._pending_stop_deadline = 0.0
                    self._pending_stop_reason = ""
                    self._enqueue_event_unlocked(
                        "recording_state",
                        {"state": "recording", "reason": "match_running_resumed"},
                    )
                if (
                    self._state == "armed"
                    and self._start_confirm_count
                    >= max(int(self.get_parameter("start_confirm_frames").value), 1)
                ):
                    self._start_session_unlocked("game_progress_4", data)
                return

            self._start_confirm_count = 0
            if self._state not in {"recording", "post_roll"}:
                return
            stop_progress = int(self.get_parameter("stop_game_progress").value)
            if progress != stop_progress:
                self._stop_confirm_count = 0
                return
            self._stop_confirm_count += 1
            if self._stop_confirm_count < max(
                int(self.get_parameter("stop_confirm_frames").value),
                1,
            ):
                return
            if self._state != "post_roll":
                self._state = "post_roll"
                self._pending_stop_deadline = time.monotonic() + max(
                    float(self.get_parameter("post_roll_sec").value),
                    0.0,
                )
                self._pending_stop_reason = f"game_progress_{progress}"
                self._enqueue_event_unlocked(
                    "recording_state",
                    {
                        "state": "post_roll",
                        "reason": self._pending_stop_reason,
                        "deadline_monotonic": self._pending_stop_deadline,
                    },
                )

    def _next_session_id_unlocked(self) -> str:
        base = (
            f"{self.match_run_id}_radio"
            if self.match_run_id
            else datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_match")
        )
        candidate = base
        suffix = 1
        while recording_session_dir(self.record_root, candidate).exists():
            candidate = f"{base}_{suffix:02d}"
            suffix += 1
        return candidate

    def _start_session_unlocked(self, reason: str, game_status: Optional[dict[str, Any]]) -> bool:
        if self._state not in {"armed"}:
            return False
        try:
            self.record_root.mkdir(parents=True, exist_ok=True)
            free_bytes = free_space_bytes(self.record_root)
            if free_bytes < self.min_free_bytes:
                raise OSError(
                    f"free space {free_bytes / 1_000_000_000:.1f} GB is below "
                    f"{self.min_free_bytes / 1_000_000_000:g} GB reserve"
                )
            session_id = self._next_session_id_unlocked()
            session_dir = recording_session_dir(self.record_root, session_id)
            session_dir.mkdir(parents=True, exist_ok=False)
            config_path = Path(str(self.get_parameter("config_path").value)).expanduser()
            if config_path.is_file():
                shutil.copy2(config_path, session_dir / "config_snapshot.yaml")
            self._event_writer.start(session_dir)
            self._session_id = session_id
            self._session_dir = session_dir
            self._session_started_at = time.time()
            self._session_stopped_at = 0.0
            self._seen_active_roles.clear()
            self._stopped_roles.clear()
            self._state = "recording"
            self._last_error = ""
            self._pending_stop_deadline = 0.0
            self._pending_stop_reason = ""
            manifest = self._manifest_unlocked("recording", reason=reason)
            manifest["start_game_status"] = dict(game_status or {})
            atomic_write_json(session_dir / "manifest.json", manifest)
            atomic_write_json(
                session_dir / "iq_location.json",
                {
                    "schema": "shark.radio.iq_location.v1",
                    "match_run_id": self.match_run_id or None,
                    "session_id": session_id,
                    "iq_record_root": str(self.iq_record_root),
                    "iq_session_dir": str(recording_session_dir(self.iq_record_root, session_id)),
                },
            )
            self._enqueue_event_unlocked(
                "session_start",
                {
                    "reason": reason,
                    "session_id": session_id,
                    "game_status": dict(game_status or {}),
                },
            )
            self._publish_control_unlocked("start", reason)
            self.get_logger().warning(f"match recording started: {session_dir}")
            return True
        except Exception as exc:
            self._state = "error"
            self._last_error = f"failed to start match recording: {exc}"
            self._event_writer.stop(timeout=2.0)
            self.get_logger().error(self._last_error)
            return False

    def _begin_finalize_unlocked(self, reason: str) -> bool:
        if self._state not in {"recording", "post_roll"} or not self._session_id:
            return False
        self._state = "finalizing"
        self._pending_stop_deadline = 0.0
        self._pending_stop_reason = ""
        self._finalize_deadline = time.monotonic() + max(
            float(self.get_parameter("finalize_timeout_sec").value),
            0.5,
        )
        self._enqueue_event_unlocked("session_stop_requested", {"reason": reason})
        self._publish_control_unlocked("stop", reason)
        return True

    def _complete_finalize_unlocked(self, reason: str) -> None:
        if self._state != "finalizing" or not self._session_id:
            return
        if reason == "iq_finalize_timeout":
            missing = sorted(self._seen_active_roles - self._stopped_roles)
            if missing:
                self._last_error = (
                    "IQ writer finalization acknowledgement timed out: "
                    + ", ".join(missing)
                )
        self._session_stopped_at = time.time()
        self._enqueue_event_unlocked(
            "session_complete",
            {
                "reason": reason,
                "iq_roles_seen": sorted(self._seen_active_roles),
                "iq_roles_stopped": sorted(self._stopped_roles),
            },
        )
        self._event_writer.stop(timeout=8.0)
        event_status = self._event_writer.snapshot()
        if event_status.get("last_error"):
            self._last_error = str(event_status["last_error"])
        session_dir = self._session_dir
        if session_dir is not None:
            manifest = self._manifest_unlocked("complete", reason=reason)
            manifest["event_writer"] = event_status
            atomic_write_json(session_dir / "manifest.json", manifest)
            atomic_write_json(
                session_dir / "completed.json",
                {
                    "schema": "shark.radio.match_recording.complete.v1",
                    "match_run_id": self.match_run_id or None,
                    "session_id": self._session_id,
                    "completed_at": self._session_stopped_at,
                    "reason": reason,
                    "last_error": self._last_error,
                },
            )
        self.get_logger().warning(f"match recording completed: {session_dir}")
        self._state = "armed" if self.enabled else "disabled"
        self._session_id = ""
        self._session_dir = None
        self._session_started_at = 0.0
        self._pending_stop_deadline = 0.0
        self._finalize_deadline = 0.0
        self._start_confirm_count = 0
        self._stop_confirm_count = 0

    def _publish_control_unlocked(self, action: str, reason: str) -> None:
        payload = {
            "schema": "shark.radio.recording_control.v1",
            "action": action,
            "match_run_id": self.match_run_id or None,
            "session_id": self._session_id,
            "timestamp": time.time(),
            "reason": reason,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.control_pub.publish(msg)

    def _enqueue_event_unlocked(self, kind: str, detail: dict[str, Any]) -> None:
        self._event_writer.enqueue(
            {
                "schema": "shark.radio.match_event.v1",
                "timestamp": time.time(),
                "monotonic_ns": time.monotonic_ns(),
                "match_run_id": self.match_run_id or None,
                "kind": kind,
                **detail,
            }
        )

    def _manifest_unlocked(self, state: str, *, reason: str) -> dict[str, Any]:
        return {
            "schema": "shark.radio.match_recording.v1",
            "session_id": self._session_id,
            "match_run_id": self.match_run_id or None,
            "state": state,
            "reason": reason,
            "host": socket.gethostname(),
            "pid": os.getpid(),
            "record_root": str(self.record_root),
            "iq_record_root": str(self.iq_record_root),
            "iq_session_dir": (
                str(recording_session_dir(self.iq_record_root, self._session_id))
                if self._session_id
                else None
            ),
            "session_dir": str(self._session_dir) if self._session_dir else None,
            "config_path": str(self.get_parameter("config_path").value),
            "started_at": self._session_started_at or None,
            "stopped_at": self._session_stopped_at or None,
            "last_game_status": dict(self._last_game_status),
            "iq_roles_seen": sorted(self._seen_active_roles),
            "iq_roles_stopped": sorted(self._stopped_roles),
            "last_error": self._last_error,
        }

    def _state_tick(self) -> None:
        with self._lock:
            now = time.monotonic()
            event_status = self._event_writer.snapshot()
            if event_status.get("last_error"):
                self._last_error = str(event_status["last_error"])
            elif int(event_status.get("events_dropped", 0)) > 0:
                self._last_error = (
                    "event recorder dropped "
                    f"{int(event_status['events_dropped'])} event(s)"
                )
            if self._state in {"recording", "post_roll"}:
                max_duration = max(float(self.get_parameter("max_duration_sec").value), 0.0)
                if (
                    max_duration > 0
                    and self._session_started_at
                    and time.time() - self._session_started_at >= max_duration
                ):
                    self._begin_finalize_unlocked("max_duration")
                    return
            if (
                self._state == "post_roll"
                and self._pending_stop_deadline
                and now >= self._pending_stop_deadline
            ):
                self._begin_finalize_unlocked(self._pending_stop_reason or "post_roll_complete")
                return
            if self._state == "finalizing":
                # Only writers that actually acknowledged this session can be
                # required to stop. A missing RX node must not turn a clean
                # partial recording into a five-second timeout/error.
                all_seen_stopped = self._stopped_roles.issuperset(
                    self._seen_active_roles
                )
                if all_seen_stopped:
                    self._complete_finalize_unlocked("iq_writers_stopped")
                elif now >= self._finalize_deadline:
                    self._complete_finalize_unlocked("iq_finalize_timeout")

    def _status_payload_unlocked(self) -> dict[str, Any]:
        try:
            disk_free = free_space_bytes(self.record_root) if self.enabled else None
        except Exception:
            disk_free = None
        return {
            "schema": "shark.radio.match_recorder_status.v1",
            "enabled": self.enabled,
            "auto_start": self.auto_start,
            "state": self._state,
            "started": True,
            "session_id": self._session_id or None,
            "match_run_id": self.match_run_id or None,
            "session_dir": str(self._session_dir) if self._session_dir else None,
            "session_started_at": self._session_started_at or None,
            "duration_sec": (
                max(0.0, time.time() - self._session_started_at)
                if self._session_started_at
                else 0.0
            ),
            "record_root": str(self.record_root),
            "iq_record_root": str(self.iq_record_root),
            "disk_free_bytes": disk_free,
            "min_free_bytes": int(self.min_free_bytes),
            "start_confirm_count": self._start_confirm_count,
            "stop_confirm_count": self._stop_confirm_count,
            "post_roll_remaining_sec": (
                max(0.0, self._pending_stop_deadline - time.monotonic())
                if self._pending_stop_deadline
                else 0.0
            ),
            "iq_roles_seen": sorted(self._seen_active_roles),
            "iq_roles_stopped": sorted(self._stopped_roles),
            "events": self._event_writer.snapshot(),
            "last_game_status": dict(self._last_game_status),
            "last_error": self._last_error,
        }

    def _publish_status(self) -> None:
        with self._lock:
            payload = self._status_payload_unlocked()
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.status_pub.publish(msg)

    def _handle_manual_start(self, _request, response):
        with self._lock:
            ok = self._start_session_unlocked("manual", self._last_game_status)
            response.success = ok
            response.message = str(self._session_dir) if ok else self._last_error or self._state
        return response

    def _handle_manual_stop(self, _request, response):
        with self._lock:
            ok = self._begin_finalize_unlocked("manual")
            response.success = ok
            response.message = "finalizing" if ok else f"not recording ({self._state})"
        return response

    def destroy_node(self) -> bool:
        with self._lock:
            if self._session_id and self._state in {"recording", "post_roll", "finalizing"}:
                if self._state != "finalizing":
                    self._publish_control_unlocked("stop", "node_shutdown")
                self._session_stopped_at = time.time()
                self._enqueue_event_unlocked("session_incomplete", {"reason": "node_shutdown"})
                self._event_writer.stop(timeout=3.0)
                if self._session_dir is not None:
                    atomic_write_json(
                        self._session_dir / "manifest.json",
                        self._manifest_unlocked("incomplete", reason="node_shutdown"),
                    )
            else:
                self._event_writer.stop(timeout=1.0)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node: Optional[MatchRecorderNode] = None
    try:
        node = MatchRecorderNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # ROS 2 Humble can surface RCLError from spin after its SIGTERM handler
        # has already invalidated the context. Suppress only that shutdown race.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
