import gzip
import json

import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter
from std_msgs.msg import String

from rm_radio_ros.nodes.match_recorder_node import MatchRecorderNode


def _parameter(name, parameter_type, value):
    return Parameter(name, parameter_type, value)


def test_match_recorder_auto_starts_and_finalizes_from_game_status(tmp_path):
    context = Context()
    rclpy.init(context=context)
    node = MatchRecorderNode(
        context=context,
        parameter_overrides=[
            _parameter(
                "record_root",
                Parameter.Type.STRING,
                str(tmp_path),
            ),
            _parameter("min_free_gb", Parameter.Type.DOUBLE, 0.0),
            _parameter("post_roll_sec", Parameter.Type.DOUBLE, 0.0),
            _parameter("event_compression", Parameter.Type.STRING, "gzip"),
            _parameter("event_topics", Parameter.Type.STRING, ""),
        ],
    )
    try:
        running = String(
            data=json.dumps(
                {
                    "game_progress": 4,
                    "match_running": True,
                    "stage_remain_time": 420,
                }
            )
        )
        node._handle_game_status(running)
        assert node._state == "armed"
        node._handle_game_status(running)
        assert node._state == "recording"
        session_dir = node._session_dir
        assert session_dir is not None
        assert (session_dir / "config_snapshot.yaml").exists() is False

        ended = String(
            data=json.dumps(
                {
                    "game_progress": 5,
                    "match_running": False,
                    "stage_remain_time": 0,
                }
            )
        )
        node._handle_game_status(ended)
        node._handle_game_status(ended)
        assert node._state == "post_roll"
        node._state_tick()
        assert node._state == "finalizing"
        with node._lock:
            node._seen_active_roles.update(("rx1", "rx2"))
            node._stopped_roles.update(("rx1", "rx2"))
        node._state_tick()

        assert node._state == "armed"
        manifest = json.loads((session_dir / "manifest.json").read_text(encoding="utf-8"))
        assert manifest["state"] == "complete"
        assert (session_dir / "completed.json").exists()
        with gzip.open(session_dir / "events.jsonl.gz", "rt", encoding="utf-8") as stream:
            events = [json.loads(line) for line in stream]
        assert any(event["kind"] == "session_start" for event in events)
        assert any(event["kind"] == "session_complete" for event in events)
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)


def test_partial_recording_does_not_wait_for_never_started_iq_role(tmp_path):
    event_root = tmp_path / "events"
    iq_root = tmp_path / "iq"
    context = Context()
    rclpy.init(context=context)
    node = MatchRecorderNode(
        context=context,
        parameter_overrides=[
            _parameter("record_root", Parameter.Type.STRING, str(event_root)),
            _parameter("iq_record_root", Parameter.Type.STRING, str(iq_root)),
            _parameter("match_run_id", Parameter.Type.STRING, "match_shared_001"),
            _parameter("min_free_gb", Parameter.Type.DOUBLE, 0.0),
            _parameter("event_compression", Parameter.Type.STRING, "gzip"),
            _parameter("event_topics", Parameter.Type.STRING, ""),
        ],
    )
    try:
        with node._lock:
            assert node._start_session_unlocked("manual", None)
            session_id = node._session_id
            session_dir = node._session_dir
            assert session_id == "match_shared_001_radio"
            node._seen_active_roles.add("rx1")
            node._stopped_roles.add("rx1")
            assert node._begin_finalize_unlocked("manual")

        node._state_tick()

        assert node._state == "armed"
        manifest = json.loads((session_dir / "manifest.json").read_text(encoding="utf-8"))
        assert manifest["reason"] == "iq_writers_stopped"
        assert manifest["match_run_id"] == "match_shared_001"
        assert manifest["iq_record_root"] == str(iq_root.resolve())
        assert manifest["iq_roles_seen"] == ["rx1"]
        assert manifest["last_error"] == ""
        iq_location = json.loads(
            (session_dir / "iq_location.json").read_text(encoding="utf-8")
        )
        assert iq_location["iq_session_dir"] == str(iq_root / session_id)
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)
