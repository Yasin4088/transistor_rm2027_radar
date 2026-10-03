from pathlib import Path
import os
import threading

from rm_radio_ros.app_support.match_dashboard_node import (
    DashboardState,
    RadioDashboardNode,
    _update_simple_yaml_scalars,
)
from std_msgs.msg import String


def _node_without_init():
    node = object.__new__(RadioDashboardNode)
    node.state = DashboardState()
    node._lock = threading.RLock()
    node._stream_condition = threading.Condition(node._lock)
    node._processes = {}
    node._process_logs = {}
    node._frame_sequence = 0
    node._stream_generation = 0
    node._radar_generation = 0
    node._sdr_check_lock = threading.Lock()
    node._event = lambda *args, **kwargs: None
    return node


def _set_started_rx(node, role, *, now=100.0, **data):
    payload = {"started": True, **data}
    node.state.statuses[role] = {"timestamp": now, "data": payload}
    node.state.status_first_seen_at[role] = now - 10.0


def test_dashboard_stream_endpoint_is_incremental_and_state_tail_is_bounded():
    node = _node_without_init()
    callback = RadioDashboardNode._frames_callback(node, "rx1")
    for index in range(20):
        msg = String()
        msg.data = '{"cmd_id": %d}' % index
        callback(msg)
        node.state.bridge_outputs.appendleft({"index": index})

    node.state.spectra = {
        "rx1": {"timestamp": 10.0, "data": {"power_dbfs": [-90.0, -80.0]}},
        "rx2": {"timestamp": 20.0, "data": {"power_dbfs": [-70.0, -60.0]}},
    }
    stream = RadioDashboardNode._stream_snapshot(
        node,
        after_sequence=15,
        spectrum_after={"rx1": 10.0, "rx2": 19.0},
    )
    snapshot = RadioDashboardNode._snapshot(node)

    assert stream["frame_sequence"] == 20
    assert [item["seq"] for item in stream["frames"]] == [20, 19, 18, 17, 16]
    assert list(stream["spectra"]) == ["rx2"]
    assert len(snapshot["frames"]) == 12
    assert len(snapshot["bridge_outputs"]) == 12
    assert snapshot["stream"]["frame_sequence"] == 20


def test_dashboard_reports_decoded_rx_rates_from_one_second_window_only():
    node = _node_without_init()
    node.state.frames.extend([
        {"timestamp": 100.0, "role": "rx1"},
        {"timestamp": 99.8, "role": "rx1"},
        {"timestamp": 99.4, "role": "rx2"},
        {"timestamp": 99.0, "role": "rx2"},
        {"timestamp": 100.0, "role": "referee"},
        {"timestamp": 98.9, "role": "rx1"},
        {"timestamp": 100.1, "role": "rx2"},
    ])

    rates = RadioDashboardNode._reception_rates_unlocked(node, now=100.0)

    assert rates == {"window_sec": 1.0, "rx1_hz": 2.0, "rx2_hz": 2.0}


def test_dashboard_warns_when_either_rx_rate_is_below_two_fps():
    for deficient_role in ("rx1", "rx2"):
        node = _node_without_init()
        healthy_role = "rx2" if deficient_role == "rx1" else "rx1"
        _set_started_rx(node, "rx1")
        _set_started_rx(node, "rx2")
        # Keep every expected command fresh while controlling only the 1-second
        # rate window used by this assertion.
        node.state.frames.extend(
            {"timestamp": 98.5, "role": "rx1", "frame": {"cmd_id": command}}
            for command in range(0x0A01, 0x0A06)
        )
        node.state.frames.append({"timestamp": 98.5, "role": "rx2", "frame": {"cmd_id": 0x0A06}})
        node.state.frames.extend([
            {"timestamp": 100.0, "role": deficient_role, "frame": {"cmd_id": 0x0A01 if deficient_role == "rx1" else 0x0A06}},
            {"timestamp": 100.0, "role": healthy_role, "frame": {"cmd_id": 0x0A01 if healthy_role == "rx1" else 0x0A06}},
            {"timestamp": 99.5, "role": healthy_role, "frame": {"cmd_id": 0x0A02 if healthy_role == "rx1" else 0x0A06}},
        ])

        health = RadioDashboardNode._health_unlocked(node, now=100.0)

        deficient = health["modules"][f"{deficient_role}_frame_rate"]
        healthy = health["modules"][f"{healthy_role}_frame_rate"]
        assert deficient["state"] == "warn"
        assert "1.0 帧/s" in deficient["message"]
        assert "低于 2 帧/s 门限" in deficient["message"]
        assert healthy["state"] == "ok"
        assert "2.0 帧/s" in healthy["message"]
        assert health["state"] == "warn"


def test_dashboard_health_distinguishes_startup_unknown_from_missing_runtime_data():
    node = _node_without_init()
    node.state.statuses["rx1"] = {"timestamp": 100.0, "data": {"started": True}}
    node.state.status_first_seen_at["rx1"] = 98.0

    startup = RadioDashboardNode._health_unlocked(node, now=100.0)["modules"]
    assert startup["rx1_business_frames"]["state"] == "unknown"
    assert startup["rx1_signal"]["state"] == "unknown"

    node.state.status_first_seen_at["rx1"] = 90.0
    running = RadioDashboardNode._health_unlocked(node, now=100.0)["modules"]
    assert running["rx1_business_frames"]["state"] == "warn"
    assert running["rx1_tuning"]["state"] == "warn"
    assert running["rx1_signal"]["state"] == "warn"
    assert running["rx1_spectrum"]["state"] == "warn"


def test_dashboard_missing_rx_traffic_is_capped_at_warning():
    node = _node_without_init()
    _set_started_rx(node, "rx1")
    _set_started_rx(node, "rx2")

    modules = RadioDashboardNode._health_unlocked(node, now=100.0)["modules"]

    for role in ("rx1", "rx2"):
        assert modules[f"{role}_frame_rate"]["state"] == "warn"
        assert modules[f"{role}_business_frames"]["state"] == "warn"


def test_dashboard_health_binds_business_tuning_signal_decode_and_frequency_errors():
    node = _node_without_init()
    _set_started_rx(
        node,
        "rx1",
        last_setters={"cen_f": 432_000_000, "Gain": 20},
        decoder={
            "iq_diagnostics": {"sample_count": 4096, "signal_state": "clipped", "clip_fraction": 0.01, "headroom_db": -0.2},
            "iq_decode_diagnostics": {"frequency_shift_hz": 125_000},
        },
        reception_pipeline={
            "estimate_available": True,
            "suspected_frames": 20,
            "end_to_end_loss_rate": 0.25,
            "crc_rejection_rate": 0.10,
            "candidate_sequence": {"duplicate_frames": 1, "out_of_order_frames": 2},
        },
    )
    node.state.frames.append({"timestamp": 100.0, "role": "rx1", "frame": {"cmd_id": 0x0A02}})

    modules = RadioDashboardNode._health_unlocked(node, now=100.0)["modules"]

    assert modules["rx1_business_frames"]["state"] == "warn"
    assert "0x0A01" in modules["rx1_business_frames"]["message"]
    assert modules["rx1_tuning"]["state"] == "bad"
    assert modules["rx1_signal"]["state"] == "bad"
    assert "削顶" in modules["rx1_signal"]["message"]
    assert modules["rx1_decode_quality"]["state"] == "bad"
    assert "丢包 25.0%" in modules["rx1_decode_quality"]["message"]
    assert "rx1_frequency_offset" not in modules
    assert "rx2_frequency_offset" not in modules


def test_dashboard_tuning_uses_applied_setters_after_intelligent_side_switch():
    node = _node_without_init()
    node.state.intelligent_mode = True
    node.state.effective_radio_side = "blue"
    node.state.effective_interference_level = 2
    node.state.broadcast_gain = 32.0
    _set_started_rx(
        node,
        "rx1",
        radio_side="red",
        last_setters={"cen_f": 433_920_000, "Gain": 32.0},
        rx_gain={"gain_db": 32.0},
    )
    _set_started_rx(
        node,
        "rx2",
        radio_side="red",
        interference_level=1,
        last_setters={"cen_f": 434_620_000},
    )

    assert RadioDashboardNode._tuning_health_unlocked(node, "rx1", 100.0)["state"] == "ok"
    assert RadioDashboardNode._tuning_health_unlocked(node, "rx2", 100.0)["state"] == "ok"


def test_dashboard_tuning_rejects_actual_frequency_mismatch_after_side_switch():
    node = _node_without_init()
    node.state.intelligent_mode = True
    node.state.effective_radio_side = "blue"
    node.state.effective_interference_level = 1
    _set_started_rx(
        node,
        "rx1",
        radio_side="red",
        last_setters={"cen_f": 433_200_000, "Gain": 20.0},
        rx_gain={"gain_db": 20.0},
    )

    health = RadioDashboardNode._tuning_health_unlocked(node, "rx1", 100.0)

    assert health["state"] == "bad"
    assert "实际 433.200 MHz，期望 433.920 MHz" in health["message"]


def test_dashboard_health_binds_serial_output_coverage_vision_and_browser_errors():
    node = _node_without_init()
    node.state.statuses["referee"] = {
        "timestamp": 100.0,
        "data": {"serial_open": True, "serial_open_count": 4, "serial_last_close_time": 95.0},
    }
    node.state.statuses["radar_integration"] = {
        "timestamp": 100.0,
        "data": {
            "schema": "shark.radar.fusion.v1",
            "side": "red",
            "send_allowed": True,
            "vision": {"online": True},
            "robots": {"B1": {"final": {"valid": False}}},
            "tx_0305": {"request_count": 8, "success_count": 0, "actual_rate_hz": 0, "configured_rate_hz": 4.8},
        },
    }
    node.state.statuses["vision_radar"] = {
        "timestamp": 100.0,
        "data": {"schema": "shark.radar.telemetry.v1", "vision": {"camera_ready": True, "fps": 4.0, "inference_ms": 300}},
    }
    node.state.status_first_seen_at["vision_radar"] = 90.0
    node.state.display_metrics = {
        "received_at": 100.0,
        "active_page": "radio",
        "stream_state": "reconnecting",
        "latency_ms": 1800,
        "render_fps": 8,
        "dropped_rows_delta": 12,
    }

    modules = RadioDashboardNode._health_unlocked(node, now=100.0)["modules"]

    assert modules["referee_stability"]["state"] == "bad"
    assert modules["radar_output"]["state"] == "bad"
    assert "尚无成功" in modules["radar_output"]["message"]
    assert modules["coordinate_coverage"]["state"] == "bad"
    assert modules["vision_performance"]["state"] == "bad"
    assert modules["display_performance"]["state"] == "bad"
    assert "SSE reconnecting" in modules["display_performance"]["message"]


def test_dashboard_health_detects_slow_camera_despite_fast_processing():
    node = _node_without_init()
    node.state.statuses["vision_radar"] = {
        "timestamp": 100.0,
        "data": {
            "schema": "shark.radar.telemetry.v1",
            "vision": {
                "camera_ready": True,
                "camera_fps": 3.1,
                "processing_fps": 84.3,
                "fps": 84.3,
                "inference_ms": 11.9,
            },
        },
    }
    node.state.status_first_seen_at["vision_radar"] = 90.0

    module = RadioDashboardNode._vision_performance_health_unlocked(
        node, now=100.0
    )

    assert module["state"] == "bad"
    assert "相机 3.1 FPS" in module["message"]
    assert "处理 84.3 FPS" in module["message"]


def test_dashboard_status_error_is_not_hidden_by_stale_timestamp():
    node = _node_without_init()
    node.state.statuses["rx1"] = {"timestamp": 80.0, "data": {"started": True, "last_error": "解码线程崩溃"}}

    health = RadioDashboardNode._status_health(node, "rx1", 100.0)

    assert health["state"] == "bad"
    assert "解码线程崩溃" in health["message"]
    assert "20.0 秒未更新" in health["message"]


def test_dashboard_spectrum_updates_wake_latest_only_stream():
    node = _node_without_init()
    callback = RadioDashboardNode._spectrum_callback(node, "rx1")
    msg = String()
    msg.data = '{"timestamp": 12.5, "power_dbfs": [-90, -80]}'

    callback(msg)
    stream = RadioDashboardNode._wait_stream_snapshot(
        node,
        after_generation=0,
        after_sequence=0,
        spectrum_after={"rx1": 0.0, "rx2": 0.0},
        timeout=0.01,
    )

    assert stream["generation"] == 1
    assert stream["spectra"]["rx1"]["data"]["timestamp"] == 12.5


def test_dashboard_can_stream_filtered_spectrum_independently():
    node = _node_without_init()
    callback = RadioDashboardNode._spectrum_callback(node, "rx1", tap="filtered")
    msg = String()
    msg.data = '{"timestamp": 13.5, "tap": "filtered", "power_dbfs": [-80, -70]}'

    callback(msg)
    stream = RadioDashboardNode._stream_snapshot(
        node,
        spectrum_after={"rx1": 0.0, "rx2": 0.0},
        spectrum_tap="filtered",
    )

    assert stream["spectrum_tap"] == "filtered"
    assert stream["spectra"]["rx1"]["data"]["tap"] == "filtered"


def test_dashboard_radar_stream_is_latest_only_and_wakes_on_fusion_updates():
    node = _node_without_init()
    callback = RadioDashboardNode._radar_callback(node, "radar_integration")
    first = String()
    first.data = '{"schema":"shark.radar.fusion.v1","robots":{"B1":{"final":{"source":"vision"}}}}'
    second = String()
    second.data = '{"schema":"shark.radar.fusion.v1","robots":{"B1":{"final":{"source":"radio"}}}}'

    callback(first)
    callback(second)
    stream = RadioDashboardNode._radar_stream_snapshot(node)

    assert stream["generation"] == 2
    assert stream["radar"]["data"]["robots"]["B1"]["final"]["source"] == "radio"
    assert "history" not in stream


def test_dashboard_health_exposes_vision_and_radar_integration_modules():
    node = _node_without_init()
    vision = String()
    vision.data = '{"schema":"shark.radar.telemetry.v1","vision":{"camera_ready":true}}'
    fusion = String()
    fusion.data = '{"schema":"shark.radar.fusion.v1","health":"ok","side_mismatch":false}'
    RadioDashboardNode._radar_callback(node, "vision_radar")(vision)
    RadioDashboardNode._radar_callback(node, "radar_integration")(fusion)

    modules = RadioDashboardNode._snapshot(node)["health"]["modules"]
    assert modules["vision_radar"]["state"] == "ok"
    assert modules["radar_integration"]["state"] == "ok"


def test_dashboard_does_not_report_bridge_warn_for_missing_radio_fusion_input():
    node = _node_without_init()
    fusion = String()
    fusion.data = (
        '{"schema":"shark.radar.fusion.v1","health":"warn",'
        '"side_mismatch":false,"radio":{"fresh":false}}'
    )
    RadioDashboardNode._radar_callback(node, "radar_integration")(fusion)

    module = RadioDashboardNode._snapshot(node)["health"]["modules"]["radar_integration"]
    assert module["state"] == "ok"
    assert module["message"] == "ROS 融合桥接在线"


def test_dashboard_accepts_bounded_browser_performance_metrics():
    node = _node_without_init()

    result = RadioDashboardNode._update_client_metrics(node, {
        "render_fps": 59.8,
        "rx1_hz": 19.5,
        "rx2_hz": 20.2,
        "latency_ms": -10,
        "transport_ms": 4.5,
        "dropped_waterfall_rows": 3,
        "stream_state": "open",
        "active_page": "radar",
    })

    assert result["metrics"]["render_fps"] == 59.8
    assert result["metrics"]["latency_ms"] == 0.0
    assert result["metrics"]["stream_state"] == "open"
    assert result["metrics"]["active_page"] == "radar"
    assert result["metrics"]["dropped_rows_delta"] == 3
    assert node.state.display_metrics["rx2_hz"] == 20.2


def test_dashboard_schedules_sdr_check_off_executor_thread():
    node = _node_without_init()
    entered = threading.Event()
    release = threading.Event()

    def blocking_check():
        entered.set()
        release.wait(1.0)

    node._check_sdrs = blocking_check

    RadioDashboardNode._schedule_sdr_check(node)
    assert entered.wait(0.2)
    assert node._sdr_check_lock.locked()
    RadioDashboardNode._schedule_sdr_check(node)
    release.set()
    for _ in range(100):
        if not node._sdr_check_lock.locked():
            break
        threading.Event().wait(0.005)
    assert not node._sdr_check_lock.locked()


def test_dashboard_match_rx_launch_uses_two_rx_uris():
    node = _node_without_init()
    node._workspace_root = lambda: Path("/tmp/radio-workspace")
    node.state.radio_side = "blue"
    node.state.interference_level = 3
    node.state.sdr_uris.update({
        "rx1": "ip:192.168.22.1",
        "rx2": "ip:192.168.33.1",
    })

    command = RadioDashboardNode._rx_launch_command_unlocked(node)

    assert "rx2.launch.py" in command
    assert "radio_side:=blue" in command
    assert "interference_level:=3" in command
    assert "rx1_uri:=ip:192.168.22.1" in command
    assert "rx2_uri:=ip:192.168.33.1" in command
    assert any(part.startswith("flowgraph_dir:=") for part in command)
    assert not any("ganraoyuan" in part for part in command)


def test_dashboard_referee_launch_passes_level_confirm_parameters():
    node = _node_without_init()

    command = RadioDashboardNode._referee_launch_command_unlocked(node)

    assert "auto_rx_interference_level:=true" in command
    assert "apply_referee_level_only_when_running:=false" in command
    assert "single_rx_auto_broadcast_after_level3:=false" in command
    assert "single_rx_auto_allow_return_to_interference:=true" in command
    assert "referee_level_confirm_count:=2" in command
    assert "require_serial_open_on_start:=false" in command
    assert "frame_timeout_sec:=2.0" in command
    assert "auto_send_invincible_targets:=true" in command
    assert "invincible_targets_data_cmd_id:=564" in command
    assert "invincible_targets_send_rate_hz:=3.0" in command
    assert "invincible_targets_freshness_sec:=1.0" in command
    assert "interference_rx_control_topic:=/rm_gfsk_interference_node/setters_json" in command


def test_dashboard_loads_only_match_rx_sdr_defaults(monkeypatch):
    node = _node_without_init()
    monkeypatch.setenv("RX1_URI", "ip:192.168.2.1")
    monkeypatch.setenv("RX2_URI", "ip:192.168.3.1")
    monkeypatch.setenv("BROADCAST_TX_URI", "ip:192.168.1.10")
    monkeypatch.setenv("INTERFERENCE_TX_URI", "ip:192.168.4.1")

    RadioDashboardNode._load_env_defaults(node)

    assert node.state.sdr_uris == {
        "rx1": "ip:192.168.2.1",
        "rx2": "ip:192.168.3.1",
    }


def test_dashboard_rx_env_enables_tracking_override(monkeypatch):
    node = _node_without_init()
    monkeypatch.delenv("INTERFERENCE_TX_URI", raising=False)

    env = RadioDashboardNode._base_env(node, "rx")

    assert env["RM_RADIO_RX_TRACKING"] == "true"
    assert "INTERFERENCE_TX_URI" not in env


def test_dashboard_manual_mode_publishes_two_rx_setter_updates():
    node = _node_without_init()
    node.state.intelligent_mode = True
    published = {"rx1": [], "rx2": []}

    class Publisher:
        def __init__(self, role):
            self.role = role

        def publish(self, msg):
            published[self.role].append(msg.data)

    node.broadcast_rx_setters_pub = Publisher("rx1")
    node.interference_rx_setters_pub = Publisher("rx2")

    snapshot = RadioDashboardNode._update_config(node, {
        "mode": "match_rx",
        "intelligent_mode": False,
        "radio_side": "blue",
        "interference_level": 2,
        "sdr_uris": {"rx1": "ip:rx1", "rx2": "ip:rx2", "tx": "ip:ignored"},
    })

    assert snapshot["radio_side"] == "blue"
    assert snapshot["interference_level"] == 2
    assert snapshot["configured"] == {"radio_side": "blue", "interference_level": 2}
    assert snapshot["effective"] == {"radio_side": "blue", "interference_level": 2}
    assert snapshot["sdr_uris"] == {"rx1": "ip:rx1", "rx2": "ip:rx2"}
    assert published["rx1"] and "433920000" in published["rx1"][0]
    assert published["rx2"] and "434620000" in published["rx2"][0]


def test_dashboard_rx1_gain_is_bounded_and_hot_published():
    node = _node_without_init()
    node.state.intelligent_mode = False
    published = []

    class Publisher:
        def publish(self, msg):
            published.append(msg.data)

    node.broadcast_rx_setters_pub = Publisher()
    node.interference_rx_setters_pub = Publisher()

    snapshot = RadioDashboardNode._update_config(node, {
        "mode": "match_rx",
        "broadcast_gain": 73,
    })

    assert snapshot["rx1_gain"]["configured_db"] == 73
    assert snapshot["rx1_gain"]["min_db"] == -1
    assert snapshot["rx1_gain"]["max_db"] == 73
    assert published and '"Gain": 73.0' in published[-1]

    snapshot = RadioDashboardNode._update_config(node, {
        "mode": "match_rx",
        "broadcast_gain": -1,
    })
    assert snapshot["rx1_gain"]["configured_db"] == -1
    assert '"Gain": -1.0' in published[-1]

    try:
        RadioDashboardNode._update_config(node, {
            "mode": "match_rx",
            "broadcast_gain": 74,
        })
    except ValueError as exc:
        assert "-1..73" in str(exc)
    else:
        raise AssertionError("dashboard accepted RX1 gain above 73 dB")


def test_dashboard_save_persists_git_tracked_match_config(tmp_path):
    config_path = tmp_path / "config.yaml"
    config_path.write_text(
        """# keep this deployment comment
radio:
  side: red
  interference_level: 1
  intelligent_mode: true
  rx1_uri: ip:old-rx1
  rx2_uri: ip:old-rx2
  broadcast_gain: 20
referee:
  port: /dev/ttyUSB0
  baudrate: 115200
decoder:
  broadcast:
    demod_mode: mm
""",
        encoding="utf-8",
    )
    node = _node_without_init()
    node.config_path = config_path
    published = []

    class Publisher:
        def publish(self, msg):
            published.append(msg.data)

    node.broadcast_rx_setters_pub = Publisher()
    node.interference_rx_setters_pub = Publisher()

    snapshot = RadioDashboardNode._update_config(node, {
        "mode": "match_rx",
        "persist": True,
        "intelligent_mode": False,
        "radio_side": "blue",
        "interference_level": 3,
        "broadcast_gain": 37,
        "sdr_uris": {"rx1": "ip:192.168.9.110", "rx2": "ip:192.168.9.111"},
        "referee": {"port": "/dev/serial/by-id/radar", "baudrate": 230400},
    })

    saved = config_path.read_text(encoding="utf-8")
    assert "# keep this deployment comment" in saved
    assert "side: blue" in saved
    assert "interference_level: 3" in saved
    assert "intelligent_mode: false" in saved
    assert "rx1_uri: ip:192.168.9.110" in saved
    assert "rx2_uri: ip:192.168.9.111" in saved
    assert "broadcast_gain: 37" in saved
    assert "port: /dev/serial/by-id/radar" in saved
    assert "baudrate: 230400" in saved
    assert "demod_mode: mm" in saved
    assert snapshot["runtime"]["config_path"] == str(config_path)
    assert published


def test_simple_yaml_persistence_rejects_unknown_fields(tmp_path):
    config_path = tmp_path / "config.yaml"
    config_path.write_text("radio:\n  side: red\n", encoding="utf-8")

    try:
        _update_simple_yaml_scalars(config_path, {"radio.broadcast_gain": 20})
    except KeyError as exc:
        assert "radio.broadcast_gain" in str(exc)
    else:
        raise AssertionError("missing deployment field was silently appended")


def test_referee_auto_status_updates_effective_but_never_manual_config():
    node = _node_without_init()
    node.state.radio_side = "red"
    node.state.interference_level = 1
    node.state.effective_radio_side = "red"
    node.state.effective_interference_level = 1
    node.state.intelligent_mode = True
    published = {"rx1": [], "rx2": []}

    class Publisher:
        def __init__(self, role):
            self.role = role

        def publish(self, msg):
            published[self.role].append(msg.data)

    node.broadcast_rx_setters_pub = Publisher("rx1")
    node.interference_rx_setters_pub = Publisher("rx2")
    callback = RadioDashboardNode._status_callback(node, "referee")
    msg = String()
    msg.data = '{"effective_radio_side":"blue","rx_interference_level":3}'

    callback(msg)
    snapshot = RadioDashboardNode._snapshot(node)

    assert snapshot["configured"] == {"radio_side": "red", "interference_level": 1}
    assert snapshot["effective"] == {"radio_side": "blue", "interference_level": 3}
    assert snapshot["frequency_plan"]["broadcast_hz"] == 433920000
    assert snapshot["frequency_plan"]["interference"]["level"] == 3
    assert published["rx1"] and '"cen_f": 433920000' in published["rx1"][-1]
    assert published["rx2"] and '"cen_f": 434320000' in published["rx2"][-1]


def test_referee_side_only_change_republishes_both_rx_setters():
    node = _node_without_init()
    node.state.radio_side = "red"
    node.state.interference_level = 2
    node.state.effective_radio_side = "red"
    node.state.effective_interference_level = 2
    node.state.intelligent_mode = True
    published = {"rx1": [], "rx2": []}

    class Publisher:
        def __init__(self, role):
            self.role = role

        def publish(self, msg):
            published[self.role].append(msg.data)

    node.broadcast_rx_setters_pub = Publisher("rx1")
    node.interference_rx_setters_pub = Publisher("rx2")
    callback = RadioDashboardNode._status_callback(node, "referee")
    msg = String()
    msg.data = '{"effective_radio_side":"blue","rx_interference_level":2}'

    callback(msg)

    assert published["rx1"] and '"cen_f": 433920000' in published["rx1"][-1]
    assert published["rx2"] and '"cen_f": 434620000' in published["rx2"][-1]


def test_disabling_intelligent_mode_restores_manual_baseline():
    node = _node_without_init()
    node.state.radio_side = "red"
    node.state.interference_level = 2
    node.state.effective_radio_side = "blue"
    node.state.effective_interference_level = 3
    node.state.intelligent_mode = True
    published = {"rx1": [], "rx2": []}

    class Publisher:
        def __init__(self, role):
            self.role = role

        def publish(self, msg):
            published[self.role].append(msg.data)

    node.broadcast_rx_setters_pub = Publisher("rx1")
    node.interference_rx_setters_pub = Publisher("rx2")

    snapshot = RadioDashboardNode._update_config(node, {
        "mode": "match_rx",
        "intelligent_mode": False,
    })

    assert snapshot["configured"] == {"radio_side": "red", "interference_level": 2}
    assert snapshot["effective"] == {"radio_side": "red", "interference_level": 2}
    assert published["rx1"] and published["rx2"]


def test_match_dashboard_rejects_lab_mode_tx_config_and_tx_process():
    node = _node_without_init()

    for payload in (
        {"mode": "lab_interference_tx"},
        {"mode": "match_rx", "tx_config": {"runtime": "actual_tx"}},
    ):
        try:
            RadioDashboardNode._update_config(node, payload)
        except ValueError as exc:
            assert "比赛 RX Panel" in str(exc)
        else:
            raise AssertionError("match dashboard accepted a laboratory TX setting")

    try:
        RadioDashboardNode._process_command_unlocked(node, "tx")
    except ValueError as exc:
        assert "未知链路" in str(exc)
    else:
        raise AssertionError("match dashboard accepted a TX process")


def test_dashboard_snapshot_contains_no_tx_or_lab_state():
    node = _node_without_init()

    snapshot = RadioDashboardNode._snapshot(node)

    assert snapshot["mode"] == "match_rx"
    assert snapshot["runtime"]["match_locked"] is True
    for key in ("tx_type", "tx_enabled", "tx_config", "tx_last_setters", "simulation", "record_match", "rule_check"):
        assert key not in snapshot
    assert set(snapshot["processes"]) == {"rx", "referee"}


def test_dashboard_reports_and_protects_start_sh_supervised_processes():
    node = _node_without_init()
    node.state.process_specs["referee"] = {
        "name": "referee",
        "pid": os.getpid(),
        "externally_managed": True,
        "owner": "match_rx_start",
    }

    snapshot = RadioDashboardNode._process_snapshot_unlocked(node)

    assert snapshot["referee"]["running"] is True
    assert snapshot["referee"]["externally_managed"] is True
    for action in (RadioDashboardNode._start_process, RadioDashboardNode._stop_process):
        try:
            action(node, "referee")
        except ValueError as exc:
            assert "一键启动器托管" in str(exc)
        else:
            raise AssertionError("Dashboard changed a start.sh-supervised referee process")
