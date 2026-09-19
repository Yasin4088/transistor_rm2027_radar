import subprocess
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]


def _read_package(relative: str) -> str:
    return (PACKAGE_ROOT / relative).read_text(encoding='utf-8')


def _read_root(relative: str) -> str:
    return (WORKSPACE_ROOT / relative).read_text(encoding='utf-8')


def test_legacy_combined_rx_tx_launch_files_are_not_installed():
    for relative in ('launch/radio_pair.launch.py', 'launch/rx2_interference_tx.launch.py', 'launch/dashboard.launch.py'):
        assert not (PACKAGE_ROOT / relative).exists()

    setup = _read_package('setup.py')
    assert 'radio_pair.launch.py' not in setup
    assert 'rx2_interference_tx.launch.py' not in setup
    assert 'dashboard.launch.py' not in setup


def test_new_app_directories_and_yaml_configs_exist():
    assert (WORKSPACE_ROOT / 'apps/match_rx/start.sh').exists()
    assert (WORKSPACE_ROOT / 'apps/match_rx/doctor.sh').exists()
    assert (WORKSPACE_ROOT / 'apps/match_rx/config.yaml').exists()
    assert (WORKSPACE_ROOT / 'apps/lab_tx/start.sh').exists()
    assert (WORKSPACE_ROOT / 'apps/lab_tx/config.yaml').exists()
    assert (WORKSPACE_ROOT / 'apps/common/read_yaml.py').exists()
    assert (WORKSPACE_ROOT / 'apps/common/check_gnuradio_runtime.py').exists()
    assert not (PACKAGE_ROOT / 'config/radio_profiles.yaml').exists()


def test_match_recording_is_configured_and_launched_from_yaml():
    config = _read_root('apps/match_rx/config.yaml')
    env_source = _read_root('apps/match_rx/env.sh')
    start_source = _read_root('apps/match_rx/start.sh')
    setup_source = _read_package('setup.py')
    rx_launch = _read_package('launch/rx2.launch.py')

    for setting in (
        'recording:',
        'enabled: true',
        'root: ~/.local/share/transistor-radio/match-records',
        'root: /media/transistor/Resources/transistor-radio-iq',
        'fallback_root: ~/.local/share/transistor-radio/match-records',
        'output_sample_rate: 1000000',
        'segment_sec: 30.0',
        'compression: zlib',
        'compression: zstd',
    ):
        assert setting in config
    assert 'yaml_get recording.enabled true' in env_source
    assert 'mountpoint -q "$RM_RADIO_RECORDING_IQ_MOUNT_POINT"' in env_source
    assert '",$iq_mount_options," != *,rw,*' in env_source
    assert 'export RM_RADIO_RECORDING_IQ_ROOT="$RM_RADIO_RECORDING_IQ_FALLBACK_ROOT"' in env_source
    assert 'yaml_get recording.iq.output_sample_rate 1000000' in env_source
    assert 'rm_match_recorder' in setup_source
    assert 'start_service recorder ros2 run rm_radio_ros rm_match_recorder' in start_source
    assert 'recording_enabled:="$RUN_RECORDER"' in start_source
    assert 'recording_root:="$RM_RADIO_RECORDING_IQ_ROOT"' in start_source
    assert '-p record_root:="$RM_RADIO_RECORDING_ROOT"' in start_source
    assert '-p iq_record_root:="$RM_RADIO_RECORDING_IQ_ROOT"' in start_source
    assert "DeclareLaunchArgument('recording_output_sample_rate'" in rx_launch
    assert "'recording_role': 'rx1'" in rx_launch
    assert "'recording_role': 'rx2'" in rx_launch


def test_match_app_is_rx_only_and_does_not_reference_tx_launchers():
    start_source = _read_root('apps/match_rx/start.sh')
    dashboard_source = _read_package('rm_radio_ros/app_support/match_dashboard_node.py')

    assert 'rx2.launch.py' in start_source
    assert 'referee_serial.launch.py' in start_source
    assert 'rm_match_dashboard' in start_source
    assert 'rm_ganraoyuan_node' not in start_source
    assert 'rm_iq_replay_tx_node' not in start_source
    assert 'start_interference_tx.sh' not in start_source
    assert 'start_dual_tx.sh' not in start_source
    assert 'rm_ganraoyuan_node' not in dashboard_source
    assert 'rm_iq_replay_tx_node' not in dashboard_source
    assert 'run_virtual_link' not in dashboard_source
    assert '比赛 RX Panel 只支持 match_rx 模式' in dashboard_source
    assert '比赛 RX Panel 不接受 TX 配置' in dashboard_source


def test_match_app_keeps_serial_failure_isolated():
    start_source = _read_root('apps/match_rx/start.sh')
    referee_config = _read_root('apps/match_rx/config.yaml')

    assert 'if [[ "$name" == "referee" || "$name" == "algorithm" ]]' in start_source
    assert 'other modules continue' in start_source
    assert 'require_serial_open_on_start: false' in referee_config
    assert 'require_serial_open_on_start:="$REQUIRE_REFEREE_SERIAL_OPEN_ON_START"' in start_source
    assert 'frame_timeout_sec: 2.0' in referee_config
    assert 'frame_timeout_sec:="$REFEREE_FRAME_TIMEOUT_SEC"' in start_source
    assert 'referee fail-fast exited' in start_source


def test_match_app_wires_invincible_target_broadcast_from_yaml():
    start_source = _read_root('apps/match_rx/start.sh')
    env_source = _read_root('apps/match_rx/env.sh')
    app_config = _read_root('apps/match_rx/config.yaml')
    referee_config = _read_package('config/referee_serial.yaml')
    referee_launch = _read_package('launch/referee_serial.launch.py')

    assert 'auto_send_invincible_targets: true' in app_config
    assert 'invincible_targets_data_cmd_id: 564' in app_config
    assert 'invincible_targets_send_rate_hz: 3.0' in app_config
    assert 'invincible_targets_freshness_sec: 1.0' in app_config
    assert 'yaml_get referee.invincible_targets_data_cmd_id 564' in env_source
    assert 'invincible_targets_data_cmd_id:="$INVINCIBLE_TARGETS_DATA_CMD_ID"' in start_source
    assert 'invincible_targets_data_cmd_id: 0x0234' in referee_config
    assert "DeclareLaunchArgument('invincible_targets_data_cmd_id', default_value='564')" in referee_launch


def test_match_app_keeps_all_services_starting_when_rx_hardware_is_absent():
    start_source = _read_root('apps/match_rx/start.sh')
    flowgraph_source = _read_package('rm_radio_ros/nodes/flowgraph_node.py')

    assert 'probe_rx_device "$RX1_URI"' in start_source
    assert 'probe_rx_device "$RX2_URI"' in start_source
    assert 'timeout --foreground --signal=TERM --kill-after=1s' in start_source
    assert '以设备降级状态继续启动' in start_source
    assert 'ROS、裁判系统、视觉通信和面板继续启动' in start_source
    assert '硬件回读失败；拒绝启动' not in start_source
    assert "self.declare_parameter('auto_start_retry_sec', 5.0)" in flowgraph_source
    assert 'self._retry_auto_start' in flowgraph_source
    assert 'ROS node remains active and will' in flowgraph_source
    assert 'flowgraph recovered after device became available' in flowgraph_source


def test_match_app_passes_side_aware_referee_sender_and_receiver_ids():
    env_source = _read_root('apps/match_rx/env.sh')
    start_source = _read_root('apps/match_rx/start.sh')

    assert 'default_radar_sender_id=9' in env_source
    assert 'default_radar_sender_id=109' in env_source
    assert 'RADAR_SENDER_ID=' in env_source
    assert 'REFEREE_RECEIVER_ID=' in env_source
    assert 'sender_id:="$RADAR_SENDER_ID"' in start_source
    assert 'receiver_id:="$REFEREE_RECEIVER_ID"' in start_source


def test_lab_tx_app_is_separate_and_requires_confirmation():
    env_source = _read_root('apps/lab_tx/env.sh')
    start_source = _read_root('apps/lab_tx/start.sh')
    dashboard_source = _read_root('apps/lab_tx/radio_tx_dashboard.py')

    assert 'rm_radio_ros.nodes.ganraoyuan_node' in _read_root('apps/lab_tx/start_dual_tx.sh')
    assert 'RM_RADIO_LAB_TX_ANTENNA_CONFIRM' in env_source
    assert 'require_lab_tx_confirmation' in env_source
    assert 'Refusing to start lab TX.' in env_source
    assert 'exec "$SCRIPT_DIR/start_dashboard.sh"' in start_source
    assert 'referee_serial.launch.py' not in start_source
    assert 'rx2.launch.py' not in start_source
    assert 'LabTX 仅限实验室使用' in dashboard_source


def test_all_real_radio_entrypoints_reject_known_bad_gnuradio_runtime():
    audit_path = 'apps/common/check_gnuradio_runtime.py'
    assert 'REJECTED_GNURADIO_PREFIXES = ((3, 10, 1),)' in _read_root(audit_path)
    assert '"ldd", str(extension)' in _read_root(audit_path)
    for relative in (
        'apps/match_rx/start.sh',
        'apps/lab_tx/start_dual_tx.sh',
        'apps/lab_tx/start_interference_tx.sh',
        'apps/lab_tx/start_iq_replay_tx.sh',
    ):
        source = _read_root(relative)
        assert 'check_gnuradio_runtime.py' in source
        assert 'RM_RADIO_REQUIRE_LATEST_RUNTIME' in source


def test_iq_replay_tx_uses_hardware_backpressure_and_buffer_params():
    node_source = _read_package('rm_radio_ros/nodes/iq_replay_tx_node.py')
    top_block_source = node_source[
        node_source.index('class IqReplayTopBlock') : node_source.index('class IqReplayTxNode')
    ]
    env_source = _read_root('apps/lab_tx/env.sh')
    start_source = _read_root('apps/lab_tx/start_iq_replay_tx.sh')

    assert 'blocks.throttle' not in top_block_source
    assert 'tx_buffer_size' in top_block_source
    assert 'cache_iq_to_tmp' in node_source
    assert 'iq_cache_dir' in node_source
    assert 'hashlib.sha256' in node_source
    assert 'IQ_REPLAY_TX_BUFFER_SIZE="${IQ_REPLAY_TX_BUFFER_SIZE:-$(yaml_get iq_replay.buffer_size 1048576)}"' in env_source
    assert 'IQ_REPLAY_CACHE_TO_TMP="${IQ_REPLAY_CACHE_TO_TMP:-$(yaml_get iq_replay.cache_to_tmp true)}"' in env_source
    assert 'IQ_REPLAY_TX_URI="$BROADCAST_TX_URI"' in start_source
    assert 'IQ_REPLAY_TX_URI="$INTERFERENCE_TX_URI"' in start_source
    assert 'tx_uri:=\\"$IQ_REPLAY_TX_URI\\"' in start_source
    assert '-p tx_buffer_size:="$IQ_REPLAY_TX_BUFFER_SIZE"' in start_source
    assert '-p cache_iq_to_tmp:="$IQ_REPLAY_CACHE_TO_TMP"' in start_source
    assert '-p iq_cache_dir:="$IQ_REPLAY_CACHE_DIR"' in start_source


def test_gfsk_and_rx2_launch_accept_spectrum_and_flowgraph_overrides():
    gfsk = _read_package('launch/gfsk.launch.py')
    rx2 = _read_package('launch/rx2.launch.py')
    params = _read_package('config/gfsk.yaml')
    pure_rx = _read_package('flowgraphs/gfsk/RX.py')

    for source in (gfsk, rx2):
        assert "DeclareLaunchArgument('setters_apply_settle_sec', default_value='0.15')" in source
        assert "DeclareLaunchArgument('spectrum_enabled', default_value='true')" in source
        assert "DeclareLaunchArgument('spectrum_period_sec', default_value='0.5')" in source

    assert "flowgraph_dir = LaunchConfiguration('flowgraph_dir')" in rx2
    assert 'flowgraph_module: RX' in params
    assert 'flowgraph_class: RadioRx' in params
    assert "'flowgraph_module': 'RX'" in rx2
    assert "'flowgraph_class': 'RadioRx'" in rx2
    assert 'digital.gfsk_demod' in pure_rx
    assert 'ContinuousGfskDemod' in pure_rx
    assert 'digital.gfsk_mod' not in pure_rx
    assert 'fmcomms2_sink' not in pure_rx
    assert 'from jiang import' not in pure_rx
    assert "'flowgraph_dir': ParameterValue(flowgraph_dir, value_type=str)" in rx2

    gfsk_dir = PACKAGE_ROOT / 'flowgraphs/gfsk'
    assert not (gfsk_dir / 'RM.py').exists()
    assert not (gfsk_dir / 'jiang.py').exists()
    assert not (gfsk_dir / 'EGO.grc').exists()
    assert not (gfsk_dir / 'Hier_Block/untitled.grc').exists()


def test_dual_rx_nodes_respawn_after_ethernet_sdr_disconnect():
    rx2 = _read_package('launch/rx2.launch.py')

    # A disconnected libiio network source terminates only its child node while
    # ros2 launch stays alive. Both receiver children must therefore be respawned
    # by launch so reconnecting the cable creates a fresh IIO context.
    assert rx2.count('respawn=True') == 2
    assert rx2.count('respawn_delay=2.0') == 2


def test_match_app_wires_continuous_broadcast_demod_profile():
    config = _read_root('apps/match_rx/config.yaml')
    env = _read_root('apps/match_rx/env.sh')
    start = _read_root('apps/match_rx/start.sh')
    rx2 = _read_package('launch/rx2.launch.py')
    web = _read_root('apps/match_rx/web/app.js')
    html = _read_root('apps/match_rx/web/index.html')

    assert 'broadcast_gain_mode: manual' in config
    assert 'broadcast_gain:' in config
    assert 'broadcast_gain_max: 73' in config
    assert 'demod_mode: mm' in config
    assert 'frontend_profile: broadcast' in config
    assert 'frontend_profile: interference' in config
    assert 'input_sps: 94' in config
    assert 'iq_snapshot_enabled: false' in config
    assert 'iio_capture_enabled: false' in config
    assert 'iio_capture_only: false' in config
    assert 'is_true "$RUN_RX" && [[ "${BROADCAST_RX_GAIN_MODE,,}" != "manual" ]]' in start
    assert 'RX1 信息波接收机必须使用 manual' in start
    assert 'init_ad9361_rx "$RX1_URI" "$broadcast_freq" 540000 manual' in start
    assert start.count('"${RX_RF_PORT:-B_BALANCED}" defer') == 2
    assert 'RX1 信息波 manual/${BROADCAST_RX_GAIN}dB 已由硬件回读确认' in start
    assert 'RX1 信息波 manual/${BROADCAST_RX_GAIN}dB 硬件回读失败；以设备降级状态继续启动' in start
    assert 'RM_RADIO_RX_SKIP_PREINIT=true：跳过 RX2 预初始化' in start
    assert start.count('\\"rx_tracking\\":true') == 3
    assert 'self.rx_tracking = True' in _read_package('flowgraphs/gfsk/RX.py')
    assert 'iq_low_pass_hz: 260000' in config
    assert 'BROADCAST_DEMOD_MODE' in env
    assert 'BROADCAST_IQ_DECODER_ENABLED' in env
    assert 'BROADCAST_IIO_CAPTURE_DECODER_ENABLED' in env
    assert 'broadcast_demod_mode:="$BROADCAST_DEMOD_MODE"' in start
    assert 'broadcast_frontend_profile:="$BROADCAST_FRONTEND_PROFILE"' in start
    assert 'interference_frontend_profile:="$INTERFERENCE_FRONTEND_PROFILE"' in start
    assert 'broadcast_input_sps:="$BROADCAST_INPUT_SPS"' in start
    assert 'interference_input_sps:="$INTERFERENCE_INPUT_SPS"' in start
    assert 'broadcast_iq_decoder_enabled:="$BROADCAST_IQ_DECODER_ENABLED"' in start
    assert 'iio_capture_decoder_enabled:="$BROADCAST_IIO_CAPTURE_DECODER_ENABLED"' in start
    assert 'broadcast_iq_decoder_autotune_mode:="$BROADCAST_IQ_DECODER_AUTOTUNE_MODE"' in start
    assert "DeclareLaunchArgument('broadcast_demod_mode', default_value='mm')" in rx2
    assert "DeclareLaunchArgument('interference_demod_mode', default_value='legacy')" in rx2
    assert "DeclareLaunchArgument('broadcast_frontend_profile', default_value='broadcast')" in rx2
    assert "DeclareLaunchArgument('interference_frontend_profile', default_value='interference')" in rx2
    assert "DeclareLaunchArgument('broadcast_input_sps', default_value='94')" in rx2
    assert "DeclareLaunchArgument('interference_input_sps', default_value='94')" in rx2
    assert "DeclareLaunchArgument('broadcast_iq_decoder_autotune_mode'" in rx2
    assert "DeclareLaunchArgument('interference_iq_decoder_autotune_mode'" in rx2
    assert 'parsed.robot_main_status || {}' in web
    assert 'enhanced_offensive: "强化进攻姿态"' in web
    assert 'app.js?v=rx-20260730-link-recovery-v12' in html
    assert 'vision.camera_fps' in web
    assert 'vision.processing_fps ?? vision.fps' in web
    assert 'function nodeRuntimeLabel' in web
    assert 'externallyManaged' in web
    assert 'persist: true' in web
    assert '保存到 config.yaml' in html
    assert 'styles.css?v=rx-20260717-rx-windows-v10' in html
    assert 'let activePage = location.hash === "#radio" ? "radio" : "radar"' in web
    assert 'id="radarPage" class="app-page radar-page active"' in html
    assert 'id="radioPage" class="app-page" hidden' in html
    assert 'mark-switch-row' in web
    assert '0x0105 · 飞镖选定目标' in web
    assert 'id="rx2FrameRate"' in html
    assert 'function frameRateForRole' in web
    assert 'frameRateForRole("rx2", now).toFixed(1)' in web
    assert 'id="rx1Arrivals"' in html
    assert '<details class="broadcast-details">' not in html
    assert 'class="decoded-body rx1-decoded-window"' in html
    assert 'id="rx1Decoded"' in html
    assert 'function renderBroadcastArrivals' in web
    assert 'FRAME_RATE_WINDOW_SEC = 1' in web
    assert 'frameArrivalHistory' in web
    assert '1s 平均帧率' in web
    assert 'rx1_frame_rate: "RX1 信息波帧率"' in web
    assert 'rx2_frame_rate: "RX2 干扰波帧率"' in web
    assert 'healthSnapshot.state === "warn"' in web
    assert 'fmtArrivalClock' not in web
    assert 'WATERFALL_DBFS_MIN = -120' in web
    assert 'WATERFALL_DBFS_MAX = 0' in web
    assert 'desiredFloor' not in web
    assert 'renderDecodedRole($("rx2Decoded")' in web
    left_column = html.index('<section class="col col-left">')
    middle_column = html.index('<section class="col col-mid">')
    right_column = html.index('<section class="col col-right">')
    loss_diagnostics = html.index('id="lossDiagnostics"')
    assert left_column < middle_column < loss_diagnostics < right_column


def test_ros_setup_isolates_incompatible_user_setuptools_for_colcon_develop():
    setup_source = _read_package('setup.py')

    assert "os.environ.get('COLCON') == '1'" in setup_source
    assert "'--editable' in sys.argv" in setup_source
    assert "clean_env['PYTHONNOUSERSITE'] = '1'" in setup_source
    assert 'os.execve(sys.executable' in setup_source


def test_rx_preinit_requires_gain_mode_hardware_readback():
    script = r'''
iio_init="$1"
gain_mode_readback="$2"
source "$iio_init"

iio_info() { :; }
iio_attr() {
  case " $* " in
    *" gain_control_mode "*) printf '%s\n' "$gain_mode_readback" ;;
    *" rf_port_select "*) printf '%s\n' 'B_BALANCED' ;;
    *" sampling_frequency "*) printf '%s\n' '2000000' ;;
    *" rf_bandwidth "*) printf '%s\n' '540000' ;;
    *" frequency "*) printf '%s\n' '433200000' ;;
  esac
}

init_ad9361_rx ip:mock 433200000 540000 slow_attack 20 B_BALANCED 2000000
'''
    iio_init = str(WORKSPACE_ROOT / 'apps/common/iio_init.sh')

    confirmed = subprocess.run(
        ['bash', '-c', script, 'bash', iio_init, 'slow_attack'],
        capture_output=True,
        text=True,
        check=False,
    )
    assert confirmed.returncode == 0, confirmed.stderr
    assert 'gain_mode=slow_attack' in confirmed.stderr

    rejected = subprocess.run(
        ['bash', '-c', script, 'bash', iio_init, 'fast_attack'],
        capture_output=True,
        text=True,
        check=False,
    )
    assert rejected.returncode != 0
    assert 'init_ad9361_rx verification failed' in rejected.stderr
    assert 'actual_gain_mode=fast_attack' in rejected.stderr


def test_rx_preinit_accepts_only_small_ad9361_frequency_quantization():
    iio_init = _read_root('apps/common/iio_init.sh')

    assert 'RM_RADIO_RX_FREQUENCY_READBACK_TOLERANCE_HZ:-5' in iio_init


def test_tx_init_accepts_only_small_ad9361_frequency_quantization():
    script = r'''
iio_init="$1"
frequency_readback="$2"
source "$iio_init"

iio_info() { :; }
timeout() {
  shift 4
  "$@"
}
iio_attr() {
  case " $* " in
    *" altvoltage1 frequency "*) printf '%s\n' "$frequency_readback" ;;
    *" altvoltage1 powerdown "*) printf '%s\n' '0' ;;
    *" rf_port_select "*) printf '%s\n' 'A' ;;
    *" hardwaregain "*) printf '%s\n' '-11.000000 dB' ;;
    *" sampling_frequency "*) printf '%s\n' '2500000' ;;
    *" rf_bandwidth "*) printf '%s\n' '250000' ;;
  esac
}

init_ad9361_tx ip:mock 434320000 250000 11.0 A 2500000
'''
    iio_init = str(WORKSPACE_ROOT / 'apps/common/iio_init.sh')

    quantized = subprocess.run(
        ['bash', '-c', script, 'bash', iio_init, '434319998'],
        capture_output=True,
        text=True,
        check=False,
    )
    assert quantized.returncode == 0, quantized.stderr

    outside_tolerance = subprocess.run(
        ['bash', '-c', script, 'bash', iio_init, '434319994'],
        capture_output=True,
        text=True,
        check=False,
    )
    assert outside_tolerance.returncode != 0
    assert 'actual_freq=434319994' in outside_tolerance.stderr


def test_tx_init_accepts_z103_sample_rate_quantization_within_two_hz():
    script = r'''
iio_init="$1"
sample_rate_readback="$2"
source "$iio_init"

iio_info() { :; }
timeout() {
  shift 4
  "$@"
}
iio_attr() {
  case " $* " in
    *" altvoltage1 frequency "*) printf '%s\n' '433200000' ;;
    *" altvoltage1 powerdown "*) printf '%s\n' '0' ;;
    *" rf_port_select "*) printf '%s\n' 'A' ;;
    *" hardwaregain "*) printf '%s\n' '-61.000000 dB' ;;
    *" sampling_frequency "*) printf '%s\n' "$sample_rate_readback" ;;
    *" rf_bandwidth "*) printf '%s\n' '540000' ;;
  esac
}

init_ad9361_tx ip:mock 433200000 540000 61.0 A 2489362
'''
    iio_init = str(WORKSPACE_ROOT / 'apps/common/iio_init.sh')

    quantized = subprocess.run(
        ['bash', '-c', script, 'bash', iio_init, '2489361'],
        capture_output=True,
        text=True,
        check=False,
    )
    assert quantized.returncode == 0, quantized.stderr

    outside_tolerance = subprocess.run(
        ['bash', '-c', script, 'bash', iio_init, '2489359'],
        capture_output=True,
        text=True,
        check=False,
    )
    assert outside_tolerance.returncode != 0
    assert 'actual_sample_rate=2489359' in outside_tolerance.stderr


def test_rx_preinit_can_defer_low_sample_rate_until_gnuradio_loads_fir():
    script = r'''
iio_init="$1"
requested_rate="$2"
source "$iio_init"

iio_info() { :; }
iio_attr() {
  case " $* " in
    *" gain_control_mode "*) printf '%s\n' 'slow_attack' ;;
    *" rf_port_select "*) printf '%s\n' 'B_BALANCED' ;;
    *" sampling_frequency "*) printf '%s\n' '30720000' ;;
    *" rf_bandwidth "*) printf '%s\n' '540000' ;;
    *" frequency "*) printf '%s\n' '433200000' ;;
  esac
}

init_ad9361_rx ip:mock 433200000 540000 slow_attack 20 B_BALANCED "$requested_rate"
'''
    iio_init = str(WORKSPACE_ROOT / 'apps/common/iio_init.sh')

    deferred = subprocess.run(
        ['bash', '-c', script, 'bash', iio_init, 'defer'],
        capture_output=True,
        text=True,
        check=False,
    )
    assert deferred.returncode == 0, deferred.stderr
    assert 'sample_rate=30720000' in deferred.stderr

    strict = subprocess.run(
        ['bash', '-c', script, 'bash', iio_init, '2000000'],
        capture_output=True,
        text=True,
        check=False,
    )
    assert strict.returncode != 0
    assert 'requested_sample_rate=2000000' in strict.stderr
    assert 'actual_sample_rate=30720000' in strict.stderr


def test_tx_preinit_defers_low_rate_write_until_gnuradio_auto_fir():
    script = r'''
iio_init="$1"
mode="$2"
source "$iio_init"

iio_info() { :; }
timeout() {
  shift 4
  "$@"
}
iio_attr() {
  if [[ " $* " == *" sampling_frequency 1000000 "* ]]; then
    printf '%s\n' 'unexpected-low-rate-write' >&2
    return 9
  fi
  case " $* " in
    *" altvoltage1 frequency "*) printf '%s\n' '433200000' ;;
    *" altvoltage1 powerdown "*) printf '%s\n' '0' ;;
    *" rf_port_select "*) printf '%s\n' 'A' ;;
    *" hardwaregain "*) printf '%s\n' '-61.000000 dB' ;;
    *" sampling_frequency "*) printf '%s\n' '30720000' ;;
    *" rf_bandwidth "*) printf '%s\n' '540000' ;;
  esac
}

init_ad9361_tx ip:mock 433200000 540000 61.0 A 1000000 "$mode"
'''
    iio_init = str(WORKSPACE_ROOT / 'apps/common/iio_init.sh')

    deferred = subprocess.run(
        ['bash', '-c', script, 'bash', iio_init, 'defer'],
        capture_output=True,
        text=True,
        check=False,
    )
    assert deferred.returncode == 0, deferred.stderr
    assert 'unexpected-low-rate-write' not in deferred.stderr
    assert 'requested_sample_rate=1000000' in deferred.stderr
    assert 'sample_rate_mode=defer' in deferred.stderr
    assert 'sample_rate=30720000' in deferred.stderr

    strict = subprocess.run(
        ['bash', '-c', script, 'bash', iio_init, 'strict'],
        capture_output=True,
        text=True,
        check=False,
    )
    assert strict.returncode != 0
    assert 'unexpected-low-rate-write' in strict.stderr
    assert 'sample_rate_mode=strict' in strict.stderr


def test_tx_preinit_mode_only_defers_below_no_fir_rate_limit():
    script = r'''
source "$1"
printf '%s %s %s\n' \
  "$(ad9361_tx_preinit_mode 1000000)" \
  "$(ad9361_tx_preinit_mode 2083333)" \
  "$(ad9361_tx_preinit_mode invalid)"
'''
    completed = subprocess.run(
        ['bash', '-c', script, 'bash', str(WORKSPACE_ROOT / 'apps/common/iio_init.sh')],
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr
    assert completed.stdout.strip() == 'defer strict strict'


def test_gnuradio_auto_fir_tx_launchers_request_deferred_low_rate_preinit():
    for relative in (
        'apps/lab_tx/start_interference_tx.sh',
        'apps/lab_tx/start_dual_tx.sh',
        'apps/lab_tx/run_generated_link_test.sh',
    ):
        source = _read_root(relative)
        assert 'ad9361_tx_preinit_mode' in source
        assert '"$tx_preinit_mode"' in source

    # Raw IQ replay has no ganraoyuan Auto-FIR handoff and must remain strict.
    replay = _read_root('apps/lab_tx/start_iq_replay_tx.sh')
    assert 'ad9361_tx_preinit_mode' not in replay


def test_match_spectrum_uses_push_stream_waterfall_and_native_fallback():
    config = _read_root('apps/match_rx/config.yaml')
    env = _read_root('apps/match_rx/env.sh')
    start = _read_root('apps/match_rx/start.sh')
    web = _read_root('apps/match_rx/web/app.js')
    html = _read_root('apps/match_rx/web/index.html')
    styles = _read_root('apps/match_rx/web/styles.css')
    flowgraph_node = _read_package('rm_radio_ros/nodes/flowgraph_node.py')

    assert 'period_sec: 0.04' in config
    assert 'fft_size: 1024' in config
    assert 'bin_count: 256' in config
    assert 'auto_open_browser: true' in config
    assert 'dedicated_browser' not in config
    assert 'native_gui: false' in config
    assert 'native_spectrum_only: true' in config
    assert 'qt_platform: offscreen' in config
    assert 'SPECTRUM_PERIOD_SEC' in env
    assert 'RM_RADIO_AUTO_OPEN_BROWSER' in env
    assert 'RM_RADIO_DEDICATED_BROWSER' not in env
    assert 'RM_RADIO_NATIVE_SPECTRUM_ONLY' in env
    assert '--native-gui' in start
    assert '--no-native-gui' in start
    assert 'prepare_graphical_session' in start
    assert 'source "$RM_RADIO_WS/apps/common/browser_tabs.sh"' in start
    assert 'rm_radio_wait_and_open_browser_tab "$url"' in start
    assert '--app="$url"' not in start
    assert '--user-data-dir' not in start
    assert '--start-maximized' not in start
    assert 'BROADCAST_IIO_CAPTURE_DECODER_ENABLED=false' in start
    assert 'disable_gui_sinks:="$DISABLE_GUI_SINKS"' in start
    assert "self._set_flowgraph_window_visible(True)" in flowgraph_node
    assert "action = getattr(self.flowgraph, 'show', None)" in flowgraph_node
    assert 'after_seq' in web
    assert 'new EventSource(`/api/spectrum-stream?' in web
    assert 'waterfallQueues' in web
    assert 'updateDisplayPerformance' in web
    assert 'apiPost("/api/client-metrics", metrics)' in web
    assert 'streamRequestBusy' in web
    assert 'spectrumStreamState !== "open"' in web
    assert 'Math.min(window.devicePixelRatio || 1, 1.5)' in web
    assert 'gain.min_db ?? -1' in web
    assert 'gain.max_db ?? 73' in web
    assert 'min="-1" max="73"' in html
    assert '.col-left {' in styles
    assert 'overflow-y: auto;' in styles
    assert 'grid-template-columns: minmax(390px, 1.35fr) minmax(310px, 0.8fr) minmax(320px, 1fr);' in styles
    assert 'flex: 1.35 1 300px;' in styles
    assert 'WATERFALL_DBFS_MIN = -120' in web
    assert 'WATERFALL_DBFS_MAX = 0' in web
    assert '@media (min-width: 1141px) and (max-height: 900px)' in styles
    assert '-1.0 <= gain <= limit <= 73.0' in start


def test_raw_capture_aborts_when_dual_tx_did_not_start():
    source = _read_root('apps/lab_tx/run_dual_tx_raw_rx1_capture.sh')

    assert 'dual TX failed to start' in source
    assert 'dual_tx.pids' in source
    assert 'kill -0 "$TX_LAUNCHER_PID"' in source
    assert 'value["best_summary"]' in source
    assert 'RX_CENTER_FREQUENCY' in source
    assert 'RUN_BROADCAST_SCAN' in source


def test_demod_ab_script_selects_each_rx1_path_explicitly():
    live = _read_root('apps/lab_tx/run_four_sdr_live_test.sh')
    ab = _read_root('apps/lab_tx/run_demod_ab_test.sh')

    assert 'BROADCAST_DEMOD_MODE="${BROADCAST_DEMOD_MODE:-mm}"' in live
    assert 'BROADCAST_IIO_CAPTURE_DECODER_ONLY="${BROADCAST_IIO_CAPTURE_DECODER_ONLY:-false}"' in live
    assert 'MODES="${DEMOD_AB_MODES:-mm capture_only legacy_stream}"' in ab
    assert 'BROADCAST_DEMOD_MODE="$demod_mode"' in ab
    assert 'BROADCAST_IIO_CAPTURE_DECODER_ONLY="$capture_only"' in ab
    assert 'lab_tx_hard_mute_and_verify' in _read_root('apps/lab_tx/run_four_sdr_live_test.sh')
    assert 'rx1_reconstructed_frame_count' in live
    assert 'rx1_tx_payload_mismatch_count' in live
    assert 'rx1_tx_payload_audit_pass' in live


def test_lab_dual_tx_launcher_is_fail_fast_and_hard_mutes_on_exit():
    source = _read_root('apps/lab_tx/start_dual_tx.sh')
    safety = _read_root('apps/common/lab_tx_safety.sh')
    iio_init = _read_root('apps/common/iio_init.sh')

    assert 'trap dual_tx_cleanup EXIT' in source
    assert 'wait -n "$broadcast_pid" "$interference_pid"' in source
    assert 'lab_tx_cleanup_residual_tx' in source
    assert 'lab_tx_hard_mute_and_verify "$BROADCAST_TX_URI" "$INTERFERENCE_TX_URI"' in source
    assert 'lab_tx_verify_no_residual_tx' in source
    assert 'hardwaregain' in safety
    assert 'tx_lo_powerdown' in safety
    assert 'init_ad9361_tx verification failed' in iio_init
    assert 'IIO TX mute verification failed' in iio_init
    assert '_iio_number_close "$actual_rate" "$sample_rate"' in iio_init
    assert '_iio_tx_run_with_timeout iio_info' in iio_init
    assert 'timeout --foreground --signal=TERM --kill-after=1s' in iio_init
    assert 'gain="$(_iio_tx_attr_get' in safety


def test_lab_main_tx_buffering_comes_from_yaml_not_stale_environment():
    script = r'''
export RM_RADIO_TX_BUFFER_SIZE=32768
export RM_RADIO_TX_PREFILL_PACKETS=3
export RADIO_TX_SKIP_LOCAL_ENV=true
source "$1"
printf '%s %s\n' "$RM_RADIO_TX_BUFFER_SIZE" "$RM_RADIO_TX_PREFILL_PACKETS"
'''
    completed = subprocess.run(
        ['bash', '-c', script, 'bash', str(WORKSPACE_ROOT / 'apps/lab_tx/env.sh')],
        cwd=str(WORKSPACE_ROOT),
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr
    assert completed.stdout.strip() == '1048576 0'


def test_lab_tx_panel_uses_three_board_single_tx_topology():
    config = _read_root('apps/lab_tx/config.yaml')
    dashboard = _read_root('apps/lab_tx/radio_tx_dashboard.py')
    html = _read_root('apps/lab_tx/web/index.html')
    web = _read_root('apps/lab_tx/web/app.js')
    launcher = _read_root('apps/lab_tx/start_interference_tx.sh')
    iq_replay = _read_root('apps/lab_tx/start_iq_replay_tx.sh')
    dashboard_launcher = _read_root('apps/lab_tx/start_dashboard.sh')

    assert 'side: blue' in config
    assert 'sample_rate: 1000000' in config
    assert 'broadcast_tx_uri: ip:192.168.3.1' in config
    assert 'interference_tx_uri: ""' in config
    assert 'broadcast_attenuation_db: 61' in config
    assert 'interference_attenuation_db: 11' in config
    assert 'Pluto / AD9361' in html
    assert 'id="interferenceTxUri" value=""' in html
    assert 'defaultTxAttenuation = { broadcast: 61, interference: 11 }' in web
    assert 'applyServerDefaults(result.state)' in web
    assert 'os.environ.get("INTERFERENCE_TX_URI", "")' in dashboard
    assert '"mode": "dual" if interference_installed else "single_broadcast"' in dashboard
    assert '_ros_string_override("interference_tx_uri", uri)' in dashboard
    assert 'def _is_tx_process_command(command: str)' in dashboard
    assert 'return "start_dual_tx.sh" in token_basenames' in dashboard
    assert '报告相同硬件序列号' in dashboard
    assert 'app.js?v=tx-20260716-protocol-ranges' in html
    assert 'id="broadcastAutoChange"' in html
    assert 'broadcast.auto_change_data = $("broadcastAutoChange").checked' in web
    assert '"$INTERFERENCE_TX_RF_PORT"' in launcher
    assert '干扰波 TX 当前未连接' in launcher
    assert 'interference_tx_uri:="$INTERFERENCE_TX_URI"' in launcher
    assert 'sample_rate:="$RM_RADIO_SAMPLE_RATE"' in launcher
    assert 'setters_json:="$INTERFERENCE_SETTERS_JSON"' in launcher
    assert 'IQ_REPLAY_TX_URI="$BROADCAST_TX_URI"' in iq_replay
    assert 'tx_uri:=\\"$IQ_REPLAY_TX_URI\\"' in iq_replay
    assert 'lab_tx_hard_mute_reachable_configured' in dashboard_launcher
    assert 'source "$RM_RADIO_WS/apps/common/browser_tabs.sh"' in dashboard_launcher
    assert '--no-browser' in dashboard_launcher
    assert 'rm_radio_wait_and_open_browser_tab' in dashboard_launcher


def test_deployment_configuration_has_no_hidden_machine_local_override():
    gitignore = _read_root('.gitignore')
    match_env = _read_root('apps/match_rx/env.sh')
    lab_env = _read_root('apps/lab_tx/env.sh')
    match_config = _read_root('apps/match_rx/config.yaml')
    lab_config = _read_root('apps/lab_tx/config.yaml')

    assert 'apps/match_rx/local.env' not in gitignore
    assert 'apps/lab_tx/local.env' not in gitignore
    assert 'MATCH_RX_ENV_FILE' in match_env
    assert 'LAB_TX_ENV_FILE' in lab_env
    assert '"$MATCH_RX_DIR/local.env"' not in match_env
    assert '"$RADIO_TX_DIR/local.env"' not in lab_env
    assert '/home/' not in match_config
    assert '/home/' not in lab_config
    assert 'intelligent_mode: true' in match_config
    assert 'broadcast_iio_capture_enabled: true' in lab_config


def test_browser_helper_opens_normal_tabs_without_private_profiles():
    helper = _read_root('apps/common/browser_tabs.sh')

    assert 'google-chrome' in helper
    assert '"$browser" "$url"' in helper
    for forbidden in ('--app', '--user-data-dir', '--incognito', '--kiosk'):
        assert forbidden not in helper


def test_lab_rf_test_scripts_require_explicit_confirmation_and_verify_shutdown():
    raw = _read_root('apps/lab_tx/run_dual_tx_raw_rx1_capture.sh')
    live = _read_root('apps/lab_tx/run_four_sdr_live_test.sh')

    for source in (raw, live):
        assert 'export RM_RADIO_LAB_TX_ANTENNA_CONFIRM=true' not in source
        assert 'require_lab_tx_confirmation' in source
        assert source.count('lab_tx_verify_dual_running') >= 2
        assert 'finalize_tx_safety' in source
        assert 'lab_tx_verify_no_residual_tx' in source
        assert 'lab_tx_hard_mute_and_verify' in source


def test_panels_replace_residual_processes_and_cleanup_on_exit():
    match = _read_root('apps/match_rx/start.sh')
    lab = _read_root('apps/lab_tx/start_dashboard.sh')
    dashboard = _read_root('apps/lab_tx/radio_tx_dashboard.py')

    assert 'export RM_RADIO_KILL_RESIDUAL=true' in match
    assert 'MATCH_RESIDUAL_PATTERNS' in match
    assert '"rm_radio_runtime_logs/match_rx_"' in match
    assert 'MATCH_WRAPPER_PID_FILE' in match
    assert 'replace_match_wrapper' in match
    assert 'flock -x "$lock_fd"' in match
    assert 'unregister_match_wrapper' in match
    assert 'exec setsid "$@"' in match
    assert 'RM_RADIO_SUPERVISED_RX_PID' in match
    assert 'RM_RADIO_SUPERVISED_REFEREE_PID' in match
    assert 'trap stop_all EXIT' in match
    assert 'rm_radio_guard_residual "match-rx-cleanup"' in match

    assert 'export RM_RADIO_KILL_RESIDUAL=true' in lab
    assert 'TX_DASHBOARD_RESIDUAL_PATTERNS' in lab
    assert 'LAB_DASHBOARD_PID_FILE' in lab
    assert 'replace_lab_dashboard_wrapper' in lab
    assert 'flock -x "$lock_fd"' in lab
    assert 'unregister_lab_dashboard_wrapper' in lab
    assert '"rm_broadcast_tx_node"' in lab
    assert '"rm_interference_tx_node"' in lab
    assert 'trap cleanup_dashboard EXIT' in lab
    assert 'setsid python3' in lab
    assert 'lab_tx_hard_mute_and_verify "${reachable[@]}"' in lab
    assert lab.count('lab_tx_hard_mute_reachable_configured') >= 3
    assert 'stop_wave(server, "broadcast")' in dashboard
    assert 'stop_wave(server, "interference")' in dashboard


def test_residual_guards_never_signal_process_groups():
    for relative in ('apps/lab_tx/env.sh', 'apps/match_rx/env.sh'):
        env = _read_root(relative)

        assert 'kill -TERM -- "-$pg"' not in env
        assert 'kill -KILL -- "-$pg"' not in env
        assert 'kill -TERM "$pid"' in env
        assert 'kill -KILL "$pid"' in env
        assert 'local self="${BASHPID:-$$}"' in env
        assert 'local ancestors=" $self "' in env
        assert 'index(ancestors, " " pid " ") > 0' in env


def test_setup_entrypoints_match_new_module_layout():
    setup = _read_package('setup.py')

    assert 'rm_gfsk_node = rm_radio_ros.nodes.gfsk_node:main' in setup
    assert 'rm_ganraoyuan_node = rm_radio_ros.nodes.ganraoyuan_node:main' in setup
    assert 'rm_match_dashboard = rm_radio_ros.app_support.match_dashboard_node:main' in setup
    assert 'rm_find_usb_serial = rm_radio_ros.core.usb_serial_finder:main' in setup
    assert 'scripts_dir' not in setup
    assert 'web_dir' not in setup


def test_algorithm_wrapper_is_archived_and_match_optional():
    match_start = _read_root('apps/match_rx/start.sh')
    optional = _read_root('apps/match_rx/optional_algorithm.sh')

    assert 'RUN_ALGORITHM' in match_start
    assert 'optional_algorithm.sh' in match_start
    assert 'RM_ALGO_DISABLE_REFEREE_SERIAL' in optional
    assert 'run_algorithm.py' in optional
