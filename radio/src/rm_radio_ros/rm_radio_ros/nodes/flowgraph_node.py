from __future__ import annotations

import importlib.util
import json
import os
import re
import signal
import sys
import threading
import time
import traceback
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ..core.match_recording import CompactIqRecorder
from ..core.referee_bridge import RefereeBridge
from ..core.reception_metrics import ReceptionMetrics
from ..core.rm_protocol import (
    ACCESS_CODES,
    RADAR_AIR_DATA_LENGTHS,
    AccessCodeAirPacketExtractor,
    RefereeFrameAssembler,
    bytes_to_bits,
)


_SETTLING_SETTER_NAMES = {
    'access',
    'BW',
    'bw_re',
    'cen_f',
    'center_f',
    'center_F',
    'Gain',
    'GainMode',
    'LowPass',
    'rx_tracking',
    'sample_rate',
    'sen_re',
    'sncy',
    'sps',
}


class FlowgraphNode(Node):
    """ROS 2 node that owns one GNU Radio top_block flowgraph."""

    def __init__(self, node_name: str = 'rm_flowgraph_node', flowgraph_name: str = ''):
        super().__init__(node_name)

        default_name = flowgraph_name or 'gfsk'
        self.declare_parameter('flowgraph_name', default_name)
        self.declare_parameter('flowgraph_dir', '')
        self.declare_parameter('flowgraph_module', 'RM')
        self.declare_parameter('flowgraph_class', 'RM')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('auto_start_retry_sec', 5.0)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('qt_platform', 'offscreen')
        self.declare_parameter('disable_gui_sinks', True)
        self.declare_parameter('rx_only', True)
        self.declare_parameter('qt_event_period_sec', 0.05)
        self.declare_parameter('rx_uri', 'ip:192.168.2.1')
        self.declare_parameter('broadcast_tx_uri', 'ip:192.168.1.10')
        self.declare_parameter('interference_tx_uri', 'ip:192.168.3.1')
        try:
            default_sample_rate = float(os.environ.get('RM_RADIO_SAMPLE_RATE', '0') or 0)
        except (TypeError, ValueError):
            default_sample_rate = 0.0
        self.declare_parameter('sample_rate', default_sample_rate)
        self.declare_parameter('radio_side', 'red')
        self.declare_parameter('rx_profile', 'broadcast')
        self.declare_parameter('frontend_profile', 'auto')
        self.declare_parameter('demod_mode', 'auto')
        self.declare_parameter('input_sps', 94.0)
        self.declare_parameter('interference_level', 1)
        self.declare_parameter('setters_json', os.environ.get('RM_RADIO_SETTERS_JSON', '{}'))
        self.declare_parameter('setters_apply_settle_sec', 0.15)
        self.declare_parameter('status_period_sec', 1.0)
        self.declare_parameter('air_extractor_max_bits', 16384)
        self.declare_parameter('air_extractor_max_access_hamming', 3)
        self.declare_parameter('air_extractor_max_length_hamming', 0)
        self.declare_parameter('air_extractor_allow_inverted', True)
        self.declare_parameter('demod_bit_diagnostics_every_n_batches', 20)
        self.declare_parameter('iq_decoder_enabled', True)
        self.declare_parameter('iq_decoder_window_sec', 0.5)
        self.declare_parameter('iq_decoder_period_sec', 0.35)
        self.declare_parameter('iq_decoder_sps_values', '92,92.5,93,93.5,94,94.5,95')
        self.declare_parameter('iq_decoder_offset_step', 4)
        self.declare_parameter('iq_decoder_decode_threshold', 3)
        self.declare_parameter('iq_decoder_max_payloads', 128)
        self.declare_parameter('iq_decoder_max_frames', 32)
        self.declare_parameter('iq_decoder_autotune_mode', 'quick')
        self.declare_parameter('iq_decoder_frequency_shift_hz', 0.0)
        self.declare_parameter('iq_decoder_low_pass_hz', 0.0)
        self.declare_parameter('iq_decoder_max_frequency_candidates', 4)
        self.declare_parameter('iq_decoder_max_candidates', 8)
        self.declare_parameter('spectrum_enabled', True)
        self.declare_parameter('spectrum_period_sec', 0.5)
        self.declare_parameter('spectrum_fft_size', 512)
        self.declare_parameter('spectrum_bin_count', 128)
        self.declare_parameter('iio_capture_decoder_enabled', False)
        self.declare_parameter('iio_capture_decoder_only', False)
        self.declare_parameter('iio_capture_seconds', 1.2)
        self.declare_parameter('iio_capture_period_sec', 1.5)
        self.declare_parameter('iio_capture_start_delay_sec', 0.0)
        self.declare_parameter('iio_capture_timeout_sec', 8.0)
        self.declare_parameter('iio_capture_buffer_size', 32768)
        self.declare_parameter('iio_attribute_timeout_sec', 3.0)
        self.declare_parameter('iio_capture_stop_timeout_sec', 12.0)
        self.declare_parameter(
            'iio_capture_rf_port',
            os.environ.get('RX_RF_PORT', 'B_BALANCED'),
        )
        # Sticky autotune: after a successful live decode, reuse the winning (shift, low_pass)
        # candidate via a single fast scan instead of re-scanning the full grid every cycle.
        # Pure RX-side latency optimization; falls back to full autotune after repeated misses.
        self.declare_parameter('iq_decoder_autotune_lock_enabled', True)
        self.declare_parameter('iq_decoder_autotune_lock_max_miss', 3)
        # F5 time-diversity voting: recover the interference password by majority-voting its
        # repeated copies. Only ever applied to the interference profile (constant frame).
        self.declare_parameter('iq_decoder_vote_repeated_frames', True)
        self.declare_parameter('recording_enabled', False)
        self.declare_parameter(
            'recording_root', '~/.local/share/shark-radio/match-records'
        )
        self.declare_parameter('recording_role', 'rx')
        self.declare_parameter('recording_control_topic', '/rm_match_recorder/control')
        self.declare_parameter('recording_output_sample_rate', 1_000_000.0)
        self.declare_parameter('recording_segment_sec', 30.0)
        self.declare_parameter('recording_queue_chunks', 64)
        self.declare_parameter('recording_min_free_gb', 15.0)
        self.declare_parameter('recording_iq_compression', 'zlib')
        self.declare_parameter('recording_iq_compression_level', 1)

        self.flowgraph = None
        self.qt_app = None
        self._qt_event_timer = None
        self.started = False
        self.loaded = False
        self._flowgraph_runtime_started = False
        self.last_error = ''
        self.last_setters: Dict[str, Any] = {}
        self.last_setters_apply_time = 0.0
        self.setters_apply_count = 0
        self._setters_lock = threading.RLock()
        self._iio_device_lock = threading.RLock()
        self.frame_assembler = RefereeFrameAssembler(allowed_cmd_lengths=RADAR_AIR_DATA_LENGTHS)
        self.iq_frame_assembler = RefereeFrameAssembler(allowed_cmd_lengths=RADAR_AIR_DATA_LENGTHS)
        self.reception_metrics = ReceptionMetrics(window_sec=10.0, idle_reset_sec=2.0)
        self.air_packet_extractor = self._make_air_packet_extractor()
        self.referee_bridge = RefereeBridge()
        self.decode_stats = self._new_decode_stats()
        self.demod_bit_diagnostics: Dict[str, Any] = {}
        self.iq_diagnostics: Dict[str, Any] = {}
        self.filtered_iq_diagnostics: Dict[str, Any] = {}
        self.iq_decode_diagnostics: Dict[str, Any] = {}
        self._iq_buffer_lock = threading.RLock()
        self._iq_decode_state_lock = threading.RLock()
        self._iq_buffer_chunks: list[tuple[int, Any]] = []
        self._iq_buffer_sample_count = 0
        self._iq_total_samples_seen = 0
        self._iq_decode_thread: Optional[threading.Thread] = None
        self._iq_decode_last_start_time = 0.0
        self._iq_seen_payload_keys: set[tuple[str, int, str]] = set()
        self._iq_seen_payload_order: list[tuple[str, int, str]] = []
        self._iq_seen_frame_keys: set[tuple[int, str]] = set()
        self._iq_seen_frame_order: list[tuple[int, str]] = []
        self._iio_capture_thread_lock = threading.RLock()
        self._iio_capture_stop_event: Optional[threading.Event] = None
        self._iio_capture_thread: Optional[threading.Thread] = None
        self._iio_capture_generation = 0
        self._iio_capture_stopping = False
        self._iio_capture_total_samples_seen = 0
        self._iq_locked_candidate: Optional[dict] = None
        self._iq_lock_miss_count = 0
        self._spectrum_last_publish_time = 0.0
        self._filtered_spectrum_last_publish_time = 0.0
        self._recording_init_error = ''
        try:
            recording_input_rate = float(self.get_parameter('sample_rate').value)
            if recording_input_rate <= 0:
                recording_input_rate = float(os.environ.get('RM_RADIO_SAMPLE_RATE', '2000000'))
            self._iq_recorder = CompactIqRecorder(
                enabled=bool(self.get_parameter('recording_enabled').value),
                root=str(self.get_parameter('recording_root').value),
                role=str(self.get_parameter('recording_role').value),
                input_sample_rate=recording_input_rate,
                output_sample_rate=float(
                    self.get_parameter('recording_output_sample_rate').value
                ),
                segment_seconds=float(self.get_parameter('recording_segment_sec').value),
                queue_chunks=int(self.get_parameter('recording_queue_chunks').value),
                min_free_gb=float(self.get_parameter('recording_min_free_gb').value),
                compression=str(self.get_parameter('recording_iq_compression').value),
                compression_level=int(
                    self.get_parameter('recording_iq_compression_level').value
                ),
            )
        except Exception as exc:  # noqa: BLE001 - recording must not take down RX
            self._recording_init_error = f'IQ recorder configuration failed: {exc}'
            self.get_logger().error(self._recording_init_error)
            self._iq_recorder = CompactIqRecorder(
                enabled=False,
                root='~/.local/share/shark-radio/match-records',
                role=str(self.get_parameter('recording_role').value),
                input_sample_rate=2_000_000.0,
                output_sample_rate=1_000_000.0,
            )

        diagnostic_qos = QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '~/status', diagnostic_qos)
        self.frames_pub = self.create_publisher(String, '~/frames', diagnostic_qos)
        self.referee_bridge_pub = self.create_publisher(String, '~/referee_bridge', diagnostic_qos)
        # Spectrum is latest-only telemetry. A volatile depth-1 queue prevents
        # stale FFT JSON from building up behind frame/status traffic.
        spectrum_qos = QoSProfile(depth=1)
        self.spectrum_pub = self.create_publisher(String, '~/spectrum', spectrum_qos)
        self.filtered_spectrum_pub = self.create_publisher(String, '~/filtered_spectrum', spectrum_qos)
        self.create_subscription(String, '~/air_payload_hex', self._handle_air_payload_hex, 10)
        self.create_subscription(String, '~/demod_bits_hex', self._handle_demod_bits_hex, 10)
        setters_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '~/setters_json', self._handle_setters_json, setters_qos)
        recording_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String,
            str(self.get_parameter('recording_control_topic').value),
            self._handle_recording_control,
            recording_qos,
        )
        self.create_service(Trigger, '~/start', self._handle_start)
        self.create_service(Trigger, '~/stop', self._handle_stop)
        self.create_service(Trigger, '~/reload', self._handle_reload)
        self.create_service(Trigger, '~/status_trigger', self._handle_status)

        status_period = float(self.get_parameter('status_period_sec').value)
        self.create_timer(max(status_period, 0.1), self._publish_status)

        self._auto_start_retry_timer = None
        if bool(self.get_parameter('auto_start').value):
            if not self.start_flowgraph():
                retry_sec = max(
                    float(self.get_parameter('auto_start_retry_sec').value),
                    0.0,
                )
                if retry_sec > 0.0:
                    self._auto_start_retry_timer = self.create_timer(
                        retry_sec,
                        self._retry_auto_start,
                    )
                    self.get_logger().warning(
                        'flowgraph unavailable; ROS node remains active and will '
                        f'retry every {retry_sec:g} seconds'
                    )

    def _default_flowgraph_dir(self, flowgraph_name: str) -> Path:
        share_dir = Path(get_package_share_directory('rm_radio_ros'))
        return share_dir / 'flowgraphs' / flowgraph_name

    def _make_air_packet_extractor(self) -> AccessCodeAirPacketExtractor:
        rx_profile = str(self.get_parameter('rx_profile').value).strip().lower()
        access_code = ACCESS_CODES.get(rx_profile)
        access_codes = [access_code] if access_code is not None else ACCESS_CODES.values()
        if access_code is None:
            self.get_logger().warning(
                f'unknown rx_profile={rx_profile!r}; matching all radar air access codes'
            )
        return AccessCodeAirPacketExtractor(
            access_codes=access_codes,
            max_bits=int(self.get_parameter('air_extractor_max_bits').value),
            max_access_hamming=int(self.get_parameter('air_extractor_max_access_hamming').value),
            max_length_hamming=int(self.get_parameter('air_extractor_max_length_hamming').value),
            allow_inverted=bool(self.get_parameter('air_extractor_allow_inverted').value),
        )

    def _set_rx_profile(self, rx_profile: str) -> None:
        clean_profile = str(rx_profile).strip().lower()
        if not clean_profile:
            raise ValueError('rx_profile must not be empty')
        self.set_parameters([
            Parameter(
                'rx_profile',
                Parameter.Type.STRING,
                clean_profile,
            )
        ])
        self.frame_assembler.clear()
        self.reception_metrics.clear()
        self.air_packet_extractor = self._make_air_packet_extractor()
        self.demod_bit_diagnostics = {}
        self._iq_locked_candidate = None
        self._iq_lock_miss_count = 0
        self.get_logger().warning(f'switched decoder rx_profile to {clean_profile}')

    def _set_interference_level(self, level: Any) -> None:
        try:
            clean_level = int(level)
        except (TypeError, ValueError) as exc:
            raise ValueError(f'interference_level must be 1, 2, or 3, got {level!r}') from exc
        if clean_level not in (1, 2, 3):
            raise ValueError(f'interference_level must be 1, 2, or 3, got {level!r}')
        current_level = int(self.get_parameter('interference_level').value)
        if current_level == clean_level:
            return
        self.set_parameters([
            Parameter(
                'interference_level',
                Parameter.Type.INTEGER,
                clean_level,
            )
        ])
        self._iq_locked_candidate = None
        self._iq_lock_miss_count = 0
        self.get_logger().warning(f'switched decoder interference_level to {clean_level}')

    @staticmethod
    def _new_decode_stats() -> Dict[str, int]:
        return {
            'manual_air_payloads': 0,
            'demod_bit_batches': 0,
            'iq_decode_batches': 0,
            'air_payloads': 0,
            'iq_air_payloads': 0,
            'iq_referee_frames': 0,
            'frames': 0,
            'bridge_outputs': 0,
            'iio_capture_batches': 0,
            'iio_capture_errors': 0,
        }

    def _reset_decode_state(self) -> None:
        self.frame_assembler.clear()
        self.iq_frame_assembler.clear()
        self.reception_metrics.clear()
        self.air_packet_extractor.clear()
        self.decode_stats = self._new_decode_stats()
        self.demod_bit_diagnostics = {}
        self.iq_diagnostics = {}
        self.filtered_iq_diagnostics = {}
        self.iq_decode_diagnostics = {}
        with self._iq_buffer_lock:
            self._iq_buffer_chunks.clear()
            self._iq_buffer_sample_count = 0
            self._iq_total_samples_seen = 0
        with self._iq_decode_state_lock:
            self._iq_seen_payload_keys.clear()
            self._iq_seen_payload_order.clear()
            self._iq_seen_frame_keys.clear()
            self._iq_seen_frame_order.clear()
            self._iq_locked_candidate = None
            self._iq_lock_miss_count = 0

    def _resolve_flowgraph_dir(self) -> Path:
        flowgraph_dir = str(self.get_parameter('flowgraph_dir').value).strip()
        if flowgraph_dir:
            return Path(flowgraph_dir).expanduser().resolve()
        flowgraph_name = str(self.get_parameter('flowgraph_name').value).strip()
        return self._default_flowgraph_dir(flowgraph_name)

    def _ensure_qt_application(self) -> None:
        qt_platform = str(self.get_parameter('qt_platform').value).strip()
        if qt_platform:
            os.environ.setdefault('QT_QPA_PLATFORM', qt_platform)

        from PyQt5 import Qt

        app = Qt.QApplication.instance()
        if app is None:
            args = ['rm_radio_ros']
            if qt_platform:
                args += ['-platform', qt_platform]
            app = Qt.QApplication(args)
        self.qt_app = app
        self._ensure_qt_event_timer()

    def _ensure_qt_event_timer(self) -> None:
        if self.qt_app is None or self._qt_event_timer is not None:
            return
        period = float(self.get_parameter('qt_event_period_sec').value)
        if period <= 0:
            return
        self._qt_event_timer = self.create_timer(max(period, 0.01), self._process_qt_events)

    def _process_qt_events(self) -> None:
        app = self.qt_app
        if app is None:
            return
        try:
            app.processEvents()
        except Exception as exc:  # noqa: BLE001 - keep ROS node alive on Qt shutdown races
            self.last_error = f'Qt event processing failed: {exc}'

    def _set_flowgraph_window_visible(self, visible: bool) -> bool:
        """Show/hide the generated GNU Radio Qt window from the ROS wrapper.

        Generated flowgraphs normally call ``show()`` in their standalone
        ``main`` function.  The ROS wrapper instantiates the top block directly,
        so native sinks were connected but their parent window stayed hidden.
        """
        if self.flowgraph is None:
            return False
        if visible and bool(self.get_parameter('disable_gui_sinks').value):
            return False
        try:
            if visible:
                profile = str(self.get_parameter('rx_profile').value).strip().lower()
                role = '信息波 RX1' if profile == 'broadcast' else '干扰波 RX2'
                set_title = getattr(self.flowgraph, 'setWindowTitle', None)
                if callable(set_title):
                    set_title(f'SharkRadio GNU Radio - {role}')
                set_geometry = getattr(self.flowgraph, 'setGeometry', None)
                app = getattr(self, 'qt_app', None)
                screen = app.primaryScreen() if app is not None else None
                if callable(set_geometry) and screen is not None:
                    available = screen.availableGeometry()
                    width = max(480, available.width() // 2)
                    x = available.x() if profile == 'broadcast' else available.x() + available.width() - width
                    set_geometry(x, available.y(), width, available.height())
                action = getattr(self.flowgraph, 'show', None)
            else:
                action = getattr(self.flowgraph, 'hide', None)
            if not callable(action):
                return False
            action()
            self._process_qt_events()
            self.get_logger().info(
                f'GNU Radio Qt window {"shown" if visible else "hidden"}'
            )
            return True
        except Exception as exc:  # noqa: BLE001 - spectrum UI must not kill RX
            self.get_logger().warning(
                f'failed to {"show" if visible else "hide"} GNU Radio Qt window: {exc}'
            )
            return False

    def _configure_flowgraph_environment(self) -> None:
        disable_gui = bool(self.get_parameter('disable_gui_sinks').value)
        os.environ['RM_RADIO_DISABLE_GUI_SINKS'] = '1' if disable_gui else '0'
        rx_only = bool(self.get_parameter('rx_only').value)
        os.environ['RM_RADIO_RX_ONLY'] = '1' if rx_only else '0'
        uri_env_map = {
            'rx_uri': 'RM_RADIO_RX_URI',
            'broadcast_tx_uri': 'RM_RADIO_BROADCAST_TX_URI',
            'interference_tx_uri': 'RM_RADIO_INTERFERENCE_TX_URI',
        }
        for param_name, env_name in uri_env_map.items():
            value = str(self.get_parameter(param_name).value).strip()
            if value:
                os.environ[env_name] = value
        sample_rate = float(self.get_parameter('sample_rate').value)
        if sample_rate > 0:
            os.environ['RM_RADIO_SAMPLE_RATE'] = (
                str(int(sample_rate)) if sample_rate.is_integer() else str(sample_rate)
            )
        os.environ['RM_RADIO_SIDE'] = str(self.get_parameter('radio_side').value).strip().lower()
        os.environ['RM_RADIO_RX_PROFILE'] = str(self.get_parameter('rx_profile').value).strip().lower()
        frontend_profile = str(self.get_parameter('frontend_profile').value).strip().lower()
        if frontend_profile not in ('auto', 'broadcast', 'interference'):
            raise ValueError(
                'frontend_profile must be auto, broadcast, or interference, '
                f'got {frontend_profile!r}'
            )
        os.environ['RM_RADIO_FRONTEND_PROFILE'] = frontend_profile
        os.environ['RM_RADIO_DEMOD_MODE'] = str(self.get_parameter('demod_mode').value).strip().lower()
        try:
            input_sps = float(self.get_parameter('input_sps').value)
        except (KeyError, TypeError, ValueError):
            input_sps = float(os.environ.get('RM_RADIO_INPUT_SPS', '94') or 94)
        if input_sps <= 1.0:
            raise ValueError(f'input_sps must be greater than 1, got {input_sps}')
        os.environ['RM_RADIO_INPUT_SPS'] = str(input_sps)
        os.environ['RM_RADIO_INTERFERENCE_LEVEL'] = str(int(self.get_parameter('interference_level').value))
        self.get_logger().info(
            'configured SDR URIs: '
            f'rx={os.environ.get("RM_RADIO_RX_URI", "")}, '
            f'broadcast_tx={os.environ.get("RM_RADIO_BROADCAST_TX_URI", "")}, '
            f'interference_tx={os.environ.get("RM_RADIO_INTERFERENCE_TX_URI", "")}, '
            f'sample_rate={os.environ.get("RM_RADIO_SAMPLE_RATE", "")}, '
            f'side={os.environ.get("RM_RADIO_SIDE", "")}, '
            f'rx_profile={os.environ.get("RM_RADIO_RX_PROFILE", "")}, '
            f'frontend_profile={os.environ.get("RM_RADIO_FRONTEND_PROFILE", "")}, '
            f'demod_mode={os.environ.get("RM_RADIO_DEMOD_MODE", "")}, '
            f'input_sps={os.environ.get("RM_RADIO_INPUT_SPS", "")}, '
            f'interference_level={os.environ.get("RM_RADIO_INTERFERENCE_LEVEL", "")}'
        )

    def _import_flowgraph_module(self, flowgraph_dir: Path):
        module_name = str(self.get_parameter('flowgraph_module').value).strip() or 'RM'
        module_file = flowgraph_dir / f'{module_name}.py'
        if not module_file.exists():
            raise FileNotFoundError(f'flowgraph module not found: {module_file}')

        # The generated RM.py imports local helpers such as `jiang` by name.
        # Put the selected flowgraph directory first so each node uses its own copy.
        sys.path.insert(0, str(flowgraph_dir))
        unique_name = f'_rm_radio_ros_{self.get_parameter("flowgraph_name").value}_{module_name}'
        spec = importlib.util.spec_from_file_location(unique_name, str(module_file))
        if spec is None or spec.loader is None:
            raise ImportError(f'cannot load module spec from {module_file}')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def _parse_setters(self) -> Dict[str, Any]:
        raw = str(self.get_parameter('setters_json').value).strip() or '{}'
        try:
            parsed = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise ValueError(f'invalid setters_json: {exc}') from exc
        if not isinstance(parsed, dict):
            raise ValueError('setters_json must be a JSON object')
        return parsed

    def _apply_setters(self) -> None:
        if self.flowgraph is None:
            return
        setters = self._parse_setters()
        self._apply_setter_values(setters)

    def _apply_setter_values(self, setters: Dict[str, Any]) -> None:
        if self.flowgraph is None:
            if self._iio_capture_only_active():
                self._apply_iio_capture_only_setter_values(setters)
            elif setters:
                self.last_error = 'flowgraph is not loaded; cannot apply setters'
            return
        setters = dict(setters)
        requested_rx_profile = setters.pop('rx_profile', None)
        requested_interference_level = setters.pop('interference_level', None)
        # 部分 setter 会重置载荷：set_cmd_id/set_payload_size 会把 payload 复位成默认值
        # （如 0x0A06 的默认 "ABC123"）。因此必须保证携带真实载荷的 payload_data/
        # command_cycle 在这些结构性 setter 之后应用，否则热重配会丢掉真实密钥/数据。
        _setter_order = {'cmd_id': 0, 'com_id': 0, 'payload_size': 0, 'payload_data': 1, 'command_cycle': 1}

        def _setter_rank(item: tuple[str, Any]) -> int:
            name = item[0]
            clean = name[4:] if name.startswith('set_') else name
            return _setter_order.get(clean, 0)

        ordered_setters = sorted(setters.items(), key=_setter_rank)
        with self._setters_lock:
            settling_setter_applied = False
            for name, value in ordered_setters:
                method_name = name if name.startswith('set_') else f'set_{name}'
                method = getattr(self.flowgraph, method_name, None)
                if method is None:
                    self.get_logger().warning(f'skip unknown flowgraph setter: {method_name}')
                    continue
                method(value)
                clean_name = name[4:] if name.startswith('set_') else name
                settling_setter_applied = settling_setter_applied or clean_name in _SETTLING_SETTER_NAMES
                self.last_setters[name] = value
                self.get_logger().info(f'applied flowgraph setter {method_name}={value!r}')

            if settling_setter_applied:
                settle_sec = max(0.0, float(self.get_parameter('setters_apply_settle_sec').value))
                if settle_sec > 0.0:
                    time.sleep(settle_sec)

            if requested_rx_profile is not None:
                self._set_rx_profile(str(requested_rx_profile))
                self.last_setters['rx_profile'] = str(requested_rx_profile)

            if requested_interference_level is not None:
                self._set_interference_level(requested_interference_level)
                self.last_setters['interference_level'] = int(requested_interference_level)

            self.setters_apply_count += 1
            self.last_setters_apply_time = time.time()
            self.last_error = ''

    def _iio_attribute_timeout(self) -> float:
        try:
            return max(0.1, float(self.get_parameter('iio_attribute_timeout_sec').value))
        except Exception:
            return 3.0

    def _run_iio_capture_attribute(self, *args: str) -> str:
        import subprocess

        rx_uri = str(self.get_parameter('rx_uri').value).strip()
        if not rx_uri:
            raise ValueError('rx_uri is empty')
        timeout_sec = self._iio_attribute_timeout()
        command = [
            'iio_attr',
            '-T',
            str(max(1, int(timeout_sec * 1000.0))),
            '-u',
            rx_uri,
            *[str(value) for value in args],
        ]
        try:
            completed = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=timeout_sec + 0.5,
            )
        except subprocess.TimeoutExpired as exc:
            operation = ' '.join(str(value) for value in args)
            raise TimeoutError(
                f'iio_attr timed out after {timeout_sec:.2f}s for {rx_uri}: {operation}'
            ) from exc
        if completed.returncode != 0:
            detail = completed.stderr.strip() or completed.stdout.strip()
            operation = ' '.join(str(value) for value in args)
            raise RuntimeError(
                detail
                or f'iio_attr failed with code {completed.returncode} for {rx_uri}: {operation}'
            )
        return completed.stdout.strip()

    def _write_iio_capture_attribute(self, *args: str) -> None:
        self._run_iio_capture_attribute(*args)

    def _read_iio_capture_attribute(self, *args: str) -> str:
        value = self._run_iio_capture_attribute(*args)
        if not value:
            operation = ' '.join(str(item) for item in args)
            raise RuntimeError(f'iio_attr returned an empty value: {operation}')
        return value

    @staticmethod
    def _clean_iio_attribute_value(value: str) -> str:
        clean = str(value).strip()
        quoted = re.search(r"\bvalue\s*'([^']*)'", clean, flags=re.IGNORECASE)
        if quoted is not None:
            return quoted.group(1).strip()
        labelled = re.search(r'\bvalue\s*:\s*(.+?)\s*$', clean, flags=re.IGNORECASE)
        if labelled is not None:
            return labelled.group(1).strip()
        lines = [line.strip() for line in clean.splitlines() if line.strip()]
        return lines[-1] if lines else ''

    def _verify_iio_capture_attribute(
        self,
        args: tuple[str, ...],
        expected: Any,
        numeric_tolerance: Optional[float] = None,
    ) -> None:
        actual_raw = self._read_iio_capture_attribute(*args)
        actual = self._clean_iio_attribute_value(actual_raw)
        operation = ' '.join(args)
        if numeric_tolerance is None:
            if actual.strip().lower() != str(expected).strip().lower():
                raise RuntimeError(
                    f'IIO readback mismatch for {operation}: expected {expected!r}, got {actual!r}'
                )
            return

        matches = re.findall(
            r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?',
            actual,
        )
        if not matches:
            raise RuntimeError(
                f'IIO readback is not numeric for {operation}: expected {expected!r}, got {actual!r}'
            )
        actual_number = float(matches[-1])
        expected_number = float(expected)
        if abs(actual_number - expected_number) > float(numeric_tolerance):
            raise RuntimeError(
                f'IIO readback mismatch for {operation}: '
                f'expected {expected_number}, got {actual_number}'
            )

    def _capture_rf_port(self) -> str:
        try:
            value = str(self.get_parameter('iio_capture_rf_port').value).strip()
        except Exception:
            value = str(os.environ.get('RX_RF_PORT', 'B_BALANCED')).strip()
        return value or 'B_BALANCED'

    def _apply_iio_capture_only_setter_values(
        self,
        setters: Dict[str, Any],
        *,
        initial: bool = False,
    ) -> None:
        """Apply RX hot-retunes when the broadcast node intentionally has no flowgraph.

        Capture-only avoids two readers fighting over one E310, but it also means
        generated flowgraph setters cannot reach the hardware.  Keep panel/side
        hot-retunes functional by writing the small supported RX subset directly
        through IIO between captures; decoder-only values are still recorded.
        """
        setters = dict(setters)
        requested_rx_profile = setters.pop('rx_profile', None)
        requested_interference_level = setters.pop('interference_level', None)
        center_frequency = setters.get(
            'cen_f',
            setters.get('center_f', setters.get('center_F')),
        )
        rf_bandwidth = setters.get('bw_re', setters.get('BW'))
        gain_mode = setters.get('GainMode')
        gain = setters.get('Gain')
        if initial:
            missing = [
                name
                for name, value in (
                    ('center frequency (cen_f/center_f/center_F)', center_frequency),
                    ('RX bandwidth (bw_re/BW)', rf_bandwidth),
                    ('GainMode', gain_mode),
                )
                if value is None
            ]
            if missing:
                raise ValueError(
                    'capture-only initial setters are incomplete: missing ' + ', '.join(missing)
                )
            if str(gain_mode).strip().lower() == 'manual' and gain is None:
                raise ValueError('capture-only manual GainMode requires an initial Gain value')

        operations: list[tuple[tuple[str, ...], str, Any, Optional[float]]] = []

        def add_operation(
            args: tuple[str, ...],
            value: Any,
            *,
            numeric_tolerance: Optional[float] = None,
        ) -> None:
            operations.append((args, str(value), value, numeric_tolerance))

        if center_frequency is not None:
            value = int(float(center_frequency))
            add_operation(
                ('-c', 'ad9361-phy', 'altvoltage0', 'frequency'),
                value,
                numeric_tolerance=1.0,
            )

        sample_rate = None
        rf_port = None
        if initial:
            sample_rate = int(float(setters.get('sample_rate', self._current_sample_rate())))
            if sample_rate <= 0:
                raise ValueError(f'capture-only sample_rate must be positive, got {sample_rate}')
            rf_port = self._capture_rf_port()

        effective_gain_mode = str(
            gain_mode if gain_mode is not None else self.last_setters.get('GainMode', '')
        ).strip().lower()
        for channel in ('voltage0', 'voltage1'):
            prefix = ('-i', '-c', 'ad9361-phy', channel)
            if sample_rate is not None:
                add_operation(
                    (*prefix, 'sampling_frequency'),
                    sample_rate,
                    numeric_tolerance=1.0,
                )
            if rf_bandwidth is not None:
                value = int(float(rf_bandwidth))
                add_operation(
                    (*prefix, 'rf_bandwidth'),
                    value,
                    numeric_tolerance=1.0,
                )
            if gain_mode is not None:
                add_operation((*prefix, 'gain_control_mode'), str(gain_mode))
            if gain is not None and effective_gain_mode == 'manual':
                add_operation(
                    (*prefix, 'hardwaregain'),
                    float(gain),
                    numeric_tolerance=0.05,
                )
            if rf_port is not None:
                add_operation((*prefix, 'rf_port_select'), rf_port)

        hardware_changed = bool(operations)

        device_lock = getattr(self, '_iio_device_lock', self._setters_lock)
        with self._setters_lock, device_lock:
            for args, value, _expected, _tolerance in operations:
                self._write_iio_capture_attribute(*args, value)

            if hardware_changed:
                settle_sec = max(0.0, float(self.get_parameter('setters_apply_settle_sec').value))
                if settle_sec > 0.0:
                    time.sleep(settle_sec)

            for args, _value, expected, tolerance in operations:
                self._verify_iio_capture_attribute(args, expected, tolerance)

            self.last_setters.update(setters)
            if requested_rx_profile is not None:
                self._set_rx_profile(str(requested_rx_profile))
                self.last_setters['rx_profile'] = str(requested_rx_profile)
            if requested_interference_level is not None:
                self._set_interference_level(requested_interference_level)
                self.last_setters['interference_level'] = int(requested_interference_level)
            if hardware_changed and not initial:
                self._reset_decode_state()
            self.setters_apply_count += 1
            self.last_setters_apply_time = time.time()
            self.last_error = ''
            mode = 'initial' if initial else 'hot'
            self.get_logger().info(
                f'applied and verified capture-only IIO {mode} setters: {setters!r}'
            )

    def load_flowgraph(self) -> bool:
        if self.loaded:
            return True
        try:
            if bool(self.get_parameter('dry_run').value):
                flowgraph_dir = self._resolve_flowgraph_dir()
                if not flowgraph_dir.exists():
                    raise FileNotFoundError(f'flowgraph_dir not found: {flowgraph_dir}')
                module = self._import_flowgraph_module(flowgraph_dir)
                class_name = str(self.get_parameter('flowgraph_class').value).strip() or 'RM'
                getattr(module, class_name)
                self.loaded = True
                self.last_error = ''
                self.get_logger().info(
                    f'dry_run enabled; imported flowgraph {class_name} from {flowgraph_dir}'
                )
                return True

            flowgraph_dir = self._resolve_flowgraph_dir()
            if not flowgraph_dir.exists():
                raise FileNotFoundError(f'flowgraph_dir not found: {flowgraph_dir}')

            self._configure_flowgraph_environment()
            if self._iio_capture_only_active():
                setters = self._parse_setters()
                self._apply_iio_capture_only_setter_values(setters, initial=True)
                self.loaded = True
                self.last_error = ''
                self.get_logger().info(
                    'IIO capture-only decoder active; verified hardware and skipped '
                    'GNU Radio flowgraph construction'
                )
                return True
            self._ensure_qt_application()
            module = self._import_flowgraph_module(flowgraph_dir)
            class_name = str(self.get_parameter('flowgraph_class').value).strip() or 'RM'
            flowgraph_cls = getattr(module, class_name)
            self.flowgraph = flowgraph_cls()
            self._attach_flowgraph_decoders()
            self._apply_setters()
            self.loaded = True
            self.last_error = ''
            self.get_logger().info(f'loaded GNU Radio flowgraph {class_name} from {flowgraph_dir}')
            return True
        except Exception as exc:  # noqa: BLE001 - report into ROS status
            # If flowgraph construction partially succeeded before an exception
            # (for example bad setters_json), release it to avoid IIO buffer
            # conflicts on the next load attempt in this same process.
            try:
                if self.flowgraph is not None:
                    stop = getattr(self.flowgraph, 'stop', None)
                    wait = getattr(self.flowgraph, 'wait', None)
                    if stop is not None:
                        stop()
                    if wait is not None:
                        wait()
            except Exception:
                pass
            self.flowgraph = None
            self.loaded = False
            self.started = False
            self.last_error = ''.join(traceback.format_exception_only(type(exc), exc)).strip()
            self.get_logger().error(f'failed to load flowgraph: {self.last_error}')
            self.get_logger().debug(traceback.format_exc())
            return False

    def _retry_auto_start(self) -> None:
        if self.started:
            timer = self._auto_start_retry_timer
            if timer is not None:
                timer.cancel()
                self._auto_start_retry_timer = None
            return
        if not self.start_flowgraph():
            return
        timer = self._auto_start_retry_timer
        if timer is not None:
            timer.cancel()
            self._auto_start_retry_timer = None
        self.get_logger().warning('flowgraph recovered after device became available')

    def _attach_flowgraph_decoders(self) -> None:
        if self.flowgraph is None:
            return
        setter = getattr(self.flowgraph, 'set_demod_bits_callback', None)
        if setter is not None:
            setter(self._handle_demod_bit_bytes)
            self.get_logger().info('attached demod bit callback decoder')
        iq_setter = getattr(self.flowgraph, 'set_iq_callback', None)
        if iq_setter is not None:
            iq_setter(self._handle_iq_samples)
            self.get_logger().info('attached raw IQ diagnostics callback')
        filtered_iq_setter = getattr(self.flowgraph, 'set_filtered_iq_callback', None)
        if filtered_iq_setter is not None:
            filtered_iq_setter(self._handle_filtered_iq_samples)
            self.get_logger().info('attached filtered IQ diagnostics callback')

    def start_flowgraph(self) -> bool:
        if bool(self.get_parameter('dry_run').value):
            self.started = True
            self._flowgraph_runtime_started = False
            self.get_logger().info('dry_run enabled; marked flowgraph as started')
            return True
        if not self.loaded and not self.load_flowgraph():
            return False
        if self.started:
            return True
        try:
            if self._iio_capture_only_active():
                self._flowgraph_runtime_started = False
            else:
                self.flowgraph.start()
                self._flowgraph_runtime_started = True
                self._set_flowgraph_window_visible(True)
            self.started = True
            self._start_iio_capture_decoder_if_needed()
            self.last_error = ''
            if self._flowgraph_runtime_started:
                self.get_logger().info('GNU Radio flowgraph started')
            else:
                self.get_logger().info('IIO capture-only decoder started without GNU Radio streaming')
            return True
        except Exception as exc:  # noqa: BLE001
            self.last_error = ''.join(traceback.format_exception_only(type(exc), exc)).strip()
            self.get_logger().error(f'failed to start flowgraph: {self.last_error}')
            return False

    def stop_flowgraph(self) -> bool:
        if bool(self.get_parameter('dry_run').value):
            self.started = False
            self._flowgraph_runtime_started = False
            return True
        if self.flowgraph is None or not self.started:
            if not self._stop_iio_capture_decoder():
                return False
            self.started = False
            self._flowgraph_runtime_started = False
            return True
        try:
            if not self._stop_iio_capture_decoder():
                return False
            if self._flowgraph_runtime_started:
                self.flowgraph.stop()
                self.flowgraph.wait()
                self._set_flowgraph_window_visible(False)
            self.started = False
            self._flowgraph_runtime_started = False
            self.last_error = ''
            self.get_logger().info('GNU Radio flowgraph stopped')
            return True
        except Exception as exc:  # noqa: BLE001
            self.last_error = ''.join(traceback.format_exception_only(type(exc), exc)).strip()
            self.get_logger().error(f'failed to stop flowgraph: {self.last_error}')
            return False

    def reload_flowgraph(self) -> bool:
        was_started = self.started
        if not self.stop_flowgraph():
            return False
        self.flowgraph = None
        self.loaded = False
        self._reset_decode_state()
        if not self.load_flowgraph():
            return False
        if was_started:
            return self.start_flowgraph()
        return True

    def _status_payload(self) -> str:
        payload = {
            'node': self.get_name(),
            'flowgraph_name': str(self.get_parameter('flowgraph_name').value),
            'flowgraph_dir': str(self._resolve_flowgraph_dir()),
            'loaded': self.loaded,
            'started': self.started,
            'dry_run': bool(self.get_parameter('dry_run').value),
            'rx_only': bool(self.get_parameter('rx_only').value),
            'radio_side': str(self.get_parameter('radio_side').value),
            'rx_profile': str(self.get_parameter('rx_profile').value),
            'frontend_profile': (
                getattr(self.flowgraph, 'get_frontend_profile', lambda: None)()
                if self.flowgraph is not None
                else str(self.get_parameter('frontend_profile').value)
            ),
            'demod_mode': (
                getattr(self.flowgraph, 'get_demod_mode', lambda: None)()
                if self.flowgraph is not None
                else str(self.get_parameter('demod_mode').value)
            ),
            'demod_configuration': (
                getattr(self.flowgraph, 'get_demod_configuration', lambda: {})()
                if self.flowgraph is not None
                else {}
            ),
            'interference_level': int(self.get_parameter('interference_level').value),
            'last_error': self.last_error,
            'decoder': {
                'max_bits': int(self.get_parameter('air_extractor_max_bits').value),
                'max_access_hamming': int(self.get_parameter('air_extractor_max_access_hamming').value),
                'max_length_hamming': int(self.get_parameter('air_extractor_max_length_hamming').value),
                'allow_inverted': bool(self.get_parameter('air_extractor_allow_inverted').value),
                'iio_capture_decoder_enabled': bool(self.get_parameter('iio_capture_decoder_enabled').value),
                'iio_capture_decoder_only': bool(self.get_parameter('iio_capture_decoder_only').value),
                'stats': dict(self.decode_stats),
                'diagnostics': dict(self.demod_bit_diagnostics),
                'extractor_diagnostics': self._extractor_diagnostics(),
                'iq_diagnostics': dict(self.iq_diagnostics),
                'filtered_iq_diagnostics': dict(self.filtered_iq_diagnostics),
                'iq_decode_diagnostics': dict(self.iq_decode_diagnostics),
            },
            'frame_validation': {
                'primary': self.frame_assembler.diagnostics(),
                'iq': self.iq_frame_assembler.diagnostics(),
            },
            'reception_pipeline': self.reception_metrics.snapshot(),
            'last_setters': dict(self.last_setters),
            'setters_apply': {
                'count': int(self.setters_apply_count),
                'timestamp': float(self.last_setters_apply_time),
            },
            'tx_stream': (
                getattr(self.flowgraph, 'get_tx_stream_diagnostics', lambda: {})()
                if self.flowgraph is not None
                else {}
            ),
            'rx_gain': (
                getattr(self.flowgraph, 'get_rx_gain_diagnostics', lambda: {})()
                if self.flowgraph is not None
                else {}
            ),
            'recording': (
                self._iq_recorder.snapshot()
                if getattr(self, '_iq_recorder', None) is not None
                else {
                    'enabled': False,
                    'active': False,
                    'last_error': getattr(self, '_recording_init_error', ''),
                }
            ),
        }
        if getattr(self, '_recording_init_error', ''):
            payload['recording']['last_error'] = self._recording_init_error
        return json.dumps(payload, ensure_ascii=False)

    def _publish_status(self) -> None:
        msg = String()
        msg.data = self._status_payload()
        self.status_pub.publish(msg)

    def _publish_frames(self, frames) -> None:
        for frame in frames:
            out = String()
            out.data = json.dumps(frame.to_dict(), ensure_ascii=False)
            self.frames_pub.publish(out)
        bridge_outputs = self.referee_bridge.process_frames(frames)
        self.decode_stats['frames'] += len(frames)
        self.decode_stats['bridge_outputs'] += len(bridge_outputs)
        for output in bridge_outputs:
            out = String()
            out.data = json.dumps(output, ensure_ascii=False)
            self.referee_bridge_pub.publish(out)

    @staticmethod
    def _decode_hex_message(msg: String) -> bytes:
        raw = ''.join(str(msg.data).split())
        return bytes.fromhex(raw)

    def _handle_air_payload_hex(self, msg: String) -> None:
        try:
            payload = self._decode_hex_message(msg)
            if len(payload) % 15 != 0:
                raise ValueError(f'hex payload must contain whole 15-byte air payloads, got {len(payload)} bytes')
            frames = []
            for offset in range(0, len(payload), 15):
                frames.extend(self.frame_assembler.push_air_payload(payload[offset : offset + 15]))
                self._record_primary_validation_events()
            self.decode_stats['manual_air_payloads'] += len(payload) // 15
        except Exception as exc:  # noqa: BLE001 - publish parse errors for operators
            self.last_error = f'air payload parse failed: {exc}'
            self.get_logger().warning(self.last_error)
            return

        self._publish_frames(frames)

    def _handle_setters_json(self, msg: String) -> None:
        try:
            raw = str(msg.data).strip() or '{}'
            setters = json.loads(raw)
            if not isinstance(setters, dict):
                raise ValueError('setters_json message must be a JSON object')
            self._apply_setter_values(setters)
        except Exception as exc:  # noqa: BLE001 - publish parse/apply errors for operators
            self.last_error = f'setters_json apply failed: {exc}'
            self.get_logger().warning(self.last_error)

    def _handle_recording_control(self, msg: String) -> None:
        recorder = getattr(self, '_iq_recorder', None)
        if recorder is None or not recorder.enabled:
            return
        try:
            payload = json.loads(str(msg.data).strip() or '{}')
            if not isinstance(payload, dict):
                raise ValueError('recording control must be a JSON object')
            action = str(payload.get('action') or '').strip().lower()
            session_id = str(payload.get('session_id') or '').strip()
            if action == 'start':
                metadata = {
                    'node': self.get_name(),
                    'rx_profile': str(self.get_parameter('rx_profile').value),
                    'frontend_profile': str(self.get_parameter('frontend_profile').value),
                    'radio_side': str(self.get_parameter('radio_side').value),
                    'interference_level': int(self.get_parameter('interference_level').value),
                    'rx_uri': str(self.get_parameter('rx_uri').value),
                    'last_setters': dict(self.last_setters),
                    'control_timestamp': payload.get('timestamp'),
                    'control_reason': payload.get('reason'),
                }
                recorder.start(session_id, metadata)
                self.get_logger().warning(
                    f'IQ match recording started: role={recorder.role} session={session_id}'
                )
            elif action == 'stop':
                current_session = str(recorder.snapshot().get('session_id') or '')
                if not session_id or not current_session or session_id == current_session:
                    recorder.stop(str(payload.get('reason') or 'control_stop'))
                    self.get_logger().warning(
                        f'IQ match recording stopped: role={recorder.role} '
                        f'session={current_session}'
                    )
            else:
                raise ValueError(f'unknown recording action: {action!r}')
        except Exception as exc:  # noqa: BLE001 - recording cannot disrupt RX
            self._recording_init_error = f'IQ recording control failed: {exc}'
            self.get_logger().error(self._recording_init_error)

    def _handle_demod_bits_hex(self, msg: String) -> None:
        try:
            bit_bytes = self._decode_hex_message(msg)
            frames = self._decode_demod_bit_bytes(bit_bytes)
        except Exception as exc:  # noqa: BLE001 - publish parse errors for operators
            self.last_error = f'demod bit parse failed: {exc}'
            self.get_logger().warning(self.last_error)
            return

        self._publish_frames(frames)

    def _decode_demod_bit_bytes(self, bit_bytes: bytes):
        self.decode_stats['demod_bit_batches'] += 1
        if self._should_update_demod_bit_diagnostics():
            self._update_demod_bit_diagnostics(bit_bytes)
        payloads = self.air_packet_extractor.push_bit_bytes(bit_bytes)
        self.decode_stats['air_payloads'] += len(payloads)
        frames = []
        for payload in payloads:
            frames.extend(self.frame_assembler.push_air_payload(payload))
            self._record_primary_validation_events()
        return frames

    def _record_primary_validation_events(self) -> None:
        self.reception_metrics.record_many(self.frame_assembler.pop_diagnostic_events())

    def _should_update_demod_bit_diagnostics(self) -> bool:
        try:
            period = int(self.get_parameter('demod_bit_diagnostics_every_n_batches').value)
        except Exception:
            period = 20
        if period <= 0:
            return False
        batches = int(self.decode_stats.get('demod_bit_batches', 0))
        return batches == 1 or batches % period == 0

    def _update_demod_bit_diagnostics(self, bit_bytes: bytes) -> None:
        bits = [1 if int(value) else 0 for value in bit_bytes]
        result: Dict[str, Any] = {
            'sample_bits': len(bits),
            'ones': sum(bits),
            'zeros': len(bits) - sum(bits),
        }
        for name, access in ACCESS_CODES.items():
            pattern = bytes_to_bits(access)
            normal = self._best_hamming(bits, pattern)
            inverted = self._best_hamming(bits, [1 - bit for bit in pattern])
            result[name] = {
                'best_hamming': normal[0],
                'best_index': normal[1],
                'best_hamming_inverted': inverted[0],
                'best_index_inverted': inverted[1],
            }
        self.demod_bit_diagnostics = result

    def _extractor_diagnostics(self) -> Dict[str, Any]:
        bits = list(getattr(self.air_packet_extractor, '_bits', []))
        result: Dict[str, Any] = {
            'buffer_bits': len(bits),
            'ones': sum(bits),
            'zeros': len(bits) - sum(bits),
        }
        for name, access in ACCESS_CODES.items():
            pattern = bytes_to_bits(access)
            normal = self._best_hamming(bits, pattern)
            inverted = self._best_hamming(bits, [1 - bit for bit in pattern])
            result[name] = {
                'best_hamming': normal[0],
                'best_index': normal[1],
                'best_hamming_inverted': inverted[0],
                'best_index_inverted': inverted[1],
            }
        return result

    @staticmethod
    def _best_hamming(bits, pattern) -> tuple[Optional[int], Optional[int]]:
        if not pattern or len(bits) < len(pattern):
            return None, None
        best_distance = None
        best_index = None
        last_start = len(bits) - len(pattern)
        for start in range(last_start + 1):
            distance = sum(1 for bit, expected in zip(bits[start : start + len(pattern)], pattern) if int(bit) != int(expected))
            if best_distance is None or distance < best_distance:
                best_distance = distance
                best_index = start
        return best_distance, best_index

    def _handle_demod_bit_bytes(self, bit_bytes: bytes) -> None:
        try:
            frames = self._decode_demod_bit_bytes(bit_bytes)
        except Exception as exc:  # noqa: BLE001 - GNU Radio callback must not raise
            self.last_error = f'flowgraph demod bit decode failed: {exc}'
            self.get_logger().warning(self.last_error)
            return
        self._publish_frames(frames)

    @staticmethod
    def _iq_diagnostics_payload(data, source: str) -> Dict[str, Any]:
        import numpy as np

        samples = np.asarray(data, dtype=np.complex64)
        if samples.size == 0:
            return {}
        i_values = np.real(samples)
        q_values = np.imag(samples)
        mag = np.abs(samples)
        rms_abs = float(np.sqrt(np.mean(np.square(mag))))
        max_abs = float(np.max(mag))
        component_peak = float(max(np.max(np.abs(i_values)), np.max(np.abs(q_values))))
        near_clip = (np.abs(i_values) >= 0.95) | (np.abs(q_values) >= 0.95)
        clipped = (np.abs(i_values) >= 0.999) | (np.abs(q_values) >= 0.999)
        near_clip_fraction = float(np.mean(near_clip))
        clip_fraction = float(np.mean(clipped))
        headroom_db = float(-20.0 * np.log10(max(component_peak, 1e-12)))
        if clip_fraction > 0.0:
            signal_state = 'clipped'
        elif near_clip_fraction > 0.0 or headroom_db < 3.0:
            signal_state = 'warning'
        else:
            signal_state = 'ok'
        result: Dict[str, Any] = {
            'sample_count': int(samples.size),
            'mean_abs': float(np.mean(mag)),
            'max_abs': max_abs,
            'rms_abs': rms_abs,
            'rms_dbfs': float(20.0 * np.log10(max(rms_abs, 1e-12))),
            'component_peak': component_peak,
            'headroom_db': headroom_db,
            'crest_factor_db': float(20.0 * np.log10(max(max_abs / max(rms_abs, 1e-12), 1e-12))),
            'near_clip_fraction': near_clip_fraction,
            'clip_fraction': clip_fraction,
            'signal_state': signal_state,
            'mean_i': float(np.mean(i_values)),
            'mean_q': float(np.mean(q_values)),
            'source': source,
        }
        if samples.size > 1:
            phase_step = np.angle(samples[1:] * np.conj(samples[:-1]))
            result.update({
                'mean_phase_step': float(np.mean(phase_step)),
                'std_phase_step': float(np.std(phase_step)),
                'min_phase_step': float(np.min(phase_step)),
                'max_phase_step': float(np.max(phase_step)),
            })
        return result

    def _handle_iq_samples(self, samples) -> None:
        try:
            import numpy as np

            data = np.asarray(samples, dtype=np.complex64)
            if data.size == 0:
                return
            self.iq_diagnostics = self._iq_diagnostics_payload(data, 'flowgraph_raw')
            self._maybe_publish_spectrum(data, tap='raw')
            self._buffer_iq_samples(data.copy())
        except Exception as exc:  # noqa: BLE001 - GNU Radio callback must not raise
            self.last_error = f'flowgraph raw IQ diagnostics failed: {exc}'

    def _handle_filtered_iq_samples(self, samples) -> None:
        try:
            import numpy as np

            data = np.asarray(samples, dtype=np.complex64)
            if data.size == 0:
                return
            recorder = getattr(self, '_iq_recorder', None)
            if recorder is not None:
                recorder.enqueue(data)
            self.filtered_iq_diagnostics = self._iq_diagnostics_payload(
                data,
                'flowgraph_filtered',
            )
            self._maybe_publish_spectrum(data, tap='filtered')
        except Exception as exc:  # noqa: BLE001 - GNU Radio callback must not raise
            self.last_error = f'flowgraph filtered IQ diagnostics failed: {exc}'

    def _start_iio_capture_decoder_if_needed(self) -> None:
        if not bool(self.get_parameter('iio_capture_decoder_enabled').value):
            return
        if self._current_rx_profile() != 'broadcast':
            return
        thread_lock = getattr(self, '_iio_capture_thread_lock', self._setters_lock)
        with thread_lock:
            if getattr(self, '_iio_capture_stopping', False):
                raise RuntimeError('IIO capture thread is still stopping')
            existing = self._iio_capture_thread
            if existing is not None and existing.is_alive():
                return
            self._iio_capture_generation += 1
            generation = self._iio_capture_generation
            stop_event = threading.Event()
            thread = threading.Thread(
                target=self._run_iio_capture_decoder_loop,
                args=(stop_event, generation),
                name=f'iio-capture-{generation}',
                daemon=True,
            )
            self._iio_capture_stop_event = stop_event
            self._iio_capture_thread = thread
            try:
                thread.start()
            except Exception:
                self._iio_capture_stop_event = None
                self._iio_capture_thread = None
                raise
        self.get_logger().info('started IIO capture IQ decoder')

    def _iio_capture_only_active(self) -> bool:
        return (
            bool(self.get_parameter('iio_capture_decoder_enabled').value)
            and bool(self.get_parameter('iio_capture_decoder_only').value)
            and self._current_rx_profile() == 'broadcast'
        )

    def _iio_capture_stop_timeout(self) -> float:
        try:
            return max(0.1, float(self.get_parameter('iio_capture_stop_timeout_sec').value))
        except Exception:
            return 12.0

    def _stop_iio_capture_decoder(self) -> bool:
        thread_lock = getattr(self, '_iio_capture_thread_lock', self._setters_lock)
        with thread_lock:
            thread = self._iio_capture_thread
            stop_event = self._iio_capture_stop_event
            if thread is None:
                self._iio_capture_stopping = False
                return True
            self._iio_capture_stopping = True
            if stop_event is not None:
                stop_event.set()
        if thread is threading.current_thread():
            self.last_error = 'IIO capture thread cannot synchronously stop itself'
            self.get_logger().error(self.last_error)
            return False
        if thread.is_alive():
            thread.join(timeout=self._iio_capture_stop_timeout())
        if thread.is_alive():
            self.last_error = (
                'timed out waiting for IIO capture thread to stop; '
                'reload/start is blocked to prevent concurrent iio_readdev readers'
            )
            self.get_logger().error(self.last_error)
            return False
        with thread_lock:
            if self._iio_capture_thread is thread:
                self._iio_capture_thread = None
                self._iio_capture_stop_event = None
            self._iio_capture_stopping = False
        return True

    def _run_iio_capture_decoder_loop(
        self,
        stop_event: threading.Event,
        generation: int,
    ) -> None:
        decode_condition = threading.Condition()
        decode_state: dict[str, Any] = {"pending": None}
        decode_stop = threading.Event()

        def decode_worker() -> None:
            while True:
                with decode_condition:
                    decode_condition.wait_for(
                        lambda: decode_state["pending"] is not None or decode_stop.is_set()
                    )
                    if decode_stop.is_set() and decode_state["pending"] is None:
                        return
                    task = decode_state["pending"]
                    decode_state["pending"] = None
                if task is None:
                    continue
                samples, start_sample, sample_rate = task
                try:
                    self._decode_iq_snapshot(samples, start_sample, sample_rate)
                    self.decode_stats['iio_capture_batches'] += 1
                except Exception as exc:  # noqa: BLE001 - latest-only decode worker
                    self.decode_stats['iio_capture_errors'] += 1
                    self.last_error = f'IIO capture IQ decode failed: {exc}'
                    self.iq_decode_diagnostics = {'last_error': self.last_error}

        decoder = threading.Thread(
            target=decode_worker,
            name=f'iio-capture-decode-{generation}',
            daemon=True,
        )
        decoder.start()
        try:
            start_delay = max(
                0.0,
                float(self.get_parameter('iio_capture_start_delay_sec').value),
            )
            if start_delay > 0.0 and stop_event.wait(start_delay):
                return
            while not stop_event.is_set():
                start_time = time.time()
                try:
                    device_lock = getattr(self, '_iio_device_lock', self._iq_buffer_lock)
                    # Keep a hot retune from landing between capture and decode;
                    # otherwise one batch from the previous side/frequency could be
                    # published after the operator already switched profiles.
                    with device_lock:
                        samples = self._capture_iio_samples(
                            stop_event=stop_event,
                            spectrum_callback=self._maybe_publish_spectrum,
                        )
                        if stop_event.is_set():
                            return
                        if samples is not None and int(samples.size) >= 1024:
                            start_sample = self._iio_capture_total_samples_seen
                            self._iio_capture_total_samples_seen += int(samples.size)
                            self._update_iq_diagnostics(samples)
                            # Decoding a 0.6 s snapshot can be CPU-heavy. Queue
                            # only the latest complete capture so acquisition,
                            # FFT publication and the waterfall never pause.
                            with decode_condition:
                                decode_state["pending"] = (
                                    samples,
                                    start_sample,
                                    self._current_sample_rate(),
                                )
                                decode_condition.notify()
                except Exception as exc:  # noqa: BLE001 - capture decoder is best-effort.
                    if not stop_event.is_set():
                        self.decode_stats['iio_capture_errors'] += 1
                        self.last_error = f'IIO capture IQ decode failed: {exc}'
                        self.iq_decode_diagnostics = {'last_error': self.last_error}
                period = max(0.1, float(self.get_parameter('iio_capture_period_sec').value))
                elapsed = time.time() - start_time
                stop_event.wait(max(0.0, period - elapsed))
        finally:
            decode_stop.set()
            with decode_condition:
                # Shutdown favors releasing the SDR promptly over decoding an
                # obsolete queued batch. A currently running decode may finish.
                decode_state["pending"] = None
                decode_condition.notify_all()
            if decoder.is_alive():
                decoder.join(timeout=self._iio_capture_stop_timeout())
            thread_lock = getattr(self, '_iio_capture_thread_lock', self._setters_lock)
            current = threading.current_thread()
            with thread_lock:
                if (
                    self._iio_capture_thread is current
                    and self._iio_capture_generation == generation
                ):
                    self._iio_capture_thread = None
                    self._iio_capture_stop_event = None

    def _capture_iio_samples(
        self,
        *,
        stop_event: Optional[threading.Event] = None,
        spectrum_callback=None,
    ):
        import subprocess
        import numpy as np

        rx_uri = str(self.get_parameter('rx_uri').value).strip()
        if not rx_uri:
            raise ValueError('rx_uri is empty')
        sample_rate = self._current_sample_rate()
        capture_seconds = max(0.1, float(self.get_parameter('iio_capture_seconds').value))
        sample_count = max(1024, int(capture_seconds * sample_rate))
        timeout_sec = max(1.0, float(self.get_parameter('iio_capture_timeout_sec').value))
        buffer_size = max(1024, int(self.get_parameter('iio_capture_buffer_size').value))
        cmd = [
            'timeout',
            f'{timeout_sec:.3f}s',
            'iio_readdev',
            '-u',
            rx_uri,
            '-b',
            str(buffer_size),
            '-s',
            str(sample_count),
            'cf-ad9361-lpc',
            'voltage0',
            'voltage1',
        ]
        device_lock = getattr(self, '_iio_device_lock', self._iq_buffer_lock)
        with device_lock:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=True,
            )
            raw_bytes = bytearray()
            carry = b''
            read_size = max(16_384, min(buffer_size * 4, 262_144))
            assert proc.stdout is not None
            while True:
                chunk = proc.stdout.read(read_size)
                if not chunk:
                    break
                raw_bytes.extend(chunk)
                if spectrum_callback is not None:
                    block = carry + chunk
                    usable = len(block) - (len(block) % 4)
                    carry = block[usable:]
                    if usable >= 4096:
                        raw_chunk = np.frombuffer(block[:usable], dtype='<i2')
                        iq_chunk = raw_chunk.reshape(-1, 2).astype(np.float32) / 32768.0
                        spectrum_callback(
                            (iq_chunk[:, 0] + 1j * iq_chunk[:, 1]).astype(np.complex64)
                        )
                if stop_event is not None and stop_event.is_set():
                    try:
                        os.killpg(proc.pid, signal.SIGTERM)
                    except (ProcessLookupError, PermissionError):
                        pass
                    break
            try:
                returncode = proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass
                returncode = proc.wait(timeout=1.0)
            stderr = proc.stderr.read().decode('utf-8', errors='replace').strip() if proc.stderr is not None else ''
        raw = np.frombuffer(raw_bytes, dtype='<i2')
        if raw.size % 2:
            raw = raw[:-1]
        if raw.size < 2048:
            raise RuntimeError(stderr or f'iio_readdev returned {returncode}')
        iq = raw.reshape(-1, 2).astype(np.float32) / 32768.0
        return (iq[:, 0] + 1j * iq[:, 1]).astype(np.complex64)

    def _update_iq_diagnostics(self, data) -> None:
        self.iq_diagnostics = self._iq_diagnostics_payload(data, 'iio_readdev')
        self._maybe_publish_spectrum(data, tap='raw')

    def _maybe_publish_spectrum(self, data, tap: str = 'raw') -> None:
        try:
            if not bool(self.get_parameter('spectrum_enabled').value):
                return
            now = time.time()
            period = max(0.025, float(self.get_parameter('spectrum_period_sec').value))
            filtered = str(tap).strip().lower() == 'filtered'
            last_publish = (
                self._filtered_spectrum_last_publish_time
                if filtered
                else self._spectrum_last_publish_time
            )
            if now - last_publish < period:
                return
            payload = self._spectrum_payload(data, now, tap='filtered' if filtered else 'raw')
            if payload is None:
                return
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            publisher = self.filtered_spectrum_pub if filtered else self.spectrum_pub
            publisher.publish(msg)
            if filtered:
                self._filtered_spectrum_last_publish_time = now
            else:
                self._spectrum_last_publish_time = now
        except Exception as exc:  # noqa: BLE001 - spectrum is diagnostic only
            self.last_error = f'spectrum publish failed: {exc}'

    def _spectrum_payload(self, data, timestamp: float, tap: str = 'raw') -> Optional[dict]:
        import numpy as np

        samples = np.asarray(data, dtype=np.complex64)
        if samples.size < 8:
            return None
        fft_size = max(8, int(self.get_parameter('spectrum_fft_size').value))
        fft_size = min(fft_size, int(samples.size))
        bin_count = max(8, int(self.get_parameter('spectrum_bin_count').value))
        segment = samples[-fft_size:]
        window = np.hanning(fft_size).astype(np.float32) if fft_size > 1 else np.ones(fft_size, dtype=np.float32)
        spectrum = np.fft.fftshift(np.fft.fft(segment * window))
        scale = max(float(np.sum(window)), 1e-12)
        power_dbfs = 20.0 * np.log10((np.abs(spectrum) / scale) + 1e-12)
        power_dbfs = np.nan_to_num(power_dbfs, nan=-240.0, neginf=-240.0, posinf=0.0)
        sample_rate = float(self._current_sample_rate())
        offsets = np.fft.fftshift(np.fft.fftfreq(fft_size, d=1.0 / sample_rate))
        peak_index = int(np.argmax(power_dbfs))

        if fft_size > bin_count:
            edges = np.linspace(0, fft_size, bin_count + 1, dtype=int)
            out_offsets = []
            out_power = []
            for start, end in zip(edges[:-1], edges[1:]):
                if end <= start:
                    continue
                out_offsets.append(float(np.mean(offsets[start:end])))
                out_power.append(float(np.mean(power_dbfs[start:end])))
        else:
            out_offsets = [float(value) for value in offsets]
            out_power = [float(value) for value in power_dbfs]

        center_frequency_hz = self._current_center_frequency()
        peak_offset_hz = float(offsets[peak_index])
        payload = {
            'node': self.get_name(),
            'timestamp': float(timestamp),
            'tap': str(tap),
            'rx_profile': self._current_rx_profile(),
            'sample_rate_hz': sample_rate,
            'center_frequency_hz': center_frequency_hz,
            'fft_size': int(fft_size),
            'bin_count': len(out_offsets),
            'offset_hz': out_offsets,
            'power_dbfs': out_power,
            'peak_offset_hz': peak_offset_hz,
            'peak_frequency_hz': None if center_frequency_hz is None else float(center_frequency_hz) + peak_offset_hz,
            'peak_dbfs': float(power_dbfs[peak_index]),
            'noise_floor_dbfs': float(np.median(power_dbfs)),
        }
        return payload

    def _buffer_iq_samples(self, data) -> None:
        if not bool(self.get_parameter('iq_decoder_enabled').value):
            return
        if bool(self.get_parameter('iio_capture_decoder_only').value):
            return
        sample_rate = self._current_sample_rate()
        window_sec = max(0.05, float(self.get_parameter('iq_decoder_window_sec').value))
        window_samples = max(1024, int(sample_rate * window_sec))
        now = time.time()
        snapshot = None
        snapshot_start_sample = 0
        with self._iq_buffer_lock:
            start_sample = self._iq_total_samples_seen
            self._iq_total_samples_seen += int(data.size)
            self._iq_buffer_chunks.append((start_sample, data))
            self._iq_buffer_sample_count += int(data.size)
            while (
                len(self._iq_buffer_chunks) > 1
                and self._iq_buffer_sample_count - int(self._iq_buffer_chunks[0][1].size) >= window_samples
            ):
                _old_start, old_data = self._iq_buffer_chunks.pop(0)
                self._iq_buffer_sample_count -= int(old_data.size)

            period = max(0.05, float(self.get_parameter('iq_decoder_period_sec').value))
            thread_alive = self._iq_decode_thread is not None and self._iq_decode_thread.is_alive()
            if thread_alive or now - self._iq_decode_last_start_time < period:
                return
            if self._iq_buffer_sample_count < min(window_samples, max(8192, int(data.size))):
                return
            snapshot, snapshot_start_sample = self._copy_iq_window_locked(window_samples)
            self._iq_decode_last_start_time = now

        if snapshot is None or int(snapshot.size) < 1024:
            return
        worker = threading.Thread(
            target=self._decode_iq_snapshot,
            args=(snapshot, snapshot_start_sample, sample_rate),
            daemon=True,
        )
        with self._iq_buffer_lock:
            self._iq_decode_thread = worker
        worker.start()

    def _copy_iq_window_locked(self, window_samples: int):
        import numpy as np

        remaining = int(window_samples)
        pieces = []
        for _start, chunk in reversed(self._iq_buffer_chunks):
            if remaining <= 0:
                break
            take = min(int(chunk.size), remaining)
            pieces.append(chunk[-take:] if take < int(chunk.size) else chunk)
            remaining -= take
        if not pieces:
            return None, 0
        pieces.reverse()
        snapshot = np.concatenate(pieces).astype(np.complex64, copy=False)
        snapshot_start_sample = max(0, self._iq_total_samples_seen - int(snapshot.size))
        return snapshot, snapshot_start_sample

    def _decode_iq_snapshot(self, samples, start_sample: int, sample_rate: float) -> None:
        try:
            from ..core.offline_iq_scan import scan_iq_samples
            from ..core.rx_autotune import autotune_iq_samples

            threshold = int(self.get_parameter('iq_decoder_decode_threshold').value)
            profile = self._current_rx_profile()
            level = self._current_interference_level()
            frequency_shift_hz = float(self.get_parameter('iq_decoder_frequency_shift_hz').value)
            low_pass_hz = self._iq_decoder_low_pass_hz(profile, level)
            autotune_mode = self._iq_decoder_autotune_mode()
            lock_enabled = bool(self.get_parameter('iq_decoder_autotune_lock_enabled').value)
            locked = self._iq_locked_candidate if lock_enabled else None
            used_lock = False
            # F5 time-diversity voting only for the interference wave, whose password frame is
            # retransmitted unchanged (constant content); never for cycling info-wave commands.
            vote_frames = (
                bool(self.get_parameter('iq_decoder_vote_repeated_frames').value)
                and profile == 'interference'
            )
            if autotune_mode == 'off':
                result = scan_iq_samples(
                    samples,
                    label='live_iq',
                    sample_rate=sample_rate,
                    start_second=float(start_sample) / float(sample_rate),
                    sps_values=self._iq_decoder_sps_values(),
                    offset_step=int(self.get_parameter('iq_decoder_offset_step').value),
                    decode=True,
                    decode_threshold=threshold,
                    max_decoded_payloads=int(self.get_parameter('iq_decoder_max_payloads').value),
                    max_decoded_frames=int(self.get_parameter('iq_decoder_max_frames').value),
                    aggregate_decode=True,
                    frequency_shift_hz=frequency_shift_hz,
                    low_pass_hz=low_pass_hz,
                    start_sample=int(start_sample),
                    allowed_access_names=(profile,),
                    vote_repeated_frames=vote_frames,
                )
                autotune = None
            elif locked is not None:
                # Sticky fast path: reuse the previously locked winning candidate with a
                # single scan (full SPS sweep, one shift/low_pass) instead of the whole grid.
                result = scan_iq_samples(
                    samples,
                    label='live_iq',
                    sample_rate=sample_rate,
                    start_second=float(start_sample) / float(sample_rate),
                    sps_values=self._iq_decoder_sps_values(),
                    offset_step=int(self.get_parameter('iq_decoder_offset_step').value),
                    decode=True,
                    decode_threshold=threshold,
                    max_decoded_payloads=int(self.get_parameter('iq_decoder_max_payloads').value),
                    max_decoded_frames=int(self.get_parameter('iq_decoder_max_frames').value),
                    aggregate_decode=True,
                    frequency_shift_hz=float(locked.get('frequency_shift_hz', frequency_shift_hz)),
                    low_pass_hz=float(locked.get('low_pass_hz', low_pass_hz)),
                    start_sample=int(start_sample),
                    allowed_access_names=(profile,),
                    vote_repeated_frames=vote_frames,
                )
                autotune = None
                used_lock = True
            else:
                autotune = autotune_iq_samples(
                    samples,
                    label='live_iq',
                    profile=profile,
                    level=level,
                    sample_rate=sample_rate,
                    base_low_pass_hz=low_pass_hz,
                    requested_frequency_shift_hz=frequency_shift_hz,
                    start_second=float(start_sample) / float(sample_rate),
                    start_sample=int(start_sample),
                    sps_values=self._iq_decoder_sps_values(),
                    offset_step=int(self.get_parameter('iq_decoder_offset_step').value),
                    decode_threshold=threshold,
                    max_decoded_payloads=int(self.get_parameter('iq_decoder_max_payloads').value),
                    max_decoded_frames=int(self.get_parameter('iq_decoder_max_frames').value),
                    frame_limit=8,
                    mode=autotune_mode,
                    max_frequency_candidates=int(self.get_parameter('iq_decoder_max_frequency_candidates').value),
                    max_candidates=int(self.get_parameter('iq_decoder_max_candidates').value),
                    vote_repeated_frames=vote_frames,
                )
                result = autotune['best_result']
            decoded = result.get('decoded') or {}
            frames = self._collect_iq_decoded_frames(decoded)
            referee_frame_count = int(decoded.get('referee_frame_count', 0) or 0)
            if lock_enabled and autotune_mode != 'off':
                if used_lock:
                    if referee_frame_count > 0:
                        self._iq_lock_miss_count = 0
                    else:
                        self._iq_lock_miss_count += 1
                        max_miss = max(1, int(self.get_parameter('iq_decoder_autotune_lock_max_miss').value))
                        if self._iq_lock_miss_count >= max_miss:
                            self._iq_locked_candidate = None
                            self._iq_lock_miss_count = 0
                elif autotune is not None and referee_frame_count > 0:
                    best_candidate = autotune.get('best_candidate') or {}
                    if best_candidate:
                        self._iq_locked_candidate = {
                            'frequency_shift_hz': float(best_candidate.get('frequency_shift_hz', frequency_shift_hz)),
                            'low_pass_hz': float(best_candidate.get('low_pass_hz', low_pass_hz)),
                        }
                        self._iq_lock_miss_count = 0
            self.decode_stats['iq_decode_batches'] += 1
            self.iq_decode_diagnostics = {
                'sample_count': int(samples.size),
                'start_sample': int(start_sample),
                'best_access_match': result.get('best_access_match'),
                'air_payload_count': int(decoded.get('air_payload_count', 0) or 0),
                'referee_frame_count': int(decoded.get('referee_frame_count', 0) or 0),
                'published_frames': len(frames),
                'autotune': None
                if autotune is None
                else {
                    'mode': autotune.get('mode'),
                    'candidate_count': autotune.get('candidate_count'),
                    'scanned_candidate_count': autotune.get('scanned_candidate_count'),
                    'peak_offsets_hz': autotune.get('peak_offsets_hz'),
                    'best_candidate': autotune.get('best_candidate'),
                    'best_score': autotune.get('best_score'),
                    'best_summary': autotune.get('best_summary'),
                },
                'frequency_shift_hz': result.get('frequency_shift_hz'),
                'low_pass_hz': result.get('low_pass_hz'),
                'used_lock': bool(used_lock),
                'locked_candidate': self._iq_locked_candidate,
            }
            if frames:
                self._publish_frames(frames)
        except Exception as exc:  # noqa: BLE001 - background decoder must not kill the node
            self.last_error = f'live IQ decode failed: {exc}'
            self.iq_decode_diagnostics = {'last_error': self.last_error}

    def _push_iq_decoded_payloads(self, payloads) -> list:
        frames = []
        ordered_payloads = sorted(
            payloads,
            key=lambda item: int(item.get('absolute_bit_index', item.get('bit_index', 0)) or 0),
        )
        with self._iq_decode_state_lock:
            for payload in ordered_payloads:
                payload_hex = str(payload.get('air_payload_hex', ''))
                if len(payload_hex) != 30:
                    continue
                absolute_bit_index = int(payload.get('absolute_bit_index', payload.get('bit_index', 0)) or 0)
                key = (
                    str(payload.get('access', '')),
                    int(round(absolute_bit_index / 8.0)),
                    payload_hex,
                )
                if key in self._iq_seen_payload_keys:
                    continue
                self._remember_iq_payload_key(key)
                air_payload = bytes.fromhex(payload_hex)
                frames.extend(self.iq_frame_assembler.push_air_payload(air_payload))
                self.decode_stats['iq_air_payloads'] += 1
        return frames

    def _collect_iq_decoded_frames(self, decoded: dict) -> list:
        direct_frames = self._push_iq_decoded_referee_frames(decoded.get('referee_frames') or [])
        payload_frames = self._push_iq_decoded_payloads(decoded.get('air_payloads') or [])
        direct_raw = {frame.raw for frame in direct_frames}
        return direct_frames + [frame for frame in payload_frames if frame.raw not in direct_raw]

    def _push_iq_decoded_referee_frames(self, frame_dicts) -> list:
        frames = []
        ordered_frames = sorted(
            frame_dicts,
            key=lambda item: int(
                item.get(
                    'source_air_payload_absolute_bit_index',
                    item.get('source_air_payload_bit_index', 0),
                )
                or 0
            ),
        )
        with self._iq_decode_state_lock:
            for frame_dict in ordered_frames:
                raw_hex = ''.join(str(frame_dict.get('raw_hex', '')).split())
                if not raw_hex:
                    continue
                try:
                    raw = bytes.fromhex(raw_hex)
                    source_bit_index = int(
                        frame_dict.get(
                            'source_air_payload_absolute_bit_index',
                            frame_dict.get('source_air_payload_bit_index', -1),
                        )
                        or -1
                    )
                except (TypeError, ValueError):
                    continue
                key = (source_bit_index, raw.hex())
                if key in self._iq_seen_frame_keys:
                    continue
                parsed = RefereeFrameAssembler(
                    max_buffer_size=max(4096, len(raw) + 16),
                    allowed_cmd_lengths=RADAR_AIR_DATA_LENGTHS,
                ).push_bytes(raw)
                if len(parsed) != 1 or parsed[0].raw != raw:
                    continue
                self._remember_iq_frame_key(key)
                frames.append(parsed[0])
                self.decode_stats['iq_referee_frames'] += 1
        return frames

    def _remember_iq_payload_key(self, key: tuple[str, int, str]) -> None:
        self._iq_seen_payload_keys.add(key)
        self._iq_seen_payload_order.append(key)
        while len(self._iq_seen_payload_order) > 4096:
            old = self._iq_seen_payload_order.pop(0)
            self._iq_seen_payload_keys.discard(old)

    def _remember_iq_frame_key(self, key: tuple[int, str]) -> None:
        self._iq_seen_frame_keys.add(key)
        self._iq_seen_frame_order.append(key)
        while len(self._iq_seen_frame_order) > 4096:
            old = self._iq_seen_frame_order.pop(0)
            self._iq_seen_frame_keys.discard(old)

    def _current_sample_rate(self) -> float:
        if self.flowgraph is not None:
            getter = getattr(self.flowgraph, 'get_sample_rate', None)
            if getter is not None:
                try:
                    value = float(getter())
                    if value > 0:
                        return value
                except Exception:
                    pass
        try:
            return float(os.environ.get('RM_RADIO_SAMPLE_RATE', '2000000'))
        except ValueError:
            return 2_000_000.0

    def _current_center_frequency(self) -> Optional[float]:
        # ``cen_f`` is the setter that drives the AD9361 hardware in RX.py.
        # Prefer it over legacy display-only aliases if a payload contains more
        # than one spelling, so spectrum diagnostics report the tuned RF value.
        for key in ('cen_f', 'center_f', 'center_F'):
            try:
                value = float(self.last_setters.get(key, 0.0))
                if value > 0:
                    return value
            except Exception:
                pass
        if self.flowgraph is not None:
            for getter_name in ('get_cen_f', 'get_center_f', 'get_center_F'):
                getter = getattr(self.flowgraph, getter_name, None)
                if getter is None:
                    continue
                try:
                    value = float(getter())
                    if value > 0:
                        return value
                except Exception:
                    pass
        try:
            from ..core.radio_config import BROADCAST_FREQUENCIES, interference_setters_for_side_level

            side = str(self.get_parameter('radio_side').value).strip().lower()
            profile = self._current_rx_profile()
            if profile == 'interference':
                setters = interference_setters_for_side_level(side, self._current_interference_level())
                return float(setters.get('center_f'))
            return float(BROADCAST_FREQUENCIES.get(side))
        except Exception:
            return None

    def _current_rx_profile(self) -> str:
        profile = str(self.get_parameter('rx_profile').value).strip().lower()
        return profile if profile in {'broadcast', 'interference'} else 'broadcast'

    def _current_interference_level(self) -> int:
        try:
            level = int(self.get_parameter('interference_level').value)
        except Exception:
            level = 1
        return min(3, max(1, level))

    def _iq_decoder_autotune_mode(self) -> str:
        mode = str(self.get_parameter('iq_decoder_autotune_mode').value or 'quick').strip().lower()
        if mode in {'0', 'false', 'no', 'off', 'disabled'}:
            return 'off'
        if mode not in {'quick', 'full'}:
            return 'quick'
        return mode

    def _iq_decoder_low_pass_hz(self, profile: str, level: int) -> float:
        try:
            configured = float(self.get_parameter('iq_decoder_low_pass_hz').value)
            if configured > 0:
                return configured
        except Exception:
            pass
        try:
            current = float(self.last_setters.get('LowPass', 0.0))
            if current > 0:
                return current
        except Exception:
            pass
        if str(profile).strip().lower() == 'interference':
            return 160_000.0 if int(level) == 3 else 500_000.0
        return 320_000.0

    def _iq_decoder_sps_values(self) -> list[float]:
        raw = self.get_parameter('iq_decoder_sps_values').value
        if isinstance(raw, (list, tuple)):
            return [float(value) for value in raw]
        values = [float(item.strip()) for item in str(raw).split(',') if item.strip()]
        return values or [92.0, 92.5, 93.0, 93.5, 94.0]

    def _handle_start(self, _request, response):
        ok = self.start_flowgraph()
        response.success = ok
        response.message = 'started' if ok else self.last_error
        return response

    def _handle_stop(self, _request, response):
        ok = self.stop_flowgraph()
        response.success = ok
        response.message = 'stopped' if ok else self.last_error
        return response

    def _handle_reload(self, _request, response):
        ok = self.reload_flowgraph()
        response.success = ok
        response.message = 'reloaded' if ok else self.last_error
        return response

    def _handle_status(self, _request, response):
        response.success = self.loaded and (self.started or bool(self.get_parameter('dry_run').value))
        response.message = self._status_payload()
        return response

    def destroy_node(self) -> bool:
        self.stop_flowgraph()
        recorder = getattr(self, '_iq_recorder', None)
        if recorder is not None:
            recorder.stop('node_shutdown', timeout=3.0)
        return super().destroy_node()


def run_node(node_name: str = 'rm_flowgraph_node', flowgraph_name: str = '') -> None:
    rclpy.init()
    node: Optional[FlowgraphNode] = None
    try:
        node = FlowgraphNode(node_name=node_name, flowgraph_name=flowgraph_name)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    run_node()
