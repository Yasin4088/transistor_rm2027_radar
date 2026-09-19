import os
import threading

import pytest

rclpy = pytest.importorskip('rclpy')

from rm_radio_ros.nodes.flowgraph_node import FlowgraphNode
from rm_radio_ros.core.rm_protocol import RADAR_AIR_DATA_LENGTHS, RefereeFrameAssembler, build_referee_frame


class FakeParameter:
    def __init__(self, value):
        self.value = value


class FakeLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []

    def info(self, msg):
        self.infos.append(msg)

    def warning(self, msg):
        self.warnings.append(msg)


class FakeFlowgraph:
    def __init__(self):
        self.calls = []

    def set_cen_f(self, value):
        self.calls.append(('cen_f', value))

    def set_BW(self, value):
        self.calls.append(('BW', value))


class FakeFlowgraphNodeLogger(FakeLogger):
    def __init__(self):
        super().__init__()
        self.errors = []
        self.debugs = []

    def error(self, msg):
        self.errors.append(msg)

    def debug(self, msg):
        self.debugs.append(msg)


class FakeTimer:
    def __init__(self):
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


@pytest.mark.parametrize("recovered", [False, True])
def test_auto_start_retry_keeps_node_alive_until_flowgraph_recovers(recovered):
    node = FlowgraphNode.__new__(FlowgraphNode)
    node.started = False
    timer = FakeTimer()
    node._auto_start_retry_timer = timer
    attempts = []
    node.start_flowgraph = lambda: attempts.append(True) or recovered
    logger = FakeFlowgraphNodeLogger()
    node.get_logger = lambda: logger

    FlowgraphNode._retry_auto_start(node)

    assert attempts == [True]
    assert timer.cancelled is recovered
    assert (node._auto_start_retry_timer is None) is recovered
    assert bool(logger.warnings) is recovered


def test_ros_wrapper_shows_and_hides_native_gnuradio_window():
    class FakeNativeWindow:
        def __init__(self):
            self.calls = []

        def setWindowTitle(self, title):
            self.calls.append(('title', title))

        def show(self):
            self.calls.append(('show',))

        def hide(self):
            self.calls.append(('hide',))

    node = FlowgraphNode.__new__(FlowgraphNode)
    node.flowgraph = FakeNativeWindow()
    node._parameters = {'disable_gui_sinks': False, 'rx_profile': 'broadcast'}
    node.get_parameter = lambda name: FakeParameter(node._parameters[name])
    logger = FakeFlowgraphNodeLogger()
    node.get_logger = lambda: logger
    qt_events = []
    node._process_qt_events = lambda: qt_events.append(True)

    assert FlowgraphNode._set_flowgraph_window_visible(node, True) is True
    assert node.flowgraph.calls[:2] == [
        ('title', 'SharkRadio GNU Radio - 信息波 RX1'),
        ('show',),
    ]
    assert FlowgraphNode._set_flowgraph_window_visible(node, False) is True
    assert node.flowgraph.calls[-1] == ('hide',)
    assert qt_events == [True, True]

    node._parameters['disable_gui_sinks'] = True
    assert FlowgraphNode._set_flowgraph_window_visible(node, True) is False
    assert node.flowgraph.calls[-1] == ('hide',)


def test_flowgraph_environment_uses_explicit_tx_sample_rate_parameter(monkeypatch):
    node = FlowgraphNode.__new__(FlowgraphNode)
    node._parameters = {
        'disable_gui_sinks': True,
        'rx_only': False,
        'rx_uri': 'ip:192.168.9.110',
        'broadcast_tx_uri': 'ip:192.168.2.1',
        'interference_tx_uri': 'ip:192.168.3.1',
        'sample_rate': 1000000.0,
        'radio_side': 'red',
        'rx_profile': 'interference',
        'frontend_profile': 'broadcast',
        'demod_mode': 'legacy',
        'input_sps': 93.0,
        'interference_level': 2,
    }
    node.get_parameter = lambda name: FakeParameter(node._parameters[name])
    node.get_logger = lambda: FakeLogger()
    monkeypatch.delenv('RM_RADIO_SAMPLE_RATE', raising=False)

    FlowgraphNode._configure_flowgraph_environment(node)

    assert os.environ['RM_RADIO_INTERFERENCE_TX_URI'] == 'ip:192.168.3.1'
    assert os.environ['RM_RADIO_SAMPLE_RATE'] == '1000000'
    assert os.environ['RM_RADIO_FRONTEND_PROFILE'] == 'broadcast'
    assert os.environ['RM_RADIO_INPUT_SPS'] == '93.0'


def test_flowgraph_environment_rejects_unknown_frontend_profile():
    node = FlowgraphNode.__new__(FlowgraphNode)
    node._parameters = {
        'disable_gui_sinks': True,
        'rx_only': True,
        'rx_uri': 'ip:192.168.9.111',
        'broadcast_tx_uri': 'ip:192.168.2.1',
        'interference_tx_uri': 'ip:192.168.3.1',
        'sample_rate': 2_000_000.0,
        'radio_side': 'blue',
        'rx_profile': 'interference',
        'frontend_profile': 'unknown',
        'demod_mode': 'legacy',
        'input_sps': 94.0,
        'interference_level': 1,
    }
    node.get_parameter = lambda name: FakeParameter(node._parameters[name])
    node.get_logger = lambda: FakeLogger()

    with pytest.raises(ValueError, match='frontend_profile'):
        FlowgraphNode._configure_flowgraph_environment(node)


def _flowgraph_node_without_init(settle_sec=0.15):
    node = FlowgraphNode.__new__(FlowgraphNode)
    node.flowgraph = FakeFlowgraph()
    node.last_error = ''
    node.last_setters = {}
    node.last_setters_apply_time = 0.0
    node.setters_apply_count = 0
    node._setters_lock = threading.RLock()
    node._set_rx_profile_calls = []
    node._set_rx_profile = lambda profile: node._set_rx_profile_calls.append(profile)
    node._parameters = {'setters_apply_settle_sec': settle_sec, 'interference_level': 1}
    node.get_parameter = lambda name: FakeParameter(node._parameters[name])

    def set_parameters(parameters):
        for parameter in parameters:
            node._parameters[parameter.name] = parameter.value
        return []

    node.set_parameters = set_parameters
    node.get_logger = lambda: FakeLogger()
    return node


def test_flowgraph_setters_apply_hardware_before_rx_profile_and_settle(monkeypatch):
    sleeps = []
    now = [1000.0]
    monkeypatch.setattr('rm_radio_ros.nodes.flowgraph_node.time.sleep', lambda value: sleeps.append(value))
    monkeypatch.setattr('rm_radio_ros.nodes.flowgraph_node.time.time', lambda: now[0])
    node = _flowgraph_node_without_init(settle_sec=0.15)
    setters = {'rx_profile': 'broadcast', 'cen_f': 433200000, 'BW': 540000}

    FlowgraphNode._apply_setter_values(node, setters)

    assert setters == {'rx_profile': 'broadcast', 'cen_f': 433200000, 'BW': 540000}
    assert node.flowgraph.calls == [('cen_f', 433200000), ('BW', 540000)]
    assert node._set_rx_profile_calls == ['broadcast']
    assert sleeps == [0.15]
    assert node.last_setters['rx_profile'] == 'broadcast'
    assert node.setters_apply_count == 1
    assert node.last_setters_apply_time == 1000.0


def test_flowgraph_setters_do_not_sleep_for_non_rf_setter(monkeypatch):
    sleeps = []
    monkeypatch.setattr('rm_radio_ros.nodes.flowgraph_node.time.sleep', lambda value: sleeps.append(value))
    node = _flowgraph_node_without_init(settle_sec=0.15)

    node.flowgraph.set_payload_data = lambda value: node.flowgraph.calls.append(('payload_data', value))

    FlowgraphNode._apply_setter_values(node, {'payload_data': [1, 2, 3]})

    assert sleeps == []
    assert node.flowgraph.calls == [('payload_data', [1, 2, 3])]


def test_flowgraph_setters_sync_interference_level_parameter(monkeypatch):
    sleeps = []
    monkeypatch.setattr('rm_radio_ros.nodes.flowgraph_node.time.sleep', lambda value: sleeps.append(value))
    node = _flowgraph_node_without_init(settle_sec=0.15)
    node._parameters['interference_level'] = 2

    FlowgraphNode._apply_setter_values(node, {'cen_f': 432800000, 'interference_level': 3})

    assert node.flowgraph.calls == [('cen_f', 432800000)]
    assert node._parameters['interference_level'] == 3
    assert node.last_setters['interference_level'] == 3
    assert sleeps == [0.15]


def test_live_iq_autotune_parameter_defaults():
    node = FlowgraphNode.__new__(FlowgraphNode)
    values = {
        'rx_profile': 'broadcast',
        'interference_level': 2,
        'iq_decoder_autotune_mode': 'bad-value',
        'iq_decoder_low_pass_hz': 0.0,
    }
    node.last_setters = {}
    node.get_parameter = lambda name: FakeParameter(values[name])

    assert FlowgraphNode._current_rx_profile(node) == 'broadcast'
    assert FlowgraphNode._current_interference_level(node) == 2
    assert FlowgraphNode._iq_decoder_autotune_mode(node) == 'quick'
    assert FlowgraphNode._iq_decoder_low_pass_hz(node, 'broadcast', 2) == 320_000.0

    values['rx_profile'] = 'interference'
    values['interference_level'] = 3
    assert FlowgraphNode._current_rx_profile(node) == 'interference'
    assert FlowgraphNode._iq_decoder_low_pass_hz(node, 'interference', 3) == 160_000.0

    values['iq_decoder_autotune_mode'] = 'off'
    values['iq_decoder_low_pass_hz'] = 260_000.0
    assert FlowgraphNode._iq_decoder_autotune_mode(node) == 'off'
    assert FlowgraphNode._iq_decoder_low_pass_hz(node, 'broadcast', 2) == 260_000.0


def test_iio_capture_only_active_only_for_broadcast():
    node = FlowgraphNode.__new__(FlowgraphNode)
    values = {
        'rx_profile': 'broadcast',
        'iio_capture_decoder_enabled': True,
        'iio_capture_decoder_only': True,
    }
    node.get_parameter = lambda name: FakeParameter(values[name])

    assert FlowgraphNode._iio_capture_only_active(node) is True

    values['rx_profile'] = 'interference'
    assert FlowgraphNode._iio_capture_only_active(node) is False

    values['rx_profile'] = 'broadcast'
    values['iio_capture_decoder_only'] = False
    assert FlowgraphNode._iio_capture_only_active(node) is False


def test_capture_only_hot_setters_reconfigure_iio_and_reset_decoder(monkeypatch):
    node = FlowgraphNode.__new__(FlowgraphNode)
    node.flowgraph = None
    node.last_error = ''
    node.last_setters = {'GainMode': 'slow_attack'}
    node.last_setters_apply_time = 0.0
    node.setters_apply_count = 0
    node._setters_lock = threading.RLock()
    node._iio_device_lock = threading.RLock()
    node._parameters = {
        'rx_uri': 'ip:192.168.9.110',
        'rx_profile': 'broadcast',
        'iio_capture_decoder_enabled': True,
        'iio_capture_decoder_only': True,
        'setters_apply_settle_sec': 0.0,
    }
    node.get_parameter = lambda name: FakeParameter(node._parameters[name])
    node.get_logger = lambda: FakeLogger()
    writes = []
    attributes = {}

    def write_attribute(*args):
        writes.append(args)
        attributes[args[:-1]] = args[-1]

    node._write_iio_capture_attribute = write_attribute
    node._read_iio_capture_attribute = lambda *args: attributes[args]
    resets = []
    node._reset_decode_state = lambda: resets.append(True)
    monkeypatch.setattr('rm_radio_ros.nodes.flowgraph_node.time.time', lambda: 123.0)

    FlowgraphNode._apply_setter_values(node, {
        'cen_f': 433_920_000,
        'bw_re': 540_000,
        'LowPass': 260_000,
        'GainMode': 'manual',
        'Gain': 45,
    })

    assert writes[0][-2:] == ('frequency', '433920000')
    assert sum('rf_bandwidth' in call for call in writes) == 2
    assert sum('gain_control_mode' in call for call in writes) == 2
    assert sum('hardwaregain' in call for call in writes) == 2
    assert resets == [True]
    assert node.last_setters['LowPass'] == 260_000
    assert node.last_setters_apply_time == 123.0
    assert node.last_error == ''


def _capture_only_load_node(tmp_path):
    node = FlowgraphNode.__new__(FlowgraphNode)
    values = {
        'dry_run': False,
        'flowgraph_module': 'RM',
        'flowgraph_class': 'RM',
        'disable_gui_sinks': True,
        'rx_only': True,
        'rx_uri': 'ip:192.168.9.110',
        'broadcast_tx_uri': 'ip:192.168.1.10',
        'interference_tx_uri': 'ip:192.168.3.1',
        'sample_rate': 0.0,
        'radio_side': 'red',
        'rx_profile': 'broadcast',
        'frontend_profile': 'broadcast',
        'demod_mode': 'mm',
        'input_sps': 94.0,
        'interference_level': 2,
        'iio_capture_decoder_enabled': True,
        'iio_capture_decoder_only': True,
        'iio_capture_rf_port': 'B_BALANCED',
        'setters_apply_settle_sec': 0.0,
        'setters_json': (
            '{"cen_f":433200000,"bw_re":540000,"GainMode":"slow_attack",'
            '"Gain":20,"LowPass":260000,"sample_rate":2000000}'
        ),
    }
    logger = FakeFlowgraphNodeLogger()
    node.loaded = False
    node.started = False
    node.flowgraph = None
    node.last_error = ''
    node.last_setters = {}
    node.last_setters_apply_time = 0.0
    node.setters_apply_count = 0
    node._setters_lock = threading.RLock()
    node._iio_device_lock = threading.RLock()
    node.get_parameter = lambda name: FakeParameter(values[name])
    node.get_logger = lambda: logger
    node._resolve_flowgraph_dir = lambda: tmp_path
    node._ensure_qt_application = lambda: (_ for _ in ()).throw(AssertionError('should not build Qt'))
    node._import_flowgraph_module = lambda _path: (_ for _ in ()).throw(AssertionError('should not import RM'))
    writes = []
    attributes = {}

    def write_attribute(*args):
        writes.append(args)
        attributes[args[:-1]] = args[-1]

    node._write_iio_capture_attribute = write_attribute
    node._read_iio_capture_attribute = lambda *args: attributes[args]
    return node, logger, writes, attributes


def test_iio_capture_only_load_writes_and_verifies_hardware_before_loaded(tmp_path):
    node, _logger, writes, _attributes = _capture_only_load_node(tmp_path)

    assert FlowgraphNode.load_flowgraph(node) is True
    assert node.loaded is True
    assert node.flowgraph is None
    assert node.last_setters['LowPass'] == 260000
    assert any(call[-2:] == ('frequency', '433200000') for call in writes)
    assert sum('sampling_frequency' in call for call in writes) == 2
    assert sum('rf_bandwidth' in call for call in writes) == 2
    assert sum('gain_control_mode' in call for call in writes) == 2
    assert sum('rf_port_select' in call for call in writes) == 2


def test_iio_capture_only_load_rejects_readback_mismatch(tmp_path):
    node, logger, _writes, attributes = _capture_only_load_node(tmp_path)

    def mismatched_read(*args):
        if args[-1] == 'frequency':
            return '432000000'
        return attributes[args]

    node._read_iio_capture_attribute = mismatched_read

    assert FlowgraphNode.load_flowgraph(node) is False
    assert node.loaded is False
    assert node.started is False
    assert 'IIO readback mismatch' in node.last_error
    assert logger.errors


def test_iio_attribute_timeout_is_bounded_and_visible(monkeypatch):
    import subprocess

    node = FlowgraphNode.__new__(FlowgraphNode)
    values = {
        'rx_uri': 'ip:192.168.9.110',
        'iio_attribute_timeout_sec': 0.2,
    }
    node.get_parameter = lambda name: FakeParameter(values[name])
    observed = {}

    def timed_out(command, **kwargs):
        observed['command'] = command
        observed['timeout'] = kwargs['timeout']
        raise subprocess.TimeoutExpired(command, kwargs['timeout'])

    monkeypatch.setattr(subprocess, 'run', timed_out)

    with pytest.raises(TimeoutError, match=r'iio_attr timed out.*192\.168\.9\.110'):
        FlowgraphNode._write_iio_capture_attribute(
            node,
            '-c',
            'ad9361-phy',
            'altvoltage0',
            'frequency',
            '433200000',
        )

    assert observed['timeout'] == pytest.approx(0.7)
    assert observed['command'][:4] == ['iio_attr', '-T', '200', '-u']


def test_iio_capture_stop_timeout_blocks_restart_until_old_thread_exits():
    node = FlowgraphNode.__new__(FlowgraphNode)
    values = {
        'iio_capture_decoder_enabled': True,
        'rx_profile': 'broadcast',
        'iio_capture_start_delay_sec': 0.0,
        'iio_capture_period_sec': 0.1,
        'iio_capture_stop_timeout_sec': 0.1,
    }
    logger = FakeFlowgraphNodeLogger()
    node.get_parameter = lambda name: FakeParameter(values[name])
    node.get_logger = lambda: logger
    node._setters_lock = threading.RLock()
    node._iio_device_lock = threading.RLock()
    node._iq_buffer_lock = threading.RLock()
    node._iio_capture_thread_lock = threading.RLock()
    node._iio_capture_thread = None
    node._iio_capture_stop_event = None
    node._iio_capture_generation = 0
    node._iio_capture_stopping = False
    node._iio_capture_total_samples_seen = 0
    node.decode_stats = FlowgraphNode._new_decode_stats()
    node.last_error = ''
    node.iq_decode_diagnostics = {}
    capture_entered = threading.Event()
    release_capture = threading.Event()
    capture_count = []

    def blocking_capture(**_kwargs):
        capture_count.append(True)
        capture_entered.set()
        release_capture.wait(2.0)
        return None

    node._capture_iio_samples = blocking_capture

    FlowgraphNode._start_iio_capture_decoder_if_needed(node)
    old_thread = node._iio_capture_thread
    assert old_thread is not None
    assert capture_entered.wait(1.0)

    assert FlowgraphNode._stop_iio_capture_decoder(node) is False
    assert node._iio_capture_thread is old_thread
    with pytest.raises(RuntimeError, match='still stopping'):
        FlowgraphNode._start_iio_capture_decoder_if_needed(node)
    assert capture_count == [True]

    release_capture.set()
    old_thread.join(timeout=1.0)
    assert not old_thread.is_alive()
    assert node._iio_capture_thread is None
    assert FlowgraphNode._stop_iio_capture_decoder(node) is True
    assert node._iio_capture_stopping is False


def test_iio_capture_streams_chunks_to_spectrum_callback(monkeypatch):
    import io
    import numpy as np
    import subprocess

    samples = np.column_stack((
        np.arange(2048, dtype=np.int16),
        -np.arange(2048, dtype=np.int16),
    ))
    raw_bytes = samples.astype('<i2').tobytes()
    observed = {'commands': [], 'chunks': []}

    class FakeProcess:
        pid = 4321

        def __init__(self):
            self.stdout = io.BytesIO(raw_bytes)
            self.stderr = io.BytesIO(b'')

        def wait(self, timeout=None):
            return 0

    def fake_popen(command, **_kwargs):
        observed['commands'].append(command)
        return FakeProcess()

    monkeypatch.setattr(subprocess, 'Popen', fake_popen)
    node = FlowgraphNode.__new__(FlowgraphNode)
    values = {
        'rx_uri': 'ip:192.168.9.110',
        'iio_capture_seconds': 0.1,
        'iio_capture_timeout_sec': 1.0,
        'iio_capture_buffer_size': 1024,
    }
    node.get_parameter = lambda name: FakeParameter(values[name])
    node._current_sample_rate = lambda: 20_000.0
    node._iio_device_lock = threading.RLock()
    node._iq_buffer_lock = threading.RLock()

    captured = FlowgraphNode._capture_iio_samples(
        node,
        spectrum_callback=lambda chunk: observed['chunks'].append(chunk.copy()),
    )

    assert captured.size == 2048
    assert sum(chunk.size for chunk in observed['chunks']) == 2048
    assert 'iio_readdev' in observed['commands'][0]
    assert np.isclose(captured[1].real, 1 / 32768.0)


def test_reload_aborts_without_clearing_flowgraph_when_stop_fails():
    node = FlowgraphNode.__new__(FlowgraphNode)
    original_flowgraph = object()
    node.started = True
    node.loaded = True
    node.flowgraph = original_flowgraph
    node.stop_flowgraph = lambda: False
    node._reset_decode_state = lambda: (_ for _ in ()).throw(
        AssertionError('reload must not reset state after a failed stop')
    )

    assert FlowgraphNode.reload_flowgraph(node) is False
    assert node.flowgraph is original_flowgraph
    assert node.loaded is True


def test_spectrum_payload_is_lightweight_summary():
    import numpy as np

    node = FlowgraphNode.__new__(FlowgraphNode)
    values = {
        'spectrum_fft_size': 64,
        'spectrum_bin_count': 16,
        'rx_profile': 'broadcast',
        'radio_side': 'red',
        'interference_level': 1,
    }
    node.flowgraph = None
    node.last_setters = {'center_f': 433_200_000}
    node.get_name = lambda: 'rm_gfsk_node'
    node.get_parameter = lambda name: FakeParameter(values[name])
    samples = np.exp(1j * 2 * np.pi * 0.1 * np.arange(256)).astype(np.complex64)

    payload = FlowgraphNode._spectrum_payload(node, samples, 123.0)

    assert payload['node'] == 'rm_gfsk_node'
    assert payload['timestamp'] == 123.0
    assert payload['center_frequency_hz'] == 433_200_000
    assert payload['fft_size'] == 64
    assert payload['bin_count'] == 16
    assert len(payload['offset_hz']) == 16
    assert len(payload['power_dbfs']) == 16
    assert payload['peak_dbfs'] > payload['noise_floor_dbfs']


def test_spectrum_center_prefers_hardware_cen_f_over_legacy_center_alias():
    node = FlowgraphNode.__new__(FlowgraphNode)
    node.flowgraph = None
    node.last_setters = {
        'cen_f': 434_320_000,
        'center_F': 433_920_000,
    }

    assert FlowgraphNode._current_center_frequency(node) == 434_320_000


def test_spectrum_publisher_emits_json(monkeypatch):
    import json
    import numpy as np

    node = FlowgraphNode.__new__(FlowgraphNode)
    values = {
        'spectrum_enabled': True,
        'spectrum_period_sec': 0.5,
        'spectrum_fft_size': 64,
        'spectrum_bin_count': 16,
        'rx_profile': 'broadcast',
        'radio_side': 'red',
        'interference_level': 1,
    }
    published = []

    class Pub:
        def publish(self, msg):
            published.append(msg.data)

    node.flowgraph = None
    node.last_setters = {'center_f': 433_200_000}
    node.last_error = ''
    node.spectrum_pub = Pub()
    node._spectrum_last_publish_time = 0.0
    node.get_name = lambda: 'rm_gfsk_node'
    node.get_parameter = lambda name: FakeParameter(values[name])
    monkeypatch.setattr('rm_radio_ros.nodes.flowgraph_node.time.time', lambda: 10.0)

    FlowgraphNode._maybe_publish_spectrum(node, np.ones(128, dtype=np.complex64))

    assert len(published) == 1
    assert json.loads(published[0])['node'] == 'rm_gfsk_node'
    assert node._spectrum_last_publish_time == 10.0


def test_iq_diagnostics_report_dbfs_headroom_and_clipping():
    import numpy as np

    safe = np.full(128, 0.1 + 0.1j, dtype=np.complex64)
    clipped = safe.copy()
    clipped[:4] = 1.0 + 0.0j

    safe_result = FlowgraphNode._iq_diagnostics_payload(safe, 'test_raw')
    clipped_result = FlowgraphNode._iq_diagnostics_payload(clipped, 'test_raw')

    assert safe_result['source'] == 'test_raw'
    assert safe_result['rms_dbfs'] < -15.0
    assert safe_result['headroom_db'] > 15.0
    assert safe_result['clip_fraction'] == 0.0
    assert safe_result['signal_state'] == 'ok'
    assert clipped_result['clip_fraction'] == 4 / 128
    assert clipped_result['signal_state'] == 'clipped'


def test_filtered_spectrum_uses_separate_latest_only_publisher(monkeypatch):
    import json
    import numpy as np

    node = FlowgraphNode.__new__(FlowgraphNode)
    values = {
        'spectrum_enabled': True,
        'spectrum_period_sec': 0.5,
        'spectrum_fft_size': 64,
        'spectrum_bin_count': 16,
        'rx_profile': 'broadcast',
        'radio_side': 'red',
        'interference_level': 1,
    }
    published = []

    class Pub:
        def publish(self, msg):
            published.append(msg.data)

    node.flowgraph = None
    node.last_setters = {'center_f': 433_200_000}
    node.last_error = ''
    node.filtered_spectrum_pub = Pub()
    node._filtered_spectrum_last_publish_time = 0.0
    node.get_name = lambda: 'rm_gfsk_node'
    node.get_parameter = lambda name: FakeParameter(values[name])
    monkeypatch.setattr('rm_radio_ros.nodes.flowgraph_node.time.time', lambda: 10.0)

    FlowgraphNode._maybe_publish_spectrum(
        node,
        np.ones(128, dtype=np.complex64),
        tap='filtered',
    )

    assert json.loads(published[0])['tap'] == 'filtered'
    assert node._filtered_spectrum_last_publish_time == 10.0


def _iq_decode_node_without_init():
    node = FlowgraphNode.__new__(FlowgraphNode)
    node.iq_frame_assembler = RefereeFrameAssembler(allowed_cmd_lengths=RADAR_AIR_DATA_LENGTHS)
    node.decode_stats = FlowgraphNode._new_decode_stats()
    node._iq_decode_state_lock = threading.RLock()
    node._iq_seen_payload_keys = set()
    node._iq_seen_payload_order = []
    node._iq_seen_frame_keys = set()
    node._iq_seen_frame_order = []
    return node


def test_live_iq_collects_direct_referee_frames_from_autotune_result():
    node = _iq_decode_node_without_init()
    raw = build_referee_frame(0x0A02, bytes(range(12)), seq=7)
    decoded = {
        'air_payloads': [],
        'referee_frames': [
            {
                'raw_hex': raw.hex(),
                'source_air_payload_absolute_bit_index': 123456,
            },
        ],
    }

    frames = FlowgraphNode._collect_iq_decoded_frames(node, decoded)

    assert len(frames) == 1
    assert frames[0].cmd_id == 0x0A02
    assert frames[0].seq == 7
    assert frames[0].raw == raw
    assert node.decode_stats['iq_referee_frames'] == 1


def test_live_iq_direct_referee_frames_are_deduped_by_source_position():
    node = _iq_decode_node_without_init()
    raw = build_referee_frame(0x0A06, b'KEY123', seq=3)
    decoded = {
        'air_payloads': [],
        'referee_frames': [
            {
                'raw_hex': raw.hex(),
                'source_air_payload_absolute_bit_index': 42,
            },
            {
                'raw_hex': raw.hex(),
                'source_air_payload_absolute_bit_index': 42,
            },
        ],
    }

    assert len(FlowgraphNode._collect_iq_decoded_frames(node, decoded)) == 1
    assert FlowgraphNode._collect_iq_decoded_frames(node, decoded) == []
    assert node.decode_stats['iq_referee_frames'] == 1


def _sticky_decode_node(monkeypatch, autotune_frames, scan_frames_seq, lock_max_miss=2):
    """Wire a node so FlowgraphNode._decode_iq_snapshot exercises only the sticky-autotune
    state machine. The full-autotune path returns ``autotune_frames`` CRC frames; each locked
    fast-scan pops the next value from ``scan_frames_seq``. Returns (node, calls)."""
    import numpy as np  # noqa: F401 - ensures numpy present for the caller's samples

    node = FlowgraphNode.__new__(FlowgraphNode)
    node._parameters = {
        'iq_decoder_decode_threshold': 3,
        'iq_decoder_frequency_shift_hz': 0.0,
        'iq_decoder_offset_step': 2,
        'iq_decoder_max_payloads': 512,
        'iq_decoder_max_frames': 128,
        'iq_decoder_max_frequency_candidates': 4,
        'iq_decoder_max_candidates': 8,
        'iq_decoder_autotune_lock_enabled': True,
        'iq_decoder_autotune_lock_max_miss': lock_max_miss,
        'iq_decoder_vote_repeated_frames': False,
    }
    node.get_parameter = lambda name: FakeParameter(node._parameters[name])
    node.get_logger = lambda: FakeFlowgraphNodeLogger()
    node._iq_locked_candidate = None
    node._iq_lock_miss_count = 0
    node.decode_stats = {'iq_decode_batches': 0}
    node.iq_decode_diagnostics = {}
    node.last_error = ''
    node._current_rx_profile = lambda: 'interference'
    node._current_interference_level = lambda: 2
    node._iq_decoder_low_pass_hz = lambda profile, level: 500000.0
    node._iq_decoder_autotune_mode = lambda: 'quick'
    node._iq_decoder_sps_values = lambda: [94.0]
    node._collect_iq_decoded_frames = lambda decoded: []
    node._publish_frames = lambda frames: None

    calls = []
    best = {'frequency_shift_hz': 12000.0, 'low_pass_hz': 480000.0}

    def fake_autotune(*args, **kwargs):
        calls.append('autotune')
        return {
            'mode': 'quick', 'candidate_count': 8, 'scanned_candidate_count': 8,
            'peak_offsets_hz': [], 'best_candidate': dict(best),
            'best_score': {}, 'best_summary': {},
            'best_result': {
                'decoded': {'referee_frame_count': autotune_frames, 'air_payload_count': 1},
                'best_access_match': {}, 'frequency_shift_hz': best['frequency_shift_hz'],
                'low_pass_hz': best['low_pass_hz'],
            },
        }

    def fake_scan(*args, **kwargs):
        calls.append(('scan', kwargs.get('frequency_shift_hz'), kwargs.get('low_pass_hz')))
        frames = scan_frames_seq.pop(0) if scan_frames_seq else 0
        return {
            'decoded': {'referee_frame_count': frames, 'air_payload_count': 1},
            'best_access_match': {}, 'frequency_shift_hz': kwargs.get('frequency_shift_hz'),
            'low_pass_hz': kwargs.get('low_pass_hz'),
        }

    monkeypatch.setattr('rm_radio_ros.core.rx_autotune.autotune_iq_samples', fake_autotune)
    monkeypatch.setattr('rm_radio_ros.core.offline_iq_scan.scan_iq_samples', fake_scan)
    return node, calls


def test_sticky_autotune_locks_after_hit_then_reuses_with_fast_scan(monkeypatch):
    import numpy as np

    node, calls = _sticky_decode_node(monkeypatch, autotune_frames=5, scan_frames_seq=[3])
    samples = np.zeros(1024, dtype=np.complex64)

    # Cycle 1: no lock -> full autotune; a hit locks the winning candidate.
    FlowgraphNode._decode_iq_snapshot(node, samples, 0, 2_000_000.0)
    assert calls == ['autotune']
    assert node._iq_locked_candidate == {'frequency_shift_hz': 12000.0, 'low_pass_hz': 480000.0}
    assert node._iq_lock_miss_count == 0

    # Cycle 2: locked -> single fast scan reusing the locked candidate (no autotune).
    FlowgraphNode._decode_iq_snapshot(node, samples, 1024, 2_000_000.0)
    assert calls == ['autotune', ('scan', 12000.0, 480000.0)]
    assert node._iq_locked_candidate is not None
    assert node.iq_decode_diagnostics['used_lock'] is True


def test_sticky_autotune_unlocks_after_consecutive_misses(monkeypatch):
    import numpy as np

    node, calls = _sticky_decode_node(monkeypatch, autotune_frames=5, scan_frames_seq=[0, 0], lock_max_miss=2)
    samples = np.zeros(1024, dtype=np.complex64)

    FlowgraphNode._decode_iq_snapshot(node, samples, 0, 2_000_000.0)   # autotune hit -> lock
    assert node._iq_locked_candidate is not None
    FlowgraphNode._decode_iq_snapshot(node, samples, 1, 2_000_000.0)   # locked miss #1
    assert node._iq_locked_candidate is not None and node._iq_lock_miss_count == 1
    FlowgraphNode._decode_iq_snapshot(node, samples, 2, 2_000_000.0)   # locked miss #2 == max -> unlock
    assert node._iq_locked_candidate is None and node._iq_lock_miss_count == 0
    assert calls == ['autotune', ('scan', 12000.0, 480000.0), ('scan', 12000.0, 480000.0)]


def test_sticky_autotune_disabled_always_runs_full_autotune(monkeypatch):
    import numpy as np

    node, calls = _sticky_decode_node(monkeypatch, autotune_frames=5, scan_frames_seq=[])
    node._parameters['iq_decoder_autotune_lock_enabled'] = False
    samples = np.zeros(1024, dtype=np.complex64)

    FlowgraphNode._decode_iq_snapshot(node, samples, 0, 2_000_000.0)
    FlowgraphNode._decode_iq_snapshot(node, samples, 1, 2_000_000.0)
    assert calls == ['autotune', 'autotune']
    assert node._iq_locked_candidate is None
