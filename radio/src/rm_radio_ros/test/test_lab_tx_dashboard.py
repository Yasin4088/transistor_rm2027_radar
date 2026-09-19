import importlib.util
import json
import subprocess
import threading
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
MODULE_PATH = WORKSPACE_ROOT / 'apps/lab_tx/radio_tx_dashboard.py'
SPEC = importlib.util.spec_from_file_location('test_radio_tx_dashboard_module', MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
dashboard = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(dashboard)


class FakeRunningProcess:
    pid = 4242

    @staticmethod
    def poll():
        return None


def _server_without_socket():
    server = object.__new__(dashboard.RadioTxHttpServer)
    server.tx_lock = threading.RLock()
    server.tx_process = None
    server.tx_last_returncode = None
    server.dual_tx_process = None
    server.dual_tx_last_returncode = None
    server.dual_tx_log_path = None
    server.wave_processes = {'broadcast': None, 'interference': None}
    server.wave_last_state = {'broadcast': None, 'interference': None}
    server.wave_last_returncode = {'broadcast': None, 'interference': None}
    server.last_tx_state = None
    server.last_dual_tx_state = None
    server.tx_hardware_cache = {'uris': [], 'all_muted': None, 'results': []}
    server.tx_hardware_cache_time = 0.0
    return server


def test_generated_tx_ros_args_explicitly_include_uri_rate_and_setters():
    plan = {
        'side': 'red',
        'level': 2,
        'modulation': {'sample_rate': 1000000},
        'tx_setters': {'center_f': 433200000, 'Period': 100.0},
    }

    args, setters_json = dashboard._generated_tx_ros_args(
        plan,
        'ip:192.168.2.1',
        node_name='rm_broadcast_tx_node',
    )

    uri_arg = next(item for item in args if item.startswith('interference_tx_uri:='))
    setters_arg = next(item for item in args if item.startswith('setters_json:='))
    assert json.loads(uri_arg.split(':=', 1)[1]) == 'ip:192.168.2.1'
    assert 'sample_rate:=1000000.0' in args
    assert json.loads(setters_arg.split(':=', 1)[1]) == setters_json
    assert json.loads(setters_json) == plan['tx_setters']
    assert '__node:=rm_broadcast_tx_node' in args


def test_dashboard_defers_only_low_rate_tx_preinit_for_auto_fir():
    assert dashboard._tx_preinit_sample_rate_mode(1_000_000) == 'defer'
    assert dashboard._tx_preinit_sample_rate_mode(2_083_332) == 'defer'
    assert dashboard._tx_preinit_sample_rate_mode(2_083_333) == 'strict'
    assert dashboard._tx_preinit_sample_rate_mode(2_500_000) == 'strict'


def test_broadcast_auto_change_uses_randomized_payload_specs():
    state = dashboard.default_state()
    state['broadcast']['auto_change_data'] = True

    plan = dashboard.build_plan(state)
    cycle = plan['tx_setters']['command_cycle']

    assert [item['payload_size'] for item in cycle] == [24, 12, 10, 8, 41]
    assert all(item['randomize_payload'] is True for item in cycle)
    assert all('payload_data' not in item for item in cycle)


def test_broadcast_fixed_mode_keeps_edited_payload_data():
    state = dashboard.default_state()
    state['broadcast']['auto_change_data'] = False
    state['broadcast']['macro']['total_coins'] = 2000
    state['broadcast']['macro']['remaining_coins'] = 1234

    cycle = dashboard.build_plan(state)['tx_setters']['command_cycle']
    macro = next(item for item in cycle if item['cmd_id'] == [0x0A, 0x04])

    assert macro['payload_data'][:2] == [0xD2, 0x04]
    assert 'randomize_payload' not in macro


def test_randomize_broadcast_stays_inside_all_rulebook_ranges():
    state = dashboard.default_state()
    for _ in range(100):
        broadcast = dashboard.randomize_broadcast_config(state)
        for position in broadcast['robots'].values():
            assert 0 <= position['x'] <= dashboard.POSITION_X_MAX_CM
            assert 0 <= position['y'] <= dashboard.POSITION_Y_MAX_CM
        assert all(
            0 <= broadcast['hp'][key] <= maximum
            for key, maximum in dashboard.HP_MAX.items()
        )
        assert broadcast['hp']['reserved'] == 0
        assert all(
            0 <= broadcast['bullets'][key] <= maximum
            for key, maximum in dashboard.BULLET_MAX.items()
        )
        macro = broadcast['macro']
        assert 0 <= macro['remaining_coins'] <= macro['total_coins'] <= 0xFFFF
        assert macro['occupation_bits'] == dashboard.normalize_occupation_bits(macro['occupation_bits'])
        for key, item in broadcast['buffs'].items():
            for field, maxima in dashboard.BUFF_FIELD_MAX.items():
                assert 0 <= item[field] <= maxima[key]
        assert 1 <= broadcast['sentry_mode'] <= 6
        assert all(0 <= value <= 3 for value in broadcast['robot_main_status'].values())


def test_broadcast_config_clamps_direct_api_values_before_building_frames():
    state = dashboard.default_state()
    state['broadcast'].update(
        {
            'enabled_cmds': ['0x0A01', '0xDEAD'],
            'robots': {'opponent_hero': {'x': 99999, 'y': 99999}},
            'hp': {'opponent_engineer': 99999, 'reserved': 123},
            'bullets': {'opponent_aerial': 99999},
            'macro': {
                'remaining_coins': 500,
                'total_coins': 100,
                'occupation_bits': 0xFFFFFFFF,
            },
            'buffs': {
                'opponent_hero': {
                    'hp_recovery_percent': 999,
                    'shooting_heat_cooling': 999,
                    'defense_percent': 999,
                    'negative_defense_percent': 999,
                    'attack_percent': 999,
                },
            },
            'sentry_mode': 99,
            'robot_main_status': {'opponent_hero': 99},
            'buff_raw': [255] * 41,
        }
    )

    config = dashboard._broadcast_config(state)

    assert config['enabled_cmds'] == ['0x0A01']
    assert config['robots']['opponent_hero'] == {'x': 2800, 'y': 1500}
    assert config['hp']['opponent_engineer'] == 250
    assert config['hp']['reserved'] == 0
    assert config['bullets']['opponent_aerial'] == 750
    assert config['macro']['remaining_coins'] == config['macro']['total_coins'] == 100
    assert config['macro']['occupation_bits'] == dashboard.normalize_occupation_bits(0xFFFFFFFF)
    assert config['buffs']['opponent_hero'] == {
        field: maxima['opponent_hero']
        for field, maxima in dashboard.BUFF_FIELD_MAX.items()
    }
    assert config['sentry_mode'] == 6
    assert config['robot_main_status']['opponent_hero'] == 3
    assert 'buff_raw' not in config


def test_lab_tx_waveform_rate_is_fixed_to_official_value():
    state = dashboard.default_state()
    state['tx']['sample_rate'] = 9_999_999
    state['four_sdr']['broadcast_tx']['sample_rate'] = 9_999_999

    plan = dashboard.build_plan(state)
    config = dashboard._four_sdr_config(state)

    assert plan['modulation']['sample_rate'] == 1_000_000
    assert plan['modulation']['sps'] == 47
    assert config['broadcast_tx']['sample_rate'] == 1_000_000


def test_tx_start_confirmation_requires_loaded_stream_rate_sps_and_auto_fir(monkeypatch):
    status = {
        'confirmed': True,
        'topic': '/rm_broadcast_tx_node/status',
        'status': {
            'loaded': True,
            'started': True,
            'last_error': '',
            'tx_stream': {
                'mode': 'scheduler_clocked_continuous_bits',
                'configured_sample_rate_hz': 1_000_000,
                'samples_per_symbol': 47,
                'iio_filter_mode': 'Auto',
            },
        },
    }

    def completed(*_args, **_kwargs):
        return subprocess.CompletedProcess(
            args=['bash'],
            returncode=0,
            stdout='ROS startup noise\n' + json.dumps(status, ensure_ascii=False) + '\n',
            stderr='',
        )

    monkeypatch.setattr(dashboard.subprocess, 'run', completed)

    result = dashboard._wait_for_tx_node_started(
        'rm_broadcast_tx_node',
        1_000_000,
        47,
        timeout=1.0,
    )

    assert result['ok'] is True
    assert result['confirmation']['status']['tx_stream']['samples_per_symbol'] == 47


def test_tx_start_confirmation_rejects_pid_only_without_status(monkeypatch):
    monkeypatch.setattr(
        dashboard.subprocess,
        'run',
        lambda *_args, **_kwargs: subprocess.CompletedProcess(
            args=['bash'],
            returncode=2,
            stdout=json.dumps(
                {
                    'confirmed': False,
                    'reason': '未收到节点状态',
                    'status': {},
                },
                ensure_ascii=False,
            ),
            stderr='',
        ),
    )

    result = dashboard._wait_for_tx_node_started(
        'rm_broadcast_tx_node',
        1_000_000,
        47,
        timeout=1.0,
    )

    assert result['ok'] is False
    assert result['confirmation']['reason'] == '未收到节点状态'


def test_tx_process_detection_accepts_ps_rendering_of_python_c_argument():
    # ps flattens argv and does not preserve the quotes around Python's -c
    # source argument. The detector must therefore match the reconstructed
    # command text, not require the full source to remain one shlex token.
    command = (
        'python3 -c from rm_radio_ros.nodes.ganraoyuan_node import main; main() '
        '--ros-args -r __node:=rm_broadcast_tx_node '
        '-p interference_tx_uri:="ip:192.168.2.1"'
    )

    assert dashboard._is_tx_process_command(command) is True
    assert dashboard._tx_uri_from_process_command(command) == 'ip:192.168.2.1'


def test_dashboard_never_probes_managed_transmitting_sdr(monkeypatch):
    server = _server_without_socket()
    state = dashboard.default_state()
    state['four_sdr']['broadcast_tx']['uri'] = 'ip:192.168.2.1'
    server.wave_processes['broadcast'] = FakeRunningProcess()
    server.wave_last_state['broadcast'] = state
    monkeypatch.setattr(dashboard, '_detect_tx_processes', lambda: [])

    def forbidden_probe(*_args, **_kwargs):
        raise AssertionError('Dashboard opened IIO on a transmitting SDR')

    monkeypatch.setattr(dashboard, 'check_sdr_uri', forbidden_probe)

    result = server.check_sdr_uri_safely('ip:192.168.2.1')

    assert result['busy'] is True
    assert result['available'] is None


def test_four_sdr_check_skips_only_busy_tx_uri(monkeypatch):
    state = dashboard.default_state()
    state['four_sdr']['broadcast_tx']['uri'] = 'ip:192.168.2.1'
    state['four_sdr']['interference_tx']['uri'] = 'ip:192.168.3.1'
    probed = []

    def probe(uri):
        probed.append(uri)
        return {'uri': uri, 'available': True, 'message': 'ok', 'summary': []}

    monkeypatch.setattr(dashboard, 'check_sdr_uri', probe)

    result = dashboard.check_four_sdr(state, busy_tx_uris={'ip:192.168.2.1'})

    assert result['broadcast_tx']['busy'] is True
    assert probed == ['ip:192.168.3.1']


def test_four_sdr_check_skips_busy_role_even_when_config_uses_uri_alias(monkeypatch):
    state = dashboard.default_state()
    state['four_sdr']['broadcast_tx']['uri'] = 'usb:3.40.3'
    state['four_sdr']['interference_tx']['uri'] = 'ip:192.168.3.1'
    probed = []
    monkeypatch.setattr(
        dashboard,
        'check_sdr_uri',
        lambda uri: probed.append(uri) or {
            'uri': uri,
            'available': True,
            'message': 'ok',
            'summary': [],
        },
    )

    result = dashboard.check_four_sdr(
        state,
        busy_tx_uris={'ip:192.168.2.1'},
        busy_tx_roles={'broadcast_tx'},
    )

    assert result['broadcast_tx']['busy'] is True
    assert probed == ['ip:192.168.3.1']


def test_tx_hardware_status_does_not_read_busy_sdr(monkeypatch):
    server = _server_without_socket()
    server.tx_hardware_uris = lambda: ['ip:192.168.2.1', 'ip:192.168.3.1']
    server.busy_tx_uris = lambda: {'ip:192.168.2.1'}
    observed = []

    def hardware_status(uris):
        observed.extend(uris)
        return {
            'uris': list(uris),
            'all_muted': True,
            'results': [{'uri': uri, 'ok': True, 'muted': True} for uri in uris],
        }

    monkeypatch.setattr(dashboard, 'tx_hardware_status', hardware_status)

    result = server.refresh_tx_hardware_status()

    assert observed == ['ip:192.168.3.1']
    assert result['busy_uris'] == ['ip:192.168.2.1']
    assert result['all_muted'] is None
    busy = next(item for item in result['results'] if item['uri'] == 'ip:192.168.2.1')
    assert busy['busy'] is True
    assert busy['muted'] is None


def test_tx_log_diagnostics_counts_gr_iio_underflow_markers(tmp_path):
    log_path = tmp_path / 'tx.log'
    log_path.write_bytes(
        b'launcher=wave:interference\n'
        b'Upagesize : debug: Setting pagesize\n'
        b'[INFO] flowgraph started\n'
        b'UUU'
    )

    result = dashboard._tx_log_diagnostics(log_path)

    assert result['underflow_available'] is True
    assert result['underflow_count'] == 4
    assert result['underflow_detected'] is True
    assert result['sample_supply_mode'] == 'scheduler_clocked_continuous_bits'


def test_underflow_observation_distinguishes_active_stable_and_historical():
    observation = {}
    diagnostics = {
        'underflow_available': True,
        'underflow_count': 3,
        'underflow_detected': True,
        'log': '/tmp/tx.log',
    }

    active = dashboard._update_underflow_observation(
        diagnostics,
        observation,
        running=True,
        now_monotonic=10.0,
        stable_after_sec=5.0,
    )
    assert active['underflow_state'] == 'active'
    assert active['underflow_count_delta'] == 3

    stable = dashboard._update_underflow_observation(
        diagnostics,
        observation,
        running=True,
        now_monotonic=16.0,
        stable_after_sec=5.0,
    )
    assert stable['underflow_state'] == 'stable'
    assert stable['underflow_count_delta'] == 0
    assert stable['underflow_last_change_age_sec'] == 6.0

    increased = dashboard._update_underflow_observation(
        {**diagnostics, 'underflow_count': 4},
        observation,
        running=True,
        now_monotonic=17.0,
        stable_after_sec=5.0,
    )
    assert increased['underflow_state'] == 'active'
    assert increased['underflow_count_delta'] == 1

    historical = dashboard._update_underflow_observation(
        {**diagnostics, 'underflow_count': 4},
        observation,
        running=False,
        now_monotonic=30.0,
        stable_after_sec=5.0,
    )
    assert historical['underflow_state'] == 'historical'
