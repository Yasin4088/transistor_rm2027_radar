from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare('rm_radio_ros'), 'config', 'gfsk.yaml'])
    dry_run = LaunchConfiguration('dry_run')
    qt_platform = LaunchConfiguration('qt_platform')
    disable_gui_sinks = LaunchConfiguration('disable_gui_sinks')
    qt_event_period_sec = LaunchConfiguration('qt_event_period_sec')
    radio_side = LaunchConfiguration('radio_side')
    rx1_uri = LaunchConfiguration('rx1_uri')
    rx2_uri = LaunchConfiguration('rx2_uri')
    interference_level = LaunchConfiguration('interference_level')
    flowgraph_dir = LaunchConfiguration('flowgraph_dir')
    setters_apply_settle_sec = LaunchConfiguration('setters_apply_settle_sec')
    broadcast_setters_json = LaunchConfiguration('broadcast_setters_json')
    interference_setters_json = LaunchConfiguration('interference_setters_json')
    broadcast_demod_mode = LaunchConfiguration('broadcast_demod_mode')
    interference_demod_mode = LaunchConfiguration('interference_demod_mode')
    broadcast_frontend_profile = LaunchConfiguration('broadcast_frontend_profile')
    interference_frontend_profile = LaunchConfiguration('interference_frontend_profile')
    broadcast_input_sps = LaunchConfiguration('broadcast_input_sps')
    interference_input_sps = LaunchConfiguration('interference_input_sps')
    air_extractor_max_access_hamming = LaunchConfiguration('air_extractor_max_access_hamming')
    air_extractor_allow_inverted = LaunchConfiguration('air_extractor_allow_inverted')
    iq_decoder_autotune_mode = LaunchConfiguration('iq_decoder_autotune_mode')
    iq_decoder_max_candidates = LaunchConfiguration('iq_decoder_max_candidates')
    spectrum_enabled = LaunchConfiguration('spectrum_enabled')
    spectrum_period_sec = LaunchConfiguration('spectrum_period_sec')
    spectrum_fft_size = LaunchConfiguration('spectrum_fft_size')
    spectrum_bin_count = LaunchConfiguration('spectrum_bin_count')
    iio_capture_decoder_enabled = LaunchConfiguration('iio_capture_decoder_enabled')
    iio_capture_decoder_only = LaunchConfiguration('iio_capture_decoder_only')
    iio_capture_seconds = LaunchConfiguration('iio_capture_seconds')
    iio_capture_period_sec = LaunchConfiguration('iio_capture_period_sec')
    iio_capture_start_delay_sec = LaunchConfiguration('iio_capture_start_delay_sec')
    iq_decoder_low_pass_hz = LaunchConfiguration('iq_decoder_low_pass_hz')
    broadcast_iq_decoder_autotune_mode = LaunchConfiguration('broadcast_iq_decoder_autotune_mode')
    broadcast_iq_decoder_enabled = LaunchConfiguration('broadcast_iq_decoder_enabled')
    broadcast_iq_decoder_low_pass_hz = LaunchConfiguration('broadcast_iq_decoder_low_pass_hz')
    broadcast_iq_decoder_sps_values = LaunchConfiguration('broadcast_iq_decoder_sps_values')
    broadcast_iq_decoder_offset_step = LaunchConfiguration('broadcast_iq_decoder_offset_step')
    broadcast_iq_decoder_max_payloads = LaunchConfiguration('broadcast_iq_decoder_max_payloads')
    broadcast_iq_decoder_max_frames = LaunchConfiguration('broadcast_iq_decoder_max_frames')
    interference_iq_decoder_autotune_mode = LaunchConfiguration('interference_iq_decoder_autotune_mode')
    interference_iq_decoder_enabled = LaunchConfiguration('interference_iq_decoder_enabled')
    interference_iq_decoder_low_pass_hz = LaunchConfiguration('interference_iq_decoder_low_pass_hz')
    recording_enabled = LaunchConfiguration('recording_enabled')
    recording_root = LaunchConfiguration('recording_root')
    recording_control_topic = LaunchConfiguration('recording_control_topic')
    recording_output_sample_rate = LaunchConfiguration('recording_output_sample_rate')
    recording_segment_sec = LaunchConfiguration('recording_segment_sec')
    recording_queue_chunks = LaunchConfiguration('recording_queue_chunks')
    recording_min_free_gb = LaunchConfiguration('recording_min_free_gb')
    recording_iq_compression = LaunchConfiguration('recording_iq_compression')
    recording_iq_compression_level = LaunchConfiguration('recording_iq_compression_level')

    common_params = {
        'flowgraph_name': 'gfsk',
        'flowgraph_module': 'RX',
        'flowgraph_class': 'RadioRx',
        'dry_run': ParameterValue(dry_run, value_type=bool),
        'qt_platform': ParameterValue(qt_platform, value_type=str),
        'disable_gui_sinks': ParameterValue(disable_gui_sinks, value_type=bool),
        'rx_only': True,
        'qt_event_period_sec': ParameterValue(qt_event_period_sec, value_type=float),
        'radio_side': ParameterValue(radio_side, value_type=str),
        'interference_level': ParameterValue(interference_level, value_type=int),
        'flowgraph_dir': ParameterValue(flowgraph_dir, value_type=str),
        'setters_apply_settle_sec': ParameterValue(setters_apply_settle_sec, value_type=float),
        'air_extractor_max_access_hamming': ParameterValue(air_extractor_max_access_hamming, value_type=int),
        'air_extractor_allow_inverted': ParameterValue(air_extractor_allow_inverted, value_type=bool),
        'iq_decoder_autotune_mode': ParameterValue(iq_decoder_autotune_mode, value_type=str),
        'iq_decoder_max_candidates': ParameterValue(iq_decoder_max_candidates, value_type=int),
        'iq_decoder_low_pass_hz': ParameterValue(iq_decoder_low_pass_hz, value_type=float),
        'spectrum_enabled': ParameterValue(spectrum_enabled, value_type=bool),
        'spectrum_period_sec': ParameterValue(spectrum_period_sec, value_type=float),
        'spectrum_fft_size': ParameterValue(spectrum_fft_size, value_type=int),
        'spectrum_bin_count': ParameterValue(spectrum_bin_count, value_type=int),
        'recording_enabled': ParameterValue(recording_enabled, value_type=bool),
        'recording_root': ParameterValue(recording_root, value_type=str),
        'recording_control_topic': ParameterValue(recording_control_topic, value_type=str),
        'recording_output_sample_rate': ParameterValue(
            recording_output_sample_rate,
            value_type=float,
        ),
        'recording_segment_sec': ParameterValue(recording_segment_sec, value_type=float),
        'recording_queue_chunks': ParameterValue(recording_queue_chunks, value_type=int),
        'recording_min_free_gb': ParameterValue(recording_min_free_gb, value_type=float),
        'recording_iq_compression': ParameterValue(recording_iq_compression, value_type=str),
        'recording_iq_compression_level': ParameterValue(
            recording_iq_compression_level,
            value_type=int,
        ),
    }

    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('qt_platform', default_value='offscreen'),
        DeclareLaunchArgument('disable_gui_sinks', default_value='true'),
        DeclareLaunchArgument('qt_event_period_sec', default_value='0.05'),
        DeclareLaunchArgument('radio_side', default_value='red'),
        DeclareLaunchArgument('rx1_uri', default_value='ip:192.168.2.1'),
        DeclareLaunchArgument('rx2_uri', default_value='ip:192.168.3.1'),
        DeclareLaunchArgument('interference_level', default_value='1'),
        DeclareLaunchArgument('flowgraph_dir', default_value=''),
        DeclareLaunchArgument('setters_apply_settle_sec', default_value='0.15'),
        DeclareLaunchArgument('broadcast_setters_json', default_value='{}'),
        DeclareLaunchArgument('interference_setters_json', default_value='{}'),
        DeclareLaunchArgument('broadcast_demod_mode', default_value='mm'),
        DeclareLaunchArgument('interference_demod_mode', default_value='legacy'),
        DeclareLaunchArgument('broadcast_frontend_profile', default_value='broadcast'),
        DeclareLaunchArgument('interference_frontend_profile', default_value='interference'),
        DeclareLaunchArgument('broadcast_input_sps', default_value='94'),
        DeclareLaunchArgument('interference_input_sps', default_value='94'),
        DeclareLaunchArgument('air_extractor_max_access_hamming', default_value='3'),
        DeclareLaunchArgument('air_extractor_allow_inverted', default_value='true'),
        DeclareLaunchArgument('iq_decoder_autotune_mode', default_value='quick'),
        DeclareLaunchArgument('iq_decoder_max_candidates', default_value='8'),
        DeclareLaunchArgument('spectrum_enabled', default_value='true'),
        DeclareLaunchArgument('spectrum_period_sec', default_value='0.5'),
        DeclareLaunchArgument('spectrum_fft_size', default_value='512'),
        DeclareLaunchArgument('spectrum_bin_count', default_value='128'),
        DeclareLaunchArgument('iio_capture_decoder_enabled', default_value='false'),
        DeclareLaunchArgument('iio_capture_decoder_only', default_value='false'),
        DeclareLaunchArgument('iio_capture_seconds', default_value='1.2'),
        DeclareLaunchArgument('iio_capture_period_sec', default_value='1.5'),
        DeclareLaunchArgument('iio_capture_start_delay_sec', default_value='0.0'),
        DeclareLaunchArgument('iq_decoder_low_pass_hz', default_value='0.0'),
        DeclareLaunchArgument('broadcast_iq_decoder_autotune_mode', default_value=iq_decoder_autotune_mode),
        DeclareLaunchArgument('broadcast_iq_decoder_enabled', default_value='false'),
        DeclareLaunchArgument('broadcast_iq_decoder_low_pass_hz', default_value=iq_decoder_low_pass_hz),
        DeclareLaunchArgument('broadcast_iq_decoder_sps_values', default_value='92,92.5,93,93.5,94,94.5,95'),
        DeclareLaunchArgument('broadcast_iq_decoder_offset_step', default_value='8'),
        DeclareLaunchArgument('broadcast_iq_decoder_max_payloads', default_value='256'),
        DeclareLaunchArgument('broadcast_iq_decoder_max_frames', default_value='64'),
        DeclareLaunchArgument('interference_iq_decoder_autotune_mode', default_value=iq_decoder_autotune_mode),
        DeclareLaunchArgument('interference_iq_decoder_enabled', default_value='true'),
        DeclareLaunchArgument('interference_iq_decoder_low_pass_hz', default_value=iq_decoder_low_pass_hz),
        DeclareLaunchArgument('recording_enabled', default_value='false'),
        DeclareLaunchArgument(
            'recording_root',
            default_value='~/.local/share/shark-radio/match-records',
        ),
        DeclareLaunchArgument(
            'recording_control_topic',
            default_value='/rm_match_recorder/control',
        ),
        DeclareLaunchArgument('recording_output_sample_rate', default_value='1000000'),
        DeclareLaunchArgument('recording_segment_sec', default_value='30'),
        DeclareLaunchArgument('recording_queue_chunks', default_value='64'),
        DeclareLaunchArgument('recording_min_free_gb', default_value='15'),
        DeclareLaunchArgument('recording_iq_compression', default_value='zlib'),
        DeclareLaunchArgument('recording_iq_compression_level', default_value='1'),
        Node(
            package='rm_radio_ros',
            executable='rm_gfsk_node',
            name='rm_gfsk_node',
            output='screen',
            # libiio aborts the receiver process when an Ethernet SDR disappears.
            # Keep the launch service alive and rebuild the whole IIO context after
            # the device becomes reachable again; a stopped source cannot reconnect
            # by reusing its old context.
            respawn=True,
            respawn_delay=2.0,
            parameters=[params, {
                **common_params,
                'rx_uri': ParameterValue(rx1_uri, value_type=str),
                'rx_profile': 'broadcast',
                'frontend_profile': ParameterValue(broadcast_frontend_profile, value_type=str),
                'demod_mode': ParameterValue(broadcast_demod_mode, value_type=str),
                'input_sps': ParameterValue(broadcast_input_sps, value_type=float),
                'iq_decoder_enabled': ParameterValue(broadcast_iq_decoder_enabled, value_type=bool),
                'iio_capture_decoder_enabled': ParameterValue(iio_capture_decoder_enabled, value_type=bool),
                'iio_capture_decoder_only': ParameterValue(iio_capture_decoder_only, value_type=bool),
                'iio_capture_seconds': ParameterValue(iio_capture_seconds, value_type=float),
                'iio_capture_period_sec': ParameterValue(iio_capture_period_sec, value_type=float),
                'iio_capture_start_delay_sec': ParameterValue(iio_capture_start_delay_sec, value_type=float),
                'iq_decoder_autotune_mode': ParameterValue(broadcast_iq_decoder_autotune_mode, value_type=str),
                'iq_decoder_low_pass_hz': ParameterValue(broadcast_iq_decoder_low_pass_hz, value_type=float),
                'iq_decoder_sps_values': ParameterValue(broadcast_iq_decoder_sps_values, value_type=str),
                'iq_decoder_offset_step': ParameterValue(broadcast_iq_decoder_offset_step, value_type=int),
                'iq_decoder_max_payloads': ParameterValue(broadcast_iq_decoder_max_payloads, value_type=int),
                'iq_decoder_max_frames': ParameterValue(broadcast_iq_decoder_max_frames, value_type=int),
                'recording_role': 'rx1',
                'setters_json': ParameterValue(broadcast_setters_json, value_type=str),
            }],
            emulate_tty=True,
        ),
        Node(
            package='rm_radio_ros',
            executable='rm_gfsk_node',
            name='rm_gfsk_interference_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[params, {
                **common_params,
                'rx_uri': ParameterValue(rx2_uri, value_type=str),
                'rx_profile': 'interference',
                'frontend_profile': ParameterValue(interference_frontend_profile, value_type=str),
                'demod_mode': ParameterValue(interference_demod_mode, value_type=str),
                'input_sps': ParameterValue(interference_input_sps, value_type=float),
                'iq_decoder_enabled': ParameterValue(interference_iq_decoder_enabled, value_type=bool),
                'iq_decoder_autotune_mode': ParameterValue(interference_iq_decoder_autotune_mode, value_type=str),
                'iq_decoder_low_pass_hz': ParameterValue(interference_iq_decoder_low_pass_hz, value_type=float),
                'recording_role': 'rx2',
                'setters_json': ParameterValue(interference_setters_json, value_type=str),
            }],
            emulate_tty=True,
        ),
    ])
