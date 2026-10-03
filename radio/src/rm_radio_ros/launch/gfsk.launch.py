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
    setters_json = LaunchConfiguration('setters_json')
    qt_platform = LaunchConfiguration('qt_platform')
    disable_gui_sinks = LaunchConfiguration('disable_gui_sinks')
    rx_only = LaunchConfiguration('rx_only')
    qt_event_period_sec = LaunchConfiguration('qt_event_period_sec')
    rx_uri = LaunchConfiguration('rx_uri')
    broadcast_tx_uri = LaunchConfiguration('broadcast_tx_uri')
    radio_side = LaunchConfiguration('radio_side')
    rx_profile = LaunchConfiguration('rx_profile')
    interference_level = LaunchConfiguration('interference_level')
    setters_apply_settle_sec = LaunchConfiguration('setters_apply_settle_sec')
    air_extractor_max_access_hamming = LaunchConfiguration('air_extractor_max_access_hamming')
    air_extractor_allow_inverted = LaunchConfiguration('air_extractor_allow_inverted')
    iq_decoder_autotune_mode = LaunchConfiguration('iq_decoder_autotune_mode')
    iq_decoder_max_candidates = LaunchConfiguration('iq_decoder_max_candidates')
    spectrum_enabled = LaunchConfiguration('spectrum_enabled')
    spectrum_period_sec = LaunchConfiguration('spectrum_period_sec')
    spectrum_fft_size = LaunchConfiguration('spectrum_fft_size')
    spectrum_bin_count = LaunchConfiguration('spectrum_bin_count')

    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('setters_json', default_value='{}'),
        DeclareLaunchArgument('qt_platform', default_value='offscreen'),
        DeclareLaunchArgument('disable_gui_sinks', default_value='true'),
        DeclareLaunchArgument('rx_only', default_value='true'),
        DeclareLaunchArgument('qt_event_period_sec', default_value='0.05'),
        DeclareLaunchArgument('rx_uri', default_value='ip:192.168.2.1'),
        DeclareLaunchArgument('broadcast_tx_uri', default_value='ip:192.168.1.10'),
        DeclareLaunchArgument('radio_side', default_value='red'),
        DeclareLaunchArgument('rx_profile', default_value='broadcast'),
        DeclareLaunchArgument('interference_level', default_value='1'),
        DeclareLaunchArgument('setters_apply_settle_sec', default_value='0.15'),
        DeclareLaunchArgument('air_extractor_max_access_hamming', default_value='3'),
        DeclareLaunchArgument('air_extractor_allow_inverted', default_value='true'),
        DeclareLaunchArgument('iq_decoder_autotune_mode', default_value='quick'),
        DeclareLaunchArgument('iq_decoder_max_candidates', default_value='8'),
        DeclareLaunchArgument('spectrum_enabled', default_value='true'),
        DeclareLaunchArgument('spectrum_period_sec', default_value='0.5'),
        DeclareLaunchArgument('spectrum_fft_size', default_value='512'),
        DeclareLaunchArgument('spectrum_bin_count', default_value='128'),
        Node(
            package='rm_radio_ros',
            executable='rm_gfsk_node',
            name='rm_gfsk_node',
            output='screen',
            parameters=[params, {
                'dry_run': ParameterValue(dry_run, value_type=bool),
                'qt_platform': ParameterValue(qt_platform, value_type=str),
                'disable_gui_sinks': ParameterValue(disable_gui_sinks, value_type=bool),
                'rx_only': ParameterValue(rx_only, value_type=bool),
                'qt_event_period_sec': ParameterValue(qt_event_period_sec, value_type=float),
                'rx_uri': ParameterValue(rx_uri, value_type=str),
                'broadcast_tx_uri': ParameterValue(broadcast_tx_uri, value_type=str),
                'radio_side': ParameterValue(radio_side, value_type=str),
                'rx_profile': ParameterValue(rx_profile, value_type=str),
                'interference_level': ParameterValue(interference_level, value_type=int),
                'setters_apply_settle_sec': ParameterValue(setters_apply_settle_sec, value_type=float),
                'air_extractor_max_access_hamming': ParameterValue(air_extractor_max_access_hamming, value_type=int),
                'air_extractor_allow_inverted': ParameterValue(air_extractor_allow_inverted, value_type=bool),
                'iq_decoder_autotune_mode': ParameterValue(iq_decoder_autotune_mode, value_type=str),
                'iq_decoder_max_candidates': ParameterValue(iq_decoder_max_candidates, value_type=int),
                'spectrum_enabled': ParameterValue(spectrum_enabled, value_type=bool),
                'spectrum_period_sec': ParameterValue(spectrum_period_sec, value_type=float),
                'spectrum_fft_size': ParameterValue(spectrum_fft_size, value_type=int),
                'spectrum_bin_count': ParameterValue(spectrum_bin_count, value_type=int),
                'setters_json': ParameterValue(setters_json, value_type=str),
            }],
            emulate_tty=True,
        ),
    ])
