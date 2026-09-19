from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare('rm_radio_ros'), 'config', 'referee_serial.yaml'])
    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')
    dry_run = LaunchConfiguration('dry_run')
    bridge_topic = LaunchConfiguration('bridge_topic')
    radar_cmd_topic = LaunchConfiguration('radar_cmd_topic')
    sender_id = LaunchConfiguration('sender_id')
    receiver_id = LaunchConfiguration('receiver_id')
    auto_send_radar_cmd = LaunchConfiguration('auto_send_radar_cmd')
    password_verify_cooldown_sec = LaunchConfiguration('password_verify_cooldown_sec')
    require_can_change_password_for_update = LaunchConfiguration('require_can_change_password_for_update')
    radar_client_max_rate_hz = LaunchConfiguration('radar_client_max_rate_hz')
    auto_send_invincible_targets = LaunchConfiguration('auto_send_invincible_targets')
    invincible_targets_data_cmd_id = LaunchConfiguration('invincible_targets_data_cmd_id')
    invincible_targets_send_rate_hz = LaunchConfiguration('invincible_targets_send_rate_hz')
    invincible_targets_freshness_sec = LaunchConfiguration('invincible_targets_freshness_sec')
    exclusive_port_lock = LaunchConfiguration('exclusive_port_lock')
    require_serial_open_on_start = LaunchConfiguration('require_serial_open_on_start')
    frame_timeout_sec = LaunchConfiguration('frame_timeout_sec')
    auto_interference_level = LaunchConfiguration('auto_interference_level')
    auto_rx_interference_level = LaunchConfiguration('auto_rx_interference_level')
    apply_referee_level_only_when_running = LaunchConfiguration('apply_referee_level_only_when_running')
    single_rx_auto_broadcast_after_level3 = LaunchConfiguration('single_rx_auto_broadcast_after_level3')
    single_rx_auto_allow_return_to_interference = LaunchConfiguration(
        'single_rx_auto_allow_return_to_interference'
    )
    referee_level_confirm_count = LaunchConfiguration('referee_level_confirm_count')
    radio_side = LaunchConfiguration('radio_side')
    auto_detect_radio_side = LaunchConfiguration('auto_detect_radio_side')
    interference_level = LaunchConfiguration('interference_level')
    interference_control_topic = LaunchConfiguration('interference_control_topic')
    interference_rx_control_topic = LaunchConfiguration('interference_rx_control_topic')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('bridge_topic', default_value='/rm_gfsk_node/referee_bridge'),
        DeclareLaunchArgument('radar_cmd_topic', default_value='/rm_radar_algorithm/radar_cmd'),
        DeclareLaunchArgument('sender_id', default_value='9'),
        DeclareLaunchArgument('receiver_id', default_value='32896'),
        DeclareLaunchArgument('auto_send_radar_cmd', default_value='true'),
        DeclareLaunchArgument('password_verify_cooldown_sec', default_value='10.0'),
        DeclareLaunchArgument('require_can_change_password_for_update', default_value='true'),
        DeclareLaunchArgument('radar_client_max_rate_hz', default_value='5.0'),
        DeclareLaunchArgument('auto_send_invincible_targets', default_value='true'),
        DeclareLaunchArgument('invincible_targets_data_cmd_id', default_value='564'),
        DeclareLaunchArgument('invincible_targets_send_rate_hz', default_value='3.0'),
        DeclareLaunchArgument('invincible_targets_freshness_sec', default_value='1.0'),
        DeclareLaunchArgument('exclusive_port_lock', default_value='true'),
        DeclareLaunchArgument('require_serial_open_on_start', default_value='false'),
        DeclareLaunchArgument('frame_timeout_sec', default_value='2.0'),
        DeclareLaunchArgument('auto_interference_level', default_value='false'),
        DeclareLaunchArgument('auto_rx_interference_level', default_value='true'),
        DeclareLaunchArgument('apply_referee_level_only_when_running', default_value='false'),
        DeclareLaunchArgument('single_rx_auto_broadcast_after_level3', default_value='false'),
        DeclareLaunchArgument('single_rx_auto_allow_return_to_interference', default_value='true'),
        DeclareLaunchArgument('referee_level_confirm_count', default_value='2'),
        DeclareLaunchArgument('radio_side', default_value='red'),
        DeclareLaunchArgument('auto_detect_radio_side', default_value='true'),
        DeclareLaunchArgument('interference_level', default_value='1'),
        DeclareLaunchArgument('interference_control_topic', default_value='/rm_ganraoyuan_node/setters_json'),
        DeclareLaunchArgument('interference_rx_control_topic', default_value='/rm_gfsk_interference_node/setters_json'),
        Node(
            package='rm_radio_ros',
            executable='rm_referee_serial_node',
            name='rm_referee_serial_node',
            output='screen',
            parameters=[params, {
                'port': ParameterValue(port, value_type=str),
                'baudrate': ParameterValue(baudrate, value_type=int),
                'dry_run': ParameterValue(dry_run, value_type=bool),
                'bridge_topic': ParameterValue(bridge_topic, value_type=str),
                'radar_cmd_topic': ParameterValue(radar_cmd_topic, value_type=str),
                'sender_id': ParameterValue(sender_id, value_type=int),
                'receiver_id': ParameterValue(receiver_id, value_type=int),
                'auto_send_radar_cmd': ParameterValue(auto_send_radar_cmd, value_type=bool),
                'password_verify_cooldown_sec': ParameterValue(password_verify_cooldown_sec, value_type=float),
                'require_can_change_password_for_update': ParameterValue(
                    require_can_change_password_for_update,
                    value_type=bool,
                ),
                'radar_client_max_rate_hz': ParameterValue(radar_client_max_rate_hz, value_type=float),
                'auto_send_invincible_targets': ParameterValue(auto_send_invincible_targets, value_type=bool),
                'invincible_targets_data_cmd_id': ParameterValue(invincible_targets_data_cmd_id, value_type=int),
                'invincible_targets_send_rate_hz': ParameterValue(
                    invincible_targets_send_rate_hz,
                    value_type=float,
                ),
                'invincible_targets_freshness_sec': ParameterValue(
                    invincible_targets_freshness_sec,
                    value_type=float,
                ),
                'exclusive_port_lock': ParameterValue(exclusive_port_lock, value_type=bool),
                'require_serial_open_on_start': ParameterValue(
                    require_serial_open_on_start,
                    value_type=bool,
                ),
                'frame_timeout_sec': ParameterValue(frame_timeout_sec, value_type=float),
                'auto_interference_level': ParameterValue(auto_interference_level, value_type=bool),
                'auto_rx_interference_level': ParameterValue(auto_rx_interference_level, value_type=bool),
                'apply_referee_level_only_when_running': ParameterValue(
                    apply_referee_level_only_when_running,
                    value_type=bool,
                ),
                'single_rx_auto_broadcast_after_level3': ParameterValue(
                    single_rx_auto_broadcast_after_level3,
                    value_type=bool,
                ),
                'single_rx_auto_allow_return_to_interference': ParameterValue(
                    single_rx_auto_allow_return_to_interference,
                    value_type=bool,
                ),
                'referee_level_confirm_count': ParameterValue(referee_level_confirm_count, value_type=int),
                'radio_side': ParameterValue(radio_side, value_type=str),
                'auto_detect_radio_side': ParameterValue(auto_detect_radio_side, value_type=bool),
                'interference_level': ParameterValue(interference_level, value_type=int),
                'interference_control_topic': ParameterValue(interference_control_topic, value_type=str),
                'interference_rx_control_topic': ParameterValue(interference_rx_control_topic, value_type=str),
            }],
            emulate_tty=True,
        ),
    ])
