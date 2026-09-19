from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare('rm_radio_ros'), 'config', 'ganraoyuan.yaml'])
    dry_run = LaunchConfiguration('dry_run')
    flowgraph_dir = LaunchConfiguration('flowgraph_dir')
    setters_json = LaunchConfiguration('setters_json')
    qt_platform = LaunchConfiguration('qt_platform')
    disable_gui_sinks = LaunchConfiguration('disable_gui_sinks')
    qt_event_period_sec = LaunchConfiguration('qt_event_period_sec')
    interference_tx_uri = LaunchConfiguration('interference_tx_uri')
    sample_rate = LaunchConfiguration('sample_rate')
    radio_side = LaunchConfiguration('radio_side')
    interference_level = LaunchConfiguration('interference_level')

    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('flowgraph_dir', default_value=''),
        DeclareLaunchArgument('setters_json', default_value='{}'),
        DeclareLaunchArgument('qt_platform', default_value='offscreen'),
        DeclareLaunchArgument('disable_gui_sinks', default_value='true'),
        DeclareLaunchArgument('qt_event_period_sec', default_value='0.05'),
        DeclareLaunchArgument('interference_tx_uri', default_value='ip:192.168.3.1'),
        DeclareLaunchArgument('sample_rate', default_value='1000000'),
        DeclareLaunchArgument('radio_side', default_value='red'),
        DeclareLaunchArgument('interference_level', default_value='1'),
        Node(
            package='rm_radio_ros',
            executable='rm_ganraoyuan_node',
            name='rm_ganraoyuan_node',
            output='screen',
            parameters=[params, {
                'dry_run': ParameterValue(dry_run, value_type=bool),
                'flowgraph_dir': ParameterValue(flowgraph_dir, value_type=str),
                'qt_platform': ParameterValue(qt_platform, value_type=str),
                'disable_gui_sinks': ParameterValue(disable_gui_sinks, value_type=bool),
                'qt_event_period_sec': ParameterValue(qt_event_period_sec, value_type=float),
                'interference_tx_uri': ParameterValue(interference_tx_uri, value_type=str),
                'sample_rate': ParameterValue(sample_rate, value_type=float),
                'radio_side': ParameterValue(radio_side, value_type=str),
                'interference_level': ParameterValue(interference_level, value_type=int),
                'setters_json': ParameterValue(setters_json, value_type=str),
            }],
            emulate_tty=True,
        ),
    ])
