import os
import sys
from pathlib import Path


# ROS 2 Humble's colcon-python-setup-py injects a custom ``develop`` command
# with ``--editable`` and ``--uninstall`` options.  A newer setuptools in the
# user's site-packages shadows Humble's compatible system setuptools, silently
# drops that command extension, and makes an otherwise valid symlink build fail.
# Re-enter only that colcon develop invocation with user site-packages disabled;
# normal packaging, tests and runtime Python environments remain untouched.
if (
    os.environ.get('COLCON') == '1'
    and 'develop' in sys.argv
    and ('--editable' in sys.argv or '--uninstall' in sys.argv)
    and os.environ.get('RM_RADIO_SETUP_REEXEC') != '1'
):
    clean_env = os.environ.copy()
    clean_env['PYTHONNOUSERSITE'] = '1'
    clean_env['RM_RADIO_SETUP_REEXEC'] = '1'
    os.execve(sys.executable, [sys.executable, *sys.argv], clean_env)

from setuptools import find_packages, setup


package_name = 'rm_radio_ros'
package_dir = Path(__file__).resolve().parent

flowgraphs_root = package_dir / 'flowgraphs'
gfsk_dir = flowgraphs_root / 'gfsk'
ganrao_dir = flowgraphs_root / 'ganraoyuan'


def flowgraph_files(name, directory, filenames, hier_filenames=()):
    files = [directory / filename for filename in filenames]
    hier_files = [directory / 'Hier_Block' / filename for filename in hier_filenames]
    rel = lambda p: os.path.relpath(p, package_dir)
    return [
        (f'share/{package_name}/flowgraphs/{name}', [rel(p) for p in files if p.exists()]),
        (f'share/{package_name}/flowgraphs/{name}/Hier_Block', [rel(p) for p in hier_files if p.exists()]),
    ]


data_files = [
    ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
    (f'share/{package_name}', ['package.xml']),
    (f'share/{package_name}/launch', [
        'launch/gfsk.launch.py',
        'launch/ganraoyuan.launch.py',
        'launch/rx2.launch.py',
        'launch/referee_serial.launch.py',
    ]),
    (f'share/{package_name}/config', [
        'config/gfsk.yaml',
        'config/ganraoyuan.yaml',
        'config/referee_serial.yaml',
        'config/tx_content_broadcast_example.json',
        'config/tx_setters_interference_example.json',
    ]),
    (f'share/{package_name}/masks', [
        'assets/double_vulnerability_trigger_mask.npz',
    ]),
]
data_files.extend(flowgraph_files(
    'gfsk',
    gfsk_dir,
    ('RX.py', 'continuous_gfsk_demod.py'),
))
data_files.extend(flowgraph_files(
    'ganraoyuan',
    ganrao_dir,
    ('RM.py', 'jiang.py', 'EGO.grc'),
    ('untitled.grc',),
))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='RoboMaster Team',
    maintainer_email='team@example.com',
    description='ROS 2 Humble nodes wrapping RoboMaster GNU Radio SDR flowgraphs.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rm_flowgraph_node = rm_radio_ros.nodes.flowgraph_node:main',
            'rm_gfsk_node = rm_radio_ros.nodes.gfsk_node:main',
            'rm_ganraoyuan_node = rm_radio_ros.nodes.ganraoyuan_node:main',
            'rm_referee_serial_node = rm_radio_ros.nodes.referee_serial_node:main',
            'rm_offline_iq_scan = rm_radio_ros.core.offline_iq_scan:main',
            'rm_find_usb_serial = rm_radio_ros.core.usb_serial_finder:main',
            'rm_virtual_link_test = rm_radio_ros.core.virtual_link_test:main',
            'rm_match_dashboard = rm_radio_ros.app_support.match_dashboard_node:main',
            'rm_auto_password_node = rm_radio_ros.nodes.auto_password_node:main',
            'rm_radar_integration = rm_radio_ros.nodes.radar_integration_node:main',
            'rm_iq_replay_tx_node = rm_radio_ros.nodes.iq_replay_tx_node:main',
            'rm_match_recorder = rm_radio_ros.nodes.match_recorder_node:main',
            'rm_vision_udp_bridge = rm_radio_ros.nodes.vision_udp_bridge_node:main',
        ],
    },
)
