from setuptools import setup
import os
from glob import glob

package_name = 'risabot_automode'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '.dashboard_panels'],
    package_data={package_name: [
        'dashboard_panels/shell/*.html',
        'dashboard_panels/left/*/*.html',
        'dashboard_panels/center/*/*.html',
        'dashboard_panels/right/*/*.html',
        'dashboard_panels/flow/*.html',
        'dashboard_panels/eventlog/*.html',
        'dashboard_panels/paramdrawer/*.html',
        'dashboard_panels/ctrldrawer/*.html',
        'dashboard_panels/script/*.js',
        'dashboard_panels/teach/*.html',
    ]},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # ADD THIS LINE BELOW:
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.xml')),
        (os.path.join('share', package_name, 'sim_views'), glob('sim_views/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@todo.todo',
    description='RisaBot Auto Mode Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # The Entry Point: Name = Package.File:Function
            'auto_driver = risabot_automode.auto_driver:main',
            'line_follower_camera = risabot_automode.line_follower_camera:main',
            'traffic_light_detector = risabot_automode.traffic_light_detector:main',
            'boom_gate_detector = risabot_automode.boom_gate_detector:main',
            'parking_controller = risabot_automode.parking_controller:main',
            'tunnel_wall_follower = risabot_automode.tunnel_wall_follower:main',
            'obstruction_avoidance = risabot_automode.obstruction_avoidance:main',
            'dashboard = risabot_automode.dashboard:main',
            'health_monitor = risabot_automode.health_monitor:main',
            'cmd_safety_controller = risabot_automode.cmd_safety_controller:main',
            'bag_regression_validator = risabot_automode.bag_regression_validator:main',
            'signage_detector = risabot_automode.signage_detector:main',
            'heading_fusion = risabot_automode.heading_fusion:main',
            'map_recorder = risabot_automode.map_recorder:main',
            'mipi_relay = risabot_automode.mipi_relay:main',
            'side_camera_manager = risabot_automode.side_camera_manager:main',
            'v4_telemetry_bridge = risabot_automode.v4_telemetry_bridge:main',
        ],
    },
)
