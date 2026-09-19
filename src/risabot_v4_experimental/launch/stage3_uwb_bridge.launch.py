import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('risabot_v4_experimental')
    params = os.path.join(package_share, 'config', 'uwb.yaml')
    enabled = LaunchConfiguration('enabled')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enabled', default_value='false',
            description='Enable diagnostic-only raw-range UWB bridge.',
        ),
        Node(
            package='risabot_v4_experimental',
            executable='uwb_bridge_shadow',
            name='v4_uwb_bridge_shadow',
            output='screen',
            parameters=[params, {'enabled': enabled}],
        ),
    ])
