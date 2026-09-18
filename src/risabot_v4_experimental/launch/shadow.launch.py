import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('risabot_v4_experimental')
    params = os.path.join(package_share, 'config', 'v4_experimental.yaml')
    enabled = LaunchConfiguration('enabled')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enabled',
            default_value='false',
            description='Enable read-only V4 shadow evaluation; never grants motion authority.',
        ),
        Node(
            package='risabot_v4_experimental',
            executable='shadow_monitor',
            name='v4_shadow_monitor',
            output='screen',
            parameters=[params, {'enabled': enabled}],
        ),
    ])
