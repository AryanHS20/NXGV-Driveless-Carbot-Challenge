import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('risabot_v4_control')
    return LaunchDescription([
        DeclareLaunchArgument('enabled', default_value='false'),
        Node(
            package='risabot_v4_control', executable='motion_executor',
            name='v4_motion_executor', output='screen',
            parameters=[
                os.path.join(share, 'config', 'v4_control.yaml'),
                {'enabled': LaunchConfiguration('enabled')},
            ],
        ),
    ])
