import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('risabot_v4_experimental')
    enabled = LaunchConfiguration('enabled')
    return LaunchDescription([
        DeclareLaunchArgument('enabled', default_value='false'),
        Node(package='risabot_v4_experimental', executable='parking_goal_source',
             name='v4_parking_goal_source', output='screen', parameters=[
                 os.path.join(share, 'config', 'v4_experimental.yaml'),
                 {'enabled': enabled, 'profile_path': os.path.join(share, 'config', 'camera_profiles.yaml')},
             ]),
    ])
