import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('risabot_v4_experimental')
    params = os.path.join(package_share, 'config', 'v4_experimental.yaml')
    profiles = os.path.join(package_share, 'config', 'camera_profiles.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enabled',
            default_value='false',
            description='Enable debug-only BEV generation; never grants motion authority.',
        ),
        Node(
            package='risabot_v4_experimental',
            executable='bev_shadow',
            name='v4_bev_shadow',
            output='screen',
            parameters=[
                params,
                {
                    'enabled': LaunchConfiguration('enabled'),
                    'profile_path': profiles,
                },
            ],
        ),
    ])
