"""Primary-camera V4 lane pipeline with no motion authority.

This launch deliberately excludes parking, reverse recovery, the secondary
camera, and Stage 8.  It is the low-load pipeline used to prove V4 lane and
local-trajectory quality before any physical control trial.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('risabot_v4_experimental')
    params = os.path.join(share, 'config', 'v4_experimental.yaml')
    default_profiles = os.path.join(share, 'config', 'camera_profiles.yaml')
    enabled = LaunchConfiguration('enabled')
    # Per-car override: risabot5 (and any future chassis) keeps its measured
    # profiles in a board-local file and passes profile_path:=<file>.
    profiles = LaunchConfiguration('profile_path')

    common = [params, {'enabled': enabled}]
    return LaunchDescription([
        DeclareLaunchArgument(
            'enabled', default_value='false',
            description='Enable the read-only primary-camera V4 lane pipeline.'),
        DeclareLaunchArgument(
            'profile_path', default_value=default_profiles,
            description='Camera profiles YAML (board-local override per chassis).'),
        Node(
            package='risabot_v4_experimental', executable='shadow_monitor',
            name='v4_shadow_monitor', output='screen', parameters=[
                params, {'enabled': enabled, 'require_secondary_camera': False,
                         'require_uwb': False},
            ]),
        Node(
            package='risabot_v4_experimental', executable='bev_shadow',
            name='v4_bev_shadow', output='screen', parameters=[
                params, {'enabled': enabled, 'profile_path': profiles,
                         'process_secondary': False},
            ]),
        Node(
            package='risabot_v4_experimental', executable='road_mask_shadow',
            name='v4_road_mask_shadow', output='screen', parameters=[
                params, {'enabled': enabled, 'profile_path': profiles,
                         'process_secondary': False},
            ]),
        Node(
            package='risabot_v4_experimental', executable='pose_shadow',
            name='v4_pose_shadow', output='screen', parameters=common),
        Node(
            package='risabot_v4_experimental', executable='trajectory_shadow',
            name='v4_trajectory_shadow', output='screen', parameters=[
                params, {'enabled': enabled, 'profile_path': profiles},
            ]),
        Node(
            package='risabot_v4_experimental', executable='arbitration_shadow',
            name='v4_arbitration_shadow', output='screen', parameters=[
                params, {'enabled': enabled, 'lane_only': True},
            ]),
    ])
