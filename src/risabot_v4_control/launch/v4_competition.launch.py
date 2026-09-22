"""One-command bringup for the legacy mission FSM with V4 lane authority."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package, launch_name, **arguments):
    path = os.path.join(get_package_share_directory(package), 'launch', launch_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path), launch_arguments=arguments.items())


def generate_launch_description():
    source = LaunchConfiguration('autonomy_source')
    enabled = LaunchConfiguration('v4_enabled')
    return LaunchDescription([
        DeclareLaunchArgument(
            'autonomy_source', default_value='legacy',
            description='Use v4 only after the Stage 8 status has no blockers'),
        DeclareLaunchArgument(
            'v4_enabled', default_value='true',
            description='Run V4 stages; physical gates still remain authoritative'),
        _include('risabot_automode', 'bringup.launch.py', autonomy_source=source),
        _include('risabot_v4_experimental', 'shadow.launch.py', enabled=enabled),
        # Stage 2 already launches its Stage 1 BEV dependency. Including the
        # standalone Stage 1 launch here creates two BEV processors with the
        # same node name and doubles the camera/CPU load.
        _include('risabot_v4_experimental', 'stage2_road_mask.launch.py', enabled=enabled),
        _include('risabot_v4_experimental', 'stage3_pose.launch.py', enabled=enabled),
        _include('risabot_v4_experimental', 'stage3_uwb_bridge.launch.py', enabled=enabled),
        _include('risabot_v4_experimental', 'stage4_trajectory.launch.py', enabled=enabled),
        _include('risabot_v4_experimental', 'stage5_goal_source.launch.py', enabled=enabled),
        _include('risabot_v4_experimental', 'stage5_parking.launch.py', enabled=enabled),
        _include('risabot_v4_experimental', 'stage6_request_source.launch.py', enabled=enabled),
        _include('risabot_v4_experimental', 'stage6_recovery.launch.py', enabled=enabled),
        _include('risabot_v4_experimental', 'stage7_arbitration.launch.py', enabled=enabled),
        _include('risabot_v4_control', 'stage8_control.launch.py', enabled=enabled),
    ])
