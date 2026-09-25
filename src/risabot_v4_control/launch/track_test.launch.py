"""Single-owner front-camera lane test, starting in joystick MANUAL mode."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from risabot_v4_control.track_test_config import track_test_overrides
from risabot_v4_experimental.bev_core import load_profiles


def _setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        value = arg(name).lower()
        if value not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        return value == 'true'

    auto = get_package_share_directory('risabot_automode')
    v4 = get_package_share_directory('risabot_v4_experimental')
    control = get_package_share_directory('risabot_v4_control')
    vehicle = arg('vehicle')
    overrides = track_test_overrides(
        vehicle, float(arg('motor_duty')), float(arg('steering_gain')),
        float(arg('minimum_turn_duty')), float(arg('steering_slowdown_gain')),
        flag('enable_reverse_recovery'),
    )
    profile_path = arg('profile_path') or os.path.join(
        v4, 'config', 'risabot5_camera_profiles.yaml' if vehicle == 'risabot5' else 'camera_profiles.yaml')
    if not os.path.isfile(profile_path):
        raise RuntimeError(f'Camera profile missing: {profile_path}. Pass profile_path:=<this car\'s YAML>.')
    profiles = load_profiles(profile_path)
    primary = profiles['primary']
    if primary.name != 'primary':
        raise RuntimeError('A primary camera profile is required')
    for name in ('v4_bev_shadow', 'v4_road_mask_shadow', 'v4_trajectory_shadow'):
        overrides[name]['profile_path'] = profile_path
    if arg('parallel_recording'):
        overrides['servo_controller']['parallel_recording'] = arg('parallel_recording')
    auto_params = os.path.join(auto, 'config', 'params.yaml')
    v4_params = os.path.join(v4, 'config', 'v4_experimental.yaml')
    control_params = os.path.join(control, 'config', 'v4_control.yaml')
    actions = [
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', os.path.join(auto, 'config', 'disable_shm.xml')),
        LogInfo(msg=f'TRACK TEST: {vehicle}, motor duty {arg("motor_duty")}%, servo center {overrides["servo_controller"]["servo_center"]}; start in MANUAL.'),
        LogInfo(msg=f'Camera: {profile_path}; calibrated={primary.calibrated}; resolution={primary.resolution}.'),
    ]
    if flag('start_camera'):
        astra = get_package_share_directory('astra_camera')
        with open(os.path.join(astra, 'params', 'astra_mini_params.yaml'), encoding='utf-8') as stream:
            camera_params = yaml.safe_load(stream)
        camera_params.update(
            enable_point_cloud=False, enable_colored_point_cloud=False,
            enable_depth=False, enable_ir=False, enable_color=True,
            use_uvc_camera=True,
            color_width=int(primary.resolution[0]), color_height=int(primary.resolution[1]), color_fps=15,
        )
        actions.append(ComposableNodeContainer(
            name='astra_camera_container', namespace='', package='rclcpp_components',
            executable='component_container', output='screen',
            composable_node_descriptions=[ComposableNode(
                package='astra_camera', plugin='astra_camera::OBCameraNodeFactory',
                name='camera', namespace='camera', parameters=[camera_params])],
        ))
    if flag('start_lidar'):
        actions.append(Node(
            package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node', output='screen', parameters=[{
                'port': arg('lidar_port'), 'baudrate': 230400, 'frame_id': 'laser_frame',
                'lidar_type': 1, 'device_type': 0, 'sample_rate': 4,
                'support_motor_dtr': True, 'intensity': True,
                'angle_max': 180.0, 'angle_min': -180.0, 'range_max': 16.0,
                'range_min': 0.02, 'frequency': 10.0, 'fixed_resolution': True,
                'reversion': True, 'inverted': True, 'auto_reconnect': True,
                'isSingleChannel': False, 'invalid_range_is_inf': False, 'abnormal_check_count': 4,
            }],
        ))
        if vehicle == 'risabot1':
            actions.append(Node(
                package='risabot_automode', executable='tunnel_wall_follower',
                name='tunnel_wall_follower', output='screen',
                parameters=[auto_params, {
                    'tunnel_hysteresis_frames': 6,
                    'forward_speed': min(float(arg('motor_duty')), 55.0) / 255.0,
                    'lidar_angle_offset': 3.1416,
                }],
            ))
    for name, executable in (
        ('v4_bev_shadow', 'bev_shadow'), ('v4_road_mask_shadow', 'road_mask_shadow'),
        ('v4_pose_shadow', 'pose_shadow'), ('v4_trajectory_shadow', 'trajectory_shadow'),
        ('v4_arbitration_shadow', 'arbitration_shadow'),
    ):
        actions.append(Node(package='risabot_v4_experimental', executable=executable,
                            name=name, output='screen', parameters=[v4_params, overrides[name]]))
    actions.append(Node(package='risabot_v4_control', executable='motion_executor',
                        name='v4_motion_executor', output='screen',
                        parameters=[control_params, overrides['v4_motion_executor']]))
    for package, name in (
        ('risabot_automode', 'auto_driver'), ('risabot_automode', 'cmd_safety_controller'),
        ('control_servo', 'servo_controller'), ('risabot_automode', 'v4_telemetry_bridge'),
    ):
        actions.append(Node(package=package, executable=name, name=name, output='screen',
                            parameters=[auto_params, overrides.get(name, {})]))
    actions.append(Node(package='joy', executable='joy_node', name='joy_node',
                        parameters=[{'deadzone': 0.12, 'autorepeat_rate': 20.0, 'coalesce_interval_ms': 1}]))
    if flag('parallel_park'):
        # Sign classifier -> 2 s hold -> servo_controller 'park_sequence'
        # (wiggle x3, 5 s pause, recorded movement). Re-run: see params.yaml note.
        actions.append(Node(package='risabot_automode', executable='signage_detector',
                            name='signage_detector', output='screen', parameters=[auto_params]))
        actions.append(Node(package='risabot_automode', executable='parallel_park_trigger',
                            name='parallel_park_trigger', output='screen', parameters=[auto_params]))
    if flag('dashboard'):
        actions.append(Node(package='risabot_automode', executable='dashboard', name='dashboard',
                            output='screen', parameters=[auto_params]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('vehicle', default_value='risabot5'),
        DeclareLaunchArgument('motor_duty', default_value='65', description='Motor duty percent, not m/s.'),
        DeclareLaunchArgument('steering_gain', default_value='2.8'),
        DeclareLaunchArgument('minimum_turn_duty', default_value='48'),
        DeclareLaunchArgument('steering_slowdown_gain', default_value='0.85'),
        DeclareLaunchArgument('enable_reverse_recovery', default_value='false'),
        DeclareLaunchArgument('profile_path', default_value='', description='Camera YAML for this chassis.'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('parallel_park', default_value='true',
                              description='Run signage detector + parallel park trigger.'),
        DeclareLaunchArgument('parallel_recording', default_value='',
                              description='Saved recording name played after the wiggle/pause.'),
        DeclareLaunchArgument('dashboard', default_value='true'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'),
        OpaqueFunction(function=_setup),
    ])
