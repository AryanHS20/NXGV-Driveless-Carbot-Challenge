import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    risabot_pkg = get_package_share_directory('risabot_automode')
    sim_pkg = get_package_share_directory('risabot_sim')
    
    # We use the main params.yaml from risabot_automode.
    # Can merge with sim_params.yaml if needed later.
    params_file = os.path.join(risabot_pkg, 'config', 'params.yaml')

    return LaunchDescription([
        # 1. Sim Servo Bridge (replaces real servo controller, publishes /auto_mode and handles joy toggles)
        Node(
            package='risabot_sim',
            executable='sim_servo_bridge',
            name='sim_servo_bridge',
            output='screen'
        ),

        # 2. IMU Converter (converts Gazebo's /imu/data_raw to /imu/rpy JSON format expected by nodes)
        Node(
            package='risabot_sim',
            executable='imu_converter',
            name='imu_converter',
            output='screen'
        ),

        # ==================== PERCEPTION ====================

        # 3. LiDAR obstacle detection
        Node(
            package='obstacle_avoidance',
            executable='obstacle_avoidance',
            name='obstacle_avoidance_node',
            output='screen',
            parameters=[params_file]
        ),

        # 4. Camera obstacle detection
        Node(
            package='obstacle_avoidance_camera',
            executable='obstacle_avoidance_camera',
            name='obstacle_avoidance_camera',
            output='screen',
            parameters=[params_file]
        ),

        # 5. Line follower camera (lane tracking)
        Node(
            package='risabot_automode',
            executable='line_follower_camera',
            name='line_follower_camera',
            output='screen',
            parameters=[params_file]
        ),

        # 6. Tunnel wall follower
        Node(
            package='risabot_automode',
            executable='tunnel_wall_follower',
            name='tunnel_wall_follower',
            output='screen',
            parameters=[params_file]
        ),

        # 7. Heading & Odometry Fusion (consumes JSON /imu/rpy + /odom)
        Node(
            package='risabot_automode',
            executable='heading_fusion',
            name='heading_fusion',
            output='screen',
            parameters=[params_file]
        ),

        # 8. Dynamic VFH+ Obstruction Avoidance
        Node(
            package='risabot_automode',
            executable='obstruction_avoidance',
            name='obstruction_avoidance',
            output='screen',
            parameters=[params_file]
        ),

        # 9. Closed-Loop Parking Controller
        Node(
            package='risabot_automode',
            executable='parking_controller',
            name='parking_controller',
            output='screen',
            parameters=[params_file]
        ),

        # ==================== CONTROL ====================

        # 10. Auto Driver (brain - coordinates challenges)
        Node(
            package='risabot_automode',
            executable='auto_driver',
            name='auto_driver',
            output='screen',
            parameters=[params_file]
        ),

        # 11. Command safety controller (emits final /cmd_vel that Gazebo Ackermann plugin consumes)
        Node(
            package='risabot_automode',
            executable='cmd_safety_controller',
            name='cmd_safety_controller',
            output='screen',
            parameters=[params_file]
        ),
    ])
