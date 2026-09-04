import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_risabot_sim = get_package_share_directory('risabot_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Path to xacro file
    xacro_file = os.path.join(pkg_risabot_sim, 'urdf', 'risabot.urdf.xacro')
    # Path to world file
    world_file = os.path.join(pkg_risabot_sim, 'worlds', 'nxgv_track.sdf')

    # Gazebo simulation
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}' # Removed -s so GUI and GPU sensors load
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]),
            'use_sim_time': True
        }]
    )

    # Spawn robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'risabot',
                   '-z', '0.1']
    )

    # ROS <-> Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/camera/color/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/imu/data_raw@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', pkg_risabot_sim),
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        bridge
    ])
