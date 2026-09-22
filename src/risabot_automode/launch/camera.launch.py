"""Launch the Astra camera without unused point-cloud components."""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    share = get_package_share_directory('astra_camera')
    mini_params = os.path.join(share, 'params', 'astra_mini_params.yaml')
    if os.path.exists(mini_params):
        with open(mini_params, encoding='utf-8') as stream:
            params = yaml.safe_load(stream)
        # Preserve the board's camera settings. The car uses RGB images and
        # LiDAR; it has no subscribers requiring the optional point clouds.
        params.update(enable_point_cloud=False, enable_colored_point_cloud=False)
        return LaunchDescription([ComposableNodeContainer(
            name='astra_camera_container', namespace='',
            package='rclcpp_components', executable='component_container',
            composable_node_descriptions=[ComposableNode(
                package='astra_camera', plugin='astra_camera::OBCameraNodeFactory',
                name='camera', namespace='camera', parameters=[params])],
            output='screen')])
    return LaunchDescription([IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(share, 'launch', 'astra_pro.launch.xml')),
        launch_arguments={'enable_point_cloud': 'false',
                          'enable_colored_point_cloud': 'false'}.items())])
