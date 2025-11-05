"""Start Intel Realsense D455 Depth Camera node."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('psaf_launch'),
                          'config', 'd455.yaml')

    return LaunchDescription([
        Node(
            name='realsense2_camera_node',
            package='realsense2_camera',
            executable='realsense2_camera_node',
            parameters=[config],
            output='screen'
        )
    ])
