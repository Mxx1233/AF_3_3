import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('psaf_launch'), 'config', 'psaf_launch.yaml'
    )
    state_estimation_node = Node(
        package='low_level_pkg',
        executable='state_estimation_node',
        name='state_estimation_node',
        parameters=[config_path],
        output='screen',
    )
    return LaunchDescription([state_estimation_node])
