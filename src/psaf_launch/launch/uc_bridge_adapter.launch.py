import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('psaf_launch'), 'config', 'psaf_launch.yaml'
    )
    uc_bridge_adapter_node = Node(
        package='low_level_pkg',
        executable='uc_bridge_adapter_node',
        name='uc_bridge_adapter_node',
        parameters=[config_path],
        output='screen',
    )
    return LaunchDescription([uc_bridge_adapter_node])
