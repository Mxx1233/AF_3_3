import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('psaf_launch'), 'config', 'psaf_launch.yaml'
    )
    detect_traffic_sign_node = Node(
        package='traffic_sign_yolo',
        executable='detect_traffic_sign_node',
        name='detect_traffic_sign_node',
        parameters=[config_path],
        output='screen',
    )
    return LaunchDescription([detect_traffic_sign_node])
