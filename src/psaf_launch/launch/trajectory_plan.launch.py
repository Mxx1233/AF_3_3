import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('psaf_launch'), 'config', 'psaf_launch.yaml'
    )

    return LaunchDescription(
        [
            Node(
                package='psaf_trajectory_plan',
                executable='trajectory_plan',
                name='trajectory_plan',
                parameters=[config_path],
                output='log',
            )
        ]
    )
