"""Start UC-Board Bridge for exchanging data on the psaf 2 car."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('psaf_launch'),
                          'config',
                          'parameters_new.yml')

    return LaunchDescription([
        Node(
            package='psaf_ucbridge',
            executable='uc_bridge',
            name='uc_bridge',
            parameters=[config],
            output='screen'
        )
    ])
