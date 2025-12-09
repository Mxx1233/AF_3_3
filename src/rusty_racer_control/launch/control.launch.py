import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # params_file = os.path.join(...)

    control_node = Node(
        package='rusty_racer_control',
        executable='control_node',
        name='control_node',
        output='screen'
        # parameters=[params_file]
    )
    return LaunchDescription([control_node])
