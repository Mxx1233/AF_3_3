from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    state_estimation_node = Node(
        package='low_level_pkg',
        executable='state_estimation_node',
        name='state_estimation_node',
        output='screen'
    )
    return LaunchDescription([state_estimation_node])
