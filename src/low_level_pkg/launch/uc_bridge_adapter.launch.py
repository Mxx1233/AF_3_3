from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    uc_bridge_adapter_node = Node(
        package='low_level_pkg',
        executable='uc_bridge_adapter_node',
        name='uc_bridge_adapter_node',
        output='screen'
    )
    return LaunchDescription([uc_bridge_adapter_node])
