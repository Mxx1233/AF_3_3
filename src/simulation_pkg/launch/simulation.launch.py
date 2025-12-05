from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # --- Start Fake Hardware Nodes ---
        Node(
            package='simulation_pkg',
            executable='fake_uc_bridge',
            name='uc_bridge',
            output='screen'
        ),
        Node(
            package='simulation_pkg',
            executable='fake_vision_node',
            name='vision_node',
            output='screen'
        ),

        # --- Start Your Application Nodes ---
        Node(
            package='low_level_pkg',
            executable='state_estimation_node',
            name='state_estimation_node',
            output='screen'
        ),
        Node(
            package='low_level_pkg',
            executable='uc_bridge_adapter_node',
            name='uc_bridge_adapter_node',
            output='screen'
        ),
        Node(
            package='rusty_racer_control',
            executable='control_node',
            name='control_node',
            output='screen'
        ),
    ])
