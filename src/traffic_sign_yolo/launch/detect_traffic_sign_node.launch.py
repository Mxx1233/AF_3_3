from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    state_estimation_node = Node(
        package='traffic_sgin_yolo',
        executable='detect_traffic_sign_node',
        name='detect_traffic_sign_node',
        output='screen'
    )
    return LaunchDescription([detect_traffic_sign_node])