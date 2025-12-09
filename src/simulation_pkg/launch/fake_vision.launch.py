import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # params_file = os.path.join(...)
    
    fake_vision_node = Node(
        package='simulation_pkg',
        executable='fake_vision_node',
        name='fake_vision_node',
        output='screen'
        # parameters=[params_file]
    )
    return LaunchDescription([fake_vision_node])
