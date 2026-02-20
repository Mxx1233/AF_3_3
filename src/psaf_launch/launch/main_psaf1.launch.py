"""Start the whole pipeline with all Nodes for the old car."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


__ucbridge = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('psaf_launch'),
            'launch',
            'ucbridge_old.launch.py')))
            
__realsense2_camera = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
       os.path.join(
            get_package_share_directory('psaf_launch'),
            'launch',
            'realsense2_camera_455.launch.py')))


__state_estimation = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(get_package_share_directory('low_level_pkg'), 'launch',
                 'state_estimation.launch.py')
))

__uc_bridge_adapter = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(get_package_share_directory('low_level_pkg'), 'launch',
                 'uc_bridge_adapter.launch.py')
))

__control = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(get_package_share_directory('rusty_racer_control'), 'launch',
                 'control.launch.py')
))

__detect_traffic_sign_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(get_package_share_directory('traffic_sign_yolo'), 'launch',
                 'detect_traffic_sign_node.launch.py')
))


def generate_launch_description():

    # record_arg = DeclareLaunchArgument(
    #     'record',
    #     default_value='false',
    #     description='Set to "true" to enable data recording, "false" to disable.')

    # recorder_node = Node(
    #     package='log_pkg',
    #     executable='logger_node',
    #     name='data_recorder',
    #     output='screen',
    #     condition=IfCondition(LaunchConfiguration('record'))
    # )

    return LaunchDescription([
        LogInfo(msg=['Start model car for the Carolo-Cup']),
        #record_arg,

        __ucbridge,
        __realsense2_camera,

        LogInfo(msg=['Start system layout']),
        __state_estimation,
        __uc_bridge_adapter,
        __control,
        __detect_traffic_sign_node,
    ])
