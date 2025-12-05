"""Start the whole pipeline with all Nodes for the old car."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node


__ucbridge = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(get_package_share_directory('psaf_launch'), 'launch', 'ucbridge_old.launch.py')))

__realsense2_camera = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(get_package_share_directory('psaf_launch'), 'launch',
                 'realsense2_camera_455.launch.py')
))

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


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=['Start model car for the Carolo-Cup']),
        __ucbridge,
        __realsense2_camera,
        
        LogInfo(msg=['Start system layout']),
        __state_estimation,
        __uc_bridge_adapter,
        __control,
    ])
