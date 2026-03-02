import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

__ucbridge = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('psaf_launch'),
            'launch',
            'ucbridge_old.launch.py',
        )
    )
)

__realsense2_camera = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('psaf_launch'),
            'launch',
            'realsense2_camera_455.launch.py',
        )
    )
)


__state_estimation = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('psaf_launch'),
            'launch',
            'state_estimation.launch.py',
        )
    )
)

__uc_bridge_adapter = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('psaf_launch'),
            'launch',
            'uc_bridge_adapter.launch.py',
        )
    )
)

__control = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('psaf_launch'), 'launch', 'control.launch.py'
        )
    )
)

__detect_traffic_sign_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('psaf_launch'),
            'launch',
            'detect_traffic_sign_node.launch.py',
        )
    )
)

__lane_detection = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('psaf_launch'),
            'launch',
            'lane_detection_outside.launch.py',
        )
    )
)

__trajectory_plan = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('psaf_launch'),
            'launch',
            'trajectory_plan.launch.py',
        )
    )
)


def generate_launch_description():
    __foxglove_bridge = Node(package='foxglove_bridge', executable='foxglove_bridge')

    return LaunchDescription(
        [
            LogInfo(msg=['Start model car for the Carolo-Cup']),
            __ucbridge,
            __realsense2_camera,
            LogInfo(msg=['Start system layout']),
            __state_estimation,
            __uc_bridge_adapter,
            __control,
            __detect_traffic_sign_node,
            __lane_detection,
            __trajectory_plan,
            __foxglove_bridge,
            LogInfo(msg=['System Initialization Done!']),
        ]
    )
