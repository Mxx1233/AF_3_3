import os
from launch import LaunchDescription
from launch_ros.actions import Node 
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
	ld = LaunchDescription()
	config = os.path.join(
		get_package_share_directory('psaf_lane_detection_outside'),
		'config',
		'psaf_lane_detection_outside.yaml'
	)

	lane_detection_outside_node =  Node(
		package='psaf_lane_detection_outside',
		executable='lane_detection_outside',
		name='lane_detection_outside',
		parameters=[config]
	)

	ld.add_action(lane_detection_outside_node)
	return ld
