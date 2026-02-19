import os
from launch import LaunchDescription
from launch_ros.actions import Node 
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
	ld = LaunchDescription()
	config = os.path.join(
		get_package_share_directory('psaf_trajectory_plan'),
		'config',
		'psaf_trajectory_plan.yaml'
	)

	trajectory_plan_node =  Node(
		package='psaf_trajectory_plan',
		executable='trajectory_plan',
		name='trajectory_plan',
		parameters=[config]
	)

	ld.add_action(trajectory_plan_node)
	return ld
