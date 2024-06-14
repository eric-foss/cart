from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='cart',
			executable='incline',
			name='incline',
			output='screen'
		),
		Node(
			package='cart',
			exectuable='servo',
			name='servo',
			output='screen'
		)
	])
