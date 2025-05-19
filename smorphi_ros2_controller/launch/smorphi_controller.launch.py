from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
	smorphi_controller_node = Node(package='smorphi_ros2_controller',executable='smorphi_controller_node',name='controller_node',output="screen")
    
	#static_tf = Node(package = "tf2_ros", executable = "static_transform_publisher",arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_link"])
		
	#rviz_display_node = Node(package='rviz2',executable="rviz2",output="screen")
	
	return LaunchDescription([smorphi_controller_node])
#rviz_display_node]
