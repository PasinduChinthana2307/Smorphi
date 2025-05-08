from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
	imu_node = Node(package='smorphi_ros_imu',executable='imu_node',name='imu',output="screen")
    
	static_tf = Node(package = "tf2_ros", executable = "static_transform_publisher",arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_link"])
		
	#rviz_display_node = Node(package='rviz2',executable="rviz2",output="screen")
	
	return LaunchDescription([imu_node,static_tf])
#rviz_display_node]
