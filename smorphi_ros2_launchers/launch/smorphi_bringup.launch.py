from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
	return LaunchDescription([
	
	IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(get_package_share_directory('smorphi_ros2_controller'),
				"launch/smorphi_controller.launch.py")
		)
	),
	IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(get_package_share_directory('mpu9250driver'),
				"launch/mpu9250driver_launch.py")
		)
	),
	IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(get_package_share_directory('robot_localization'),
				"launch/ekf.launch.py")
		)
	),
	IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(get_package_share_directory('rplidar_ros'),
				"launch/rplidar_a1_launch.py")
		)
	),

	
	])
