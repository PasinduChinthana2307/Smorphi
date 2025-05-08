import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    share_dir = get_package_share_directory('ros2_mpu6050')
    param_file_path = os.path.join(share_dir, 'config', 'params.yaml')

    mpu6050_node = Node(
        package='ros2_mpu6050',
        executable='ros2_mpu6050',
        name='mpu6050_sensor',
        output='screen',
        emulate_tty=True,
        parameters=[param_file_path]
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    )

    return LaunchDescription([mpu6050_node, static_tf])
