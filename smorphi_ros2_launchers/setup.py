from setuptools import find_packages, setup

package_name = 'smorphi_ros2_launchers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' +package_name+'/launch', ['launch/smorphi_bringup.launch.py']),
        ('share/' +package_name+'/config', ['config/smorphi_mapper_online_async.yaml']),
        ('share/' +package_name+'/launch', ['launch/smorphi_mapper_online_async_launch.py']),
        ('share/' +package_name+'/launch', ['launch/smorphi_nav2.launch.py']),
        ('share/' +package_name+'/params', ['params/smorphi.yaml']),
        ('share/' +package_name+'/map', ['map/turtlebot3_world.pgm']),
        ('share/' +package_name+'/map', ['map/turtlebot3_world.yaml']),
        ('share/' +package_name+'/rviz', ['rviz/navigation2.rviz']),
    ],
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='smorphi',
    maintainer_email='smorphi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
