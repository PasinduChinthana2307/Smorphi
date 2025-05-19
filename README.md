# smorphi_driver_ros2
## Smorphi Driver for ROS2
This tutorial will guide the users to setup and install packages to run smorphi.
## Connecting your smorphi using XRDP protocol (optional)
XRDP is a client-server application that uses the Remote Desktop Protocol (RDP) as its transport protocol. The server is implemented as a Linux daemon running on top of a DisplayLink device. The communication between the server and the client is encrypted using TLS.
### Setting up RDP server on your robot
```bash
sudo apt update
sudo apt install xrdp -y
```
Once the installation is successfully finished, the XRDP service will be started automatically. You can check the status by executing the following command:
```bash
sudo systemctl status xrdp
```
<img width="540" alt="Screenshot 2023-09-23 at 1 10 03 PM" src="https://github.com/roarLab/smorphi_driver_ros2/assets/34149646/bbe63796-ea56-4e9d-b9c6-c02c6a4b9faf">
<br>

The above output shows the Xrdp service is up and running.In order to work xrdp properly, add the xrdp user to the “ssl-cert” group with the following command.
```bash
sudo usermod -a -G ssl-cert xrdp 
sudo systemctl restart xrdp 
```
Now you need to configure system firewall. The XRDP service listens on standard remote desktop port 3389. You need to adjust the firewall to allow access to port 3389 for the remote systems.
```bash
sudo ufw allow from 192.168.1.0/24 to any port 3389 
sudo ufw reload
```
Check IP address of your smorphi robot using the following command.
```bash
ifconfig
```
<img width="540" alt="Screenshot 2023-09-23 at 2 12 35 PM" src="https://github.com/roarLab/smorphi_driver_ros2/assets/34149646/15220112-c323-4d98-9b58-25fc2568833d">
<br>

From the above image the ip address of the robot is "172.20.10.13". Connect your smorphi using RDP client from your windows/mac/linux desktop by entering the ip address of your smorphi. Ensure your desktop is connected to the same wifi network as of your robot.


## Compile & Install YDLidar SDK

ydlidar_ros2_driver depends on YDLidar-SDK library. If you have never installed YDLidar-SDK library or it is out of date, you must first install YDLidar-SDK library. If you have installed the latest version of YDLidar-SDK, skip this step and go to the next step.

1. Download or clone the [YDLIDAR/YDLidar-SDK](https://github.com/YDLIDAR/YDLidar-SDK) repository on GitHub.
2. Compile and install the YDLidar-SDK under the ***build*** directory by following steps:
  In the YDLidar SDK directory, run the following commands to compile the project:
```
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
mkdir build
cd build
cmake ..
make
sudo make install
```

## Installing Dependencies
```bash
sudo apt-get install libgeographic-dev
```
```bash
sudo apt-get install ros-humble-geographic-msgs
```
```bash
sudo apt-get install ros-humble-tf-transformations
sudo apt-get install ros-humble-diagnostics
sudo pip3 install transforms3d
pip install pyserial
```
git clone packages to the ROS2_WORKSPACE/src/ directory.
```bash
cd ROS2_WORKSPACE/src
git clone https://github.com/roarLab/smorphi_driver_ros2.git
```
## Setting up Nav2 Stack in Smorphi
Run the following commands to Install Nav2
```bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-*
```

## Build the cloned packages.
```bash
cd ..
colcon build
```
After building the packages, Source your workspace
```bash
source install/setup.bash
```
## Common Build Errors:
deprecations in setup.py
```bash
pip install setuptools==58.2.0
```
## Launching Smorphi Driver

```bash
ros2 launch smorphi_ros2_launchers smorphi_bringup.launch.py
```


## Activity 1: Teleoperating Smorphi
After connecting Masterboard, Lidar and Imu to Raspberry Pi, Run the below command to trigger the USB rules to allow access of the USB ports
```bash
sudo udevadm trigger
```
After running the above command. Close the terminal and open a new terminal to run the below commands to teleop smorphi.
```bash
ros2 launch smorphi_ros2_launchers smorphi_bringup.launch.py
```
In another terminal
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Use the keyboard to teleop your smorphi!!

## Activity 2: Mapping with Smorphi

```bash
ros2 launch smorphi_ros2_launchers smorphi_bringup.launch.py
```
In another terminal
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Open a new terminal and run slam toolbox to start mapping
```bash
ros2 launch smorphi_ros2_launchers smorphi_mapper_online_async_launch.py
```
Open a new terminal and run rviz2 to see the mapping
```bash
rviz2
```
To save map run the following command in another terminal. Replace "my_map" with the name you want to save for the map.
```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

## Activity 3: Nav2 with Smorphi

Once the map is saved copy the my_map.pgm and my_map.yaml files to smorphi_ros2_launchers/map folder in the package.
Add the map location in setup.py file as shown in the image below.

<img width="928" alt="Screenshot 2024-02-08 at 4 50 48 PM" src="https://github.com/roarLab/smorphi_driver_ros2/assets/34149646/ae478aba-118e-4bac-ba7f-10584abf4c26">

After saving the file, Build the package using the below command
```bash
colcon build
```
After the package is build run the following to launch smorphi
```bash
ros2 launch smorphi_ros2_launchers smorphi_bringup.launch.py
```
In another terminal
```bash
ros2 launch smorphi_ros2_launchers smorphi_nav2.launch.py
```
In the rviz window initialize the robot pose using 2d pose estimate and send navigation goals to smorphi for autonomous waypoint navigation.


