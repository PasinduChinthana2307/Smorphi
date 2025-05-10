cmake_minimum_required(VERSION 3.8)
project(ros2_mpu6050)

# Set C++ standard (required for ROS 2 Humble)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)  # Required for IMU data processing
find_package(tf2 REQUIRED)     # Required for transformations
find_package(tf2_ros REQUIRED) # Required for ROS 2 transforms

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  /opt/ros/humble/include
)

# Main executable
add_executable(ros2_mpu6050 
  src/mpu6050.cpp 
  src/mpu6050_hal.cpp 
  src/mpu6050_node.cpp
)
ament_target_dependencies(ros2_mpu6050
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)
target_link_libraries(ros2_mpu6050
  i2c
  ${EIGEN3_LIBRARIES}
)

# Calibration executable
add_executable(ros2_mpu6050_calibrate
  src/mpu6050.cpp 
  src/mpu6050_hal.cpp
)
ament_target_dependencies(ros2_mpu6050_calibrate
  rclcpp
  sensor_msgs
)
target_link_libraries(ros2_mpu6050_calibrate
  i2c
  ${EIGEN3_LIBRARIES}
)

# Installation
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS
  ros2_mpu6050
  ros2_mpu6050_calibrate
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
