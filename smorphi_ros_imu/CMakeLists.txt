cmake_minimum_required(VERSION 3.8)
project(smorphi_ros_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
find_package(tf2 REQUIRED)
find_package(rcl_interfaces REQUIRED)
include_directories(
  include/smorphi_ros_imu
  ${serial_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
#add_executable(imu_node "src/imu_node.cpp")
add_executable(imu_node src/imu_node.cpp)
ament_target_dependencies(imu_node
  rclcpp
  sensor_msgs
  std_msgs
  serial
  Eigen3
  tf2
  rcl_interfaces
)
target_include_directories(imu_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS imu_node
  DESTINATION lib/smorphi_ros_imu)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
