#ifndef MPU6050DRIVER_H
#define MPU6050DRIVER_H

#include "ros2_mpu6050/mpu6050.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <array>

class Mpu6050Node : public rclcpp::Node {
public:
  explicit Mpu6050Node(const std::string& name);

private:
  // ROS 2 Components
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // MPU6050 Device
  std::unique_ptr<Mpu6050> mpu6050_dev_;

  // Configuration Parameters
  int filter_update_rate_;
  bool use_madgwick_;
  double beta_;
  int device_type_;
  
  // Timing
  rclcpp::Time last_time_;
  
  // Sensor Data Processing
  double gyro_x_offset_{0.0};
  double gyro_y_offset_{0.0};
  double gyro_z_offset_{0.0};
  double accel_x_offset_{0.0};
  double accel_y_offset_{0.0};
  double accel_z_offset_{0.0};
  
  // Orientation Tracking
  double q0_{1.0}, q1_{0.0}, q2_{0.0}, q3_{0.0};
  
  // Covariance Matrices
  std::array<double, 9> imu_orientation_covariance_;
  std::array<double, 9> imu_angular_vel_covariance_;
  std::array<double, 9> imu_linear_accel_covariance_;

  // Main Callback
  void ImuPubCallback();
  
  // Filter Implementations
  void MadgwickAHRSupdate(double gx, double gy, double gz, 
                         double ax, double ay, double az, 
                         double dt);
  void ComplementaryFilter(double ax, double ay, double az,
                          double gx, double gy, double gz,
                          double dt);
  
  // Coordinate Transformation
  void TransformToROSStandard(sensor_msgs::msg::Imu& imu_msg);
};

#endif  // MPU6050DRIVER_H
