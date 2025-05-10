#include "ros2_mpu6050/mpu6050_node.h"
#include <Eigen/Geometry>
#include <tf2/tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

Mpu6050Node::Mpu6050Node(const std::string& name)
    : Node(name)
    , mpu6050_dev_{std::make_unique<Mpu6050>()}
    , filter_update_rate_{100}  // 100Hz = 10ms timer
    , last_time_{this->get_clock()->now()}
{
    // Declare parameters
    this->declare_parameter<int>("gyro_fs_sel", 0);
    this->declare_parameter<int>("accel_afs_sel", 0);
    this->declare_parameter<int>("dlpf_cfg", 0);
    this->declare_parameter<int>("clock_src", 0);
    this->declare_parameter<double>("gyro_x_offset", 0.0);
    this->declare_parameter<double>("gyro_y_offset", 0.0);
    this->declare_parameter<double>("gyro_z_offset", 0.0);
    this->declare_parameter<double>("accel_x_offset", 0.0);
    this->declare_parameter<double>("accel_y_offset", 0.0);
    this->declare_parameter<double>("accel_z_offset", 0.0);
    this->declare_parameter<bool>("use_madgwick", true);
    this->declare_parameter<double>("madgwick_beta", 0.1);
    this->declare_parameter<int>("device_type", 1);  // 0=raw, 1=ROS standard

    // Get parameters
    gyro_x_offset_ = this->get_parameter("gyro_x_offset").as_double();
    gyro_y_offset_ = this->get_parameter("gyro_y_offset").as_double();
    gyro_z_offset_ = this->get_parameter("gyro_z_offset").as_double();
    accel_x_offset_ = this->get_parameter("accel_x_offset").as_double();
    accel_y_offset_ = this->get_parameter("accel_y_offset").as_double();
    accel_z_offset_ = this->get_parameter("accel_z_offset").as_double();
    use_madgwick_ = this->get_parameter("use_madgwick").as_bool();
    beta_ = this->get_parameter("madgwick_beta").as_double();
    device_type_ = this->get_parameter("device_type").as_int();

    // Sensor configuration
    mpu6050_dev_->Mpu6050_GyroFsSel(static_cast<Mpu6050::Mpu6050_FsSel_t>(this->get_parameter("gyro_fs_sel").as_int()));
    mpu6050_dev_->Mpu6050_AccelFsSel(static_cast<Mpu6050::Mpu6050_AfsSel_t>(this->get_parameter("accel_afs_sel").as_int()));
    mpu6050_dev_->Mpu6050_DlpfConfig(static_cast<Mpu6050::Mpu6050_DlpfCfg_t>(this->get_parameter("dlpf_cfg").as_int()));
    mpu6050_dev_->Mpu6050_ClockSelect(static_cast<Mpu6050::Mpu6050_ClkSrc_t>(this->get_parameter("clock_src").as_int()));

    // Publishers
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);
    
    // Covariance matrices (example values - adjust based on your sensor specs)
    imu_orientation_covariance_ = {0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025};
    imu_angular_vel_covariance_ = {0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025};
    imu_linear_accel_covariance_ = {0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025};

    // Initialize Madgwick filter
    q0_ = 1.0; q1_ = 0.0; q2_ = 0.0; q3_ = 0.0;

    // Timer for publishing
    timer_ = this->create_wall_timer(10ms, std::bind(&Mpu6050Node::ImuPubCallback, this));
}

void Mpu6050Node::ImuPubCallback()
{
    auto message = sensor_msgs::msg::Imu();
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "imu_link";  // Changed to match reference

    // Read raw IMU data
    Mpu6050::Mpu6050_AccelData_t AccelData;
    Mpu6050::Mpu6050_GyroData_t GyroData;
    mpu6050_dev_->Mpu6050_GetAccelData(AccelData);
    mpu6050_dev_->Mpu6050_GetGyroData(GyroData);

    // Apply offsets and convert units
    double ax = AccelData.Accel_X - accel_x_offset_;
    double ay = AccelData.Accel_Y - accel_y_offset_;
    double az = AccelData.Accel_Z - accel_z_offset_;
    
    double gx = (GyroData.Gyro_X - gyro_x_offset_) * (M_PI / 180.0);
    double gy = (GyroData.Gyro_Y - gyro_y_offset_) * (M_PI / 180.0);
    double gz = (GyroData.Gyro_Z - gyro_z_offset_) * (M_PI / 180.0);

    // Calculate time step
    auto now = this->get_clock()->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    // Estimate orientation
    if (use_madgwick_) {
        MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, dt);
    } else {
        // Alternative: Complementary filter
        ComplementaryFilter(ax, ay, az, gx, gy, gz, dt);
    }

    // Set orientation quaternion
    message.orientation.x = q1_;
    message.orientation.y = q2_;
    message.orientation.z = q3_;
    message.orientation.w = q0_;

    // Set angular velocity (rad/s)
    message.angular_velocity.x = gx;
    message.angular_velocity.y = gy;
    message.angular_velocity.z = gz;

    // Set linear acceleration (m/s²)
    message.linear_acceleration.x = ax;
    message.linear_acceleration.y = ay;
    message.linear_acceleration.z = az;

    // Apply coordinate transformation if needed (ROS standard)
    if (device_type_ == 1) {
        TransformToROSStandard(message);
    }

    // Set covariance matrices
    message.orientation_covariance = imu_orientation_covariance_;
    message.angular_velocity_covariance = imu_angular_vel_covariance_;
    message.linear_acceleration_covariance = imu_linear_accel_covariance_;

    // Publish message
    imu_pub_->publish(message);
}

void Mpu6050Node::MadgwickAHRSupdate(double gx, double gy, double gz, double ax, double ay, double az, double dt)
{
    // Implementation of Madgwick filter - simplified version
    double recipNorm;
    double s0, s1, s2, s3;
    double qDot1, qDot2, qDot3, qDot4;
    double _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5 * (-q1_ * gx - q2_ * gy - q3_ * gz);
    qDot2 = 0.5 * (q0_ * gx + q2_ * gz - q3_ * gy);
    qDot3 = 0.5 * (q0_ * gy - q1_ * gz + q3_ * gx);
    qDot4 = 0.5 * (q0_ * gz + q1_ * gy - q2_ * gx);

    // Compute feedback only if accelerometer measurement valid
    if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {
        // Normalize accelerometer measurement
        recipNorm = 1.0 / sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0 * q0_;
        _2q1 = 2.0 * q1_;
        _2q2 = 2.0 * q2_;
        _2q3 = 2.0 * q3_;
        _4q0 = 4.0 * q0_;
        _4q1 = 4.0 * q1_;
        _4q2 = 4.0 * q2_;
        _8q1 = 8.0 * q1_;
        _8q2 = 8.0 * q2_;
        q0q0 = q0_ * q0_;
        q1q1 = q1_ * q1_;
        q2q2 = q2_ * q2_;
        q3q3 = q3_ * q3_;

        // Gradient descent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1_ - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0 * q0q0 * q2_ + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0 * q1q1 * q3_ - _2q1 * ax + 4.0 * q2q2 * q3_ - _2q2 * ay;
        
        // Normalize step magnitude
        recipNorm = 1.0 / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta_ * s0;
        qDot2 -= beta_ * s1;
        qDot3 -= beta_ * s2;
        qDot4 -= beta_ * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0_ += qDot1 * dt;
    q1_ += qDot2 * dt;
    q2_ += qDot3 * dt;
    q3_ += qDot4 * dt;

    // Normalize quaternion
    recipNorm = 1.0 / sqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
    q0_ *= recipNorm;
    q1_ *= recipNorm;
    q2_ *= recipNorm;
    q3_ *= recipNorm;
}

void Mpu6050Node::TransformToROSStandard(sensor_msgs::msg::Imu& imu_msg)
{
    // Create Eigen quaternion from message
    Eigen::Quaterniond q_orig(imu_msg.orientation.w,
                             imu_msg.orientation.x,
                             imu_msg.orientation.y,
                             imu_msg.orientation.z);

    // Create transformation quaternions
    Eigen::Quaterniond q_r = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
    
    Eigen::Quaterniond q_rr = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

    // Apply transformation
    Eigen::Quaterniond q_out = q_r * q_orig * q_rr;

    // Update message with transformed orientation
    imu_msg.orientation.x = q_out.x();
    imu_msg.orientation.y = q_out.y();
    imu_msg.orientation.z = q_out.z();
    imu_msg.orientation.w = q_out.w();

    // Transform angular velocity (invert Y and Z)
    imu_msg.angular_velocity.y *= -1;
    imu_msg.angular_velocity.z *= -1;

    // Transform linear acceleration (invert X)
    imu_msg.linear_acceleration.x *= -1;
}

void Mpu6050Node::ComplementaryFilter(double ax, double ay, double az, double gx, double gy, double gz, double dt)
{
    // Simple complementary filter implementation
    double roll, pitch;
    double accel_roll, accel_pitch;
    
    // Calculate angles from accelerometer
    accel_roll = atan2(ay, az);
    accel_pitch = atan2(-ax, sqrt(ay * ay + az * az));
    
    // Integrate gyro rates to get angles
    roll += gx * dt;
    pitch += gy * dt;
    
    // Apply complementary filter
    roll = 0.98 * roll + 0.02 * accel_roll;
    pitch = 0.98 * pitch + 0.02 * accel_pitch;
    
    // Convert to quaternion
    tf2::Quaternion q;
    q.setRPY(roll, pitch, gz * dt);  // Yaw from gyro integration
    
    q0_ = q.getW();
    q1_ = q.getX();
    q2_ = q.getY();
    q3_ = q.getZ();
}
