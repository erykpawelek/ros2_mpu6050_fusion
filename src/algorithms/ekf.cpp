#include <cmath>
#include <eigen3/Eigen/Dense>

#include "imu_sensor_cpp/algorithms/ekf.hpp"

namespace ekf
{
    ExtendedKalmanFilter::ExtendedKalmanFilter(
        const MeasurementMatrix & R_noise,
        double process_noise_variance)
    : R_(R_noise)
    {
        Q_.setIdentity();
        Q_ *= process_noise_variance;
        
        P_.setIdentity();
    }

    void ExtendedKalmanFilter::init(const mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>::ImuData & imu_data)
    {
        // Calculating initial state prediction
        double roll = std::atan2(imu_data.accel_y, imu_data.accel_z);
        double pitch = std::atan2(-imu_data.accel_x, std::sqrt(imu_data.accel_y * imu_data.accel_y +
                                                               imu_data.accel_z * imu_data.accel_z));
        double yaw = 0.0;

        // Calculating quaternion represetation of initial prediction using eigen tools
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

        Eigen::Quaternion q = yawAngle * pitchAngle * rollAngle;

        // Injecting initial quaternion into state vector class member variable
        x_ << q.w(), q.x(), q.y(), q.z();
    }

    void ExtendedKalmanFilter::predict(const mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>::ImuData & imu_data, double dt)
    {
        // Refactoring state format for quaternion operations
        Eigen::Quaterniond q_prev(x_(0), x_(1), x_(2), x_(3));
        // Angular velocity quaternion
        Eigen::Quaterniond q_omega(0.0, imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);
        // Quaternion derivative based ona angular velocity
        Eigen::Quaterniond q_dot = q_prev * q_omega;
        // New quaternion prediction
        Eigen::Quaterniond q_new;
        // Integrating to get new orientation and normalizing
        q_new.coeffs() = q_prev.coeffs() + 0.5 * q_dot.coeffs() * dt;
        q_new.normalize();
        // Updating state vector
        x_ << q_new.w(), q_new.x(), q_new.y(), q_new.z();
    }

    void ExtendedKalmanFilter::update(const mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>::ImuData & imu_data)
    {
        auto GRAVITY_VECTOR = Eigen::Vector3d(0.0, 0.0, 1.0);

        // Refactoring state format for quaternion operations
        Eigen::Quaterniond q_prev(x_(0), x_(1), x_(2), x_(3));
        // Measurement vector from accelerometer
        Eigen::Vector3d z_meas;
        z_meas << imu_data.accel_x, imu_data.accel_y, imu_data.accel_z;
        z_meas.normalize();
        // Predicted measurement based on current state
        Eigen::Vector3d h_x = q_prev.inverse() * GRAVITY_VECTOR;
        // Innovation calculation
        Eigen::Vector3d innovation = z_meas - h_x;
        
        Eigen::Matrix<double, 3, 4> H;

        H(0, 0) = -2 * q_prev.y();
        H(0, 1) =  2 * q_prev.z();
        H(0, 2) = -2 * q_prev.w();
        H(0, 3) =  2 * q_prev.x();

        H(1, 0) =  2 * q_prev.x();
        H(1, 1) =  2 * q_prev.w();
        H(1, 2) =  2 * q_prev.z();
        H(1, 3) =  2 * q_prev.y();

        H(2, 0) =  0.0;
        H(2, 1) = -4 * q_prev.x();
        H(2, 2) = -4 * q_prev.y();
        H(2, 3) =  0.0;
    }
} // namespace ekf