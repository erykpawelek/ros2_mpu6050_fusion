#include <cmath>
#include <eigen3/Eigen/Dense>

#include "imu_sensor_cpp/algorithms/ekf.hpp"
#include "imu_sensor_cpp/driver_core/mpu6050_driver.hpp"

namespace imu_ekf
{
    ExtendedKalmanFilter::ExtendedKalmanFilter(
        const ExtendedKalmanFilter::MeasurementMatrix & R,
        const ExtendedKalmanFilter::StateMatrix & Q,
        const double& magnitude_low_threshold,
        const double & magnitude_high_threshold)
    : 
    R_(R),
    Q_(Q),
    first_run_(true),
    magnitude_low_threshold_(magnitude_low_threshold),
    magnitude_high_threshold_(magnitude_high_threshold)
    {
        P_.setIdentity();
        P_ = P_ * 0.001;
    }

    void ExtendedKalmanFilter::initialize_if_needed(const mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>::ImuData & imu_data)
    {
        if (first_run_){
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

            first_run_ = false;
        }
    }

    void ExtendedKalmanFilter::predict(const mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>::ImuData & imu_data, double dt)
    {
        // Refactoring state format for quaternion operations
        Eigen::Quaterniond q_prev(x_(0), x_(1), x_(2), x_(3));

        // Angular velocity quaternion
        Eigen::Quaterniond q_omega(0.0, imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);

        // Quaternion derivative based ona angular velocity (1/2 component is transfered to integration step line:63)
        Eigen::Quaterniond q_dot = q_prev * q_omega;
        
        // New quaternion prediction
        Eigen::Quaterniond q_new;

        // Integrating to get new orientation and normalizing
        q_new.coeffs() = q_prev.coeffs() + 0.5 * q_dot.coeffs() * dt;
        q_new.normalize();
        // Updating state vector
        x_ << q_new.w(), q_new.x(), q_new.y(), q_new.z();
        x_.normalize();

        // Calculating omega matrix
        Eigen::Matrix4d omega;
        omega(0,0) =  0.0;
        omega(0,1) = -imu_data.gyro_x;
        omega(0,2) = -imu_data.gyro_y;
        omega(0,3) = -imu_data.gyro_z;

        omega(1,0) =  imu_data.gyro_x;
        omega(1,1) =  0.0;
        omega(1,2) =  imu_data.gyro_z;
        omega(1,3) = -imu_data.gyro_y;

        omega(2,0) =  imu_data.gyro_y;
        omega(2,1) = -imu_data.gyro_z;
        omega(2,2) =  0.0;
        omega(2,3) =  imu_data.gyro_x;

        omega(3,0) =  imu_data.gyro_z;
        omega(3,1) =  imu_data.gyro_y;
        omega(3,2) = -imu_data.gyro_x;
        omega(3,3) =  0.0;

        // Calculating transition matrix F
        Eigen::Matrix4d F = Eigen::Matrix4d::Identity() + (0.5 * dt * omega);

        // Calculating new value of prediction error covariance matrix P
        P_ = F * P_ * F.transpose() + Q_;

    }

    void ExtendedKalmanFilter::update(const mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>::ImuData & imu_data)
    {   
        // Magnitude treshold check
        if (!magnitude_check(imu_data)) {
            return;
        }   

        // Working on norm gravity vector to be roboust against linear accelerations
        auto GRAVITY_VECTOR = Eigen::Vector3d(0.0, 0.0, 1.0);

        // Refactoring state format for quaternion operations
        Eigen::Quaterniond q_prev(x_(0), x_(1), x_(2), x_(3));

        // Measurement vector from accelerometer
        Eigen::Vector3d z_meas;
        z_meas << imu_data.accel_x, imu_data.accel_y, imu_data.accel_z;
        if(z_meas.norm() < 1e-6) return; //Prevention against deviding by 0
        z_meas.normalize();

        // Predicted measurement based on current state
        Eigen::Vector3d h_x = q_prev.inverse() * GRAVITY_VECTOR;

        // Innovation calculation
        Eigen::Vector3d innovation = z_meas - h_x;
        
        // Jacobian matrix calculation
        Eigen::Matrix<double, 3, 4> H;

        H(0, 0) = -2 * q_prev.y();
        H(0, 1) =  2 * q_prev.z();
        H(0, 2) = -2 * q_prev.w();
        H(0, 3) =  2 * q_prev.x();

        H(1, 0) =  2 * q_prev.x();
        H(1, 1) =  2 * q_prev.w();
        H(1, 2) =  2 * q_prev.z();
        H(1, 3) =  2 * q_prev.y();

        H(2, 0) =  2 * q_prev.w();
        H(2, 1) = -2 * q_prev.x();
        H(2, 2) = -2 * q_prev.y();
        H(2, 3) =  2 * q_prev.z();

        // Calculating covariance of innovation
        Eigen::Matrix3d S = H * P_ * H.transpose() + R_;

        // Kalman Gain calculation
        Eigen::Matrix<double, 4, 3> K = P_ * H.transpose() * S.inverse();

        // Calculating new state estimate
        x_ = x_ + (K * innovation);
        x_.normalize();

        // Calculating new error covariance matrix
        P_ = (Eigen::Matrix4d::Identity() - K * H) * P_;
        // Keeping P symetric
        P_ = 0.5 * (P_ + P_.transpose());
    }

    void ExtendedKalmanFilter::init_first_run(){
        first_run_ = true;
    }

    ExtendedKalmanFilter::StateVector ExtendedKalmanFilter::get_state() const{
        return x_;
    }

    void ExtendedKalmanFilter::setR(std::vector<double> & R_vector){
        R_ = Eigen::Map<const Eigen::Vector3d>(R_vector.data()).asDiagonal();
    }

    void ExtendedKalmanFilter::setQ(std::vector<double> & Q_vector){
        Q_ = Eigen::Map<const Eigen::Vector4d>(Q_vector.data()).asDiagonal();
    }

    bool ExtendedKalmanFilter::magnitude_check(const mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>::ImuData & imu_data){
        double accel_norm = std::sqrt(imu_data.accel_x * imu_data.accel_x + 
                                      imu_data.accel_y * imu_data.accel_y + 
                                      imu_data.accel_z * imu_data.accel_z);

        if (accel_norm < magnitude_low_threshold_ || accel_norm > magnitude_high_threshold_) {
            return false;
        } else {
            return true;
        }  
    }

    void ExtendedKalmanFilter::set_magnitude_low_threshold(double magnitude_low_threshold){
        magnitude_low_threshold_ = magnitude_low_threshold;
    }

    void ExtendedKalmanFilter::set_magnitude_high_threshold(double magnitude_high_threshold){
        magnitude_high_threshold_ = magnitude_high_threshold;
    }
} // namespace ekf