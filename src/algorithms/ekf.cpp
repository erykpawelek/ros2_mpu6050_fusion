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

    ExtendedKalmanFilter::init(
        const mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>::ImuData & imu_data);
}