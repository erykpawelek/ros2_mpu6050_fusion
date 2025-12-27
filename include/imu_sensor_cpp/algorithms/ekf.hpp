#ifndef EXTENDED_KALMAN_FILTER
#define EXTENDED_KALMAN_FILTER

#include <eigen3/Eigen/Dense>

#include "imu_sensor_cpp/driver_core/mpu6050_driver.hpp"

namespace ekf
{

class ExtendedKalmanFilter{
public:

    using StateVector = Eigen::Matrix<double, 4, 1>;

    using StateMatrix = Eigen::Matrix<double, 4, 4>;

    using MeasurementMatrix = Eigen::Matrix<double, 3, 3>;

    ExtendedKalmanFilter(
        const MeasurementMatrix& R_noise,
        double process_noise_variance);

    void init(const mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>::ImuData & imu_data);

private:

    StateVector x_;

    StateMatrix Q_;

    StateMatrix P_;

    MeasurementMatrix R_;

};

}
#endif 