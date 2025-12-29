#ifndef EXTENDED_KALMAN_FILTER
#define EXTENDED_KALMAN_FILTER

#include <eigen3/Eigen/Dense>
#include <cmath>
#include <vector>

#include "imu_sensor_cpp/driver_core/mpu6050_driver.hpp"

namespace imu_ekf
{
/**
* @class ExtendedKalmanFilter
* @brief Implementation of the Extended Kalman Filter (EKF) for AHRS orientation estimation.
*
* This class estimates the 3D orientation of an object represented by a quaternion.
* It uses gyroscope data for prediction (process model) and accelerometer data
* for correction (measurement model).
*
* @details
* The filter operates on a 4x1 state vector (Quaternion).
* - Prediction step: Integrates angular velocity.
* - Correction step: Corrects drift using the gravity vector.
*/
class ExtendedKalmanFilter{
public:
    /**
    * @brief State Vector [4x1].
    * Represents the orientation quaternion q = [qw, qx, qy, qz]^T.
    * qw - scalar part, qx,qy,qz - vector part.
    */
    using StateVector = Eigen::Matrix<double, 4, 1>;

    /**
    * @brief State/Covariance Matrix [4x4].
    * Used for the Error Covariance Matrix (P) and Process Noise Covariance (Q).
    */
    using StateMatrix = Eigen::Matrix<double, 4, 4>;

    /**
    * @brief Measurement Matrix [3x3].
    * Used for the Measurement Noise Covariance (R).
    */
    using MeasurementMatrix = Eigen::Matrix<double, 3, 3>;

    /**
    * @brief Constructor for the Extended Kalman Filter.
    * Initializes covariance matrices and noise parameters.
    * @param R_noise Measurement Noise Covariance Matrix (3x3). 
    * @param process_noise_variance Variance scalar for process noise.
    */
    ExtendedKalmanFilter(
        const MeasurementMatrix& R_noise, 
        const StateMatrix & Q);

    /**
    * @brief Initializes the state using the first accelerometer reading.
    * Calculates initial Roll and Pitch angles based on the gravity vector
    * (assuming the robot is stationary) and converts them to the initial Quaternion.
    * Yaw is initialized to 0.0.
    * @param imu_data Structure containing raw IMU data (accelerometer).
    */
    void init(const mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>::ImuData & imu_data);

    /**
     * @brief Calculates state prediction along with prediction covariance matrix P_.
     *  Prediction in this implementation is orientation calculated based on gyroscope data.
     * @param imu_data MPU6050CustomDriver class data struct containing raw imu sensor data.
     * @param dt Time step between iterations.
     */
    void predict(const mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>::ImuData & imu_data, double dt);

    /**
     * @brief Corrects the predicted state using accelerometer measurements.
     * * This step computes the Kalman Gain and updates the state vector and 
     * covariance matrix by comparing the predicted gravity vector with the actual accelerometer reading.
     * @param dt Time step between iterations.
     */
    void update(const mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>::ImuData & imu_data);

    /**
    * @brief Resets the initialization flag to true, forcing the execution of init() on the next iteration.
    */
    void init_first_run();

    /**
     * @brief Returns current state vector x_, containing orientation in quaternion form.
     */
    StateVector get_state() const;

    /**
     * @brief Sets measurement covariance matrix R_. 
     * * In this implementation we assume that axis covariances are independent of each other.
     * @param R_vector Vector containing diagonal elements of R_ class member matrix. It has to be 3 element vector.
     */
    void setR(std::vector<double> R_vector);
    
    /**
     * @brief Sets prediction covariance matrix Q_. 
     * * In this implementation we assume that axis covariances are independent of each other.
     * @param Q_vector Vector containing diagonal elements of Q_ class member matrix. It has to be 4 element vector.
     */
    void setQ(std::vector<double> Q_vector);

private:

    /**
    * @brief Current State Vector (Quaternion).
    * x = [qw, qx, qy, qz]^T
    */
    StateVector x_;

    /**
    * @brief Measurement Noise Covariance Matrix.
    * Represents the uncertainty of the sensor (accelerometer).
    */
    MeasurementMatrix R_;

    /**
    * @brief Process Noise Covariance Matrix.
    * Represents the uncertainty of the physical model (gyro integration).
    * Constant in this implementation.
    */
    StateMatrix Q_;

    /**
    * @brief Error Covariance Matrix.
    * Represents the current estimation uncertainty.
    * This matrix "breathes": grows during prediction, shrinks during correction.
    */
    StateMatrix P_;

    /**
     * @brief First iteration initialization flag.
     * It's responsible for runing init function in first iteration.
     */ 
    bool first_run_;
};
}
#endif 