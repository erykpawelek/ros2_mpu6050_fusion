#include <cmath>

#include "imu_sensor_cpp/algorithms/complementary_filter.hpp"

namespace imu_complementary
{
    ComplementaryFilter::ComplementaryFilter(ComplementaryFilterConfig comp_filter_config)
    : 
    comp_filter_config_(comp_filter_config),
    last_orientation_({0.0, 0.0, 0.0})
    {}

    void ComplementaryFilter::update(const mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>::ImuData & imu_data,
                double dt)
    {
        float var_alfa_x, var_alfa_y;
        double roll, pitch, yaw;
        double accel_roll_part;

        // Calculating magnitude of acceleration vector, to prevent errors during large accelerations
        float magnitude = std::sqrt(imu_data.accel_x * imu_data.accel_x +
                                    imu_data.accel_y * imu_data.accel_y +
                                    imu_data.accel_z * imu_data.accel_z);
        bool magnitude_ok = (magnitude < comp_filter_config_.magnitude_high_threshold &&
                             magnitude > comp_filter_config_.magnitude_low_threshold);

        // Gimball lock prevention
        bool orientation_x_ok = (imu_data.accel_x < comp_filter_config_.gimbal_lock_threshold &&
                                 imu_data.accel_x > -comp_filter_config_.gimbal_lock_threshold);

        if (magnitude_ok){
            var_alfa_y = comp_filter_config_.alpha;
            if (orientation_x_ok){
                var_alfa_x = comp_filter_config_.alpha;
                accel_roll_part = std::atan2(imu_data.accel_y, imu_data.accel_z);
            } else {
                var_alfa_x = 1.0; // Gyro only
                accel_roll_part = 0.0;
            }
        } else {
            var_alfa_x = 1.0; // Gyro only
            var_alfa_y = 1.0; // Gyro only
            accel_roll_part = 0.0;
        }

        // ---- ROLL CALCULATIONS ----
    
        double prediction_x = last_orientation_.roll + (imu_data.gyro_x * dt);
        double error_x = accel_roll_part - prediction_x;
        // Using atan2() normalization to prevent discontuinities in angle calculation when close to 180/-180 degree
        if (error_x < -M_PI){
            error_x = error_x + 2 * M_PI;
        } else if (error_x > M_PI){
            error_x = error_x - 2 * M_PI;
        } 
        roll = prediction_x + (1.0 - var_alfa_x) * (error_x);

        // ---- PITCH CALCULATIONS ----

        double prediction_y = last_orientation_.pitch + (imu_data.gyro_y * dt);
        double error_y = (std::atan2(-imu_data.accel_x, std::sqrt(imu_data.accel_y * imu_data.accel_y +
                                                                  imu_data.accel_z * imu_data.accel_z))) - prediction_y;
        // Using atan2() normalization to prevent discontuinities in angle calculation when close to 180/-180 degree
        if (error_y < -M_PI){
            error_y = error_y + 2 * M_PI;
        } else if (error_y > M_PI){
            error_y = error_y - 2 * M_PI;
        } 
        pitch = prediction_y + (1.0 - var_alfa_y) * (error_y);

        // ---- YAW CALCULATIONS ----

        yaw = last_orientation_.yaw + (imu_data.gyro_z * dt);

        // Saving final values to calss struct 
        last_orientation_.roll = roll;
        last_orientation_.pitch = pitch;
        last_orientation_.yaw = yaw;
    }

    ComplementaryFilter::Quaternion ComplementaryFilter::euler_to_quaternion(EulerAngles orientation)
    {
        ComplementaryFilter::Quaternion quaternion;

        // Defining quaternion's sinus and cosinus values form RPY angles
        double cy = cos(orientation.yaw * 0.5);
        double sy = sin(orientation.yaw * 0.5);
        double cr = cos(orientation.roll * 0.5);
        double sr = sin(orientation.roll * 0.5);
        double cp = cos(orientation.pitch * 0.5);
        double sp = sin(orientation.pitch * 0.5);

        // Calculating union quaternion elements
        quaternion.w = cy * cr * cp + sy * sr * sp; 
        quaternion.x = cy * sr * cp - sy * cr * sp; 
        quaternion.y = cy * cr * sp + sy * sr * cp; 
        quaternion.z = sy * cr * cp - cy * sr * sp; 

        return quaternion;
    }

    ComplementaryFilter::EulerAngles ComplementaryFilter::get_current_orientation() const
    {
        return last_orientation_;
    }

    void ComplementaryFilter::update_alpha(double alpha)
    {
        comp_filter_config_.alpha = alpha;
    }

    void ComplementaryFilter::update_magnitude_low_threshold(double magnitude_low_threshold)
    {
        comp_filter_config_.magnitude_low_threshold = magnitude_low_threshold;
    }

    void ComplementaryFilter::update_magnitude_high_threshold(double magnitude_high_threshold)
    {
        comp_filter_config_.magnitude_high_threshold = magnitude_high_threshold;
    }

    void ComplementaryFilter::update_gimbal_lock_threshold(double gimbal_lock_threshold)
    {
        comp_filter_config_.gimbal_lock_threshold = gimbal_lock_threshold;
    }
}