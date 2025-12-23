#include <cmath>
#include <string>

#include "imu_sensor_cpp/algorithms/madgwick_filter.hpp"

namespace imu_madgwick
{
    MadgwickFilter::MadgwickFilter(double beta, Quaternion initial_pose)
    : beta_(beta),
    q_previous_(initial_pose)
    {}

    bool MadgwickFilter::update(
        const mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>::ImuData& imu_data,
        rclcpp::Duration dt)
    {
        double norm = std::sqrt(imu_data.accel_x * imu_data.accel_x +
                                imu_data.accel_y * imu_data.accel_y +
                                imu_data.accel_z * imu_data.accel_z);

        if (norm > EPSILON){
            // Normalizing the gravity vector from imu to ensure it's length is 1.0 
            double norm_ax = imu_data.accel_x / norm;
            double norm_ay = imu_data.accel_y / norm;
            double norm_az = imu_data.accel_z / norm;

            // Calculating gravity vector prediction optimalizing calculations by calculating only last collumn of rotation matrix from quaternon form due to the fact that gravity vector in word coordinate is always 0,0,1 so by that using ratation matrix calculations we reduce first columns
            double vx = 2 * (q_previous_.x * q_previous_.z + q_previous_.y * q_previous_.w);
            double vy = 2 * (q_previous_.y * q_previous_.z - q_previous_.x * q_previous_.w);
            double vz = 1 - 2 * q_previous_.x * q_previous_.x - 2 * q_previous_.y * q_previous_.y;

            //Calculating error function
            double f_x = vx - norm_ax;
            double f_y = vy - norm_ay;
            double f_z = vz - norm_az;

            // Calculating gradient of cost function
            double qw = q_previous_.w;
            double qx = q_previous_.x;
            double qy = q_previous_.y;
            double qz = q_previous_.z;

            double grad_w = -2.0f * qy * f_x + 2.0f * qx * f_y;
            double grad_x =  2.0f * qz * f_x + 2.0f * qw * f_y - 4.0f * qx * f_z;
            double grad_y = -2.0f * qw * f_x + 2.0f * qz * f_y - 4.0f * qy * f_z;
            double grad_z =  2.0f * qx * f_x + 2.0f * qy * f_y;

            // Gradient normalizing to guarantee that it's magnitude is 1.0
            double grad_norm = std::sqrt(grad_w * grad_w +
                                        grad_x * grad_x + 
                                        grad_y * grad_y +
                                        grad_z * grad_z);
                                        
            if (grad_norm > EPSILON){
                double norm_grad_w = grad_w / grad_norm;
                double norm_grad_x = grad_x / grad_norm;
                double norm_grad_y = grad_y / grad_norm;
                double norm_grad_z = grad_z / grad_norm;

                double gx = imu_data.gyro_x;
                double gy = imu_data.gyro_y;
                double gz = imu_data.gyro_z;

                // Calculating first derivative of gyroscope quaternion measurement
                double q_prim_w = 0.5 * (-qx*gx - qy*gy - qz*gz);
                double q_prim_x = 0.5 * ( qw*gx + qy*gz - qz*gy);
                double q_prim_y = 0.5 * ( qw*gy - qx*gz + qz*gx);
                double q_prim_z = 0.5 * ( qw*gz + qx*gy - qy*gx);

                double q_correction_w = q_prim_w - beta_* norm_grad_w;
                double q_correction_x = q_prim_x - beta_* norm_grad_x;
                double q_correction_y = q_prim_y - beta_* norm_grad_y;
                double q_correction_z = q_prim_z - beta_* norm_grad_z;

                // Integrating to get new quaternion orientation
                Quaternion q_current;
                q_current.w = q_previous_.w + q_correction_w * dt.seconds();
                q_current.x = q_previous_.x + q_correction_x * dt.seconds();
                q_current.y = q_previous_.y + q_correction_y * dt.seconds();
                q_current.z = q_previous_.z + q_correction_z * dt.seconds();

                // Normalizing current quaternion
                double current_norm = std::sqrt( q_current.w * q_current.w +
                                            q_current.x * q_current.x +
                                            q_current.y * q_current.y +
                                            q_current.z * q_current.z);
                
                if (current_norm > EPSILON){
                    q_current.w = q_current.w / current_norm,
                    q_current.x = q_current.x / current_norm,
                    q_current.y = q_current.y / current_norm,
                    q_current.z = q_current.z / current_norm;

                    q_previous_ = q_current;

                    return true;
                } else {
                    return false;
                }
            } else {
                return false;
            }
        } else {
            return false;
        }
    }
}