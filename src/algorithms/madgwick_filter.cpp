#include <cmath>

#include "imu_sensor_cpp/algorithms/madgwick_filter.hpp"

namespace imu_madgwick
{
    MadgwickFilter::MadgwickFilter(double beta, Quaternion initial_pose)
    : beta_(beta),
    q_previous_(initial_pose)
    {}

    bool MadgwickFilter::update(
        const mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>::ImuData& imu_data,
        double dt)
    {
        // Placeholder for normalized gradient
        double norm_grad_w = 0.0;
        double norm_grad_x = 0.0;
        double norm_grad_y = 0.0;
        double norm_grad_z = 0.0;

        // Creating local copy for clarity 
        double qw = q_previous_.w;
        double qx = q_previous_.x;
        double qy = q_previous_.y;
        double qz = q_previous_.z;
        
        double norm = std::sqrt(imu_data.accel_x * imu_data.accel_x +
                                imu_data.accel_y * imu_data.accel_y +
                                imu_data.accel_z * imu_data.accel_z);

        if (norm > EPSILON){
            // Normalizing the gravity vector from imu to ensure it's length is 1.0 
            double norm_ax = imu_data.accel_x / norm;
            double norm_ay = imu_data.accel_y / norm;
            double norm_az = imu_data.accel_z / norm;

            // Calculating gravity vector prediction optimalizing calculations by calculating only last collumn of rotation matrix from quaternon form due to the fact that gravity vector in word coordinate is always 0,0,1 so by that using ratation matrix calculations we reduce first columns
            double vx = 2.0 * (q_previous_.x * q_previous_.z + q_previous_.y * q_previous_.w);
            double vy = 2.0 * (q_previous_.y * q_previous_.z - q_previous_.x * q_previous_.w);
            double vz = 1.0 - 2.0 * q_previous_.x * q_previous_.x - 2.0 * q_previous_.y * q_previous_.y;

            //Calculating error function
            double f_x = vx - norm_ax;
            double f_y = vy - norm_ay;
            double f_z = vz - norm_az;

            // Calculating gradient of cost function
            double grad_w =  2.0 * qy * f_x - 2.0 * qx * f_y;           
            double grad_x =  2.0 * qz * f_x - 2.0 * qw * f_y - 4.0 * qx * f_z; 
            double grad_y =  2.0 * qw * f_x + 2.0 * qz * f_y - 4.0 * qy * f_z;
            double grad_z =  2.0 * qx * f_x + 2.0 * qy * f_y;         
             
            // Gradient normalizing to guarantee that it's magnitude is 1.0
            double grad_norm = std::sqrt(grad_w * grad_w +
                                         grad_x * grad_x + 
                                         grad_y * grad_y +
                                         grad_z * grad_z);
                                        
            if (grad_norm > EPSILON){
                norm_grad_w = grad_w / grad_norm;
                norm_grad_x = grad_x / grad_norm;
                norm_grad_y = grad_y / grad_norm;
                norm_grad_z = grad_z / grad_norm;
            }
        }
        // Calculating first derivative of gyroscope quaternion measurement
        double gx = imu_data.gyro_x;
        double gy = imu_data.gyro_y;
        double gz = imu_data.gyro_z;

        double q_prim_w = 0.5 * (-qx*gx - qy*gy - qz*gz);
        double q_prim_x = 0.5 * ( qw*gx + qy*gz - qz*gy);
        double q_prim_y = 0.5 * ( qw*gy - qx*gz + qz*gx);
        double q_prim_z = 0.5 * ( qw*gz + qx*gy - qy*gx);
        
        // Calculating corection derivative
        double q_correction_w = q_prim_w - beta_* norm_grad_w;
        double q_correction_x = q_prim_x - beta_* norm_grad_x;
        double q_correction_y = q_prim_y - beta_* norm_grad_y;
        double q_correction_z = q_prim_z - beta_* norm_grad_z;

        // Integrating to get new quaternion orientation
        Quaternion q_current;
        q_current.w = q_previous_.w + q_correction_w * dt;
        q_current.x = q_previous_.x + q_correction_x * dt;
        q_current.y = q_previous_.y + q_correction_y * dt;
        q_current.z = q_previous_.z + q_correction_z * dt;

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
    }   

    MadgwickFilter::Quaternion MadgwickFilter::get_current_orientation() const{
        return q_previous_;
    }

    void MadgwickFilter::set_beta(double beta){
        beta_ = beta;
    }
}