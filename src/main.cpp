#include "imu_sensor_cpp/mpu6050_driver.hpp"
#include <stdio.h>
#include <stdexcept>
#include <unistd.h>

int main()
{   
    try{
        auto mpu6050 = mpu6050cust_driver::MPU6050CustomDriver(
            0x68, 
            1, 
            mpu6050cust_driver::MPU6050CustomDriver::DLPF_94_BAND,
            mpu6050cust_driver::MPU6050CustomDriver::GYRO_RANGE_500,
            mpu6050cust_driver::MPU6050CustomDriver::ACCEL_RANGE_4); 

        while (1){
        auto data = mpu6050.getAllData(false);
        printf("Accelerations:\n X: %f,  Y: %f,  Z: %f\n", data.accel_x, data.accel_y, data.accel_z);
        printf("Gyroscope:    \n X: %f,  Y: %f,  Z: %f\n", data.gyro_x, data.gyro_y, data.gyro_z);
        printf("Temperature: %f\n", data.temperature);
        usleep(100000);
        }
    }
    catch (const std::exception& e) {
        printf("Error: %s\n", e.what());
        return -1; 
    }
    return 0;
}