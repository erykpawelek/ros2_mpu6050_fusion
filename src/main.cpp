#include "imu_sensor_cpp/mpu6050_driver.hpp"
#include <stdio.h>

int main()
{
    auto mpu6050 = mpu6050cust_driver::MPU6050CustomDriver(0x68, 1); 
    
    while (1){
        auto accelX = mpu6050.getAccelerationX(); 
        printf("Acceleration on x axe:  %f\n", accelX);
    }
    return 0;
}