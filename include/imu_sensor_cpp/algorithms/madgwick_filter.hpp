#ifndef IMU_MADGWICK
#define IMU_MADGWICK

#include "imu_sensor_cpp/driver_core/mpu6050_driver.hpp"
#include "rclcpp/rclcpp.hpp"

namespace imu_madgwick
{
    class MadgwickFilter
    {
    public:
        /**Quaternion struct */
        struct Quaternion
        {
            double w;
            double x;
            double y;
            double z;
        };

        MadgwickFilter(double beta, Quaternion initial_pose);

        void update(
            const mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>::ImuData& imu_data,
            rclcpp::Duration dt);

        Quaternion normalize(const Quaternion & data);

    private:
        /**Previous orientation of the IMU modlue */
        Quaternion q_previous_;
        /**Filter coefficient */
        double beta_;
    };
}
#endif