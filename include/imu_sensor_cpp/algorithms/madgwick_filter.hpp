#ifndef IMU_MADGWICK
#define IMU_MADGWICK

#include "imu_sensor_cpp/driver_core/mpu6050_driver.hpp"

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

        static constexpr Quaternion INITIAL_POSE = {1.0, 0.0, 0.0, 0.0};

        MadgwickFilter(double beta, Quaternion initial_pose);

        bool update(
            const mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>::ImuData& imu_data,
            double dt);

        Quaternion get_current_orientation() const;

        void set_beta(double beta);
 

    private:

        /**Filter coefficient */
        double beta_;
        /**Previous orientation of the IMU modlue */
        Quaternion q_previous_;
        /**Numeric error const */
        static constexpr double EPSILON = 1e-6;
    };
}
#endif