#ifndef IMU_COMPLEMENTARY
#define IMU_COMPLEMENTARY

#include "imu_sensor_cpp/driver_core/mpu6050_driver.hpp"

namespace imu_complementary
{
    class ComplementaryFilter
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
        /**Euler angles struct */
        struct EulerAngles
        {
            double roll;
            double pitch;
            double yaw;
        };

        /**
        * @brief Configuration struct for Complementary Filter parameters.
        */
        struct ComplementaryFilterConfig {
            /** Complementary filter weight (0.0 - 1.0). Higher value trusts Gyro more. */
            double alpha; 
            /** Minimum acceptable gravity vector length [g] for correction. */
            double magnitude_low_threshold; 
            /** Maximum acceptable gravity vector length [g] for correction. */
            double magnitude_high_threshold;
            /** Threshold for Gimbal Lock detection on X-axis [g] (e.g., 0.97). */
            double gimbal_lock_threshold; 
        };   

        ComplementaryFilter(ComplementaryFilterConfig comp_filter_config);

        void update(const mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>::ImuData & imu_data,
                    double dt);

        static Quaternion euler_to_quaternion(EulerAngles orientation);

        EulerAngles get_current_orientation() const;

        void update_alpha(double alfa);

        void update_magnitude_low_threshold(double magnitude_low_threshold);

        void update_magnitude_high_threshold(double magnitude_high_threshold);

        void update_gimbal_lock_threshold(double gimbal_lock_threshold);
            
    private:
        /** Complementary Filter configuration parameters */
        ComplementaryFilterConfig comp_filter_config_;
        /**Last orientation of the sensor in Euler's angles form */
        EulerAngles last_orientation_;
    };
}
#endif