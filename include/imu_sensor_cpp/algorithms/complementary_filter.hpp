#ifndef IMU_COMPLEMENTARY
#define IMU_COMPLEMENTARY

#include "imu_sensor_cpp/driver_core/mpu6050_driver.hpp"

namespace imu_complementary
{
    /**
     * @brief Robust Complementary Filter implementation for IMU sensor fusion.
     * * This class estimates orientation (Roll, Pitch, Yaw) by fusing Gyroscope (high-frequency) 
     * and Accelerometer (low-frequency reference) data.
     * * Key features:
     * - Acceleration Magnitude Check: Ignores accelerometer correction during dynamic motion/impacts.
     * - Gimbal Lock Prevention: Disables Roll correction when Pitch approaches +/- 90 degrees.
     * - Angle Wrapping: Handles discontinuities at +/- 180 degrees (PI).
     */
    class ComplementaryFilter
    {
    public:
        /** @brief Structure representing rotation as a Quaternion (w, x, y, z). */
        struct Quaternion
        {
            double w;
            double x;
            double y;
            double z;
        };
        /** @brief Structure representing orientation as Euler Angles (radians). */
        struct EulerAngles
        {
            double roll;
            double pitch;
            double yaw;
        };

        /**
         * @brief Configuration structure for filter tuning and safety thresholds.
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

        /**
         * @brief Construct a new Complementary Filter object.
         * @param comp_filter_config Initial configuration parameters.
         */
        ComplementaryFilter(ComplementaryFilterConfig comp_filter_config);

        /**
         * @brief Updates the filter state with new sensor data.
         * * @details This function performs the sensor fusion. It checks if the accelerometer
         * data is reliable (based on magnitude thresholds). If the robot is experiencing 
         * high dynamic acceleration (free fall or impact), the accelerometer correction 
         * is temporarily disabled to prevent orientation estimation errors.
         * * @param imu_data Raw sensor data struct from MPU6050 driver.
         * @param dt Time delta (in seconds) since the last update.
         */
        void update(const mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>::ImuData & imu_data,
                    double dt);

        /**
         * @brief Helper utility to convert Euler Angles to Quaternion.
         * @note Static method, does not depend on the filter instance state.
         * @param orientation Input orientation in Euler Angles (radians).
         * @return Quaternion representation of the orientation.
         */
        static Quaternion euler_to_quaternion(EulerAngles orientation);

        /**
         * @brief Get the current estimated orientation.
         * @return Current orientation in Euler Angles (Roll, Pitch, Yaw).
         */
        EulerAngles get_current_orientation() const;

        /**
         * @brief Updates the alpha coefficient at runtime.
         * @param alpha New alpha value (0.0 - 1.0).
         */
        void update_alpha(double alfa);

        /**
         * @brief Updates the low threshold for gravity vector validation.
         * @param magnitude_low_threshold Lower limit in g (e.g. 0.8).
         */
        void update_magnitude_low_threshold(double magnitude_low_threshold);

        /**
         * @brief Updates the high threshold for gravity vector validation.
         * @param magnitude_high_threshold Upper limit in g (e.g. 1.2).
         */
        void update_magnitude_high_threshold(double magnitude_high_threshold);

        /**
         * @brief Updates the gimbal lock safety threshold.
         * @param gimbal_lock_threshold X-axis acceleration limit (e.g. 0.97).
         */
        void update_gimbal_lock_threshold(double gimbal_lock_threshold);
            
    private:
        /** Internal copy of configuration parameters */
        ComplementaryFilterConfig comp_filter_config_;
        
        /** Current state of orientation (accumulated over time) */
        EulerAngles last_orientation_;
    };
}
#endif