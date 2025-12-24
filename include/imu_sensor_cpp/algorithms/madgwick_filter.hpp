#ifndef IMU_MADGWICK
#define IMU_MADGWICK

#include "imu_sensor_cpp/driver_core/mpu6050_driver.hpp"

namespace imu_madgwick
{
    /**
     * @brief Madgwick filter calculation class.
     */
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
        /** * Initial quaternion (Identity). 
         * Corresponds to "zero rotation" state where sensor frame aligns with world frame.
         */
        static constexpr Quaternion INITIAL_POSE = {1.0, 0.0, 0.0, 0.0};

        /**
         * @brief MadgwickFilter class constructor.
         * @param beta Value of beta coefficient in Madgwick filter structure.
         * @param initial_pose Value of expected gravity vector in first iteration, as we use right-front-up convention it's quaterion with only w = 1.0 end other coefficients are equals to 0.0
         */
        MadgwickFilter(double beta, Quaternion initial_pose);

        /**
         * @brief Updates the filter with new IMU measurement data.
         * * @details This implementation is robust against invalid accelerometer data.
         * If accelerometer norm is close to zero (e.g., free fall or sensor error),
         * the filter falls back to pure gyroscope integration to maintain continuity.
         * * @param imu_data Raw sensor data containing acceleration and angular velocity.
         * @param dt Time delta (in seconds) since the last update.
         * @return true if orientation was updated successfully (even without correction).
         * @return false only if a critical mathematical error occurred (normalization failure).
         */
        bool update(
            const mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>::ImuData& imu_data,
            double dt);

        /**
         * @brief Acquire current psition class member.
         * @return Current orientation in quaternion form.
         */
        Quaternion get_current_orientation() const;

        /**
         * @brief Updates the beta gain coefficient at runtime.
         * @param beta New beta value (higher = trust accel more, lower = trust gyro more).
         */
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