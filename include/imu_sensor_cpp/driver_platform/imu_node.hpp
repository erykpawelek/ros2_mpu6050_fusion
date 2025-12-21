#ifndef IMU_SENSOR_NODE_HPP
#define IMU_SENSOR_NODE_HPP

#include <string>
#include <memory>
#include <vector>
#include <mutex>
#include <cmath>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/qos.hpp"

#include "imu_sensor_cpp/driver_core/mpu6050_driver.hpp"

namespace imu_sensor_cpp
{   

    class ImuNode : public rclcpp_lifecycle::LifecycleNode
    {
    public:
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
        
        /**
         * @brief ImuNode class constructor.
         * @param node_name Name of the node.
         * @param options Additional options to control creation of the node.
         */
        ImuNode(const std::string & node_name, const rclcpp::NodeOptions & options);

        /**
         * @brief on_configure lifecycle transition callback.
         * @param previous_state Previous state of the node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &) override;
        
        /**
         * @brief on_activate lifecycle transition callback.
         * @param previous_state Previous state of the node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &) override;
        
        /**
         * @brief on_deactivate lifecycle transition callback.
         * @param previous_state Previous state of the node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &) override;

        /**
         * @brief on_cleanup lifecycle transition callback.
         * @param previous_state Previous state of the node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &) override;

        /**
         * @brief Publisher callback function. 
         * Reads data from IMU sensor, processes it and publishes as Imu message.
         */
        void publisher_callback();
        
        /**
         * @brief Complementary filter implementation for orientation estimation.
         * Performs sensor fusion using accelerometer and gyroscope data.
         * Handles mathematical singularities (Gimbal Lock) and vibration rejection.
         * @param imu_data Struct containing raw IMU data (accel in g, gyro in rad/s).
         * @param alfa Filter coefficient (weight for gyroscope integration).
         * @return sensor_msgs::msg::Imu message with estimated orientation (Quaternion).
         */
        sensor_msgs::msg::Imu complementary_filter(const mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>::ImuData & imu_data,
            ImuNode::ComplementaryFilterConfig comp_filter_config_copy);
        
        /**
         * @brief Converts Euler angles to Quaternion representation.
         * Follows ROS REP-103 standard (Z-Y-X sequence implied).
         * @param roll Roll angle [rad]
         * @param pitch Pitch angle [rad]
         * @param yaw Yaw angle [rad]
         * @return geometry_msgs::msg::Quaternion
         */
        geometry_msgs::msg::Quaternion euler_to_quaternion(double roll, double pitch, double yaw);

    private:
        // --- ROS Interfaces ---
        /** ROS Quality of Service profile */
        rclcpp::QoS qos_policy_;
        /** ROS Lifecycle publisher */
        rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
        /** ROS timer for periodic data acquisition */
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

        // --- Hardware Drivers ---
        /** Linux I2C interface wrapper */
        std::unique_ptr<mpu6050cust_driver::LinuxI2C> i2c_interface_;
        /** MPU6050 sensor driver logic */
        std::unique_ptr<mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>> imu_driver_;

        //**Parameters mutex */
        mutable std::mutex param_mutex_;

        // --- Internal State ---
        /** Timestamp of the previous iteration for dt calculation */
        builtin_interfaces::msg::Time last_time_;
        /** Initialization flag to skip first dt calculation */
        bool first_run_ = true;
        
        
        /** Current Roll angle estimation [rad] */
        double last_roll_ = 0.0;
        /** Current Pitch angle estimation [rad] */
        double last_pitch_ = 0.0;
        /** Current Yaw angle estimation [rad] */
        double last_yaw_ = 0.0;
        
        // --- Parameters ---
        /** Frame ID for IMU messages */
        std::string frame_id_;
        /** Flag to delete calibration data */
        bool delete_calibration_data_;
        /** Complementary Filter configuration parameters */
        ComplementaryFilterConfig comp_filter_config_;
    };
}
#endif //IMU_SENSOR_NODE_HPP