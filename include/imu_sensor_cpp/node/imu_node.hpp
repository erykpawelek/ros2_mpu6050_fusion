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

#include "imu_sensor_cpp/algorithms/madgwick_filter.hpp"
#include "imu_sensor_cpp/algorithms/complementary_filter.hpp"
#include "imu_sensor_cpp/algorithms/ekf.hpp"

namespace imu_sensor_cpp
{   

    class ImuNode : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        
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
         * @brief Calculates time step between each cycle.
         * @return rclcpp::Duration class object.
         */
        rclcpp::Duration get_dt();

    private:
        // --- ROS Interfaces ---
        /** ROS Quality of Service profile */
        rclcpp::QoS qos_policy_;
        /** ROS Lifecycle publisher */
        rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
        /** ROS timer for periodic data acquisition */
        rclcpp::TimerBase::SharedPtr timer_;
        /** Parameter change callback handle */
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

        // --- Hardware Drivers ---
        /** Linux I2C interface wrapper */
        std::unique_ptr<mpu6050cust_driver::LinuxI2C> i2c_interface_;
        /** MPU6050 sensor driver logic */
        std::unique_ptr<mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>> imu_driver_;
        /** Complementary filter math class */
        std::unique_ptr<imu_complementary::ComplementaryFilter> complementary_filter_;
        /** Madgwick filter math class */
        std::unique_ptr<imu_madgwick::MadgwickFilter> madgwick_filter_;
        /** Extended Kalman Filter class */
        std::unique_ptr<imu_ekf::ExtendedKalmanFilter> extended_kalman_filter_;

        // --- Internal State ---
        /** Timestamp of the previous iteration for dt calculation */
        builtin_interfaces::msg::Time last_time_;
        /** Initialization flag to skip first dt calculation */
        bool first_run_ = true;
        /**Madgwick failure count*/
        int accel_er_count_;
        
            
        // --- Parameters ---
        /** Frame ID for IMU messages */
        std::string frame_id_;
        /** Flag to delete calibration data */
        bool delete_calibration_data_;
        /**Operating filter mode*/
        std::string mode_;
        /**Gyroscope dead zone range range: (0, +-gyro_deadzone) */
        double gyro_deadzone_;
    };
}
#endif //IMU_SENSOR_NODE_HPP