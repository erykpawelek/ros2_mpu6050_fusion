#ifndef IMU_SENSOR_NODE_HPP
#define IMU_SENSOR_NODE_HPP

#include <string>
#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/qos.hpp"

#include "imu_sensor_cpp/core/mpu6050_driver.hpp"

namespace imu_sensor_cpp
{
    class ImuNode : public rclcpp_lifecycle::LifecycleNode
    {
    public:
    
        ImuNode(const std::string & node_name, const rclcpp::NodeOptions & options);


        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &) override;

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &) override;
            
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &) override;

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &) override;

        void publisher_callback();

        sensor_msgs::msg::Imu complementary_filter(const mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>::ImuData & imu_data, float alfa);

    private:
    rclcpp::QoS qos_policy_;
    std::unique_ptr<mpu6050cust_driver::LinuxI2C> i2c_interface_;
    std::unique_ptr<mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>> imu_driver_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    builtin_interfaces::msg::Time last_time_;
    double last_roll_ = 0.0;
    double last_pitch_ = 0.0;
    double last_yaw_ = 0.0;
    
    };
    
}
#endif //IMU_SENSOR_NODE_HPP