#include <string>
#include <memory>
#include <stdexcept>
#include <chrono>
#include <cmath>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/qos.hpp"

#include "imu_sensor_cpp/core/mpu6050_driver.hpp"
#include "imu_sensor_cpp/platform/imu_node.hpp"

using namespace imu_sensor_cpp;

ImuNode::ImuNode(const std::string & node_name, const rclcpp::NodeOptions & options)
:   
rclcpp_lifecycle::LifecycleNode(node_name, options),
qos_policy_(rclcpp::QoS(1).best_effort().durability_volatile())
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ImuNode::on_configure(
    const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "Configuring...");
    try{
        // Driver interface initialization
        i2c_interface_ = std::make_unique<mpu6050cust_driver::LinuxI2C>(mpu6050cust_driver::LinuxI2C(1));
        // Driver initialization
        imu_driver_ = std::make_unique<mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>>(
            mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>(
                *i2c_interface_,
                0x68,
                mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>::DLPF_94_BAND,
                mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>::GYRO_RANGE_250,
                mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>::ACCEL_RANGE_4
            )
        );
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", qos_policy_);

    } catch(const std::exception & e){
        RCLCPP_ERROR_STREAM(this->get_logger(), "Configuration error: " << e.what());
        return CallbackReturn::FAILURE;
    }

    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ImuNode::on_activate(
    const rclcpp_lifecycle::State &)
{
    imu_publisher_->on_activate();

    timer_ = this->create_timer(
        std::chrono::milliseconds(100),
        std::bind(&ImuNode::publisher_callback, this));
    
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ImuNode::on_deactivate(
    const rclcpp_lifecycle::State &)
{   
    try{
        imu_publisher_->on_deactivate();
        timer_->reset();
    } catch(const std::exception & e){
        RCLCPP_ERROR_STREAM(this->get_logger(), "Deactivation error: " << e.what());
        return CallbackReturn::FAILURE;
    }
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ImuNode::on_cleanup(
    const rclcpp_lifecycle::State &)
{
    imu_driver_.reset();
    i2c_interface_.reset();
    imu_publisher_.reset();

    return CallbackReturn::SUCCESS;
}

void ImuNode::publisher_callback()
{   /** 
    auto imu_data = imu_driver_->getAllData(true);
    RCLCPP_INFO_STREAM(this->get_logger(), "Accelerations\nx: " << imu_data.accel_x << "\ty: " << imu_data.accel_y << "\tz: " << imu_data.accel_z);
    RCLCPP_INFO_STREAM(this->get_logger(), "Angilar velocities\nx: " << imu_data.gyro_x << "\ty: " << imu_data.gyro_y << "\tz: " << imu_data.gyro_z);
    */
}

sensor_msgs::msg::Imu ImuNode::complementary_filter(
    const mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>::ImuData & imu_data, float alfa)
{
    sensor_msgs::msg::Imu imu_msg;

    // Geting time delta 
    auto current_time = this->get_clock()->now();
    auto dt = current_time - last_time_;
    last_time_ = current_time;
    // Calculating euler angles
    double roll = (alfa) *(last_roll_ + (imu_data.gyro_x * dt.seconds())) + (1.0 - alfa) * (std::atan2(imu_data.accel_y, imu_data.accel_z));

    double pitch = (alfa) * (last_pitch_ + (imu_data.gyro_y * dt.seconds())) + (1.0 - alfa) * (std::atan2(-imu_data.accel_x, std::sqrt(
        imu_data.accel_y * imu_data.accel_y + imu_data.accel_z * imu_data.accel_z)));

    double yaw = last_yaw_ + (imu_data.gyro_z * dt.seconds());

    last_roll_ = roll;
    last_pitch_ = pitch;
    last_yaw_ = yaw;


    return imu_msg;
}


