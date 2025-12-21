#include <string>
#include <memory>
#include <stdexcept>
#include <chrono>
#include <cmath>
#include <mutex>
#include <functional>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/qos.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "imu_sensor_cpp/driver_core/mpu6050_driver.hpp"
#include "imu_sensor_cpp/driver_platform/imu_node.hpp"

using namespace imu_sensor_cpp;

ImuNode::ImuNode(const std::string & node_name, const rclcpp::NodeOptions & options)
:   
rclcpp_lifecycle::LifecycleNode(node_name, options),
qos_policy_(rclcpp::QoS(1).best_effort().durability_volatile()),
last_time_(this->get_clock()->now())
{
    this->declare_parameter<double>("alfa", 0.98);
    this->declare_parameter<double>("magnitude_low_threshold", 0.85);
    this->declare_parameter<double>("magnitude_high_threshold", 1.15);
    this->declare_parameter<double>("gimbal_lock_threshold", 0.97);
    this->declare_parameter<std::string>("frame_id", "imu_link");
    this->declare_parameter<bool>("start_calibration", false);
    this->declare_parameter<bool>("delete_calibration_data", false);

    this->get_parameter("alfa", comp_filter_config_.alpha);
    this->get_parameter("magnitude_low_threshold", comp_filter_config_.magnitude_low_threshold);
    this->get_parameter("magnitude_high_threshold", comp_filter_config_.magnitude_high_threshold);
    this->get_parameter("gimbal_lock_threshold", comp_filter_config_.gimbal_lock_threshold);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("delete_calibration_data", delete_calibration_data_);

    param_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> & parameters)
        {   
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            {
                std::lock_guard<std::mutex> lock(this->param_mutex_);

                for (const auto & param : parameters){
                    if (param.get_name() == "alfa"){
                        double val = param.as_double();
                        if (val < 0.0 || val > 1.0) {
                            result.successful = false;
                            result.reason = "Alfa must be between 0.0 and 1.0";
                            return result; 
                        }
                        comp_filter_config_.alpha = val;
                    } else if (param.get_name() == "magnitude_low_threshold"){
                        comp_filter_config_.magnitude_low_threshold = param.as_double();
                    } else if (param.get_name() == "magnitude_high_threshold"){
                        comp_filter_config_.magnitude_high_threshold = param.as_double();
                    } else if (param.get_name() == "gimbal_lock_threshold"){
                        double val = param.as_double();
                        if (val < 0.7 || val > 1.0) {
                            result.successful = false;
                            result.reason = "gimbal_lock_threshold must be between 0.7 and 1.0";
                            return result; 
                        }
                        comp_filter_config_.gimbal_lock_threshold = val;
                    } else if (param.get_name() == "frame_id"){
                        frame_id_ = param.as_string();
                    } else if (param.get_name() == "start_calibration"){
                        if (param.as_bool() == true){
                            if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE){
                                RCLCPP_WARN(this->get_logger(), "Calibration started...");
                                imu_driver_->calibrate();
                                RCLCPP_INFO(this->get_logger(), "Calibration DONE.");
                            }
                        }
                    } else if (param.get_name() == "delete_calibration_data"){
                           if (param.as_bool() == true){
                            if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE){
                                RCLCPP_WARN(this->get_logger(), "Calibration started...");
                                imu_driver_->delete_calibration_data();
                                RCLCPP_INFO(this->get_logger(), "Calibration DONE.");
                            }
                        }
                    }
                }
            }
            return result;
        }
    );
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
        std::chrono::milliseconds(10),
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
{   
    // Copying parameters under mutex to prevent race conditions when using multi threading Nodes.
    ImuNode::ComplementaryFilterConfig comp_filter_config_copy;
    {
        std::lock_guard<std::mutex> lock(param_mutex_);
        comp_filter_config_copy = comp_filter_config_;

    }

    auto imu_data = imu_driver_->getAllData(true);
    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "\nAccelerations\nx: %.2f g\ty: %.2f g\tz: %.2f g\nAngilar velocities\nx: %.2f rad/s\ty: %.2f rad/s\tz: %.2f rad/s",
        imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
        imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);

    sensor_msgs::msg::Imu imu_msg = complementary_filter(imu_data, comp_filter_config_copy);
    imu_msg.header.frame_id = frame_id_;
    imu_publisher_->publish(imu_msg);
}

sensor_msgs::msg::Imu ImuNode::complementary_filter(
    const mpu6050cust_driver::MPU6050CustomDriver<mpu6050cust_driver::LinuxI2C>::ImuData & imu_data,
    ImuNode::ComplementaryFilterConfig comp_filter_config_copy)
{
    sensor_msgs::msg::Imu imu_msg;
    // First iteration empty for time data initialization
    if (first_run_){
        last_time_ = this->get_clock()->now();
        first_run_ = false;
        imu_msg.header.stamp = last_time_;
        imu_msg.header.frame_id = "imu_link";
        imu_msg.orientation.set__w(1.0);
        return imu_msg;
    }

    float var_alfa_x, var_alfa_y;
    double roll, pitch, yaw;
    double accel_roll_part;

    // Geting time delta 
    auto current_time = this->get_clock()->now();
    auto dt = current_time - last_time_;
    last_time_ = current_time;

    // Calculating magnitude of acceleration vector, to prevent errors during large accelerations
    float magnitude = std::sqrt(imu_data.accel_x * imu_data.accel_x +
                                imu_data.accel_y * imu_data.accel_y +
                                imu_data.accel_z * imu_data.accel_z);
    bool magnitude_ok = (magnitude < comp_filter_config_copy.magnitude_high_threshold &&
         magnitude > comp_filter_config_copy.magnitude_low_threshold);

    // Gimball lock prevention
    bool orientation_x_ok = (imu_data.accel_x < comp_filter_config_copy.gimbal_lock_threshold &&
         imu_data.accel_x > -comp_filter_config_copy.gimbal_lock_threshold);

    if (magnitude_ok){
        var_alfa_y = comp_filter_config_copy.alpha;
        if (orientation_x_ok){
            var_alfa_x = comp_filter_config_copy.alpha;
            accel_roll_part = (std::atan2(imu_data.accel_y, imu_data.accel_z));
        } else {
            var_alfa_x = 1.0; // Gyro only
            accel_roll_part = 0.0;
        }
    } else {
        var_alfa_x = 1.0; // Gyro only
        var_alfa_y = 1.0; // Gyro only
        accel_roll_part = 0.0;
    }
    

    // ---- ROLL CALCULATIONS ----
    
    double prediction_x = last_roll_ + (imu_data.gyro_x * dt.seconds());
    double error_x = accel_roll_part - prediction_x;
    // Using atan2() normalization to prevent discontuinities in angle calculation when close to 180/-180 degree
    if (error_x < -M_PI){
        error_x = error_x + 2 * M_PI;
    } else if (error_x > M_PI){
        error_x = error_x - 2 * M_PI;
    } 
    roll = prediction_x + (1.0 - var_alfa_x) * (error_x);

    // ---- PITCH CALCULATIONS ----

    double prediction_y = last_pitch_ + (imu_data.gyro_y * dt.seconds());
    double error_y = (std::atan2(-imu_data.accel_x, std::sqrt(imu_data.accel_y * imu_data.accel_y +
                                                              imu_data.accel_z * imu_data.accel_z)))
        - prediction_y  ;

    if (error_y < -M_PI){
        error_y = error_y + 2 * M_PI;
    } else if (error_y > M_PI){
        error_y = error_y - 2 * M_PI;
    } 
    pitch = prediction_y + (1.0 - var_alfa_y) * (error_y);

    // ---- YAW CALCULATIONS ----

    yaw = last_yaw_ + (imu_data.gyro_z * dt.seconds());

    // Saving last angles as calss members for safety
    last_roll_ = roll;
    last_pitch_ = pitch;
    last_yaw_ = yaw;

    // Message packing
    imu_msg.orientation = euler_to_quaternion(last_roll_, last_pitch_, last_yaw_);
    imu_msg.header.stamp = current_time;
    
    return imu_msg;
}

geometry_msgs::msg::Quaternion ImuNode::euler_to_quaternion(double roll, double pitch, double yaw)
{
    geometry_msgs::msg::Quaternion quaternion;
    // Defining quaternion's sinus and cosinus values form RPY angles
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    // Calculating union quaternion elements
    quaternion.w = cy * cr * cp + sy * sr * sp; // w
    quaternion.x = cy * sr * cp - sy * cr * sp; // x
    quaternion.y = cy * cr * sp + sy * sr * cp; // y
    quaternion.z = sy * cr * cp - cy * sr * sp; // z

    return quaternion;
}


