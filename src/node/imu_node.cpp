#include <string>
#include <memory>
#include <stdexcept>
#include <chrono>
#include <cmath>
#include <functional>
#include <eigen3/Eigen/Dense>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/qos.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "imu_sensor_cpp/driver_core/mpu6050_driver.hpp"
#include "imu_sensor_cpp/node/imu_node.hpp"

#include "imu_sensor_cpp/algorithms/madgwick_filter.hpp"
#include "imu_sensor_cpp/algorithms/complementary_filter.hpp"
#include "imu_sensor_cpp/algorithms/ekf.hpp"

using namespace imu_sensor_cpp;

ImuNode::ImuNode(const std::string & node_name, const rclcpp::NodeOptions & options)
:   
rclcpp_lifecycle::LifecycleNode(node_name, options),
qos_policy_(rclcpp::QoS(1).best_effort().durability_volatile()),
last_time_(this->get_clock()->now()),
accel_er_count_(0)
{
    imu_complementary::ComplementaryFilter::ComplementaryFilterConfig init_comp_config;
    double init_beta;
    std::vector<double> R_vector;
    std::vector<double> Q_vector;
    imu_ekf::ExtendedKalmanFilter::MeasurementMatrix R;
    imu_ekf::ExtendedKalmanFilter::StateMatrix Q;

    // Complementary filter parameters
    this->declare_parameter<double>("alfa", 0.98);
    this->declare_parameter<double>("magnitude_low_threshold", 0.85); // Used by ekf and comp
    this->declare_parameter<double>("magnitude_high_threshold", 1.15);// Used by ekf and comp
    this->declare_parameter<double>("gimbal_lock_threshold", 0.97);
    // Madgwick filter parameter
    this->declare_parameter<double>("beta", 0.1);
    // EKF filter parameters
    this->declare_parameter<std::vector<double>>("R", std::vector<double>{6.0, 6.0, 6.0});
    this->declare_parameter<std::vector<double>>("Q", std::vector<double>{0.01, 0.01, 0.01, 0.01});
    // State machine parameter
    this->declare_parameter<std::string>("mode", "ekf");
    // Publisher parameters
    this->declare_parameter<std::string>("frame_id", "imu_link");
    // MPU6050 calibration parameters
    this->declare_parameter<bool>("start_calibration", false);
    this->declare_parameter<bool>("delete_calibration_data", false);

    this->get_parameter("alfa", init_comp_config.alpha);
    this->get_parameter("magnitude_low_threshold", init_comp_config.magnitude_low_threshold);
    this->get_parameter("magnitude_high_threshold", init_comp_config.magnitude_high_threshold);
    this->get_parameter("gimbal_lock_threshold", init_comp_config.gimbal_lock_threshold);
    this->get_parameter("beta", init_beta);
    this->get_parameter("R", R_vector);
    this->get_parameter("Q", Q_vector);
    this->get_parameter("mode", mode_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("delete_calibration_data", delete_calibration_data_);

    // On set parameters callback handle
    param_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> & parameters)
        {   
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;

            for (const auto & param : parameters){
                if (param.get_name() == "alfa"){
                    double val = param.as_double();
                    if (val < 0.0 || val > 1.0) {
                        result.successful = false;
                        result.reason = "Alfa must be between 0.0 and 1.0";
                        return result; 
                    }
                    complementary_filter_->update_alpha(val);
                } else if (param.get_name() == "magnitude_low_threshold"){
                    complementary_filter_->update_magnitude_low_threshold(param.as_double());
                } else if (param.get_name() == "magnitude_high_threshold"){
                    complementary_filter_->update_magnitude_high_threshold(param.as_double());
                } else if (param.get_name() == "gimbal_lock_threshold"){
                    double val = param.as_double();
                    if (val < 0.7 || val > 1.0) {
                        result.successful = false;
                        result.reason = "gimbal_lock_threshold must be between 0.7 and 1.0";
                        return result; 
                    }
                    complementary_filter_->update_gimbal_lock_threshold(val);
                } else if (param.get_name() == "beta"){
                    double val = param.as_double();
                    if (val <= 0.0) {
                        result.successful = false;
                        result.reason = "Beta must be greater than 0.0";
                        return result; 
                    }
                    madgwick_filter_->set_beta(val);
                } else if (param.get_name() == "R"){
                    std::vector<double> val = param.as_double_array();
                    if (val.size() != 3){
                        result.successful = false;
                        result.reason = "R must be 3 element vector";
                        return result;
                    }
                    extended_kalman_filter_->setR(val);
                } else if (param.get_name() == "Q"){
                    std::vector<double> val = param.as_double_array();
                    if (val.size() != 4){
                        result.successful = false;
                        result.reason = "Q must be 3 element vector";
                        return result;
                    }
                    extended_kalman_filter_->setQ(val);   
                } else if (param.get_name() == "mode"){
                    std::string val_str = param.as_string();
                    if (val_str == "comp" || val_str == "madg" || val_str == "ekf"){
                        if (val_str == "ekf"){
                            extended_kalman_filter_->init_first_run();
                        } 
                        mode_ = val_str;
                    } else {
                        result.successful = false;
                        result.reason = "mode must be either 'comp', 'madg' or 'ekf'";
                        return result;
                    }
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
            return result;
        }
    );

    // Initializing complementary filter class
    complementary_filter_ = std::make_unique<imu_complementary::ComplementaryFilter>(
        imu_complementary::ComplementaryFilter(init_comp_config));

    // Initializing madgwick filter class
    madgwick_filter_ = std::make_unique<imu_madgwick::MadgwickFilter>(
        imu_madgwick::MadgwickFilter(init_beta ,imu_madgwick::MadgwickFilter::INITIAL_POSE));

    //Initializing ekf filter class
    //Maping vector into eigen objects
    R = Eigen::Map<const Eigen::Vector3d>(R_vector.data()).asDiagonal();
    Q = Eigen::Map<const Eigen::Vector4d>(Q_vector.data()).asDiagonal();
    extended_kalman_filter_ = std::make_unique<imu_ekf::ExtendedKalmanFilter>(
        imu_ekf::ExtendedKalmanFilter(R, Q));
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
    if (first_run_){
        last_time_ = this->get_clock()->now();
        first_run_ = false;

    } else {
        sensor_msgs::msg::Imu imu_msg;
        bool publish_data = false;

        try{
            auto imu_data = imu_driver_->getAllData(true);
            auto dt = get_dt();
            // Calculating unexpected accelerometer measurement for system health monitoring
            double norm = std::sqrt(imu_data.accel_x * imu_data.accel_x +
                                    imu_data.accel_y * imu_data.accel_y +
                                    imu_data.accel_z * imu_data.accel_z);

            if(norm < 1e-4){
                accel_er_count_ += 1;
                if (accel_er_count_ > 15){
                    RCLCPP_WARN(this->get_logger(),
                                "Accelerometer periodic zero measurement, perform basic diagnostics");
                    accel_er_count_ = 0;
                }
            } else {
                accel_er_count_ = 0;
            }

            // Logging raw data at 1 second intervals
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "\nAccelerations\nx: %.2f g\ty: %.2f g\tz: %.2f g\nAngilar velocities\nx: %.2f rad/s\ty: %.2f rad/s\tz: %.2f rad/s",
                imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
                imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);

            // State machine
            if (mode_ == "comp"){
                complementary_filter_->update(imu_data, dt.seconds());
                auto q_current = complementary_filter_->euler_to_quaternion(complementary_filter_->get_current_orientation());
                imu_msg.orientation.w = q_current.w;
                imu_msg.orientation.x = q_current.x;
                imu_msg.orientation.y = q_current.y;
                imu_msg.orientation.z = q_current.z;
                imu_msg.header.stamp = last_time_;
                publish_data = true;
            } else if (mode_ == "madg"){
                if (madgwick_filter_->update(imu_data, dt.seconds())){
                    auto q_current = madgwick_filter_->get_current_orientation();
                    imu_msg.orientation.w = q_current.w;
                    imu_msg.orientation.x = q_current.x;
                    imu_msg.orientation.y = q_current.y;
                    imu_msg.orientation.z = q_current.z;
                    imu_msg.header.stamp = last_time_;
                    publish_data = true;
                } else {
                    RCLCPP_ERROR(this->get_logger(), 
                                "Madgwick filter error occured (normalization error).");
                    publish_data = false;
                }
            } else if (mode_ == "ekf"){
                extended_kalman_filter_->init(imu_data);
                extended_kalman_filter_->predict(imu_data, dt.seconds());
                extended_kalman_filter_->update(imu_data);
                auto q = extended_kalman_filter_->get_state();
                imu_msg.orientation.w = q(0);
                imu_msg.orientation.x = q(1);
                imu_msg.orientation.y = q(2);
                imu_msg.orientation.z = q(3);
                imu_msg.header.stamp = last_time_;
                publish_data = true;
            }
            if (publish_data){
                imu_msg.header.frame_id = frame_id_;
                imu_publisher_->publish(imu_msg);
            }
        } catch (std::exception& e){
            RCLCPP_ERROR(this->get_logger(), "IMU sensor node malfunction: %s", e.what());
        }
    }
}

rclcpp::Duration ImuNode::get_dt(){
    auto current_time = this->get_clock()->now();
    auto dt = current_time - last_time_;
    last_time_ = current_time;
    return dt;
}