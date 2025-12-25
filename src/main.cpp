#include "rclcpp/rclcpp.hpp"
#include "imu_sensor_cpp/node/imu_node.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

int main(int argc, char ** argv)
{
   rclcpp::init(argc, argv);
   
    auto imu_node = std::make_shared<imu_sensor_cpp::ImuNode>("imu_node", rclcpp::NodeOptions());

    RCLCPP_INFO(rclcpp::get_logger("main"), "Attempting to configure...");
    imu_node->configure(); 

    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Attempting to activate...");
    imu_node->activate();

    rclcpp::spin(imu_node->get_node_base_interface());

    rclcpp::shutdown();
    return 0;
}