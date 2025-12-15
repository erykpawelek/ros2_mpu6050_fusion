#include "rclcpp/rclcpp.hpp"
#include "imu_sensor_cpp/platform/imu_node.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

int main(int argc, char ** argv)
{
   rclcpp::init(argc, argv);

    // 2. Tworzenie węzła
    // Używamy make_shared, aby mieć shared_ptr, który jest potrzebny do spinowania
    auto imu_node = std::make_shared<imu_sensor_cpp::ImuNode>("imu_node", rclcpp::NodeOptions());

    // 3. AUTOMATYCZNA KONFIGURACJA
    // Wymuszamy przejście do stanu Inactive
    RCLCPP_INFO(rclcpp::get_logger("main"), "Attempting to configure...");
    imu_node->configure(); 

    // 4. AUTOMATYCZNA AKTYWACJA
    // Wymuszamy przejście do stanu Active (teraz zaczną lecieć dane!)
    RCLCPP_INFO(rclcpp::get_logger("main"), "Attempting to activate...");
    imu_node->activate();

    // 5. Uruchomienie pętli (Spin)
    // Uwaga: LifecycleNode wymaga pobrania "bazy" do spina
    rclcpp::spin(imu_node->get_node_base_interface());

    // 6. Sprzątanie
    rclcpp::shutdown();
    return 0;
}