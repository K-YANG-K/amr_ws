#include <memory>
#include "amr_lifecycle_manager/lifecycle_manager.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<amr_lifecycle_manager::LifecycleManager>();
    rclcpp::spin(node);
    return 0;
}