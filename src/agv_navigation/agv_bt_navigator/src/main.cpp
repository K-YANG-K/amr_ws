#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "agv_bt_navigator/agv_bt_navigator.hpp"

int main(int argc,char* argv[]){
    
    rclcpp::init(argc,argv);
    agv_util::InitGlog(argv, "agv_bt_navigator", "");
    std::cout << "ROS 2 initialized" << std::endl;

    std::shared_ptr<agv_bt_navigator::AgvBtNavigator> node;
    node = std::make_shared<agv_bt_navigator::AgvBtNavigator>();

    std::cout << "Node created" << std::endl;

    // 检查接口是否有效
    auto node_base_interface = node->get_node_base_interface();
    if (!node_base_interface) {
        std::cerr << "Error: node_base_interface is null!" << std::endl;
        return 1;
    }

    std::cout << "Starting spin..." << std::endl;
    rclcpp::spin(node_base_interface);

    std::cout << "Spin finished" << std::endl;
    return 0;
}
