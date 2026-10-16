#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "amr_bt_navigator/amr_bt_navigator.hpp"

int main(int argc,char* argv[]){
    
    rclcpp::init(argc,argv);
    std::shared_ptr<amr_bt_navigator::AmrBtNavigator> node;
    node = std::make_shared<amr_bt_navigator::AmrBtNavigator>();
    auto node_base_interface = node->get_node_base_interface();
    if (node_base_interface == nullptr) {
        LOG(ERROR) <<"amr_bt_navigator 节点创建失败!";
        return 1;
    }
    rclcpp::spin(node_base_interface);
    return 0;
}
