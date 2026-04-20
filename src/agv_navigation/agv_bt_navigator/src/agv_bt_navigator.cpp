#include "agv_bt_navigator/agv_bt_navigator.hpp"

#include <memory>
#include <string>

namespace agv_bt_navigator{
    AgvBtNavigator::AgvBtNavigator(rclcpp::NodeOptions options):
    agv_util::LifecycleNode("agv_bt_navigator", "", options){
        LOG(INFO)<<"创建bt!";
    }

    AgvBtNavigator::~AgvBtNavigator() = default;
}