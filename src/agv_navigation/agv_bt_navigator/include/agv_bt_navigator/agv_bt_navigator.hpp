#ifndef BT_NAVIGATOR_HPP_
#define BT_NAVIGATOR_HPP_

#include <memory>
#include <string>
#include <vector>
#include <glog/logging.h>
#include "agv_util/lifecycle_node.hpp"
#include "agv_util/agv_glog.hpp"
namespace agv_bt_navigator{
class AgvBtNavigator : public agv_util::LifecycleNode
{
public:
    explicit AgvBtNavigator(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    ~AgvBtNavigator();
};
}

#endif // BT_NAVIGATOR_HPP_