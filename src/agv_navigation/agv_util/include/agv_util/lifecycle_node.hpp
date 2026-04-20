#ifndef AGV_UTIL_LIFECYCLE_NODE_HPP_
#define AGV_UTIL_LIFECYCLE_NODE_HPP_
#include <memory>
#include <string>
#include <thread>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "bondcpp/bond.hpp"
#include "bond/msg/constants.hpp"

#include "glog/logging.h"
namespace agv_util
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
class LifecycleNode : public rclcpp_lifecycle::LifecycleNode{
  public:
    LifecycleNode(
        const std::string & node_name,
        const std::string & node_namespace = "",
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~LifecycleNode();
    
    std::shared_ptr<agv_util::LifecycleNode> shared_from_this(){
        return std::static_pointer_cast<agv_util::LifecycleNode>(
            rclcpp_lifecycle::LifecycleNode::shared_from_this());
    }
    void createBond();
    void destroyBond();
  protected:
    std::unique_ptr<bond::Bond> bond_ = nullptr;

};
}
#endif