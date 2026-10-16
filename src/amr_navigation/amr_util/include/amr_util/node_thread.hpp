#pragma once 
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "glog/logging.h"
namespace amr_util
{
class NodeThread{
public:
  explicit NodeThread(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base);
  explicit NodeThread(rclcpp::executors::SingleThreadedExecutor::SharedPtr executor);
  template<typename NodeT>
  explicit NodeThread(NodeT node): NodeThread(node->get_node_base_interface()){}

  ~NodeThread();

protected:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;
  std::unique_ptr<std::thread> thread_;
  rclcpp::Executor::SharedPtr executor_;
};
}