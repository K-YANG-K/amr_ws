#pragma once


#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "glog/logging.h"
#include "rclcpp/rclcpp.hpp"

namespace amr_behavior_tree
{


enum class BtStatus { SUCCEEDED, FAILED, CANCELED };


class BehaviorTreeEngine
{
public:

  explicit BehaviorTreeEngine(
    const std::vector<std::string> & bt_nodes);
  virtual ~BehaviorTreeEngine() {}

  BtStatus run(
    BT::Tree * tree,
    std::function<void()> onLoop,
    std::function<bool()> cancelRequested,
    std::chrono::milliseconds loopTimeout = std::chrono::milliseconds(10));

  BT::Tree createTreeFromText(
    const std::string & xml_string,
    BT::Blackboard::Ptr blackboard);

  BT::Tree createTreeFromFile(
    const std::string & file_path,
    BT::Blackboard::Ptr blackboard);

  void haltAllActions(BT::TreeNode * root_node);

protected:
  
  BT::BehaviorTreeFactory factory_;
};

} 

