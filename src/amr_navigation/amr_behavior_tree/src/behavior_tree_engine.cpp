#include "amr_behavior_tree/behavior_tree_engine.hpp"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "rclcpp/rclcpp.hpp"

namespace amr_behavior_tree{
BehaviorTreeEngine::BehaviorTreeEngine(const std::vector<std::string>& bt_nodes){
    BT::SharedLibrary loader;
    for(const auto & node : bt_nodes){
        factory_.registerFromPlugin(loader.getOSName(node));
    }
}
//tick整个行为树，控制执行频率
BtStatus BehaviorTreeEngine::run(
    BT::Tree* tree,
    std::function<void()> onLoop,
    std::function<bool()> cancelRequested,
    std::chrono::milliseconds loopTimeout
){
    rclcpp::WallRate loopRate(loopTimeout);
    BT::NodeStatus result = BT::NodeStatus::RUNNING;
    try{
        while(rclcpp::ok() && result == BT::NodeStatus::RUNNING){
            if(cancelRequested()){
                tree->rootNode()->halt();
                return BtStatus::CANCELED;
            }
            result = tree->tickRoot();
            onLoop();
            if(!loopRate.sleep()){
                LOG(WARNING)<<"行为树执行频率:"<<1.0/(loopRate.period().count()*1.0e-9)<<"频率较低!";
            }
        }
    }catch (const std::exception & ex){
        LOG(ERROR)<<"行为树异常: "<<ex.what();
        return BtStatus::FAILED;
    }

    return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

BT::Tree BehaviorTreeEngine::createTreeFromText(
  const std::string & xml_string,
  BT::Blackboard::Ptr blackboard)
{
  return factory_.createTreeFromText(xml_string, blackboard);
}

BT::Tree BehaviorTreeEngine::createTreeFromFile(
  const std::string & file_path,
  BT::Blackboard::Ptr blackboard)
{
  return factory_.createTreeFromFile(file_path, blackboard);
}

void BehaviorTreeEngine::haltAllActions(BT::TreeNode * root_node)
{
  if (!root_node) {
    return;
  }

  root_node->halt();

  auto visitor = [](BT::TreeNode * node) {
      if (node->status() == BT::NodeStatus::RUNNING) {
        node->halt();
      }
    };
  BT::applyRecursiveVisitor(root_node, visitor);
}

}