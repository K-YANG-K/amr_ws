#include <memory>
#include <string>

#include "amr_behavior_tree/plugins/action/generate_follow_path_action.hpp"

namespace amr_behavior_tree
{
GenerateFollowPathAction::GenerateFollowPathAction(
      const std::string & xml_tag_name,
      const std::string & action_name,
      const BT::NodeConfiguration & conf)
      : amr_behavior_tree::BtActionNode<amr_msgs::action::GenerateFollowPath>(xml_tag_name, action_name, conf){}//调用父类构造函数创建client来send goal

void GenerateFollowPathAction::on_tick(){
    getInput("nav_a_goal",goal_.nav_a_goal);
    getInput("planner_id",goal_.planner_id);
    getInput("nav_speed",goal_.nav_speed);
}
BT::NodeStatus GenerateFollowPathAction::on_success(){
    setOutput("global_path",result_.result->global_path);
    setOutput("nav_status", result_.result->nav_status);
    return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus GenerateFollowPathAction::on_aborted(){
    nav_msgs::msg::Path empty_path;
    setOutput("nav_a_goal",empty_path);
    setOutput("nav_status", result_.result->nav_status);
    return BT::NodeStatus::FAILURE;
}
BT::NodeStatus GenerateFollowPathAction::on_cancelled(){
    nav_msgs::msg::Path empty_path;
    setOutput("nav_a_goal", empty_path);
    return BT::NodeStatus::SUCCESS;
}

void GenerateFollowPathAction::halt(){
    nav_msgs::msg::Path empty_path;
    setOutput("nav_a_goal", empty_path);
    amr_behavior_tree::BtActionNode<amr_msgs::action::GenerateFollowPath>::halt();
}
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<amr_behavior_tree::GenerateFollowPathAction>(
        name, amr_msgs::action_names::GENERATE_FOLLOW_PATH, config);
    };

  factory.registerBuilder<amr_behavior_tree::GenerateFollowPathAction>(
    "GenerateFollowPath", builder);
}
