#include <memory>
#include <string>

#include "amr_behavior_tree/plugins/action/plan_global_path_action.hpp"

namespace amr_behavior_tree
{
PlanGlobalPathAction::PlanGlobalPathAction(
      const std::string & xml_tag_name,
      const std::string & action_name,
      const BT::NodeConfiguration & conf)
      : amr_behavior_tree::BtActionNode<amr_msgs::action::PlanGlobalPath>(xml_tag_name, action_name, conf){}//调用父类构造函数创建client来send goal

void PlanGlobalPathAction::on_tick(){
    getInput("nav_b_goal",goal_.nav_b_goal);
    getInput("planner_id",goal_.planner_id);
    getInput("nav_speed",goal_.nav_speed);
}
BT::NodeStatus PlanGlobalPathAction::on_success(){
    setOutput("global_path",result_.result->global_path);
    setOutput("nav_status", result_.result->nav_status);
    return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus PlanGlobalPathAction::on_aborted(){
    nav_msgs::msg::Path empty_path;
    setOutput("nav_b_goal",empty_path);
    setOutput("nav_status", result_.result->nav_status);
    return BT::NodeStatus::FAILURE;
}
BT::NodeStatus PlanGlobalPathAction::on_cancelled(){
    nav_msgs::msg::Path empty_path;
    setOutput("nav_b_goal", empty_path);
    return BT::NodeStatus::SUCCESS;
}

void PlanGlobalPathAction::halt(){
    nav_msgs::msg::Path empty_path;
    setOutput("nav_b_goal", empty_path);
    amr_behavior_tree::BtActionNode<amr_msgs::action::PlanGlobalPath>::halt();
}
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<amr_behavior_tree::PlanGlobalPathAction>(
        name, amr_msgs::action_names::PLAN_GLOBAL_PATH, config);
    };

  factory.registerBuilder<amr_behavior_tree::PlanGlobalPathAction>(
    "PlanGlobalPath", builder);
}
