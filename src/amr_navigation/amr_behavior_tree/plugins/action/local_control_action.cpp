#include <memory>
#include <string>

#include "amr_behavior_tree/plugins/action/local_control_action.hpp"

namespace amr_behavior_tree{
LocalControlAction::LocalControlAction(  
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
    : BtActionNode<amr_msgs::action::LocalControl>(xml_tag_name, action_name, conf)
{
}

void LocalControlAction::on_tick(){
    getInput("global_path",goal_.global_path);
    getInput("nav_speed",goal_.nav_speed);
    getInput("controller_id", goal_.controller_id);
    getInput("goal_checker_id", goal_.goal_checker_id);
}

BT::NodeStatus LocalControlAction::on_success() {
    setOutput("nav_status", result_.result->nav_status);
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus LocalControlAction::on_aborted() {
    setOutput("nav_status", result_.result->nav_status);
    return BT::NodeStatus::FAILURE;
}

void LocalControlAction::on_wait_for_result(
  std::shared_ptr<const amr_msgs::action::LocalControl::Feedback>/*feedback*/)
{

  nav_msgs::msg::Path new_path;
  getInput("global_path", new_path);

  if (goal_.global_path != new_path && new_path != nav_msgs::msg::Path()) {
    goal_.global_path = new_path;
    goal_updated_ = true;
  }

  std::string new_controller_id;
  getInput("controller_id", new_controller_id);

  if (goal_.controller_id != new_controller_id) {
    goal_.controller_id = new_controller_id;
    goal_updated_ = true;
  }

  std::string new_goal_checker_id;
  getInput("goal_checker_id", new_goal_checker_id);

  if (goal_.goal_checker_id != new_goal_checker_id) {
    goal_.goal_checker_id = new_goal_checker_id;
    goal_updated_ = true;
  }
}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<amr_behavior_tree::LocalControlAction>(
        name, amr_msgs::action_names::LOCAL_CONTROL, config);
    };

  factory.registerBuilder<amr_behavior_tree::LocalControlAction>(
    "LocalControl", builder);
}
