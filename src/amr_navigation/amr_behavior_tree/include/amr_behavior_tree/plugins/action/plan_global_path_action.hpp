#pragma once
#include <string>

#include "amr_behavior_tree/bt_action_node.hpp"
#include "amr_msgs/action/plan_global_path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32.hpp"
#include "amr_msgs/action_names.hpp"
namespace amr_behavior_tree
{
class PlanGlobalPathAction : public amr_behavior_tree::BtActionNode<amr_msgs::action::PlanGlobalPath>
{
public:
    PlanGlobalPathAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

    void on_tick() override;

    BT::NodeStatus on_success() override;

    BT::NodeStatus on_aborted() override;


    BT::NodeStatus on_cancelled() override;

    void halt() override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
        {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("nav_b_goal", "from tcs goal pose"),
            BT::InputPort<std::string>("planner_id", "NavBPlanner", "planner plugin name"),
            BT::InputPort<std_msgs::msg::Float32>("nav_speed","nav a speed"),
            BT::OutputPort<nav_msgs::msg::Path>("global_path", "Path created by PlanGlobalPath node"),
            BT::OutputPort<uint16_t>("nav_status", "nav status code"),

        });
    }
};

}
