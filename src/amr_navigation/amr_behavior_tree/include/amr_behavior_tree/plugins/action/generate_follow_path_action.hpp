#pragma once
#include <string>

#include "amr_behavior_tree/bt_action_node.hpp"
#include "amr_msgs/action/generate_follow_path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32.hpp"
#include "amr_msgs/action_names.hpp"

namespace amr_behavior_tree
{
class GenerateFollowPathAction : public amr_behavior_tree::BtActionNode<amr_msgs::action::GenerateFollowPath>
{
public:
    GenerateFollowPathAction(
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
            BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("nav_a_goal", "from tcs goal path"),
            BT::InputPort<std::string>("planner_id", "NavAPlanner", "planner plugin name"),
            BT::InputPort<std_msgs::msg::Float32>("nav_speed","nav a speed"),
            BT::OutputPort<nav_msgs::msg::Path>("global_path", "Path created by GenerateFollowPath node"),
            BT::OutputPort<uint16_t>("nav_status", "nav status code"),

        });
    }
};

}
