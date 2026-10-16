#pragma once

#include <string>
#include <memory>

#include "amr_msgs/action/local_control.hpp"
#include "amr_behavior_tree/bt_action_node.hpp"
#include "amr_msgs/action_names.hpp"
#include "std_msgs/msg/float32.hpp"

namespace amr_behavior_tree{
class LocalControlAction : public BtActionNode<amr_msgs::action::LocalControl>{
public: 
    LocalControlAction(const std::string & xml_tag_name,
    const std::string & action_name,const BT::NodeConfiguration & conf);
    void on_tick() override;
    BT::NodeStatus on_success() override;
    BT::NodeStatus on_aborted() override;
   // BT::NodeStatus on_cancelled() override;
    void on_wait_for_result(std::shared_ptr<const amr_msgs::action::LocalControl::Feedback> feedback) override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
        {
            BT::InputPort<nav_msgs::msg::Path>("global_path", "Path to follow"),
            BT::InputPort<std::string>("controller_id", "MPCController"),
            BT::InputPort<std::string>("goal_checker_id", ""),
            BT::InputPort<std_msgs::msg::Float32>("nav_speed","nav_a speed"),
            BT::OutputPort<uint16_t>("nav_status","nav status code")
        });
    }
};
}
