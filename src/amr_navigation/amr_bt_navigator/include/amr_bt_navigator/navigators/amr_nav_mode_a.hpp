#pragma once
/*
    导航模式A,接受TCS下发的路线进行FollowPath
*/

#include <string>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "amr_bt_navigator/amr_navigator.hpp"
#include "amr_msgs/action/amr_nav_mode_a.hpp"
#include "amr_msgs/action_names.hpp"
#include "amr_util/robot_utils.hpp"
#include "glog/logging.h"
#include "nav_msgs/msg/path.hpp"

namespace amr_bt_navigator{

class AmrNavModeANavigator : public amr_bt_navigator::Navigator<amr_msgs::action::AmrNavModeA>{
    using ActionT = amr_msgs::action::AmrNavModeA;
public:
    AmrNavModeANavigator() : Navigator(){}
    bool configure(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;
    bool cleanup() override;
    std::string getBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;
    std::string getMode() override;
protected:
    bool goalReceived(typename ActionT::Goal::ConstSharedPtr goal) override;
    void onLoop() override;
    void onPreempt(typename ActionT::Goal::ConstSharedPtr goal) override;
    void goalCompleted(typename ActionT::Result::SharedPtr result, const amr_behavior_tree::BtStatus final_bt_status) override;
    bool initializeGoalPose(ActionT::Goal::ConstSharedPtr goal);
    rclcpp::Time start_time_;
    rclcpp_action::Client<ActionT>::SharedPtr amr_nav_mode_a_client_;

    std::string goal_blackboard_id_;
    std::string speed_blackboard_id_;
    std::string amr_nav_mode_a_bt_xml_;
    std::string amr_nav_mode_a_bt_xml_filepath_;
;
};

}
