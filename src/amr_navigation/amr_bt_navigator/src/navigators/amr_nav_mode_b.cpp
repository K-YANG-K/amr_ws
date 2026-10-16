#include <string>
#include <vector>
#include <memory>
#include <vector>
#include "amr_bt_navigator/navigators/amr_nav_mode_b.hpp"


namespace amr_bt_navigator{
    bool AmrNavModeBNavigator::configure(rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node){
        LOG(INFO)<<"初始化导航B";
        auto node = parent_node.lock();
        amr_util::declare_parameter_if_not_declared(node, "nav_b_goal_blackboard_id", rclcpp::ParameterValue(std::string("nav_b_goal")));
        amr_util::declare_parameter_if_not_declared(node, "nav_b_speed_blackboard_id", rclcpp::ParameterValue(std::string("nav_speed")));

        goal_blackboard_id_ = node->get_parameter("nav_b_goal_blackboard_id").as_string();
        speed_blackboard_id_ = node->get_parameter("nav_b_speed_blackboard_id").as_string();

        amr_nav_mode_b_client_ = rclcpp_action::create_client<ActionT>(node, getMode());
        goal_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&AmrNavModeBNavigator::onGoalPoseReceived, this, std::placeholders::_1));
        return true;
    }
    bool AmrNavModeBNavigator::cleanup(){
        amr_nav_mode_b_client_.reset();
        return true;
    }
    std::string AmrNavModeBNavigator::getBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node){

        auto node = parent_node.lock();

        std::string pkg_share_directory = ament_index_cpp::get_package_share_directory("amr_bt_navigator");

        amr_util::declare_parameter_if_not_declared(node, "amr_nav_mode_b_bt_xml",
            rclcpp::ParameterValue(std::string("amr_nav_mode_b_bt.xml")));

        node->get_parameter("amr_nav_mode_b_bt_xml", amr_nav_mode_b_bt_xml_);
        amr_nav_mode_b_bt_xml_filepath_ = std::string(pkg_share_directory + "/behavior_trees/" + amr_nav_mode_b_bt_xml_);
        LOG(INFO)<<"导航B加载行为树文件: "<<amr_nav_mode_b_bt_xml_filepath_;
        return amr_nav_mode_b_bt_xml_filepath_;
    }

    std::string AmrNavModeBNavigator::getMode(){//action的话题名
        return std::string(amr_msgs::action_names::AMR_NAV_MODE_B);
    }

    bool AmrNavModeBNavigator::goalReceived(typename ActionT::Goal::ConstSharedPtr goal){
        return initializeGoalPose(goal);
    }

    void AmrNavModeBNavigator::onLoop(){
        //TODO
    }

    void AmrNavModeBNavigator::onPreempt(typename ActionT::Goal::ConstSharedPtr goal){
        //TODO
    }

    void AmrNavModeBNavigator::goalCompleted(typename ActionT::Result::SharedPtr result, const amr_behavior_tree::BtStatus final_bt_status){
        auto blackboard = bt_action_server_->getBlackboard();
        uint16_t status = 0;
        blackboard->get<uint16_t>("nav_status", status);
        LOG(INFO) << "goalCompleted: blackboard nav_status = " << status
                  << ", bt_status = " << static_cast<int>(final_bt_status);
        result->nav_status = status;
    }

    bool
    AmrNavModeBNavigator::initializeGoalPose(ActionT::Goal::ConstSharedPtr goal)
    {
        geometry_msgs::msg::PoseStamped current_pose;
        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose = goal->nav_b_goal;
      
  
        amr_util::getCurrentPose(
            current_pose, *feedback_utils_.tf_buffer,
            feedback_utils_.global_frame, feedback_utils_.robot_frame,
            feedback_utils_.transform_tolerance);
        goal_pose.header.frame_id = feedback_utils_.global_frame;
        goal_pose.header.stamp = clock_->now();

        LOG(INFO) << "AMR执行导航B模式, 从当前位置 ("<< current_pose.pose.position.x <<", "
         << current_pose.pose.position.y << ")" << "导航至"
         <<"终点位置 (" << goal_pose.pose.position.x << ", " << goal_pose.pose.position.y
                  << ")"<< "，期望速度 " << goal->nav_speed.data << " m/s";

        start_time_ = clock_->now();
        auto blackboard = bt_action_server_->getBlackboard();
        blackboard->set<geometry_msgs::msg::PoseStamped>(goal_blackboard_id_, goal->nav_b_goal);
        blackboard->set<std_msgs::msg::Float32>(speed_blackboard_id_, goal->nav_speed);
        return true;
    }

    void
    AmrNavModeBNavigator::onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
    {

    ActionT::Goal goal;
    goal.nav_b_goal = *pose;
    amr_nav_mode_b_client_->async_send_goal(goal);
}
}