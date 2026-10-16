#include <string>
#include <vector>
#include <memory>
#include <vector>
#include "amr_bt_navigator/navigators/amr_nav_mode_a.hpp"


namespace amr_bt_navigator{
    bool AmrNavModeANavigator::configure(rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node){
        LOG(INFO)<<"初始化导航A";
        auto node = parent_node.lock();
        amr_util::declare_parameter_if_not_declared(node, "nav_a_goal_blackboard_id", rclcpp::ParameterValue(std::string("nav_a_goal")));
        amr_util::declare_parameter_if_not_declared(node, "nav_a_speed_blackboard_id", rclcpp::ParameterValue(std::string("nav_speed")));

        goal_blackboard_id_ = node->get_parameter("nav_a_goal_blackboard_id").as_string();
        speed_blackboard_id_ = node->get_parameter("nav_a_speed_blackboard_id").as_string();

       amr_nav_mode_a_client_ = rclcpp_action::create_client<ActionT>(node, getMode());
        return true;
    }
    bool AmrNavModeANavigator::cleanup(){
        amr_nav_mode_a_client_.reset();
        return true;
    }
    std::string AmrNavModeANavigator::getBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node){

        auto node = parent_node.lock();

        std::string pkg_share_directory = ament_index_cpp::get_package_share_directory("amr_bt_navigator");

        amr_util::declare_parameter_if_not_declared(node, "amr_nav_mode_a_bt_xml",
            rclcpp::ParameterValue(std::string("amr_nav_mode_a_bt.xml")));

        node->get_parameter("amr_nav_mode_a_bt_xml", amr_nav_mode_a_bt_xml_);
        amr_nav_mode_a_bt_xml_filepath_ = std::string(pkg_share_directory + "/behavior_trees/" + amr_nav_mode_a_bt_xml_);
        LOG(INFO)<<"导航A加载行为树文件: "<<amr_nav_mode_a_bt_xml_filepath_;
        return amr_nav_mode_a_bt_xml_filepath_;
    }

    std::string AmrNavModeANavigator::getMode(){//action的话题名
        return std::string(amr_msgs::action_names::AMR_NAV_MODE_A);
    }

    bool AmrNavModeANavigator::goalReceived(typename ActionT::Goal::ConstSharedPtr goal){
        return initializeGoalPose(goal);
    }

    void AmrNavModeANavigator::onLoop(){
        //TODO
    }

    void AmrNavModeANavigator::onPreempt(typename ActionT::Goal::ConstSharedPtr goal){
        //TODO
    }

    void AmrNavModeANavigator::goalCompleted(typename ActionT::Result::SharedPtr result, const amr_behavior_tree::BtStatus final_bt_status){
        auto blackboard = bt_action_server_->getBlackboard();
        uint16_t status = 0;
        blackboard->get<uint16_t>("nav_status", status);
        LOG(INFO) << "goalCompleted: blackboard nav_status = " << status
                  << ", bt_status = " << static_cast<int>(final_bt_status);
        result->nav_status = status;
    }

    bool
    AmrNavModeANavigator::initializeGoalPose(ActionT::Goal::ConstSharedPtr goal)
    {
        std::vector<geometry_msgs::msg::PoseStamped> way_poses;
        geometry_msgs::msg::PoseStamped current_pose;
        way_poses = goal->nav_a_goal;
      
  
        amr_util::getCurrentPose( 
            current_pose, *feedback_utils_.tf_buffer,
            feedback_utils_.global_frame, feedback_utils_.robot_frame,
            feedback_utils_.transform_tolerance);
        for(auto i = 0;i < way_poses.size();++i){
            way_poses[i].header.frame_id = feedback_utils_.global_frame;
            way_poses[i].header.stamp = clock_->now();
        }


        LOG(INFO) << "AMR执行导航A模式, 从当前位置 ("<< current_pose.pose.position.x <<", "
         << current_pose.pose.position.y << ")" << "导航至"
         <<"起点位置 ("
                  << way_poses.front().pose.position.x << ", " << way_poses.front().pose.position.y<< ") 终点位置 (" << way_poses.back().pose.position.x << ", " << way_poses.back().pose.position.y
                  << ")"<< "，期望速度 " << goal->nav_speed.data << " m/s";

        start_time_ = clock_->now();
        auto blackboard = bt_action_server_->getBlackboard();
        blackboard->set<std::vector<geometry_msgs::msg::PoseStamped>>(goal_blackboard_id_, goal->nav_a_goal);
        blackboard->set<std_msgs::msg::Float32>(speed_blackboard_id_, goal->nav_speed);
        return true;
    }
}