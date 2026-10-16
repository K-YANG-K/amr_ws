#include "amr_bt_navigator/amr_bt_navigator.hpp"
#include "amr_bt_navigator/navigators/amr_nav_mode_a.hpp"
#include "amr_bt_navigator/navigators/amr_nav_mode_b.hpp"

#include <memory>
#include <string>
#include <chrono>
#include <cmath>

namespace amr_bt_navigator{

AmrBtNavigator::AmrBtNavigator(rclcpp::NodeOptions options):
    amr_util::LifecycleNode("amr_bt_navigator", "", options.automatically_declare_parameters_from_overrides(true)){
    amr_util::InitGlogForNode("amr_bt_navigator");
    LOG(INFO)<<"初始化LifecycleNode--->[amr_bt_navigator]";
    declare_parameter_if_not_declared(this,"bt_nodes",rclcpp::ParameterValue(bt_nodes_));

    declare_parameter_if_not_declared(
    this, "transform_tolerance", rclcpp::ParameterValue(0.1));

    declare_parameter_if_not_declared(
    this, "global_frame", rclcpp::ParameterValue(std::string("map")));
    
    declare_parameter_if_not_declared(
    this, "robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
}

AmrBtNavigator::~AmrBtNavigator(){}

amr_util::CallbackReturn AmrBtNavigator::on_configure(const rclcpp_lifecycle::State &){
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    global_frame_ = get_parameter("global_frame").as_string();
    robot_frame_ = get_parameter("robot_base_frame").as_string();
    transform_tolerance_ = get_parameter("transform_tolerance").as_double();
    bt_nodes_ = get_parameter("bt_nodes").as_string_array();


    amr_nav_mode_a_navigator_ = std::make_unique<amr_bt_navigator::AmrNavModeANavigator>();
    amr_nav_mode_b_navigator_ = std::make_unique<amr_bt_navigator::AmrNavModeBNavigator>();

    amr_bt_navigator::FeedbackUtils feedback_utils;
    feedback_utils.tf_buffer = tf_buffer_;
    feedback_utils.robot_frame = robot_frame_;
    feedback_utils.global_frame = global_frame_;
    feedback_utils.transform_tolerance = transform_tolerance_;

    if(!amr_nav_mode_a_navigator_->on_configure(shared_from_this(),bt_nodes_,feedback_utils,&plugin_muxer_)){
        return amr_util::CallbackReturn::FAILURE;
    }

    if(!amr_nav_mode_b_navigator_->on_configure(shared_from_this(),bt_nodes_,feedback_utils,&plugin_muxer_)){
        return amr_util::CallbackReturn::FAILURE;
    }
    LOG(INFO)<<"amr_bt_navigator ---> 已配置";
    return amr_util::CallbackReturn::SUCCESS;
}

amr_util::CallbackReturn AmrBtNavigator::on_activate(const rclcpp_lifecycle::State &){
    if(!amr_nav_mode_a_navigator_->on_activate() || !amr_nav_mode_b_navigator_->on_activate()){
        LOG(ERROR)<<"导航模式激活失败!";
        return amr_util::CallbackReturn::FAILURE;
    }
    createBond();
    LOG(INFO)<<"amr_bt_navigator ---> 已激活";
    return amr_util::CallbackReturn::SUCCESS;
}

amr_util::CallbackReturn AmrBtNavigator::on_deactivate(const rclcpp_lifecycle::State &){
    if (!amr_nav_mode_a_navigator_->on_deactivate()|| !amr_nav_mode_b_navigator_->on_deactivate()) {
        return amr_util::CallbackReturn::FAILURE;
  }

    destroyBond();
    LOG(INFO)<<"amr_bt_navigator ---> 已停用";
    return amr_util::CallbackReturn::SUCCESS;
}

amr_util::CallbackReturn AmrBtNavigator::on_cleanup(const rclcpp_lifecycle::State &){
    tf_listener_.reset();
    tf_buffer_.reset();
    if(!amr_nav_mode_a_navigator_->on_cleanup() || !amr_nav_mode_b_navigator_->on_cleanup()){
        return amr_util::CallbackReturn::FAILURE;
    }
    amr_nav_mode_a_navigator_.reset();
    amr_nav_mode_b_navigator_.reset();
    LOG(INFO)<<"amr_bt_navigator ---> 已清理";
    return amr_util::CallbackReturn::SUCCESS;
}

amr_util::CallbackReturn AmrBtNavigator::on_shutdown(const rclcpp_lifecycle::State &){
    LOG(INFO)<<"amr_bt_navigator ---> 已关闭";
    return amr_util::CallbackReturn::SUCCESS;
}

}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(amr_bt_navigator::AmrBtNavigator)
