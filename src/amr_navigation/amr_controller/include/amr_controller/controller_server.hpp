#pragma once

#include <memory>
#include<string>
#include<thread>

#include "amr_core/controller.hpp"
#include "amr_core/progress_checker.hpp"
#include "amr_core/goal_checker.hpp"
#include "amr_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/transform_listener.h"
#include "amr_msgs/action/local_control.hpp"
#include "amr_msgs/msg/speed_limit.hpp"
#include "amr_util/lifecycle_node.hpp"
#include "amr_util/simple_action_server.hpp"
#include "amr_util/robot_utils.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "amr_msgs/action_names.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "amr_msgs/topic_names.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
namespace amr_controller
{

class ProgressChecker;

class ControllerServer : public amr_util::LifecycleNode
{
public:
  using ControllerMap = std::unordered_map<std::string, amr_core::Controller::Ptr>;
  using GoalCheckerMap = std::unordered_map<std::string, amr_core::GoalChecker::Ptr>;

  explicit ControllerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~ControllerServer();

protected:

  amr_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  amr_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  amr_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  amr_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  amr_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  using Action = amr_msgs::action::LocalControl;
  using ActionServer = amr_util::SimpleActionServer<Action>;

  std::unique_ptr<ActionServer> action_server_;

  void computeControl();

  bool findControllerId(const std::string & c_name, std::string & name);

  bool findGoalCheckerId(const std::string & c_name, std::string & name);

  void setPlannerPath(const nav_msgs::msg::Path & path);

  void computeAndPublishVelocity();

  void updateGlobalPath();

  void publishVelocity(const geometry_msgs::msg::TwistStamped & velocity);

  void publishZeroVelocity();

  bool isGoalReached();

  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose);

//   double getThresholdedVelocity(double velocity, double threshold)
//   {
//     return (std::abs(velocity) > threshold) ? velocity : 0.0;
//   }

//   nav_2d_msgs::msg::Twist2D getThresholdedTwist(const nav_2d_msgs::msg::Twist2D & twist)
//   {
//     nav_2d_msgs::msg::Twist2D twist_thresh;
//     twist_thresh.x = getThresholdedVelocity(twist.x, min_x_velocity_threshold_);
//     twist_thresh.y = getThresholdedVelocity(twist.y, min_y_velocity_threshold_);
//     twist_thresh.theta = getThresholdedVelocity(twist.theta, min_theta_velocity_threshold_);
//     return twist_thresh;
//   }

  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::mutex mutex_;

  std::shared_ptr<amr_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<amr_util::NodeThread> costmap_thread_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::Subscription<amr_msgs::msg::SpeedLimit>::SharedPtr speed_limit_sub_;

  pluginlib::ClassLoader<amr_core::ProgressChecker> progress_checker_loader_;
  amr_core::ProgressChecker::Ptr progress_checker_;
  std::string default_progress_checker_id_;
  std::string default_progress_checker_type_;
  std::string progress_checker_id_;
  std::string progress_checker_type_;

  pluginlib::ClassLoader<amr_core::GoalChecker> goal_checker_loader_;
  GoalCheckerMap goal_checkers_;
  std::vector<std::string> default_goal_checker_ids_;
  std::vector<std::string> default_goal_checker_types_;
  std::vector<std::string> goal_checker_ids_;
  std::vector<std::string> goal_checker_types_;
  std::string goal_checker_ids_concat_, current_goal_checker_;

  pluginlib::ClassLoader<amr_core::Controller> lp_loader_;
  ControllerMap controllers_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> controller_ids_;
  std::vector<std::string> controller_types_;
  std::string controller_ids_concat_, current_controller_;

  double controller_frequency_;
  double min_x_velocity_threshold_;
  double min_y_velocity_threshold_;
  double min_theta_velocity_threshold_;

  double failure_tolerance_;
  bool publish_zero_velocity_;

  geometry_msgs::msg::PoseStamped end_pose_;

  rclcpp::Time last_valid_cmd_time_;

  nav_msgs::msg::Path current_path_;


private:

  void speedLimitCallback(const amr_msgs::msg::SpeedLimit::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  geometry_msgs::msg::Twist odom_vel_;
};

}
