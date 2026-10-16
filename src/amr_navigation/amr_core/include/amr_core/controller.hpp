#pragma once

#include <memory>
#include <string>

#include "amr_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "amr_core/goal_checker.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace amr_core
{


class Controller
{
public:
  using Ptr = std::shared_ptr<amr_core::Controller>;


  virtual ~Controller() {}

  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string name, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<amr_costmap_2d::Costmap2DROS>) = 0;

  virtual void cleanup() = 0;

  virtual void activate() = 0;

  virtual void deactivate() = 0;

  virtual void setPlan(const nav_msgs::msg::Path & path) = 0;

  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    amr_core::GoalChecker * goal_checker) = 0;

  virtual void setSpeedLimit(const double & speed_limit, const bool & percentage) = 0;

  virtual rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
  {
    (void)parameters;
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }
};

}