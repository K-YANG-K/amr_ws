#pragma once

#include <string>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "amr_core/goal_checker.hpp"
#include "amr_costmap_2d/costmap_2d_ros.hpp"

namespace amr_controller
{

/**
 * @class PositionGoalChecker
 * @brief Goal Checker plugin that only checks XY position, ignoring orientation
 */
class PositionGoalChecker : public amr_core::GoalChecker
{
public:
  PositionGoalChecker();
  ~PositionGoalChecker() override = default;

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<amr_costmap_2d::Costmap2DROS> costmap_ros) override;

  void reset() override;

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;

  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;

  /**
   * @brief Set the XY goal tolerance
   * @param tolerance New tolerance value
   */
  void setXYGoalTolerance(double tolerance);

protected:
  double xy_goal_tolerance_;
  double xy_goal_tolerance_sq_;
  bool stateful_;
  bool position_reached_;
  std::string plugin_name_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  /**
  * @brief Callback executed when a parameter change is detected
  * @param parameters list of changed parameters
  */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

} 