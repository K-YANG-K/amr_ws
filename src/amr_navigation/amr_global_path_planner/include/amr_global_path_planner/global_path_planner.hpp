#pragma once

#include <string>
#include <memory>

#include "amr_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "amr_util/robot_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/path.hpp"
#include "amr_core/global_planner.hpp"
#include "amr_util/robot_utils.hpp"
#include "amr_costmap_2d/costmap_2d_ros.hpp"
#include "amr_util/geometry_utils.hpp"

#include <cmath>
namespace amr_follow_path_generator
{
class FollowPathGenerator :public amr_core::GlobalPlanner{
public:
    FollowPathGenerator();
    ~FollowPathGenerator();

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<amr_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  nav_msgs::msg::Path createPlan(const std::vector<geometry_msgs::msg::PoseStamped>& way_poses) override;

protected:
  nav_msgs::msg::Path computeLinePath(
      const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & end,
      int sample_num, bool reverse,
      const std_msgs::msg::Header & header);

  nav_msgs::msg::Path computeBezierPath(
      const std::vector<geometry_msgs::msg::PoseStamped> & controls,
      int sample_num, bool reverse,
      const std_msgs::msg::Header & header);

  double resolution_;
  int min_sample_num_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string global_frame_, name_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};
}