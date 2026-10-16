#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace amr_core
{

class ProgressChecker
{
public:
  typedef std::shared_ptr<amr_core::ProgressChecker> Ptr;

  virtual ~ProgressChecker() {}

  virtual void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) = 0;

  virtual bool check(geometry_msgs::msg::PoseStamped & current_pose) = 0;

  virtual void reset() = 0;
};
}  
