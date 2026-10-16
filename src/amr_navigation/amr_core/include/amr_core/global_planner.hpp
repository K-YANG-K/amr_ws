#pragma once

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "amr_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "amr_util/lifecycle_node.hpp"

namespace amr_core
{

class GlobalPlanner
{
public:
  using Ptr = std::shared_ptr<GlobalPlanner>;

  virtual ~GlobalPlanner() {}

  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<amr_costmap_2d::Costmap2DROS> costmap_ros) = 0;

  virtual void cleanup() = 0;

  virtual void activate() = 0;

  virtual void deactivate() = 0;

  virtual nav_msgs::msg::Path createPlan(const std::vector<geometry_msgs::msg::PoseStamped>& way_poses) = 0;
};

}  