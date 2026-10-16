#pragma once

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "amr_costmap_2d/costmap_2d.hpp"
#include "amr_costmap_2d/footprint_collision_checker.hpp"
#include "amr_costmap_2d/costmap_subscriber.hpp"
#include "amr_costmap_2d/footprint_subscriber.hpp"

namespace amr_costmap_2d
{

class CostmapTopicCollisionChecker
{
public:

  CostmapTopicCollisionChecker(
    CostmapSubscriber & costmap_sub,
    FootprintSubscriber & footprint_sub,
    std::string name = "collision_checker");

  ~CostmapTopicCollisionChecker() = default;

  double scorePose(
    const geometry_msgs::msg::Pose2D & pose,
    bool fetch_costmap_and_footprint = true);

  bool isCollisionFree(
    const geometry_msgs::msg::Pose2D & pose,
    bool fetch_costmap_and_footprint = true);

protected:

  Footprint getFootprint(
    const geometry_msgs::msg::Pose2D & pose,
    bool fetch_latest_footprint = true);

  std::string name_;
  CostmapSubscriber & costmap_sub_;
  FootprintSubscriber & footprint_sub_;
  FootprintCollisionChecker<std::shared_ptr<Costmap2D>> collision_checker_;
  rclcpp::Clock::SharedPtr clock_;
  Footprint footprint_;
};

}  

