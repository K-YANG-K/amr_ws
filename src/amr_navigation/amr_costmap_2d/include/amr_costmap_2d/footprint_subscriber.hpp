#pragma once
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "amr_costmap_2d/footprint.hpp"
#include "amr_util/lifecycle_node.hpp"
#include "amr_util/robot_utils.hpp"

namespace amr_costmap_2d
{

class FootprintSubscriber
{
public:

  FootprintSubscriber(
    const amr_util::LifecycleNode::WeakPtr & parent,
    const std::string & topic_name,
    tf2_ros::Buffer & tf,
    std::string robot_base_frame = "base_link",
    double transform_tolerance = 0.1);


  FootprintSubscriber(
    const rclcpp::Node::WeakPtr & parent,
    const std::string & topic_name,
    tf2_ros::Buffer & tf,
    std::string robot_base_frame = "base_link",
    double transform_tolerance = 0.1);


  ~FootprintSubscriber() {}


  bool getFootprintRaw(
    std::vector<geometry_msgs::msg::Point> & footprint,
    std_msgs::msg::Header & footprint_header);

  bool getFootprintInRobotFrame(
    std::vector<geometry_msgs::msg::Point> & footprint,
    std_msgs::msg::Header & footprint_header);

protected:

  void footprint_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);

  tf2_ros::Buffer & tf_;
  std::string robot_base_frame_;
  double transform_tolerance_;
  bool footprint_received_{false};
  geometry_msgs::msg::PolygonStamped::SharedPtr footprint_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;
};

} 