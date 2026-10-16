#pragma once
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "amr_costmap_2d/costmap_2d.hpp"
#include "amr_msgs/msg/costmap.hpp"
#include "amr_util/lifecycle_node.hpp"

namespace amr_costmap_2d
{
/**
 * @class CostmapSubscriber
 * @brief Subscribes to the costmap via a ros topic
 */
class CostmapSubscriber
{
public:

  CostmapSubscriber(
    const amr_util::LifecycleNode::WeakPtr & parent,
    const std::string & topic_name);


  CostmapSubscriber(
    const rclcpp::Node::WeakPtr & parent,
    const std::string & topic_name);

  ~CostmapSubscriber() {}


  std::shared_ptr<Costmap2D> getCostmap();


  void toCostmap2D();

  void costmapCallback(const amr_msgs::msg::Costmap::SharedPtr msg);

protected:
  std::shared_ptr<Costmap2D> costmap_;
  amr_msgs::msg::Costmap::SharedPtr costmap_msg_;
  std::string topic_name_;
  bool costmap_received_{false};
  rclcpp::Subscription<amr_msgs::msg::Costmap>::SharedPtr costmap_sub_;
};

}  

