#pragma once
#include <string>
#include <vector>
#include <unordered_set>

#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"
#include "amr_costmap_2d/costmap_2d.hpp"
#include "amr_costmap_2d/layered_costmap.hpp"
#include "amr_util/lifecycle_node.hpp"

namespace amr_costmap_2d
{
class LayeredCostmap;

class Layer
{
public:

  Layer();

  virtual ~Layer() {}

  void initialize(
    LayeredCostmap * parent,
    std::string name,
    tf2_ros::Buffer * tf,
    const amr_util::LifecycleNode::WeakPtr & node,
    rclcpp::CallbackGroup::SharedPtr callback_group);

  virtual void deactivate() {}

  virtual void activate() {}

  virtual void reset() = 0;

  virtual bool isClearable() = 0;


  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) = 0;


  virtual void updateCosts(
    Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) = 0;


  virtual void matchSize() {}


  virtual void onFootprintChanged() {}

  std::string getName() const
  {
    return name_;
  }


  bool isCurrent() const
  {
    return current_;
  }

  bool isEnabled() const
  {
    return enabled_;
  }

  const std::vector<geometry_msgs::msg::Point> & getFootprint() const;


  void declareParameter(
    const std::string & param_name,
    const rclcpp::ParameterValue & value);

  void declareParameter(
    const std::string & param_name,
    const rclcpp::ParameterType & param_type);

  bool hasParameter(const std::string & param_name);

  std::string getFullName(const std::string & param_name);

protected:
  LayeredCostmap * layered_costmap_;
  std::string name_;
  tf2_ros::Buffer * tf_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("amr_costmap_2d")};

  virtual void onInitialize() {}

  bool current_;

  bool enabled_;

  std::unordered_set<std::string> local_params_;

private:
  std::vector<geometry_msgs::msg::Point> footprint_spec_;
};

} 