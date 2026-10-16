#pragma once

#include <algorithm>
#include <string>
#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "amr_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "amr_msgs/msg/costmap.hpp"
#include "amr_msgs/srv/get_costmap.hpp"
#include "tf2/transform_datatypes.h"
#include "amr_util/lifecycle_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace amr_costmap_2d
{

class Costmap2DPublisher
{
public:

  Costmap2DPublisher(
    const amr_util::LifecycleNode::WeakPtr & parent,
    Costmap2D * costmap,
    std::string global_frame,
    std::string topic_name,
    bool always_send_full_costmap = false);

  ~Costmap2DPublisher();


  void on_configure() {}


  void on_activate()
  {
    costmap_pub_->on_activate();
    costmap_update_pub_->on_activate();
    costmap_raw_pub_->on_activate();
  }


  void on_deactivate()
  {
    costmap_pub_->on_deactivate();
    costmap_update_pub_->on_deactivate();
    costmap_raw_pub_->on_deactivate();
  }

  void updateBounds(unsigned int x0, unsigned int xn, unsigned int y0, unsigned int yn)
  {
    x0_ = std::min(x0, x0_);
    xn_ = std::max(xn, xn_);
    y0_ = std::min(y0, y0_);
    yn_ = std::max(yn, yn_);
  }

  void publishCostmap();

  bool active()
  {
    return active_;
  }

private:

  void prepareGrid();
  void prepareCostmap();

  void costmap_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<amr_msgs::srv::GetCostmap::Request> request,
    const std::shared_ptr<amr_msgs::srv::GetCostmap::Response> response);

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("amr_costmap_2d")};

  Costmap2D * costmap_;
  std::string global_frame_;
  std::string topic_name_;
  unsigned int x0_, xn_, y0_, yn_;
  double saved_origin_x_;
  double saved_origin_y_;
  bool active_;
  bool always_send_full_costmap_;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp_lifecycle::LifecyclePublisher<map_msgs::msg::OccupancyGridUpdate>::SharedPtr
    costmap_update_pub_;

  rclcpp_lifecycle::LifecyclePublisher<amr_msgs::msg::Costmap>::SharedPtr costmap_raw_pub_;

  rclcpp::Service<amr_msgs::srv::GetCostmap>::SharedPtr costmap_service_;

  float grid_resolution;
  unsigned int grid_width, grid_height;
  std::unique_ptr<nav_msgs::msg::OccupancyGrid> grid_;
  std::unique_ptr<amr_msgs::msg::Costmap> costmap_raw_;
  static char * cost_translation_table_;
};

}  

