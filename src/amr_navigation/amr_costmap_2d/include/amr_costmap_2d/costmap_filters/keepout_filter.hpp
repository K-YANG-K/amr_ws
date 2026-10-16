#pragma once
#include <string>
#include <memory>

#include "amr_costmap_2d/costmap_filters/costmap_filter.hpp"

#include "rclcpp/rclcpp.hpp"
#include "amr_msgs/msg/costmap_filter_info.hpp"

namespace amr_costmap_2d
{

class KeepoutFilter : public CostmapFilter
{
public:

  KeepoutFilter();


  void initializeFilter(
    const std::string & filter_info_topic);


  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;


  void process(
    amr_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j,
    const geometry_msgs::msg::Pose2D & pose);

  void resetFilter();

  bool isActive();

private:

  void filterInfoCallback(const amr_msgs::msg::CostmapFilterInfo::SharedPtr msg);

  void maskCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  rclcpp::Subscription<amr_msgs::msg::CostmapFilterInfo>::SharedPtr filter_info_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_sub_;

  std::unique_ptr<Costmap2D> mask_costmap_;

  std::string mask_frame_;  
  std::string global_frame_;  

  unsigned int x_{0};
  unsigned int y_{0};
  unsigned int width_{0};
  unsigned int height_{0};
  bool has_updated_data_{false};
};

}  


