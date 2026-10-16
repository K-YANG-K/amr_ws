#pragma once

#include <string>
#include <mutex>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "amr_costmap_2d/layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace amr_costmap_2d
{


class CostmapFilter : public Layer
{
public:

  CostmapFilter();

  ~CostmapFilter();

  typedef std::recursive_mutex mutex_t;

  mutex_t * getMutex()
  {
    return access_;
  }

  void onInitialize() final;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;

  void updateCosts(
    amr_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) final;

  void activate() final;

  void deactivate() final;

  void reset() final;

  bool isClearable() {return false;}

  virtual void initializeFilter(
    const std::string & filter_info_topic) = 0;

  virtual void process(
    amr_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j,
    const geometry_msgs::msg::Pose2D & pose) = 0;

  virtual void resetFilter() = 0;

protected:

  void enableCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  bool transformPose(
    const std::string global_frame,
    const geometry_msgs::msg::Pose2D & global_pose,
    const std::string mask_frame,
    geometry_msgs::msg::Pose2D & mask_pose) const;

  bool worldToMask(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr filter_mask,
    double wx, double wy, unsigned int & mx, unsigned int & my) const;

  inline int8_t getMaskData(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr filter_mask,
    const unsigned int mx, const unsigned int my) const
  {
    return filter_mask->data[my * filter_mask->info.width + mx];
  }


  std::string filter_info_topic_;

  std::string mask_topic_;

  tf2::Duration transform_tolerance_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;

private:

  geometry_msgs::msg::Pose2D latest_pose_;
  mutex_t * access_;
};

}  
