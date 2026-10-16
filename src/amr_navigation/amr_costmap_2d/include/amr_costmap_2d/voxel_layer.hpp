#pragma once

#include <vector>
#include "message_filters/subscriber.h"

#include <rclcpp/rclcpp.hpp>
#include <amr_costmap_2d/layer.hpp>
#include <amr_costmap_2d/layered_costmap.hpp>
#include <amr_costmap_2d/observation_buffer.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <amr_msgs/msg/voxel_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <amr_costmap_2d/obstacle_layer.hpp>
#include <amr_voxel_grid/voxel_grid.hpp>

namespace amr_costmap_2d
{

class VoxelLayer : public ObstacleLayer
{
public:

  VoxelLayer()
  : voxel_grid_(0, 0, 0)
  {
    costmap_ = NULL;
  }

  virtual ~VoxelLayer();

  virtual void onInitialize();

  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);

  void updateOrigin(double new_origin_x, double new_origin_y);

  bool isDiscretized()
  {
    return true;
  }

  virtual void matchSize();

  virtual void reset();

  virtual bool isClearable() {return true;}

protected:

  virtual void resetMaps();

  virtual void raytraceFreespace(
    const amr_costmap_2d::Observation & clearing_observation,
    double * min_x, double * min_y,
    double * max_x,
    double * max_y);

  bool publish_voxel_;
  rclcpp_lifecycle::LifecyclePublisher<amr_msgs::msg::VoxelGrid>::SharedPtr voxel_pub_;
  amr_voxel_grid::VoxelGrid voxel_grid_;
  double z_resolution_, origin_z_;
  int unknown_threshold_, mark_threshold_, size_z_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    clearing_endpoints_pub_;

  inline bool worldToMap3DFloat(
    double wx, double wy, double wz, double & mx, double & my,
    double & mz)
  {
    if (wx < origin_x_ || wy < origin_y_ || wz < origin_z_) {
      return false;
    }
    mx = ((wx - origin_x_) / resolution_);
    my = ((wy - origin_y_) / resolution_);
    mz = ((wz - origin_z_) / z_resolution_);
    if (mx < size_x_ && my < size_y_ && mz < size_z_) {
      return true;
    }

    return false;
  }

  inline bool worldToMap3D(
    double wx, double wy, double wz, unsigned int & mx, unsigned int & my,
    unsigned int & mz)
  {
    if (wx < origin_x_ || wy < origin_y_ || wz < origin_z_) {
      return false;
    }

    mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
    my = static_cast<unsigned int>((wy - origin_y_) / resolution_);
    mz = static_cast<unsigned int>((wz - origin_z_) / z_resolution_);

    if (mx < size_x_ && my < size_y_ && mz < (unsigned int)size_z_) {
      return true;
    }

    return false;
  }

  inline void mapToWorld3D(
    unsigned int mx, unsigned int my, unsigned int mz, double & wx,
    double & wy,
    double & wz)
  {

    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
    wz = origin_z_ + (mz + 0.5) * z_resolution_;
  }

  inline double dist(double x0, double y0, double z0, double x1, double y1, double z1)
  {
    return sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) + (z1 - z0) * (z1 - z0));
  }

  double getSizeInMetersZ() const
  {
    return (size_z_ - 1 + 0.5) * z_resolution_;
  }

  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}

