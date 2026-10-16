#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "laser_geometry/laser_geometry.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreorder"
#include "tf2_ros/message_filter.h"
#pragma GCC diagnostic pop
#include "message_filters/subscriber.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "amr_costmap_2d/costmap_layer.hpp"
#include "amr_costmap_2d/layered_costmap.hpp"
#include "amr_costmap_2d/observation_buffer.hpp"
#include "amr_costmap_2d/footprint.hpp"

namespace amr_costmap_2d
{

class ObstacleLayer : public CostmapLayer
{
public:

  ObstacleLayer()
  {
    costmap_ = NULL; 
  }


  virtual ~ObstacleLayer();
  virtual void onInitialize();

  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);

  virtual void updateCosts(
    amr_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void deactivate();

  virtual void activate();

  virtual void reset();

  virtual bool isClearable() {return true;}

  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  void resetBuffersLastUpdated();

  void laserScanCallback(
    sensor_msgs::msg::LaserScan::ConstSharedPtr message,
    const std::shared_ptr<amr_costmap_2d::ObservationBuffer> & buffer);

  void laserScanValidInfCallback(
    sensor_msgs::msg::LaserScan::ConstSharedPtr message,
    const std::shared_ptr<amr_costmap_2d::ObservationBuffer> & buffer);

  void pointCloud2Callback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message,
    const std::shared_ptr<amr_costmap_2d::ObservationBuffer> & buffer);

  void addStaticObservation(amr_costmap_2d::Observation & obs, bool marking, bool clearing);
  void clearStaticObservations(bool marking, bool clearing);

protected:

  bool getMarkingObservations(
    std::vector<amr_costmap_2d::Observation> & marking_observations) const;


  bool getClearingObservations(
    std::vector<amr_costmap_2d::Observation> & clearing_observations) const;


  virtual void raytraceFreespace(
    const amr_costmap_2d::Observation & clearing_observation,
    double * min_x, double * min_y,
    double * max_x,
    double * max_y);

  void updateRaytraceBounds(
    double ox, double oy, double wx, double wy, double max_range, double min_range,
    double * min_x, double * min_y,
    double * max_x,
    double * max_y);

  std::vector<geometry_msgs::msg::Point> transformed_footprint_;
  bool footprint_clearing_enabled_;

  void updateFootprint(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);

  std::string global_frame_;  
  double min_obstacle_height_;  
  double max_obstacle_height_;  


  laser_geometry::LaserProjection projector_;
  std::vector<std::shared_ptr<message_filters::SubscriberBase<rclcpp_lifecycle::LifecycleNode>>>
  observation_subscribers_;

  std::vector<std::shared_ptr<tf2_ros::MessageFilterBase>> observation_notifiers_;

  std::vector<std::shared_ptr<amr_costmap_2d::ObservationBuffer>> observation_buffers_;

  std::vector<std::shared_ptr<amr_costmap_2d::ObservationBuffer>> marking_buffers_;

  std::vector<std::shared_ptr<amr_costmap_2d::ObservationBuffer>> clearing_buffers_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  std::vector<amr_costmap_2d::Observation> static_clearing_observations_;
  std::vector<amr_costmap_2d::Observation> static_marking_observations_;

  bool rolling_window_;
  bool was_reset_;
  int combination_method_;
};

} 