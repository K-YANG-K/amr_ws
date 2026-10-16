#pragma once
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace amr_costmap_2d
{

class Observation
{
public:

  Observation()
  : cloud_(new sensor_msgs::msg::PointCloud2()), obstacle_max_range_(0.0), obstacle_min_range_(0.0),
    raytrace_max_range_(0.0),
    raytrace_min_range_(0.0)
  {
  }

  virtual ~Observation()
  {
    delete cloud_;
  }

  Observation & operator=(const Observation & obs)
  {
    origin_ = obs.origin_;
    cloud_ = new sensor_msgs::msg::PointCloud2(*(obs.cloud_));
    obstacle_max_range_ = obs.obstacle_max_range_;
    obstacle_min_range_ = obs.obstacle_min_range_;
    raytrace_max_range_ = obs.raytrace_max_range_;
    raytrace_min_range_ = obs.raytrace_min_range_;

    return *this;
  }


  Observation(
    geometry_msgs::msg::Point & origin, const sensor_msgs::msg::PointCloud2 & cloud,
    double obstacle_max_range, double obstacle_min_range, double raytrace_max_range,
    double raytrace_min_range)
  : origin_(origin), cloud_(new sensor_msgs::msg::PointCloud2(cloud)),
    obstacle_max_range_(obstacle_max_range), obstacle_min_range_(obstacle_min_range),
    raytrace_max_range_(raytrace_max_range), raytrace_min_range_(
      raytrace_min_range)
  {
  }

  Observation(const Observation & obs)
  : origin_(obs.origin_), cloud_(new sensor_msgs::msg::PointCloud2(*(obs.cloud_))),
    obstacle_max_range_(obs.obstacle_max_range_), obstacle_min_range_(obs.obstacle_min_range_),
    raytrace_max_range_(obs.raytrace_max_range_),
    raytrace_min_range_(obs.raytrace_min_range_)
  {
  }

  Observation(
    const sensor_msgs::msg::PointCloud2 & cloud, double obstacle_max_range,
    double obstacle_min_range)
  : cloud_(new sensor_msgs::msg::PointCloud2(cloud)), obstacle_max_range_(obstacle_max_range),
    obstacle_min_range_(obstacle_min_range),
    raytrace_max_range_(0.0), raytrace_min_range_(0.0)
  {
  }

  geometry_msgs::msg::Point origin_;
  sensor_msgs::msg::PointCloud2 * cloud_;
  double obstacle_max_range_, obstacle_min_range_, raytrace_max_range_, raytrace_min_range_;
};

} 