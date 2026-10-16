#pragma once

#include <vector>
#include <list>
#include <string>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/time.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "amr_costmap_2d/observation.hpp"
#include "amr_util/lifecycle_node.hpp"


namespace amr_costmap_2d
{

class ObservationBuffer
{
public:

  ObservationBuffer(
    const amr_util::LifecycleNode::WeakPtr & parent,
    std::string topic_name,
    double observation_keep_time,
    double expected_update_rate,
    double min_obstacle_height, double max_obstacle_height, double obstacle_max_range,
    double obstacle_min_range,
    double raytrace_max_range, double raytrace_min_range, tf2_ros::Buffer & tf2_buffer,
    std::string global_frame,
    std::string sensor_frame,
    tf2::Duration tf_tolerance);


  ~ObservationBuffer();


  void bufferCloud(const sensor_msgs::msg::PointCloud2 & cloud);


  void getObservations(std::vector<Observation> & observations);

  bool isCurrent() const;

  inline void lock()
  {
    lock_.lock();
  }


  inline void unlock()
  {
    lock_.unlock();
  }


  void resetLastUpdated();

private:

  void purgeStaleObservations();

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("amr_costmap_2d")};
  tf2_ros::Buffer & tf2_buffer_;
  const rclcpp::Duration observation_keep_time_;
  const rclcpp::Duration expected_update_rate_;
  rclcpp::Time last_updated_;
  std::string global_frame_;
  std::string sensor_frame_;
  std::list<Observation> observation_list_;
  std::string topic_name_;
  double min_obstacle_height_, max_obstacle_height_;
  std::recursive_mutex lock_; 
  double obstacle_max_range_, obstacle_min_range_, raytrace_max_range_, raytrace_min_range_;
  tf2::Duration tf_tolerance_;
};
}  