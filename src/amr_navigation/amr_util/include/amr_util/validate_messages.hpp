#pragma once
#include <cmath>
#include <iostream>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"



namespace amr_util
{


bool validateMsg(const double & num)
{
  if (std::isinf(num)) {return false;}
  if (std::isnan(num)) {return false;}
  return true;
}

template<size_t N>
bool validateMsg(const std::array<double, N> & msg)
{

  for (const auto & element : msg) {
    if (!validateMsg(element)) {return false;}
  }

  return true;
}

const int NSEC_PER_SEC = 1e9;  // 1 second = 1e9 nanosecond
bool validateMsg(const builtin_interfaces::msg::Time & msg)
{
  if (msg.nanosec >= NSEC_PER_SEC) {
    return false;                                      // invalid nanosec-stamp
  }
  return true;
}

bool validateMsg(const std_msgs::msg::Header & msg)
{
  //  check sub-type
  if (!validateMsg(msg.stamp)) {return false;}

  /*  @brief frame_id check
   *  if here'a need to check message validation
   *  it should at least have a non-empty frame_id
   *  otherwise, we regard it as an invalid message
   */
  if (msg.frame_id.empty()) {return false;}
  return true;
}

bool validateMsg(const geometry_msgs::msg::Point & msg)
{
  //  check sub-type
  if (!validateMsg(msg.x)) {return false;}
  if (!validateMsg(msg.y)) {return false;}
  if (!validateMsg(msg.z)) {return false;}
  return true;
}

const double epsilon = 1e-4;
bool validateMsg(const geometry_msgs::msg::Quaternion & msg)
{
  //  check sub-type
  if (!validateMsg(msg.x)) {return false;}
  if (!validateMsg(msg.y)) {return false;}
  if (!validateMsg(msg.z)) {return false;}
  if (!validateMsg(msg.w)) {return false;}

  if (abs(msg.x * msg.x + msg.y * msg.y + msg.z * msg.z + msg.w * msg.w - 1.0) >= epsilon) {
    return false;
  }

  return true;
}

bool validateMsg(const geometry_msgs::msg::Pose & msg)
{
  // check sub-type
  if (!validateMsg(msg.position)) {return false;}
  if (!validateMsg(msg.orientation)) {return false;}
  return true;
}

bool validateMsg(const geometry_msgs::msg::PoseWithCovariance & msg)
{
  // check sub-type
  if (!validateMsg(msg.pose)) {return false;}
  if (!validateMsg(msg.covariance)) {return false;}

  return true;
}

bool validateMsg(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  // check sub-type
  if (!validateMsg(msg.header)) {return false;}
  if (!validateMsg(msg.pose)) {return false;}
  return true;
}


// Function to verify map meta information
bool validateMsg(const nav_msgs::msg::MapMetaData & msg)
{
  // check sub-type
  if (!validateMsg(msg.origin)) {return false;}
  if (!validateMsg(msg.resolution)) {return false;}

  // logic check
  // 1> we don't need an empty map
  if (msg.height == 0 || msg.width == 0) {return false;}
  return true;
}

// for msg-type like map, costmap and others as `OccupancyGrid`
bool validateMsg(const nav_msgs::msg::OccupancyGrid & msg)
{
  // check sub-type
  if (!validateMsg(msg.header)) {return false;}
  // msg.data :  @todo any check for it ?
  if (!validateMsg(msg.info)) {return false;}

  // check logic
  if (msg.data.size() != msg.info.width * msg.info.height) {
    return false;                                                          // check map-size
  }
  return true;
}


} 
