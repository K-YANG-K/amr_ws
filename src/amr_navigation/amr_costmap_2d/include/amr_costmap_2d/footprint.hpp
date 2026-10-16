#pragma once

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "amr_util/lifecycle_node.hpp"

namespace amr_costmap_2d
{

void calculateMinAndMaxDistances(
  const std::vector<geometry_msgs::msg::Point> & footprint,
  double & min_dist, double & max_dist);


geometry_msgs::msg::Point toPoint(geometry_msgs::msg::Point32 pt);


geometry_msgs::msg::Point32 toPoint32(geometry_msgs::msg::Point pt);


geometry_msgs::msg::Polygon toPolygon(std::vector<geometry_msgs::msg::Point> pts);

std::vector<geometry_msgs::msg::Point> toPointVector(
  geometry_msgs::msg::Polygon::SharedPtr polygon);


void transformFootprint(
  double x, double y, double theta,
  const std::vector<geometry_msgs::msg::Point> & footprint_spec,
  std::vector<geometry_msgs::msg::Point> & oriented_footprint);

void transformFootprint(
  double x, double y, double theta,
  const std::vector<geometry_msgs::msg::Point> & footprint_spec,
  geometry_msgs::msg::PolygonStamped & oriented_footprint);


void padFootprint(std::vector<geometry_msgs::msg::Point> & footprint, double padding);


std::vector<geometry_msgs::msg::Point> makeFootprintFromRadius(double radius);


bool makeFootprintFromString(
  const std::string & footprint_string,
  std::vector<geometry_msgs::msg::Point> & footprint);

} 
