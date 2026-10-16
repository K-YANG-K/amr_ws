#pragma once

#include <vector>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "amr_msgs/srv/clear_costmap_except_region.hpp"
#include "amr_msgs/srv/clear_costmap_around_robot.hpp"
#include "amr_msgs/srv/clear_entire_costmap.hpp"
#include "amr_costmap_2d/costmap_layer.hpp"
#include "amr_util/lifecycle_node.hpp"

namespace amr_costmap_2d
{

class Costmap2DROS;


class ClearCostmapService
{
public:

  ClearCostmapService(const amr_util::LifecycleNode::WeakPtr & parent, Costmap2DROS & costmap);


  ClearCostmapService() = delete;


  ~ClearCostmapService();


  void clearRegion(double reset_distance, bool invert);


  void clearEntirely();

private:

  rclcpp::Logger logger_{rclcpp::get_logger("amr_costmap_2d")};


  Costmap2DROS & costmap_;

  unsigned char reset_value_;

  rclcpp::Service<amr_msgs::srv::ClearCostmapExceptRegion>::SharedPtr clear_except_service_;

  void clearExceptRegionCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<amr_msgs::srv::ClearCostmapExceptRegion::Request> request,
    const std::shared_ptr<amr_msgs::srv::ClearCostmapExceptRegion::Response> response);

  rclcpp::Service<amr_msgs::srv::ClearCostmapAroundRobot>::SharedPtr clear_around_service_;

  void clearAroundRobotCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<amr_msgs::srv::ClearCostmapAroundRobot::Request> request,
    const std::shared_ptr<amr_msgs::srv::ClearCostmapAroundRobot::Response> response);

  rclcpp::Service<amr_msgs::srv::ClearEntireCostmap>::SharedPtr clear_entire_service_;


  void clearEntireCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<amr_msgs::srv::ClearEntireCostmap::Request> request,
    const std::shared_ptr<amr_msgs::srv::ClearEntireCostmap::Response> response);

  void clearLayerRegion(
    std::shared_ptr<CostmapLayer> & costmap, double pose_x, double pose_y, double reset_distance,
    bool invert);

  bool getPosition(double & x, double & y) const;
};

}  