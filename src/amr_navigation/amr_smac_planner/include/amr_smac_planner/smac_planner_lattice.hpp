#pragma once

#include <memory>
#include <vector>
#include <string>

#include "amr_smac_planner/a_star.hpp"
#include "amr_smac_planner/smoother.hpp"
#include "amr_smac_planner/utils.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "amr_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "amr_costmap_2d/costmap_2d_ros.hpp"
#include "amr_costmap_2d/costmap_2d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "amr_util/lifecycle_node.hpp"
#include "amr_util/node_utils.hpp"
#include "tf2/utils.h"

namespace amr_smac_planner
{

class SmacPlannerLattice : public amr_core::GlobalPlanner
{
public:
  /**
   * @brief constructor
   */
  SmacPlannerLattice();

  /**
   * @brief destructor
   */
  ~SmacPlannerLattice();

  /**
   * @brief Configuring plugin
   * @param parent Lifecycle node pointer
   * @param name Name of plugin map
   * @param tf Shared ptr of TF2 buffer
   * @param costmap_ros Costmap2DROS object
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<amr_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup lifecycle node
   */
  void cleanup() override;

  /**
   * @brief Activate lifecycle node
   */
  void activate() override;

  /**
   * @brief Deactivate lifecycle node
   */
  void deactivate() override;

  /**
   * @brief Creating a plan from start and goal poses
   * @param start Start pose
   * @param goal Goal pose
   * @return amr_msgs::Path of the generated path
   */
  nav_msgs::msg::Path createPlan(const std::vector<geometry_msgs::msg::PoseStamped>& way_poses) override;

protected:
  /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  std::unique_ptr<AStarAlgorithm<NodeLattice>> _a_star;
  GridCollisionChecker _collision_checker;
  std::unique_ptr<Smoother> _smoother;
  rclcpp::Clock::SharedPtr _clock;
  rclcpp::Logger _logger{rclcpp::get_logger("SmacPlannerLattice")};
  amr_costmap_2d::Costmap2D * _costmap;
  std::shared_ptr<amr_costmap_2d::Costmap2DROS> _costmap_ros;
  MotionModel _motion_model;
  LatticeMetadata _metadata;
  std::string _global_frame, _name;
  SearchInfo _search_info;
  bool _allow_unknown;
  int _max_iterations;
  int _max_on_approach_iterations;
  float _tolerance;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr _raw_plan_publisher;
  double _max_planning_time;
  double _lookup_table_size;
  std::mutex _mutex;
  rclcpp_lifecycle::LifecycleNode::WeakPtr _node;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _dyn_params_handler;
};

} 