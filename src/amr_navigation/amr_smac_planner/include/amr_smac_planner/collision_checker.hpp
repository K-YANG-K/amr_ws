#pragma once
#include <vector>
#include "amr_costmap_2d/footprint_collision_checker.hpp"
#include "amr_smac_planner/constants.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"



namespace amr_smac_planner
{

/**
 * @class amr_smac_planner::GridCollisionChecker
 * @brief A costmap grid collision checker
 */
class GridCollisionChecker
  : public amr_costmap_2d::FootprintCollisionChecker<amr_costmap_2d::Costmap2D *>
{
public:
  /**
   * @brief A constructor for amr_smac_planner::GridCollisionChecker
   * for use when regular bin intervals are appropriate
   * @param costmap The costmap to collision check against
   * @param num_quantizations The number of quantizations to precompute footprint
   * @param node Node to extract clock and logger from
   * orientations for to speed up collision checking
   */
  GridCollisionChecker(
    amr_costmap_2d::Costmap2D * costmap,
    unsigned int num_quantizations,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  /**
   * @brief A constructor for amr_smac_planner::GridCollisionChecker
   * for use when irregular bin intervals are appropriate
   * @param costmap The costmap to collision check against
   * @param angles The vector of possible angle bins to precompute for
   * orientations for to speed up collision checking, in radians
   */
  // GridCollisionChecker(
  //   amr_costmap_2d::Costmap2D * costmap,
  //   std::vector<float> & angles);

  /**
   * @brief Set the footprint to use with collision checker
   * @param footprint The footprint to collision check against
   * @param radius Whether or not the footprint is a circle and use radius collision checking
   */
  void setFootprint(
    const amr_costmap_2d::Footprint & footprint,
    const bool & radius,
    const double & possible_inscribed_cost);

  /**
   * @brief Check if in collision with costmap and footprint at pose
   * @param x X coordinate of pose to check against
   * @param y Y coordinate of pose to check against
   * @param theta Angle bin number of pose to check against (NOT radians)
   * @param traverse_unknown Whether or not to traverse in unknown space
   * @return boolean if in collision or not.
   */
  bool inCollision(
    const float & x,
    const float & y,
    const float & theta,
    const bool & traverse_unknown);

  /**
   * @brief Check if in collision with costmap and footprint at pose
   * @param i Index to search collision status of
   * @param traverse_unknown Whether or not to traverse in unknown space
   * @return boolean if in collision or not.
   */
  bool inCollision(
    const unsigned int & i,
    const bool & traverse_unknown);

  /**
   * @brief Get cost at footprint pose in costmap
   * @return the cost at the pose in costmap
   */
  float getCost();

  /**
   * @brief Get the angles of the precomputed footprint orientations
   * @return the ordered vector of angles corresponding to footprints
   */
  std::vector<float> & getPrecomputedAngles()
  {
    return angles_;
  }

private:
  /**
   * @brief Check if value outside the range
   * @param min Minimum value of the range
   * @param max Maximum value of the range
   * @param value the value to check if it is within the range
   * @return boolean if in range or not
   */
  bool outsideRange(const unsigned int & max, const float & value);

protected:
  std::vector<amr_costmap_2d::Footprint> oriented_footprints_;
  amr_costmap_2d::Footprint unoriented_footprint_;
  double footprint_cost_;
  bool footprint_is_radius_;
  std::vector<float> angles_;
  double possible_inscribed_cost_{-1};
  rclcpp::Logger logger_{rclcpp::get_logger("SmacPlannerCollisionChecker")};
  rclcpp::Clock::SharedPtr clock_;
};

}  