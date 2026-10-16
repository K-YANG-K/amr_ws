#pragma once

#include <memory>
#include <string>

#include "amr_costmap_2d/footprint_collision_checker.hpp"
#include "amr_costmap_2d/inflation_layer.hpp"

#include "amr_mppi_controller/critic_function.hpp"
#include "amr_mppi_controller/models/state.hpp"
#include "amr_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::CostCritic
 * @brief Critic objective function for avoiding obstacles using costmap's inflated cost
 */
class CostCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;

  /**
   * @brief Evaluate cost related to obstacle avoidance
   *
   * @param costs [out] add obstacle cost values to this tensor
   */
  void score(CriticData & data) override;

protected:
  /**
    * @brief Checks if cost represents a collision
    * @param cost Point cost at pose center
    * @param x X of pose
    * @param y Y of pose
    * @param theta theta of pose
    * @return bool if in collision
    */
  bool inCollision(float cost, float x, float y, float theta);

  /**
    * @brief cost at a robot pose
    * @param x X of pose
    * @param y Y of pose
    * @return Collision information at pose
    */
  float costAtPose(float x, float y);

  /**
    * @brief Find the min cost of the inflation decay function for which the robot MAY be
    * in collision in any orientation
    * @param costmap Costmap2DROS to get minimum inscribed cost (e.g. 128 in inflation layer documentation)
    * @return double circumscribed cost, any higher than this and need to do full footprint collision checking
    * since some element of the robot could be in collision
    */
  float findCircumscribedCost(std::shared_ptr<amr_costmap_2d::Costmap2DROS> costmap);

protected:
  amr_costmap_2d::FootprintCollisionChecker<amr_costmap_2d::Costmap2D *>
  collision_checker_{nullptr};
  float possibly_inscribed_cost_;

  bool consider_footprint_{true};
  float circumscribed_radius_{0};
  float circumscribed_cost_{0};
  float collision_cost_{0};
  float critical_cost_{0};
  float weight_{0};

  float near_goal_distance_;
  std::string inflation_layer_name_;

  unsigned int power_{0};
};

} 