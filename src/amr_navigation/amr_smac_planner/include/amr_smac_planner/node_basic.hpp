#pragma once

#include <math.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <functional>
#include <queue>
#include <memory>
#include <utility>
#include <limits>

#include "ompl/base/StateSpace.h"

#include "amr_smac_planner/constants.hpp"
#include "amr_smac_planner/node_hybrid.hpp"
#include "amr_smac_planner/node_lattice.hpp"
#include "amr_smac_planner/node_2d.hpp"
#include "amr_smac_planner/types.hpp"
#include "amr_smac_planner/collision_checker.hpp"

namespace amr_smac_planner
{

/**
 * @class amr_smac_planner::NodeBasic
 * @brief NodeBasic implementation for priority queue insertion
 */
template<typename NodeT>
class NodeBasic
{
public:
  /**
   * @brief A constructor for amr_smac_planner::NodeBasic
   * @param index The index of this node for self-reference
   */
  explicit NodeBasic(const unsigned int index)
  : index(index),
    graph_node_ptr(nullptr)
  {
  }

  /**
   * @brief Take a NodeBasic and populate it with any necessary state
   * cached in the queue for NodeT.
   * @param node NodeT ptr to populate metadata into NodeBasic
   */
  void populateSearchNode(NodeT * & node);

  /**
   * @brief Take a NodeBasic and populate it with any necessary state
   * cached in the queue for NodeTs.
   * @param node Search node (basic) object to initialize internal node
   * with state
   */
  void processSearchNode();

  typename NodeT::Coordinates pose;  // Used by NodeHybrid and NodeLattice
  NodeT * graph_node_ptr;
  MotionPrimitive * prim_ptr;  // Used by NodeLattice
  unsigned int index, motion_index;
  bool backward;
};

} 
