#pragma once
#include <map>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "amr_costmap_2d/layer.hpp"
#include "amr_costmap_2d/layered_costmap.hpp"

namespace amr_costmap_2d
{

class CellData
{
public:

  CellData(double i, unsigned int x, unsigned int y, unsigned int sx, unsigned int sy)
  : index_(static_cast<unsigned int>(i)), x_(x), y_(y), src_x_(sx), src_y_(sy)
  {
  }
  unsigned int index_;
  unsigned int x_, y_;
  unsigned int src_x_, src_y_;
};


class InflationLayer : public Layer
{
public:

  InflationLayer();

  ~InflationLayer();


  void onInitialize() override;


  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) override;

  void updateCosts(
    amr_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;


  void matchSize() override;


  virtual bool isClearable() override {return false;}

  void reset() override
  {
    matchSize();
    current_ = false;
  }

  inline unsigned char computeCost(double distance) const
  {
    unsigned char cost = 0;
    if (distance == 0) {
      cost = LETHAL_OBSTACLE;
    } else if (distance * resolution_ <= inscribed_radius_) {
      cost = INSCRIBED_INFLATED_OBSTACLE;
    } else {
    
      double factor =
        exp(-1.0 * cost_scaling_factor_ * (distance * resolution_ - inscribed_radius_));
      cost = static_cast<unsigned char>((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }
    return cost;
  }

 
  typedef std::recursive_mutex mutex_t;

  mutex_t * getMutex()
  {
    return access_;
  }

protected:

  void onFootprintChanged() override;

  inline double distanceLookup(
    unsigned int mx, unsigned int my, unsigned int src_x,
    unsigned int src_y)
  {
    unsigned int dx = (mx > src_x) ? mx - src_x : src_x - mx;
    unsigned int dy = (my > src_y) ? my - src_y : src_y - my;
    return cached_distances_[dx * cache_length_ + dy];
  }


  inline unsigned char costLookup(
    unsigned int mx, unsigned int my, unsigned int src_x,
    unsigned int src_y)
  {
    unsigned int dx = (mx > src_x) ? mx - src_x : src_x - mx;
    unsigned int dy = (my > src_y) ? my - src_y : src_y - my;
    return cached_costs_[dx * cache_length_ + dy];
  }


  void computeCaches();


  int generateIntegerDistances();


  unsigned int cellDistance(double world_dist)
  {
    return layered_costmap_->getCostmap()->cellDistance(world_dist);
  }


  inline void enqueue(
    unsigned int index, unsigned int mx, unsigned int my,
    unsigned int src_x, unsigned int src_y);

  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  double inflation_radius_, inscribed_radius_, cost_scaling_factor_;
  bool inflate_unknown_, inflate_around_unknown_;
  unsigned int cell_inflation_radius_;
  unsigned int cached_cell_inflation_radius_;
  std::vector<std::vector<CellData>> inflation_cells_;

  double resolution_;

  std::vector<bool> seen_;

  std::vector<unsigned char> cached_costs_;
  std::vector<double> cached_distances_;
  std::vector<std::vector<int>> distance_matrix_;
  unsigned int cache_length_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;


  bool need_reinflation_;
  mutex_t * access_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  
