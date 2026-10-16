#pragma once

#include <rclcpp/rclcpp.hpp>
#include <amr_costmap_2d/layer.hpp>
#include <amr_costmap_2d/layered_costmap.hpp>

namespace amr_costmap_2d
{

class CostmapLayer : public Layer, public Costmap2D
{
public:

  CostmapLayer()
  : has_extra_bounds_(false),
    extra_min_x_(1e6), extra_max_x_(-1e6),
    extra_min_y_(1e6), extra_max_y_(-1e6) {}

  bool isDiscretized()
  {
    return true;
  }

  virtual void matchSize();

  virtual void clearArea(int start_x, int start_y, int end_x, int end_y, bool invert);

  void addExtraBounds(double mx0, double my0, double mx1, double my1);

protected:

  void updateWithTrueOverwrite(
    amr_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  void updateWithOverwrite(
    amr_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  void updateWithMax(
    amr_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i,
    int max_j);

  void updateWithAddition(
    amr_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i,
    int max_j);

  void touch(double x, double y, double * min_x, double * min_y, double * max_x, double * max_y);

  void useExtraBounds(double * min_x, double * min_y, double * max_x, double * max_y);
  bool has_extra_bounds_;

private:
  double extra_min_x_, extra_max_x_, extra_min_y_, extra_max_y_;
};

}