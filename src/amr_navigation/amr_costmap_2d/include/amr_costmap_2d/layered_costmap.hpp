#pragma once

#include <memory>
#include <string>
#include <vector>

#include "amr_costmap_2d/cost_values.hpp"
#include "amr_costmap_2d/layer.hpp"
#include "amr_costmap_2d/costmap_2d.hpp"

namespace amr_costmap_2d
{
class Layer;

class LayeredCostmap
{
public:

  LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown);

  ~LayeredCostmap();

  void updateMap(double robot_x, double robot_y, double robot_yaw);

  std::string getGlobalFrameID() const
  {
    return global_frame_;
  }


  void resizeMap(
    unsigned int size_x, unsigned int size_y, double resolution, double origin_x,
    double origin_y,
    bool size_locked = false);


  void getUpdatedBounds(double & minx, double & miny, double & maxx, double & maxy)
  {
    minx = minx_;
    miny = miny_;
    maxx = maxx_;
    maxy = maxy_;
  }

  bool isCurrent();


  Costmap2D * getCostmap()
  {
    return &combined_costmap_;
  }

  bool isRolling()
  {
    return rolling_window_;
  }

  bool isTrackingUnknown()
  {
    return combined_costmap_.getDefaultValue() == amr_costmap_2d::NO_INFORMATION;
  }

  std::vector<std::shared_ptr<Layer>> * getPlugins()
  {
    return &plugins_;
  }


  std::vector<std::shared_ptr<Layer>> * getFilters()
  {
    return &filters_;
  }


  void addPlugin(std::shared_ptr<Layer> plugin);


  void addFilter(std::shared_ptr<Layer> filter);

  bool isSizeLocked()
  {
    return size_locked_;
  }

  void getBounds(unsigned int * x0, unsigned int * xn, unsigned int * y0, unsigned int * yn)
  {
    *x0 = bx0_;
    *xn = bxn_;
    *y0 = by0_;
    *yn = byn_;
  }


  bool isInitialized()
  {
    return initialized_;
  }


  void setFootprint(const std::vector<geometry_msgs::msg::Point> & footprint_spec);

  const std::vector<geometry_msgs::msg::Point> & getFootprint() {return footprint_;}

  double getCircumscribedRadius() {return circumscribed_radius_;}


  double getInscribedRadius() {return inscribed_radius_;}

  bool isOutofBounds(double robot_x, double robot_y);

private:

  Costmap2D primary_costmap_, combined_costmap_;
  std::string global_frame_;

  bool rolling_window_; 

  bool current_;
  double minx_, miny_, maxx_, maxy_;
  unsigned int bx0_, bxn_, by0_, byn_;

  std::vector<std::shared_ptr<Layer>> plugins_;
  std::vector<std::shared_ptr<Layer>> filters_;

  bool initialized_;
  bool size_locked_;
  double circumscribed_radius_, inscribed_radius_;
  std::vector<geometry_msgs::msg::Point> footprint_;
};

} 
