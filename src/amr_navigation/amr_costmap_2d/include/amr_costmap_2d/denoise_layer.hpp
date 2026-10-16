#pragma once
#include "amr_costmap_2d/layer.hpp"
#include "amr_costmap_2d/denoise/image_processing.hpp"

namespace amr_costmap_2d
{

class DenoiseLayer : public Layer
{
  friend class DenoiseLayerTester;  

public:
  DenoiseLayer() = default;
  ~DenoiseLayer() = default;


  void reset() override;


  bool isClearable() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;


  void updateCosts(
    amr_costmap_2d::Costmap2D & master_grid,
    int min_x, int min_y, int max_x, int max_y) override;

protected:

  void onInitialize() override;

private:

  void denoise(Image<uint8_t> & image) const;

  void removeGroups(Image<uint8_t> & image) const;

  void removeSinglePixels(Image<uint8_t> & image) const;

  bool isBackground(uint8_t pixel) const;

private:
  size_t minimal_group_size_{};
  ConnectivityType group_connectivity_type_{ConnectivityType::Way8};
  mutable MemoryBuffer buffer_;
  imgproc_impl::GroupsRemover groups_remover_;
  bool no_information_is_obstacle_{};
};

} 

