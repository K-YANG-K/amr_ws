#pragma once

#include <cstdint>
namespace amr_costmap_2d
{
static constexpr uint8_t KEEPOUT_FILTER = 0;
static constexpr uint8_t SPEED_FILTER_PERCENT = 1;
static constexpr uint8_t SPEED_FILTER_ABSOLUTE = 2;
static constexpr uint8_t BINARY_FILTER = 3;

static constexpr double BASE_DEFAULT = 0.0;
static constexpr double MULTIPLIER_DEFAULT = 1.0;

static constexpr int8_t SPEED_MASK_UNKNOWN = -1;
static constexpr int8_t SPEED_MASK_NO_LIMIT = 0;
static constexpr double NO_SPEED_LIMIT = 0.0;

}  
