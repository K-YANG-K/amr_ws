#pragma once

#include <math.h>
#include <algorithm>
#include <vector>

#include "geometry_msgs/msg/point.hpp"

inline double sign(double x)
{
  return x < 0.0 ? -1.0 : 1.0;
}

inline double sign0(double x)
{
  return x < 0.0 ? -1.0 : (x > 0.0 ? 1.0 : 0.0);
}

inline double distance(double x0, double y0, double x1, double y1)
{
  return hypot(x1 - x0, y1 - y0);
}

double distanceToLine(double pX, double pY, double x0, double y0, double x1, double y1);

