#pragma once

#include <string>

namespace amr_smac_planner
{
enum class MotionModel
{
  UNKNOWN = 0,
  TWOD = 1,
  DUBIN = 2,
  REEDS_SHEPP = 3,
  STATE_LATTICE = 4,
};

inline std::string toString(const MotionModel & n)
{
  switch (n) {
    case MotionModel::TWOD:
      return "2D";
    case MotionModel::DUBIN:
      return "Dubin";
    case MotionModel::REEDS_SHEPP:
      return "Reeds-Shepp";
    case MotionModel::STATE_LATTICE:
      return "State Lattice";
    default:
      return "Unknown";
  }
}

inline MotionModel fromString(const std::string & n)
{
  if (n == "2D") {
    return MotionModel::TWOD;
  } else if (n == "DUBIN") {
    return MotionModel::DUBIN;
  } else if (n == "REEDS_SHEPP") {
    return MotionModel::REEDS_SHEPP;
  } else if (n == "STATE_LATTICE") {
    return MotionModel::STATE_LATTICE;
  } else {
    return MotionModel::UNKNOWN;
  }
}

const float UNKNOWN = 255.0;
const float OCCUPIED = 254.0;
const float INSCRIBED = 253.0;
const float MAX_NON_OBSTACLE = 252.0;
const float FREE = 0;

}  