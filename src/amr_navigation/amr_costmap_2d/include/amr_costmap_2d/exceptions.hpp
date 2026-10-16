#pragma once
#include <stdexcept>
#include <string>
#include <memory>

namespace amr_costmap_2d
{

class CollisionCheckerException : public std::runtime_error
{
public:
  explicit CollisionCheckerException(const std::string description)
  : std::runtime_error(description) {}
};

class IllegalPoseException : public CollisionCheckerException
{
public:
  IllegalPoseException(const std::string name, const std::string description)
  : CollisionCheckerException(description), name_(name) {}
  std::string getCriticName() const {return name_;}

protected:
  std::string name_;
};

} 
