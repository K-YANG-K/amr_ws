#pragma once

#include <stdexcept>
#include <string>
#include <memory>

namespace amr_core
{

class PlannerException : public std::runtime_error
{
public:
  explicit PlannerException(const std::string description)
  : std::runtime_error(description) {}
  using Ptr = std::shared_ptr<PlannerException>;
};

} 