#pragma once
#include <vector>
#include <string>

namespace amr_costmap_2d
{
std::vector<std::vector<float>> parseVVF(const std::string & input, std::string & error_return);
}  

