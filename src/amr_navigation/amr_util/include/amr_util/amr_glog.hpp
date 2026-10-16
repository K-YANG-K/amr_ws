#pragma once
#include <string>

namespace amr_util
{

void EnsureGlogInitialized(const std::string& program_name);
void InitGlogForNode(
    const std::string& node_name,
    const std::string& log_dir = "");
void ShutdownGlog();

}