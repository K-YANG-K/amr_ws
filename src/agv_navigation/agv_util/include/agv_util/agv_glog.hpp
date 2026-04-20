#ifndef AGV_GLOG_HPP_
#define AGV_GLOG_HPP_
#include <glog/logging.h>
#include <string>

namespace agv_util{
// 初始化 glog（确保只调用一次）
void EnsureGlogInitialized(const char* program_name = "agv_program");

void InitGlog(char** argv,
        const std::string& node_name,
        const std::string& node_namespace = "",
        const std::string& log_dir = "");  // 空字符串表示使用全局默认目录

void ShutdownGlog();
}

#endif