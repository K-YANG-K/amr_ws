#include "agv_util/agv_glog.hpp"
#include <iostream>
#include <sys/stat.h>
#include <mutex>

// 全局默认日志目录
const std::string GLOBAL_LOG_DIR = "/home/kane/tianxing/agv_ws/agv_log";

namespace agv_util{

namespace {
    // 确保只初始化一次
    std::once_flag glog_init_flag;

    // 创建目录（如果不存在）
    bool createDirectoryIfNotExists(const std::string& path) {
        struct stat info;
        if (stat(path.c_str(), &info) != 0) {
            // 目录不存在，尝试创建
            std::string cmd = "mkdir -p " + path;
            return system(cmd.c_str()) == 0;
        } else if (info.st_mode & S_IFDIR) {
            // 目录已存在
            return true;
        }
        return false;
    }
}

void EnsureGlogInitialized(const char* program_name) {
    std::call_once(glog_init_flag, [program_name]() {
        // 初始化 glog
        google::InitGoogleLogging(program_name);
        google::InstallFailureSignalHandler();

        // 设置默认配置
        FLAGS_logtostderr = 0;           // 写文件
        FLAGS_alsologtostderr = 1;       // 同时输出终端
        FLAGS_colorlogtostderr = 1;
        FLAGS_stderrthreshold = 3;       // 只有 FATAL 才只走 stderr

        FLAGS_logbufsecs = 0;            // 实时写日志
        FLAGS_max_log_size = 100;        // 日志文件大小 100MB
        FLAGS_minloglevel = 0;           // 记录 INFO 及以上级别

        // 默认临时日志目录
        FLAGS_log_dir = "/tmp";

        std::cout << "Glog initialized with program: " << program_name << std::endl;
    });
}

void InitGlog(char** argv,
    const std::string& node_name,
    const std::string& node_namespace,
    const std::string& log_dir)
{
    // 确保 glog 已初始化
    const char* program_name = (argv && argv[0]) ? argv[0] : "unknown_program";
    EnsureGlogInitialized(program_name);

    // 确定日志目录
    std::string actual_log_dir = log_dir.empty() ? GLOBAL_LOG_DIR : log_dir;

    // 确保日志目录存在
    if (!actual_log_dir.empty()) {
        createDirectoryIfNotExists(actual_log_dir);
        FLAGS_log_dir = actual_log_dir;
    }

    // 设置日志文件目的地
    google::SetLogDestination(google::GLOG_INFO,
        (actual_log_dir + "/" + node_name + ".INFO.").c_str());
    google::SetLogDestination(google::GLOG_WARNING,
        (actual_log_dir + "/" + node_name + ".WARN.").c_str());
    google::SetLogDestination(google::GLOG_ERROR,
        (actual_log_dir + "/" + node_name + ".ERROR.").c_str());

    std::cout << "Glog configured for node: " << node_name
              << ", log dir: " << actual_log_dir << std::endl;
}

void ShutdownGlog()
{
    google::ShutdownGoogleLogging();
}

}

