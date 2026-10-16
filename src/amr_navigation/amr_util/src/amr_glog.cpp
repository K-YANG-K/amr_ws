#include "amr_util/amr_glog.hpp"

#include <glog/logging.h>
#include <sys/stat.h>
#include <mutex>
#include <iostream>

namespace amr_util
{

// 保存路径
static const std::string GLOBAL_LOG_DIR =
    "/home/kane/Atlas/amr_ws/amr_log";

static std::once_flag glog_init_flag;

static bool createDirectoryIfNotExists(const std::string& path)
{
    struct stat info;
    if (stat(path.c_str(), &info) != 0)
    {
        std::string cmd = "mkdir -p " + path;
        return system(cmd.c_str()) == 0;
    }
    return (info.st_mode & S_IFDIR);
}


void EnsureGlogInitialized(const std::string& program_name)
{
    std::call_once(glog_init_flag, [&]()
    {
        google::InitGoogleLogging(program_name.c_str());
        google::InstallFailureSignalHandler();


        FLAGS_logtostderr = 0;        
        FLAGS_alsologtostderr = 1;  
        FLAGS_colorlogtostderr = 1;
        FLAGS_stderrthreshold = 3;   

        FLAGS_logbufsecs = 0;       
        FLAGS_max_log_size = 10;     
        FLAGS_minloglevel = 0;       

        FLAGS_log_dir = "/tmp";

        std::cout << "[GLOG] Process initialized: "
                  << program_name << std::endl;
    });
}


void InitGlogForNode(
    const std::string& node_name,
    const std::string& log_dir)
{
 
    EnsureGlogInitialized(node_name);


    std::string actual_dir =
        log_dir.empty() ? GLOBAL_LOG_DIR : log_dir;

    createDirectoryIfNotExists(actual_dir);
    FLAGS_log_dir = actual_dir;

    google::SetLogDestination(
        google::GLOG_INFO,
        (actual_dir + "/" + node_name + ".INFO.").c_str());

    google::SetLogDestination(
        google::GLOG_WARNING,
        (actual_dir + "/" + node_name + ".WARN.").c_str());

    google::SetLogDestination(
        google::GLOG_ERROR,
        (actual_dir + "/" + node_name + ".ERROR.").c_str());

    std::cout << "[GLOG] Node configured: "
              << node_name
              << "  log_dir=" << actual_dir
              << std::endl;
}


void ShutdownGlog()
{
    google::ShutdownGoogleLogging();
}

}