#pragma once

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <vector>
#include <glog/logging.h>
#include "amr_util/lifecycle_node.hpp"
#include "amr_util/amr_glog.hpp"
#include "amr_util/node_utils.hpp"
namespace amr_bt_navigator{
class AmrBtNavigator : public amr_util::LifecycleNode
{
public:
    explicit AmrBtNavigator(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    virtual ~AmrBtNavigator();
protected:
    amr_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    amr_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    amr_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    amr_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    amr_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
private:
    const std::vector<std::string> bt_nodes = {
        "amr_global_action_bt_node",
        "amr_local_action_bt_nodes"
    };
    std::vector<std::unique_ptr<std::thread>> cpu_stress_threads_;
    std::atomic<bool> stress_running_{false};
    void cpuStressTask(int thread_id);
    void allocateMemory(size_t size_mb);
    std::vector<std::unique_ptr<char[]>> memory_chunks_;
};
}
