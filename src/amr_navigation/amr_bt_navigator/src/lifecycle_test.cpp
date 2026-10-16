#include "amr_bt_navigator/amr_bt_navigator.hpp"

#include <memory>
#include <string>
#include <chrono>
#include <cmath>

namespace amr_bt_navigator{

AmrBtNavigator::AmrBtNavigator(rclcpp::NodeOptions options):
    amr_util::LifecycleNode("amr_bt_navigator", "", options){
    amr_util::InitGlogForNode("amr_bt_navigator");
    LOG(INFO)<<"执行amr_bt_navigator构造函数";
    declare_parameter_if_not_declared(this,"bt_nodes",rclcpp::ParameterValue(bt_nodes));

    }

AmrBtNavigator::~AmrBtNavigator(){
    stress_running_ = false;
    for(auto& thread : cpu_stress_threads_){
        if(thread && thread->joinable()){
            thread->join();
        }
    }
}

void AmrBtNavigator::allocateMemory(size_t size_mb){
    size_t size = size_mb * 1024 * 1024;
    auto chunk = std::make_unique<char[]>(size);
    memset(chunk.get(), 0xAB, size);
    memory_chunks_.push_back(std::move(chunk));
    LOG(INFO) << "已分配 " << size_mb << " MB 内存，总计 " << memory_chunks_.size() << " 块";
}

void AmrBtNavigator::cpuStressTask(int thread_id){
    LOG(INFO) << "CPU压力测试线程 " << thread_id << " 启动";
    double result = 0.0;
    int counter = 0;
    while(stress_running_){
        for(int i = 0; i < 1000000; ++i){
            result += std::sin(i) * std::cos(i);
        }
        counter++;
        if(counter % 100 == 0){
            LOG(INFO) << "线程 " << thread_id << " 计算进度: " << counter << " 次，当前值: " << result;
        }
    }
    LOG(INFO) << "CPU压力测试线程 " << thread_id << " 停止";
}

amr_util::CallbackReturn AmrBtNavigator::on_configure(const rclcpp_lifecycle::State &){
    LOG(INFO)<<"amr_bt_navigator 配置启动 - 开始加载内存资源";
    allocateMemory(250);
    allocateMemory(250);
    allocateMemory(250);
    allocateMemory(250);
    LOG(INFO)<<"amr_bt_navigator 配置完成 - 已分配 1 GB 内存资源";
    return amr_util::CallbackReturn::SUCCESS;
}

amr_util::CallbackReturn AmrBtNavigator::on_activate(const rclcpp_lifecycle::State &){
    LOG(INFO)<<"amr_bt_navigator 激活 - 开始CPU压力测试";
    int num_threads = 2;
    stress_running_ = true;
    for(int i = 0; i < num_threads; ++i){
        cpu_stress_threads_.push_back(
            std::make_unique<std::thread>(&AmrBtNavigator::cpuStressTask, this, i)
        );
    }
    LOG(INFO)<<"amr_bt_navigator 激活完成 - 已启动 " << num_threads << " 个CPU压力测试线程";
    return amr_util::CallbackReturn::SUCCESS;
}

amr_util::CallbackReturn AmrBtNavigator::on_deactivate(const rclcpp_lifecycle::State &){
    LOG(INFO)<<"amr_bt_navigator 停用 - 结束CPU压力测试";
    stress_running_ = false;
    for(auto& thread : cpu_stress_threads_){
        if(thread && thread->joinable()){
            thread->join();
        }
    }
    cpu_stress_threads_.clear();
    LOG(INFO)<<"amr_bt_navigator 停用完成 - 所有CPU压力测试线程已停止";
    return amr_util::CallbackReturn::SUCCESS;
}

amr_util::CallbackReturn AmrBtNavigator::on_cleanup(const rclcpp_lifecycle::State &){
    LOG(INFO)<<"amr_bt_navigator 清理 - 释放内存资源";
    memory_chunks_.clear();
    LOG(INFO)<<"amr_bt_navigator 清理完成 - 内存资源已释放";
    return amr_util::CallbackReturn::SUCCESS;
}

amr_util::CallbackReturn AmrBtNavigator::on_shutdown(const rclcpp_lifecycle::State &){
    LOG(INFO)<<"amr_bt_navigator 关闭";
    return amr_util::CallbackReturn::SUCCESS;
}














    
}