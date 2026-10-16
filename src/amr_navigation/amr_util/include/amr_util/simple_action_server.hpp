#pragma once 
#include <string>
#include <thread>
#include <future>
#include <mutex>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "amr_util/node_thread.hpp"
#include "glog/logging.h"
namespace amr_util{

template<class ActionT>
class SimpleActionServer{
public:
typedef std::function<void()> ExecuteCallback;
typedef std::function<void()> CompletionCallback;

template<class NodeT>
explicit SimpleActionServer(
    NodeT node,
    const std::string& action_name,
    ExecuteCallback execute_callback,
    CompletionCallback completion_callback = nullptr,
    std::chrono::milliseconds server_timeout = std::chrono::milliseconds(500),
    bool spin_thread = false,
    const rcl_action_server_options_t & options = rcl_action_server_get_default_options()
) : SimpleActionServer(
    node->get_node_base_interface(),
    node->get_node_clock_interface(),
    node->get_node_logging_interface(),
    node->get_node_waitables_interface(),
    action_name, execute_callback, completion_callback, server_timeout, spin_thread, options){}

  explicit SimpleActionServer(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
    const std::string & action_name,
    ExecuteCallback execute_callback,
    CompletionCallback completion_callback = nullptr,
    std::chrono::milliseconds server_timeout = std::chrono::milliseconds(500),
    bool spin_thread = false,
    const rcl_action_server_options_t & options = rcl_action_server_get_default_options())
  : node_base_interface_(node_base_interface),
    node_clock_interface_(node_clock_interface),
    node_logging_interface_(node_logging_interface),
    node_waitables_interface_(node_waitables_interface),
    action_name_(action_name),
    execute_callback_(execute_callback),
    completion_callback_(completion_callback),
    server_timeout_(server_timeout),
    spin_thread_(spin_thread)
  {
    using namespace std::placeholders;
    if (spin_thread_) {//内部不并发，外部不抢占
      callback_group_ = node_base_interface->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);//创建互斥回调，串行执行，防止action状态混乱
    }
    action_server_ = rclcpp_action::create_server<ActionT>(
      node_base_interface_,
      node_clock_interface_,
      node_logging_interface_,
      node_waitables_interface_,
      action_name_,
      std::bind(&SimpleActionServer::handle_goal, this, _1, _2),
      std::bind(&SimpleActionServer::handle_cancel, this, _1),
      std::bind(&SimpleActionServer::handle_accepted, this, _1),
      options,
      callback_group_);
    if (spin_thread_) {//单开线程，确保action独占
      executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_callback_group(callback_group_, node_base_interface_);
      executor_thread_ = std::make_unique<amr_util::NodeThread>(executor_);
    }
  }

~SimpleActionServer()
{
    deactivate();

    if (executor_) {
        executor_->cancel();
    }

    executor_thread_.reset();

    if (execution_future_.valid()) {
        execution_future_.wait();
    }
}
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const typename ActionT::Goal>){
        std::scoped_lock<std::recursive_mutex> lock(update_mutex_);
        if(!server_active_){
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle){
        std::scoped_lock<std::recursive_mutex> lock(update_mutex_);
        if(!handle->is_active()){
        return rclcpp_action::CancelResponse::REJECT;
        }
        LOG(WARNING) <<"目标取消";
        return rclcpp_action::CancelResponse::ACCEPT;
    }

  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle){
    std::scoped_lock<std::recursive_mutex> lock(update_mutex_);
    if (is_active(current_handle_) || is_running()) {
      
      if (is_active(pending_handle_)) {
        
        terminate(pending_handle_);
      }
      pending_handle_ = handle;
      preempt_requested_ = true;
    } else {
      if (is_active(pending_handle_)) {
        terminate(pending_handle_);
        preempt_requested_ = false;
      }

      current_handle_ = handle;
      execution_future_ = std::async(std::launch::async, [this]() {work();});
    }
}
 void work(){
    while(rclcpp::ok()&& !stop_execution_ && is_active(current_handle_)){
        try{
            execute_callback_();
        }catch(std::exception & ex){
            terminate_all();
            completion_callback_();
            return;
        }
        std::scoped_lock<std::recursive_mutex> lock(update_mutex_);
        if(stop_execution_){
            terminate_all();
            completion_callback_();
            break;
        }
        if(is_active(current_handle_)){
            terminate(current_handle_);
            completion_callback_();
        }
        if(is_active(pending_handle_)){
            accept_pending_goal();
        }else{break;}
    }
  }


  void activate(){
    std::scoped_lock<std::recursive_mutex> lock(update_mutex_);
    server_active_ = true;
    stop_execution_ = false;
}

  void deactivate(){
    std::scoped_lock<std::recursive_mutex> lock(update_mutex_);
    server_active_ = false;
    stop_execution_ = true;
        if (!execution_future_.valid()) {
      return;
    }

    if (is_running()) {
 
    }

    using namespace std::chrono;  
    auto start_time = steady_clock::now();
    while (execution_future_.wait_for(milliseconds(100)) != std::future_status::ready) {
      
      if (steady_clock::now() - start_time >= server_timeout_) {
        terminate_all();
        if (completion_callback_) {completion_callback_();}
      }
    }  
}

  bool is_running()
  {
    return execution_future_.valid() &&
           (execution_future_.wait_for(std::chrono::milliseconds(0)) ==
           std::future_status::timeout);
  }


    bool is_server_active()
  {
    std::scoped_lock<std::recursive_mutex> lock(update_mutex_);
    return server_active_;
  }


    bool is_preempt_requested() const
  {
    std::scoped_lock<std::recursive_mutex> lock(update_mutex_);
    return preempt_requested_;
  }

    const std::shared_ptr<const typename ActionT::Goal> accept_pending_goal()
  {
    std::scoped_lock<std::recursive_mutex> lock(update_mutex_);

    if (!pending_handle_ || !pending_handle_->is_active()) {
      LOG(ERROR)<<"没有可执行的抢占目标";
      return std::shared_ptr<const typename ActionT::Goal>();
    }

    if (is_active(current_handle_) && current_handle_ != pending_handle_) {
      LOG(INFO)<<"取消当前目标";
      current_handle_->abort(empty_result());
    }

    current_handle_ = pending_handle_;
    pending_handle_.reset();
    preempt_requested_ = false;

    LOG(INFO)<<"抢占成功";

    return current_handle_->get_goal();
  }

    void terminate_pending_goal()
  {
    std::scoped_lock<std::recursive_mutex> lock(update_mutex_);

    if (!pending_handle_ || !pending_handle_->is_active()) {
      LOG(ERROR)<<"没有执行的抢占目标用来丢弃";
      return;
    }

    terminate(pending_handle_);
    preempt_requested_ = false;

    LOG(INFO)<<"丢弃抢占目标";
  }

    const std::shared_ptr<const typename ActionT::Goal> get_current_goal() const
  {
    std::scoped_lock<std::recursive_mutex> lock(update_mutex_);

    if (!is_active(current_handle_)) {
      LOG(ERROR)<<"没有可执行的目标来获取当前goal";
      return std::shared_ptr<const typename ActionT::Goal>();
    }

    return current_handle_->get_goal();
  }

  const rclcpp_action::GoalUUID get_current_goal_id() const
  {
    std::scoped_lock<std::recursive_mutex> lock(update_mutex_);

    if (!is_active(current_handle_)) {
      LOG(ERROR)<<"没有可执行的目标来获取ID";
      return rclcpp_action::GoalUUID();
    }

    return current_handle_->get_goal_id();
  }

  const std::shared_ptr<const typename ActionT::Goal> get_pending_goal() const
  {
    std::scoped_lock<std::recursive_mutex> lock(update_mutex_);

    if (!pending_handle_ || !pending_handle_->is_active()) {
      LOG(ERROR)<<"没有抢占目标可用";
      return std::shared_ptr<const typename ActionT::Goal>();
    }

    return pending_handle_->get_goal();
  }

  bool is_cancel_requested() const
  {
    std::scoped_lock<std::recursive_mutex> lock(update_mutex_);

    if (current_handle_ == nullptr) {
      LOG(ERROR)<<"当前目标没有执行，无法取消";
      return false;
    }

    if (pending_handle_ != nullptr) {
      return pending_handle_->is_canceling();
    }

    return current_handle_->is_canceling();
  }

  void terminate_all(
    typename std::shared_ptr<typename ActionT::Result> result =
    std::make_shared<typename ActionT::Result>())
  {
    std::scoped_lock<std::recursive_mutex> lock(update_mutex_);
    terminate(current_handle_, result);
    terminate(pending_handle_, result);
    preempt_requested_ = false;
  }


  void terminate_current(
    typename std::shared_ptr<typename ActionT::Result> result =
    std::make_shared<typename ActionT::Result>())
  {
    std::scoped_lock<std::recursive_mutex> lock(update_mutex_);
    terminate(current_handle_, result);
  }

  void succeeded_current(
    typename std::shared_ptr<typename ActionT::Result> result =
    std::make_shared<typename ActionT::Result>())
  {
    std::scoped_lock<std::recursive_mutex> lock(update_mutex_);

    if (is_active(current_handle_)) {
      current_handle_->succeed(result);
      current_handle_.reset();
    }
  }

  void publish_feedback(typename std::shared_ptr<typename ActionT::Feedback> feedback)
  {
    if (!is_active(current_handle_)) {
      LOG(ERROR)<<"没有正在执行的goal来发布反馈";
    }

    current_handle_->publish_feedback(feedback);
  }

protected:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_;
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface_;
  std::string action_name_;

  ExecuteCallback execute_callback_;
  CompletionCallback completion_callback_;
  std::future<void> execution_future_;
  bool stop_execution_{false};

  mutable std::recursive_mutex update_mutex_;
  bool server_active_{false};
  bool preempt_requested_{false};
  std::chrono::milliseconds server_timeout_;

  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> current_handle_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> pending_handle_;

  typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;
  bool spin_thread_;
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<amr_util::NodeThread> executor_thread_;


  constexpr auto empty_result() const
  {
    return std::make_shared<typename ActionT::Result>();
  }

  constexpr bool is_active(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle) const
  {
    return handle != nullptr && handle->is_active();
  }


  void terminate(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & handle,
    typename std::shared_ptr<typename ActionT::Result> result =
    std::make_shared<typename ActionT::Result>())
  {
    std::scoped_lock<std::recursive_mutex> lock(update_mutex_);

    if (is_active(handle)) {
      if (handle->is_canceling()) {
        LOG(INFO)<<"客户端请求取消目标";
        handle->canceled(result);
      } else {
        LOG(WARNING)<<"异常情况导致导航终止";
        handle->abort(result);
      }
      handle.reset();
    }
  }





};
}