#pragma once
#include <fstream>
#include <set>
#include <exception>
#include <vector>
#include "glog/logging.h"
#include "amr_util/node_utils.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
/*
整个行为树是一个server,树中的节点是一个个的小client,来调用外部具体的server来干活
通过BtActionServer类来响应外部的action client，然后初始化行为树，最终在executeCallback中启动行为树
bt本身是一个c++类，并不具备ros的功能，该类是桥接ros和bt的，核心实现是传一个匿名节点给bt黑板
*/
namespace amr_behavior_tree{
template<class ActionT>
BtActionServer<ActionT>::BtActionServer(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & action_name,
    const std::vector<std::string> & bt_nodes,
    const std::string & bt_xml_filename,
    OnGoalReceivedCallback on_goal_received_callback,
    OnLoopCallback on_loop_callback,
    OnPreemptCallback on_preempt_callback,
    OnCompletionCallback on_completion_callback)
    : action_name_(action_name),
    bt_xml_filename_(bt_xml_filename),
    bt_nodes_(bt_nodes),
    node_(parent),
    on_goal_received_callback_(on_goal_received_callback),
    on_loop_callback_(on_loop_callback),
    on_preempt_callback_(on_preempt_callback),
    on_completion_callback_(on_completion_callback)
    {
    auto node = node_.lock();
    clock_ = node->get_clock();


    amr_util::declare_parameter_if_not_declared(node,"bt_loop_duration",rclcpp::ParameterValue(10));
    amr_util::declare_parameter_if_not_declared(node,"default_server_timeout",rclcpp::ParameterValue(20));
    amr_util::declare_parameter_if_not_declared(node,"always_reload_bt_xml",rclcpp::ParameterValue(false));
    amr_util::declare_parameter_if_not_declared(node,"wait_for_service_timeout",rclcpp::ParameterValue(1000));
    }

template<class ActionT>
BtActionServer<ActionT>::~BtActionServer(){}

template<class ActionT>
bool BtActionServer<ActionT>::on_configure(){
    auto node = node_.lock();
    if(!node){
        throw std::runtime_error{"节点获取失败"};
    }
    std::string client_node_name = action_name_;
    std::replace(client_node_name.begin(), client_node_name.end(), '/', '_');
    client_node_name.erase(0, client_node_name.find_first_not_of('_'));
    auto options = rclcpp::NodeOptions().arguments(
        {"--ros-args",
        "-r",
        std::string("__node:=") +
        std::string(node->get_name()) + "_" + client_node_name + "_rclcpp_node",
        "--"});
    client_node_ = std::make_shared<rclcpp::Node>("_", options);//为行为树节点创建内部客户端
    amr_util::declare_parameter_if_not_declared(
    node, "global_frame", rclcpp::ParameterValue(std::string("map")));
    amr_util::declare_parameter_if_not_declared(
    node, "robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
    amr_util::declare_parameter_if_not_declared(
    node, "transform_tolerance", rclcpp::ParameterValue(0.1));
    amr_util::copy_all_parameters(node, client_node_);
    action_server_ = std::make_shared<ActionServer>(
    node->get_node_base_interface(),
    node->get_node_clock_interface(),
    node->get_node_logging_interface(),
    node->get_node_waitables_interface(),
    action_name_, std::bind(&BtActionServer<ActionT>::executeCallback, this));

    int timeout;
    node->get_parameter("bt_loop_duration", timeout);
    bt_loop_duration_ = std::chrono::milliseconds(timeout);
    node->get_parameter("default_server_timeout", timeout);
    default_server_timeout_ = std::chrono::milliseconds(timeout);
    int wait_for_service_timeout;
    node->get_parameter("wait_for_service_timeout", wait_for_service_timeout);
    wait_for_service_timeout_ = std::chrono::milliseconds(wait_for_service_timeout);
    node->get_parameter("always_reload_bt_xml", always_reload_bt_xml_);

    bt_ = std::make_unique<amr_behavior_tree::BehaviorTreeEngine>(bt_nodes_);
    blackboard_ = BT::Blackboard::create();

    blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_); 
    blackboard_->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_);  
    blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_); 
    blackboard_->set<std::chrono::milliseconds>("wait_for_service_timeout",wait_for_service_timeout_);

  return true;

}

template<class ActionT>
bool BtActionServer<ActionT>::on_activate()
{
  if (!loadBehaviorTree(bt_xml_filename_)) {
    LOG(ERROR)<<"加载" << bt_xml_filename_ <<"失败";
    return false;
  }
  action_server_->activate();
  return true;
}
template<class ActionT>
bool BtActionServer<ActionT>::on_deactivate()
{
  action_server_->deactivate();
  return true;
}

template<class ActionT>
bool BtActionServer<ActionT>::on_cleanup()
{
  client_node_.reset();
  action_server_.reset();
  blackboard_.reset();
  bt_->haltAllActions(tree_.rootNode());
  bt_.reset();
  return true;
}
template<class ActionT>
bool BtActionServer<ActionT>::loadBehaviorTree(const std::string & bt_xml_filename){
    if(bt_xml_filename.empty()) return false;
    if (!always_reload_bt_xml_ && current_bt_xml_filename_ == bt_xml_filename) {
    LOG(WARNING)<<"重复加载行为树文件";
    return true;
  }
    std::ifstream xml_file(bt_xml_filename);

    if (!xml_file.good()) {
    LOG(ERROR)<<"无法打开行为树文件";
    return false;
  }
    auto xml_string = std::string(
    std::istreambuf_iterator<char>(xml_file),
    std::istreambuf_iterator<char>());

    try {
        tree_ = bt_->createTreeFromText(xml_string, blackboard_);
        for (auto & blackboard : tree_.blackboard_stack) {//给每个BT节点注入rosnode,以此实现客户端的创建
        blackboard->set<rclcpp::Node::SharedPtr>("node", client_node_);
        blackboard->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_);
        blackboard->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);
        blackboard->set<std::chrono::milliseconds>(
            "wait_for_service_timeout",
            wait_for_service_timeout_);
        }
    } catch (const std::exception & e) {
        LOG(ERROR)<<"加载BT异常:"<<e.what();
        return false;
  }
    current_bt_xml_filename_ = bt_xml_filename;
    return true;
}

template<class ActionT>
void BtActionServer<ActionT>::executeCallback()
{
  if (!on_goal_received_callback_(getCurrentGoal())) {//判断是否有不同类型的goal来执行回调
    action_server_->terminate_current();
    return;
  }

  auto is_canceling = [&]() {
      if (action_server_ == nullptr) {
        LOG(WARNING)<<"Action 服务器为空，正在取消";
        return true;
      }
      if (!action_server_->is_server_active()) {
        LOG(WARNING)<<"Action 服务器未激活，正在取消";
        return true;
      }
      return action_server_->is_cancel_requested();
    };

  auto on_loop = [&]() {
      if (action_server_->is_preempt_requested() && on_preempt_callback_) {
        on_preempt_callback_(action_server_->get_pending_goal());
      }
      on_loop_callback_();
    };

  amr_behavior_tree::BtStatus rc = bt_->run(&tree_, on_loop, is_canceling, bt_loop_duration_);

  bt_->haltAllActions(tree_.rootNode());

  auto result = std::make_shared<typename ActionT::Result>();
  on_completion_callback_(result, rc);

  switch (rc) {
    case amr_behavior_tree::BtStatus::SUCCEEDED:
      action_server_->succeeded_current(result);
      LOG(INFO)<<"目标执行成功";
      break;

    case amr_behavior_tree::BtStatus::FAILED:
      action_server_->terminate_current(result);
      LOG(INFO)<<"目标执行失败";
      break;

    case amr_behavior_tree::BtStatus::CANCELED:
      action_server_->terminate_all(result);
      LOG(INFO)<<"目标取消";
      break;
  }
}
}