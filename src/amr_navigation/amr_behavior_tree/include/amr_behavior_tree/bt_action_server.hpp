#pragma once 

#include <memory>
#include <string>
#include <vector>

#include "amr_behavior_tree/behavior_tree_engine.hpp"
#include "amr_util/lifecycle_node.hpp"
#include "amr_util/simple_action_server.hpp"
#include "amr_util/node_utils.hpp"
#include "glog/logging.h"
#include <fstream>
namespace amr_behavior_tree{

template<class ActionT>
class BtActionServer{//此类是对action_server_的封装，用于驱动bt
public:
    using ActionServer = amr_util::SimpleActionServer<ActionT>;
    typedef std::function<bool (typename ActionT::Goal::ConstSharedPtr)> OnGoalReceivedCallback;
    typedef std::function<void ()> OnLoopCallback;
    typedef std::function<void (typename ActionT::Goal::ConstSharedPtr)> OnPreemptCallback;
    typedef std::function<void (typename ActionT::Result::SharedPtr, amr_behavior_tree::BtStatus)> OnCompletionCallback;

    explicit BtActionServer(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        const std::string & action_name,
        const std::vector<std::string> & bt_nodes,
        const std::string & bt_xml_filename,
        OnGoalReceivedCallback on_goal_received_callback,
        OnLoopCallback on_loop_callback,
        OnPreemptCallback on_preempt_callback,
        OnCompletionCallback on_completion_callback);
    ~BtActionServer();

  bool on_configure();
  bool on_activate();
  bool on_deactivate();
  bool on_cleanup();
  bool loadBehaviorTree(const std::string & bt_xml_filename = "");
  BT::Blackboard::Ptr getBlackboard() const
  {
    return blackboard_;
  }

  std::string getCurrentBTFilename() const
  {
    return current_bt_xml_filename_;
  }

  std::string getDefaultBTFilename() const
  {
    return bt_xml_filename_;
  }

  const std::shared_ptr<const typename ActionT::Goal> acceptPendingGoal()
  {
    return action_server_->accept_pending_goal();
  }

  void terminatePendingGoal()
  {
    action_server_->terminate_pending_goal();
  }

  const std::shared_ptr<const typename ActionT::Goal> getCurrentGoal() const
  {
    return action_server_->get_current_goal();
  }

  const std::shared_ptr<const typename ActionT::Goal> getPendingGoal() const
  {
    return action_server_->get_pending_goal();
  }

  void publishFeedback(typename std::shared_ptr<typename ActionT::Feedback> feedback)
  {
    action_server_->publish_feedback(feedback);
  }

  const BT::Tree & getTree() const
  {
    return tree_;
  }

  void haltTree()
  {
    tree_.rootNode()->halt();
  }

protected:

  void executeCallback();
  std::string action_name_;
  std::shared_ptr<ActionServer> action_server_;
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;
  std::string current_bt_xml_filename_;
  std::string bt_xml_filename_;
  std::unique_ptr<amr_behavior_tree::BehaviorTreeEngine> bt_;
  std::vector<std::string> bt_nodes_;

  rclcpp::Node::SharedPtr client_node_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Clock::SharedPtr clock_;

  std::chrono::milliseconds bt_loop_duration_;
  std::chrono::milliseconds default_server_timeout_;
  std::chrono::milliseconds wait_for_service_timeout_;
  bool always_reload_bt_xml_ = false;

  OnGoalReceivedCallback on_goal_received_callback_;
  OnLoopCallback on_loop_callback_;
  OnPreemptCallback on_preempt_callback_;
  OnCompletionCallback on_completion_callback_;
};

} 
#include <amr_behavior_tree/bt_action_server_impl.hpp>