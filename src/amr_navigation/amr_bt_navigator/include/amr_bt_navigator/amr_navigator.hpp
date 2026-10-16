#pragma once

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "amr_behavior_tree/bt_action_server.hpp"
#include "amr_behavior_tree/behavior_tree_engine.hpp"
#include "glog/logging.h"

namespace amr_bt_navigator{

struct FeedbackUtils
{
  std::string robot_frame;
  std::string global_frame;
  double transform_tolerance;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
};

//此类用于判断是否有导航任务正在执行
class NavigatorMuxer{
public:
    NavigatorMuxer() : current_navigator_(std::string("")){}

    bool isNavigating(){
        std::scoped_lock<std::mutex> navigator_lock(mutex_);
        return !current_navigator_.empty();
    }

    void startNavigating(const std::string & navigator_name){
        std::scoped_lock<std::mutex> navigator_lock(mutex_);
        if(!current_navigator_.empty())
          LOG(ERROR)<<"当前导航任务正在执行，收到新的任务请求!";
        current_navigator_ = navigator_name;
    }
    void stopNavigating(const std::string & navigator_name){
        std::scoped_lock<std::mutex> navigator_lock(mutex_);
        if(current_navigator_ != navigator_name)
        LOG(ERROR)<<"已有导航模式在运行，需要停止当前导航!";
        else current_navigator_ = std::string("");//导航任务结束，置空导航mode
    }
protected:
    std::string current_navigator_;
    std::mutex mutex_;
};

//所有导航模式的基类
template<class ActionT>
class Navigator{
public:
    using Ptr = std::shared_ptr<amr_bt_navigator::Navigator<ActionT>>;
    Navigator(){
        plugin_muxer_ = nullptr;
    }
    virtual ~Navigator() = default;

    bool on_configure(rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
                      const std::vector<std::string> &bt_nodes,
                      const FeedbackUtils &feedback_utils,
    amr_bt_navigator::NavigatorMuxer* plugin_muxer){
        auto node = parent_node.lock();
        clock_ = node->get_clock();
        feedback_utils_ = feedback_utils;
        plugin_muxer_ = plugin_muxer;
        std::string bt_xml_filename = getBTFilepath(parent_node);
        bt_action_server_ = std::make_unique<amr_behavior_tree::BtActionServer<ActionT>>(
            node,
            getMode(),
            bt_nodes,
            bt_xml_filename,
            std::bind(&Navigator::onGoalReceived,this,std::placeholders::_1),
            std::bind(&Navigator::onLoop, this),
            std::bind(&Navigator::onPreempt,this,std::placeholders::_1),
            std::bind(&Navigator::onCompletion,this,std::placeholders::_1,std::placeholders::_2));
        bool ok = true;
        if(!bt_action_server_->on_configure()){
            ok = false;
        }
        BT::Blackboard::Ptr blackboard = bt_action_server_->getBlackboard();
        blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", feedback_utils_.tf_buffer);//从全局字典里拿tf
        return configure(parent_node) && ok;
    }

    bool on_activate()
    {
        bool ok = true;

        if (!bt_action_server_->on_activate()) {
        ok = false;
        }

        return activate() && ok;
    }

    bool on_deactivate()
    {
        bool ok = true;
        if (!bt_action_server_->on_deactivate()) {
        ok = false;
        }

        return deactivate() && ok;
    }

    bool on_cleanup()
    {
        bool ok = true;
        if (!bt_action_server_->on_cleanup()) {
        ok = false;
        }

        bt_action_server_.reset();

        return cleanup() && ok;
    }

    //获取行为树xml文件
    virtual std::string getBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) = 0;
    //获取导航模式
    virtual std::string getMode() = 0;
    //通过持有BtActionServer来实现action和BT的桥接
    std::unique_ptr<amr_behavior_tree::BtActionServer<ActionT>> & getActionServer() { return bt_action_server_; }

protected:
    //导航模式的校验，通过BtActionServer类中的executeCallback调用
    bool onGoalReceived(typename ActionT::Goal::ConstSharedPtr goal){
        if(plugin_muxer_->isNavigating()){
            LOG(ERROR)<<"当前有正在执行的导航任务: "<<getMode();
            return false;
        }
        bool goal_accepted = goalReceived(goal);
        if(goal_accepted)
            plugin_muxer_->startNavigating(getMode());
        return goal_accepted;
    }

    void onCompletion(
        typename ActionT::Result::SharedPtr result,
        const amr_behavior_tree::BtStatus final_bt_status)
    {
        plugin_muxer_->stopNavigating(getMode());
        goalCompleted(result, final_bt_status);
    }

    virtual bool goalReceived(typename ActionT::Goal::ConstSharedPtr goal) = 0;
    virtual void onLoop() = 0;
    virtual void onPreempt(typename ActionT::Goal::ConstSharedPtr goal) = 0;
    virtual void goalCompleted(
        typename ActionT::Result::SharedPtr result,
        const amr_behavior_tree::BtStatus final_bt_status) = 0;
    virtual bool configure(rclcpp_lifecycle::LifecycleNode::WeakPtr) { return true; }
    virtual bool cleanup() { return true; }
    virtual bool activate() { return true; }
    virtual bool deactivate() { return true; }

    std::unique_ptr<amr_behavior_tree::BtActionServer<ActionT>> bt_action_server_;
    rclcpp::Clock::SharedPtr clock_;
    FeedbackUtils feedback_utils_;
    NavigatorMuxer* plugin_muxer_;
};

}