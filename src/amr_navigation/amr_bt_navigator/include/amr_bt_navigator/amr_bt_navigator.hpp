#pragma once

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <glog/logging.h>
#include "amr_util/lifecycle_node.hpp"
#include "amr_util/amr_glog.hpp"
#include "amr_util/node_utils.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "amr_bt_navigator/amr_navigator.hpp"
#include "amr_msgs/action/amr_nav_mode_a.hpp"
#include "amr_msgs/action/amr_nav_mode_b.hpp"

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

    std::unique_ptr<amr_bt_navigator::Navigator<amr_msgs::action::AmrNavModeA>> amr_nav_mode_a_navigator_;

    std::unique_ptr<amr_bt_navigator::Navigator<amr_msgs::action::AmrNavModeB>> amr_nav_mode_b_navigator_;

    amr_bt_navigator::NavigatorMuxer plugin_muxer_;

    std::string robot_frame_;
    std::string global_frame_;
    double transform_tolerance_;
    std::vector<std::string> bt_nodes_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


};
}