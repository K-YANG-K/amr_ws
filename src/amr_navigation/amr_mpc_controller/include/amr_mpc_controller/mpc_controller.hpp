#pragma once

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "amr_core/controller.hpp"
#include "amr_mpc_controller/optimizer.hpp"
#include "glog/logging.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "amr_core/goal_checker.hpp"
#include "amr_util/lifecycle_node.hpp"
#include "amr_util/geometry_utils.hpp"
#include "amr_util/node_utils.hpp"
#include "amr_core/exceptions.hpp"
#include "amr_util/geometry_utils.hpp"
#include "amr_mpc_controller/optimizer.hpp"
using namespace mpc;
namespace amr_mpc_controller
{
class MPCController : public amr_core::Controller{
public:
    MPCController() = default;
    ~MPCController() override = default;

    void configure(const amr_util::LifecycleNode::WeakPtr & parent,
    std::string name,const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<amr_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed,
    amr_core::GoalChecker * goal_checker) override;

    void setPlan(const nav_msgs::msg::Path & path) override;

    void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  
    int shareCostMap();

    rcl_interfaces::msg::SetParametersResult
    dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters) override;

private:

    std::string name_;
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
    std::shared_ptr<amr_costmap_2d::Costmap2DROS> costmap_ros_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::mutex mutex_;
    double lookahead_dist_min_ = 0.0;
    double lookahead_dist_max_ = 0.0;
    double lookahead_dist_close_to_goal_ = 0.0;
    double control_frequency_{0.0};
    bool closer_to_goal_{false};
    bool slow_down_{true};
    bool no_slow_down_{false};
    geometry_msgs::msg::Pose goal_pose_;
    std::shared_ptr<Optimizer> optimizer_;

    double getLookAheadDistance();
    geometry_msgs::msg::PoseStamped getLookAheadPoint(const double & lookahead_dist);
    nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);
    bool transformPose(const std::string frame, const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

    geometry_msgs::msg::PoseStamped getLookAheadPoint(const double & lookahead_dist,const nav_msgs::msg::Path & transformed_plan);

    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;








//   nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);
  
//   bool transformPose(const std::string frame, const geometry_msgs::msg::PoseStamped & in_pose,
//     geometry_msgs::msg::PoseStamped & out_pose) const;
  
//   geometry_msgs::msg::PoseStamped getLookAheadPoint(
//     const double & lookahead_dist,
//     const nav_msgs::msg::Path & transformed_plan);
  
nav_msgs::msg::Path global_plan_;
//   std::shared_ptr<tf2_ros::Buffer> tf_;
//   std::string plugin_name_;
//   std::shared_ptr<amr_costmap_2d::Costmap2DROS> costmap_ros_;
//   geometry_msgs::msg::TwistStamped m_last_cmd_vel;
//   amr_costmap_2d::Costmap2D * costmap_;
//   rclcpp::Logger logger_ {rclcpp::get_logger("MPC")};
//   rclcpp::Clock::SharedPtr clock_;
//   rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
//  // amr::LifecycleNode::WeakPtr node_;
//   tf2::Duration transform_tolerance_;
//   //amr::ServiceClient<neo_srvs2::srv::Optimizer>::SharedPtr client;

//   rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr carrot_pub_;

//   std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsg(
//     const geometry_msgs::msg::PoseStamped & carrot_pose);

//   double getLookAheadDistance(const geometry_msgs::msg::Twist & speed);

//   geometry_msgs::msg::Pose goal_pose;
//   bool closer_to_goal = false;
//   bool slow_down_ = true;
//   bool no_slow_down_ = true;
  
//   double lookahead_dist_min_ = 0.0;
//   double lookahead_dist_max_ = 0.0;
//   double lookahead_dist_close_to_goal_ = 0.0;
//   double control_frequency = 0.0;

// //   std::unique_ptr<amr_costmap_2d::FootprintCollisionChecker<amr_costmap_2d::Costmap2D *>>
// //   collision_checker_;
//   rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
//   std::mutex mutex_;
};
}
