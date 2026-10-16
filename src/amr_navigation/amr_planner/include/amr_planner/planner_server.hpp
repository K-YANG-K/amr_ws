#pragma once

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <mutex>

#include "amr_msgs/action/generate_follow_path.hpp"
#include "amr_msgs/action/plan_global_path.hpp"
#include "amr_msgs/action_names.hpp"
#include "amr_msgs/topic_names.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "amr_util/lifecycle_node.hpp"
#include "amr_msgs/msg/costmap.hpp"
#include "amr_util/robot_utils.hpp"
#include "amr_util/simple_action_server.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "amr_costmap_2d/costmap_2d_ros.hpp"
#include "amr_costmap_2d/footprint_collision_checker.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "amr_core/global_planner.hpp"
#include "amr_msgs/srv/is_path_valid.hpp"
#include "amr_msgs/server_names.hpp"
#include "amr_util/geometry_utils.hpp"
#include "glog/logging.h"

namespace amr_planner
{
class PlannerServer :public amr_util::LifecycleNode
{
public:
    explicit PlannerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~PlannerServer();
    using PlannerMap = std::unordered_map<std::string,amr_core::GlobalPlanner::Ptr>;

    nav_msgs::msg::Path getPlan(const std::vector<geometry_msgs::msg::PoseStamped> & way_poses,const std::string & planner_id);

protected:
    amr_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    amr_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    amr_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    amr_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    amr_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    using ActionGenerateFollowPath = amr_msgs::action::GenerateFollowPath;
    using ActionServerGenerateFollowPath = amr_util::SimpleActionServer<ActionGenerateFollowPath>;

    using ActionPlanGlobalPath = amr_msgs::action::PlanGlobalPath;
    using ActionServerPlanGlobalPath = amr_util::SimpleActionServer<ActionPlanGlobalPath>;

    template<typename T>
    bool isServerInactive(std::unique_ptr<amr_util::SimpleActionServer<T>> & action_server);

    template<typename T>
    bool isCancelRequested(std::unique_ptr<amr_util::SimpleActionServer<T>> & action_server);

    void waitForCostmap();
    template<typename T>
    void getPreemptedGoalIfRequested(
    std::unique_ptr<amr_util::SimpleActionServer<T>> & action_server,
    typename std::shared_ptr<const typename T::Goal> goal);

    template<typename T>
    bool getStartPose(
    std::unique_ptr<amr_util::SimpleActionServer<T>> & action_server,
    typename std::shared_ptr<const typename T::Goal> goal,
    geometry_msgs::msg::PoseStamped & start);

    template<typename T>
    bool transformPosesToGlobalFrame(
    std::unique_ptr<amr_util::SimpleActionServer<T>> & action_server,
    geometry_msgs::msg::PoseStamped & curr_start,
    geometry_msgs::msg::PoseStamped & curr_goal);

    template<typename T>
    bool validatePath(
    std::unique_ptr<amr_util::SimpleActionServer<T>> & action_server,
    const geometry_msgs::msg::PoseStamped & curr_goal,
    const nav_msgs::msg::Path & path,
    const std::string & planner_id);

    template<typename T>
    bool validateGoal(
    std::unique_ptr<amr_util::SimpleActionServer<T>> & action_server,
    const std::vector<geometry_msgs::msg::PoseStamped> & way_poses);

    void generateFollowPath();
    void planGlobalPath();

    void isPathValid(
    const std::shared_ptr<amr_msgs::srv::IsPathValid::Request> request,
    std::shared_ptr<amr_msgs::srv::IsPathValid::Response> response);

    void publishPlan(const nav_msgs::msg::Path & path);

    std::mutex mutex_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
    rcl_interfaces::msg::SetParametersResult
    dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
    
    std::unique_ptr<ActionServerGenerateFollowPath> action_server_generate_follow_path_;
    std::unique_ptr<ActionServerPlanGlobalPath> action_server_plan_global_path_;
  
    PlannerMap planners_;
    pluginlib::ClassLoader<amr_core::GlobalPlanner> gp_loader_;

    std::vector<std::string> planner_ids_;
    std::vector<std::string> planner_types_;
    double max_planner_duration_;
    std::string planner_ids_concat_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    std::shared_ptr<amr_costmap_2d::Costmap2DROS> costmap_ros_;
    std::unique_ptr<amr_util::NodeThread> costmap_thread_;
    amr_costmap_2d::Costmap2D * costmap_;
    std::unique_ptr<amr_costmap_2d::FootprintCollisionChecker<amr_costmap_2d::Costmap2D *>>
    collision_checker_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr global_path_publisher_;
    rclcpp::Service<amr_msgs::srv::IsPathValid>::SharedPtr is_path_valid_service_;
    bool debug_enable_;
};
}