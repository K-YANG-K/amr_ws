#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "amr_util/costmap.hpp"
#include "amr_util/node_utils.hpp"
#include "amr_util/geometry_utils.hpp"
#include "amr_costmap_2d/cost_values.hpp"
#include "amr_costmap_2d/footprint_collision_checker.hpp"
#include "amr_planner/planner_server.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace amr_planner{
PlannerServer::PlannerServer(const rclcpp::NodeOptions & options) : amr_util::LifecycleNode("planner_server","",options),gp_loader_("amr_core","amr_core::GlobalPlanner"),costmap_(nullptr){
    LOG(INFO)<<"初始化LifecycleNode--->[planner_server]";

    amr_util::declare_parameter_if_not_declared(this, "debug_enable", rclcpp::ParameterValue(true));

    amr_util::declare_parameter_if_not_declared(this,"planner_plugins",planner_ids_);
    amr_util::declare_parameter_if_not_declared(this,"planner_types",planner_types_);
    amr_util::declare_parameter_if_not_declared(this,"expected_planner_frequency",rclcpp::ParameterValue(1.0));

    get_parameter("debug_enable",debug_enable_);
    get_parameter("planner_plugins",planner_ids_);
    get_parameter("planner_types",planner_types_);
    for(size_t i = 0; i < planner_ids_.size(); ++i){
        declare_parameter(planner_ids_[i] + ".plugin",planner_types_[i]);
    }
  
   costmap_ros_ = std::make_shared<amr_costmap_2d::Costmap2DROS>(
    "global_costmap", std::string{get_namespace()}, "global_costmap");
}

PlannerServer::~PlannerServer()
{
  planners_.clear();
  costmap_thread_.reset();
}

amr_util::CallbackReturn
PlannerServer::on_configure(const rclcpp_lifecycle::State & )
{

 costmap_ros_->configure();
 costmap_ = costmap_ros_->getCostmap();

  if (!costmap_ros_->getUseRadius()) {
    collision_checker_ =
      std::make_unique<amr_costmap_2d::FootprintCollisionChecker<amr_costmap_2d::Costmap2D *>>(
      costmap_);
  }

  costmap_thread_ = std::make_unique<amr_util::NodeThread>(costmap_ros_);

  LOG(INFO) << "Costmap size: " << costmap_->getSizeInCellsX()
              << ", " << costmap_->getSizeInCellsY();

 tf_buffer_ = costmap_ros_->getTfBuffer();

  planner_types_.resize(planner_ids_.size());

  auto node = shared_from_this();

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    try {
      planner_types_[i] = amr_util::get_plugin_type_param(
        node, planner_ids_[i]);
      amr_core::GlobalPlanner::Ptr planner =
        gp_loader_.createUniqueInstance(planner_types_[i]);
      LOG(INFO) << "创建全局规划器：[" << planner_ids_[i]
                << "]  类型: [ " << planner_types_[i]<<"]";
      planner->configure(node, planner_ids_[i], tf_buffer_, costmap_ros_);
      planners_.insert({planner_ids_[i], planner});
    } catch (const pluginlib::PluginlibException & ex) {
      LOG(ERROR) << "创建全局规划器失败: " << ex.what();
      return amr_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    planner_ids_concat_ += planner_ids_[i] + std::string(" ");
  }

  LOG(INFO) << "当前正在使用全局规划器 ---> " << planner_ids_concat_;

  double expected_planner_frequency = 0.0;
  get_parameter("expected_planner_frequency", expected_planner_frequency);
  if (expected_planner_frequency > 0) {
    max_planner_duration_ = 1 / expected_planner_frequency;
  } else {
    LOG(WARNING) << "规划频率为 " << expected_planner_frequency
                  << " Hz";
    max_planner_duration_ = 0.0;
  }

  global_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
        amr_msgs::topic_names::GLOBAL_PATH,
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
//对用导航A
  action_server_generate_follow_path_ = std::make_unique<ActionServerGenerateFollowPath>(
    shared_from_this(),
    amr_msgs::action_names::GENERATE_FOLLOW_PATH,
    std::bind(&PlannerServer::generateFollowPath, this),
    nullptr,
    std::chrono::milliseconds(500),
    true);
//对应导航B
  action_server_plan_global_path_ = std::make_unique<ActionServerPlanGlobalPath>(
  shared_from_this(),
  amr_msgs::action_names::PLAN_GLOBAL_PATH,
  std::bind(&PlannerServer::planGlobalPath, this),
  nullptr,
  std::chrono::milliseconds(500),
  true);
  LOG(INFO)<<"planner_server ---> 已配置";
  return amr_util::CallbackReturn::SUCCESS;
}

amr_util::CallbackReturn
PlannerServer::on_activate(const rclcpp_lifecycle::State & )
{

 if(debug_enable_){
      global_path_publisher_->on_activate();
  }

  action_server_generate_follow_path_->activate();
  action_server_plan_global_path_->activate();
  costmap_ros_->activate();

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->activate();
  }

  auto node = shared_from_this();

  is_path_valid_service_ = node->create_service<amr_msgs::srv::IsPathValid>(
    amr_msgs::server_names::AMR_IS_PATH_VALID,
    std::bind(
      &PlannerServer::isPathValid, this,
      std::placeholders::_1, std::placeholders::_2));

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&PlannerServer::dynamicParametersCallback, this, _1));

  createBond();
  LOG(INFO)<<"planner_server ---> 已激活";
  return amr_util::CallbackReturn::SUCCESS;
}

amr_util::CallbackReturn
PlannerServer::on_deactivate(const rclcpp_lifecycle::State & )
{

  action_server_generate_follow_path_->deactivate();
  action_server_plan_global_path_->deactivate();
  global_path_publisher_->on_deactivate();
  costmap_ros_->deactivate();
  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->deactivate();
  }
  dyn_params_handler_.reset();
  destroyBond();
  LOG(INFO)<<"planner_server ---> 已停用";
  return amr_util::CallbackReturn::SUCCESS;
}

amr_util::CallbackReturn
PlannerServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{

  action_server_generate_follow_path_.reset();
  action_server_plan_global_path_.reset();
  global_path_publisher_.reset();
  tf_buffer_.reset();

  costmap_ros_->cleanup();

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->cleanup();
  }

  planners_.clear();
  costmap_thread_.reset();
  costmap_ = nullptr;
  LOG(INFO)<<"planner_server ---> 已清除";
  return amr_util::CallbackReturn::SUCCESS;
}

amr_util::CallbackReturn
PlannerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  LOG(INFO)<<"planner_server ---> 已关闭";;
  return amr_util::CallbackReturn::SUCCESS;
}

template<typename T>
bool PlannerServer::isServerInactive(
  std::unique_ptr<amr_util::SimpleActionServer<T>> & action_server)
{
  if (action_server == nullptr || !action_server->is_server_active()) {
    LOG(INFO) << "Action 服务器不可用";
    return true;
  }

  return false;
}

void PlannerServer::waitForCostmap()
{
  rclcpp::Rate r(100);
  while (!costmap_ros_->isCurrent()) {
    r.sleep();
  }
}

template<typename T>
bool PlannerServer::isCancelRequested(
  std::unique_ptr<amr_util::SimpleActionServer<T>> & action_server)
{
  if (action_server->is_cancel_requested()) {
    LOG(INFO) << "目标已取消，取消规划任务";
    action_server->terminate_all();
    return true;
  }

  return false;
}

template<typename T>
void PlannerServer::getPreemptedGoalIfRequested(
  std::unique_ptr<amr_util::SimpleActionServer<T>> & action_server,
  typename std::shared_ptr<const typename T::Goal> goal)
{
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

template<typename T>
bool PlannerServer::getStartPose(
  std::unique_ptr<amr_util::SimpleActionServer<T>> & action_server,
  typename std::shared_ptr<const typename T::Goal> goal,
  geometry_msgs::msg::PoseStamped & start)
{
  if (!costmap_ros_->getRobotPose(start)) {
    action_server->terminate_current();
    return false;
  }

  return true;
}

template<typename T>
bool PlannerServer::transformPosesToGlobalFrame(
  std::unique_ptr<amr_util::SimpleActionServer<T>> & action_server,
  geometry_msgs::msg::PoseStamped & curr_start,
  geometry_msgs::msg::PoseStamped & curr_goal)
{
  if (!costmap_ros_->transformPoseToGlobalFrame(curr_start, curr_start) ||
    !costmap_ros_->transformPoseToGlobalFrame(curr_goal, curr_goal))
  {
    LOG(WARNING) << "无法转换位姿";
    action_server->terminate_current();
    return false;
  }

  return true;
}

template<typename T>
bool PlannerServer::validatePath(
  std::unique_ptr<amr_util::SimpleActionServer<T>> & action_server,
  const geometry_msgs::msg::PoseStamped & goal,
  const nav_msgs::msg::Path & path,
  const std::string & planner_id)
{
  if (path.poses.size() == 0) {
    LOG(WARNING) << "规划算法未生成有效路径";
    action_server->terminate_current();
    return false;
  }

  LOG(INFO) << "生成路径，大小 " << path.poses.size()
              << " 到 (" << goal.pose.position.x
              << ", " << goal.pose.position.y << ")";

  return true;
}


template<typename T>
bool PlannerServer::validateGoal(
std::unique_ptr<amr_util::SimpleActionServer<T>> & action_server,
const std::vector<geometry_msgs::msg::PoseStamped> & way_poses)
{
  if(way_poses.size() != 2 && way_poses.size() != 4){
    LOG(ERROR)<<"路径点数必须为2或4,当前为 "<<way_poses.size();
    return false;
  }
  return true;
}

void PlannerServer::generateFollowPath(){

  LOG(INFO)<<"执行路径生成服务...";
  std::lock_guard<std::mutex> lock(mutex_);

  auto goal = action_server_generate_follow_path_->get_current_goal();
  auto result = std::make_shared<ActionGenerateFollowPath::Result>();

  std::vector<geometry_msgs::msg::PoseStamped> way_poses = goal->nav_a_goal;
  if(!validateGoal(action_server_generate_follow_path_,way_poses)){
    result->nav_status = ActionGenerateFollowPath::Result::GENERATE_ERROR;
    action_server_generate_follow_path_->terminate_current(result);
    return;
  }

  try{

    nav_msgs::msg::Path follow_path;  
    follow_path = getPlan(way_poses,goal->planner_id);
    result->global_path = follow_path;
    publishPlan(follow_path);
    action_server_generate_follow_path_->succeeded_current(result);
  }catch (std::exception & ex) {
      LOG(WARNING)<<"导航A全局路径生成异常!"<< ex.what();
      action_server_generate_follow_path_->terminate_current();
    }
}

nav_msgs::msg::Path
PlannerServer::getPlan(
  const std::vector<geometry_msgs::msg::PoseStamped> & way_poses,
  const std::string & planner_id)
{
  if (planners_.find(planner_id) != nullptr) {
    return planners_[planner_id]->createPlan(way_poses);}
  else {
      LOG(ERROR) << "当前无规划器可用!";
    }
  

  return nav_msgs::msg::Path();
}

void
PlannerServer::publishPlan(const nav_msgs::msg::Path & path)
{
  auto msg = std::make_unique<nav_msgs::msg::Path>(path);
  if (global_path_publisher_->is_activated() && global_path_publisher_->get_subscription_count() > 0) {
    global_path_publisher_->publish(std::move(msg));
  }
}

void PlannerServer::isPathValid(
  const std::shared_ptr<amr_msgs::srv::IsPathValid::Request> request,
  std::shared_ptr<amr_msgs::srv::IsPathValid::Response> response)
{
  response->is_valid = true;

  if (request->path.poses.empty()) {
    response->is_valid = false;
    return;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  unsigned int closest_point_index = 0;
  if (costmap_ros_->getRobotPose(current_pose)) {
    float current_distance = std::numeric_limits<float>::max();
    float closest_distance = current_distance;
    geometry_msgs::msg::Point current_point = current_pose.pose.position;
    for (unsigned int i = 0; i < request->path.poses.size(); ++i) {
      geometry_msgs::msg::Point path_point = request->path.poses[i].pose.position;

      current_distance = amr_util::geometry_utils::euclidean_distance(
        current_point,
        path_point);

      if (current_distance < closest_distance) {
        closest_point_index = i;
        closest_distance = current_distance;
      }
    }

    std::unique_lock<amr_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
    unsigned int mx = 0;
    unsigned int my = 0;

    bool use_radius = costmap_ros_->getUseRadius();

    unsigned int cost = amr_costmap_2d::FREE_SPACE;

    for (unsigned int i = closest_point_index; i < request->path.poses.size(); ++i) {
      auto & position = request->path.poses[i].pose.position;
      if (use_radius) {
        if (costmap_->worldToMap(position.x, position.y, mx, my)) {
          cost = costmap_->getCost(mx, my);
        } else {
          cost = amr_costmap_2d::LETHAL_OBSTACLE;
        }
      } else {
        amr_costmap_2d::Footprint footprint = costmap_ros_->getRobotFootprint();
        auto theta = tf2::getYaw(request->path.poses[i].pose.orientation);
        cost = static_cast<unsigned int>(collision_checker_->footprintCostAtPose(
            position.x, position.y, theta, footprint));
      }

      if (use_radius &&
        (cost == amr_costmap_2d::LETHAL_OBSTACLE ||
        cost == amr_costmap_2d::INSCRIBED_INFLATED_OBSTACLE))
      {
        response->is_valid = false;
        break;
      } else if (cost == amr_costmap_2d::LETHAL_OBSTACLE) {
        response->is_valid = false;
        break;
      }
    }
  }
}


void PlannerServer::planGlobalPath(){
  LOG(INFO)<<"执行全局规划服务...";
  std::lock_guard<std::mutex> lock(mutex_);
  auto start_time = this->now();
  auto goal = action_server_plan_global_path_->get_current_goal();
  auto result = std::make_shared<ActionPlanGlobalPath::Result>();

  try{
    if(isServerInactive(action_server_plan_global_path_) || isCancelRequested(action_server_plan_global_path_)){
      return;
    }
    waitForCostmap();
    getPreemptedGoalIfRequested(action_server_plan_global_path_,goal);

    geometry_msgs::msg::PoseStamped start;
    if(!getStartPose(action_server_plan_global_path_,goal,start)){
      return;
    }

    geometry_msgs::msg::PoseStamped goal_pose = goal->nav_b_goal;
    if(!transformPosesToGlobalFrame(action_server_plan_global_path_,start,goal_pose)){
      return;
    }
    std::vector<geometry_msgs::msg::PoseStamped> way_poses;
    way_poses.push_back(start);
    way_poses.push_back(goal_pose);
    result->global_path = getPlan(way_poses,goal->planner_id);

    if(!validatePath(action_server_plan_global_path_,goal_pose,result->global_path,goal->planner_id)){return;}
    publishPlan(result->global_path);
    action_server_plan_global_path_->succeeded_current(result);
  }catch (std::exception & ex) {
      LOG(WARNING)<<"导航B全局路径规划异常!"<< ex.what();
      action_server_generate_follow_path_->terminate_current();
  }
 
}

rcl_interfaces::msg::SetParametersResult
PlannerServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (name.find('.') != std::string::npos) {
      continue;
    }

    if (!mutex_.try_lock()) {
      LOG(WARNING)<<"无法动态设置参数";
      result.successful = false;
      result.reason = "无法动态设置参数";
      return result;
    }

    if (type == ParameterType::PARAMETER_BOOL) {
      if (name == "debug_enable") {
        debug_enable_ = parameter.as_bool();
      } 
    }
    mutex_.unlock();
  }

  result.successful = true;
  return result;
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(amr_planner::PlannerServer)