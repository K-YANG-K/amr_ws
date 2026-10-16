
#include <chrono>
#include <vector>
#include <memory>
#include <string>
#include <utility>
#include <limits>

#include "lifecycle_msgs/msg/state.hpp"
#include "amr_core/exceptions.hpp"
#include "amr_util/node_utils.hpp"
#include "amr_util/geometry_utils.hpp"
#include "amr_controller/controller_server.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace amr_controller
{

ControllerServer::ControllerServer(const rclcpp::NodeOptions & options)
: amr_util::LifecycleNode("controller_server", "", options),
  progress_checker_loader_("amr_core", "amr_core::ProgressChecker"),
  goal_checker_loader_("amr_core", "amr_core::GoalChecker"),
  lp_loader_("amr_core", "amr_core::Controller"),
  default_progress_checker_type_{"amr_controller::SimpleProgressChecker"},
  default_goal_checker_types_{"amr_controller::SimpleGoalChecker"},
  default_types_{"amr_mpc_controller::MPCController"}
{
  LOG(INFO) <<"初始化LifecycleNode--->[controller_server]";

  declare_parameter("controller_frequency", 20.0);

  declare_parameter("progress_checker_plugin", "progress_checker");
  declare_parameter("goal_checker_plugins", std::vector<std::string>{"general_goal_checker"});
  declare_parameter("controller_plugins", std::vector<std::string>{"MPCController"});
  declare_parameter("min_x_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_y_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_theta_velocity_threshold", rclcpp::ParameterValue(0.0001));

  declare_parameter("speed_limit_topic", rclcpp::ParameterValue("speed_limit"));

  declare_parameter("failure_tolerance", rclcpp::ParameterValue(0.0));
  declare_parameter("publish_zero_velocity", rclcpp::ParameterValue(true));

  costmap_ros_ = std::make_shared<amr_costmap_2d::Costmap2DROS>(
    "local_costmap", std::string{get_namespace()}, "local_costmap");
}

ControllerServer::~ControllerServer()
{
  progress_checker_.reset();
  goal_checkers_.clear();
  controllers_.clear();
  costmap_thread_.reset();
}

amr_util::CallbackReturn
ControllerServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  auto node = shared_from_this();

  LOG(INFO)<<"controller_server ---> 已配置";

  get_parameter("progress_checker_plugin", progress_checker_id_);
  if (!progress_checker_id_.empty()) {
    amr_util::declare_parameter_if_not_declared(
      node, progress_checker_id_ + ".plugin",
      rclcpp::ParameterValue(default_progress_checker_type_));
  }

  get_parameter("goal_checker_plugins", goal_checker_ids_);
  if (!goal_checker_ids_.empty()) {
    for (size_t i = 0; i < goal_checker_ids_.size(); ++i) {
      amr_util::declare_parameter_if_not_declared(
        node, goal_checker_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_goal_checker_types_[i]));
    }
  }

  get_parameter("controller_plugins", controller_ids_);
  controller_types_.resize(controller_ids_.size());
  if (!controller_ids_.empty()) {
    for (size_t i = 0; i < controller_ids_.size(); ++i) {
      amr_util::declare_parameter_if_not_declared(
        node, controller_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_types_[i]));
    }
  }

  goal_checker_types_.resize(goal_checker_ids_.size());

  get_parameter("controller_frequency", controller_frequency_);
  get_parameter("min_x_velocity_threshold", min_x_velocity_threshold_);
  get_parameter("min_y_velocity_threshold", min_y_velocity_threshold_);
  get_parameter("min_theta_velocity_threshold", min_theta_velocity_threshold_);

  std::string speed_limit_topic;
  get_parameter("speed_limit_topic", speed_limit_topic);
  get_parameter("failure_tolerance", failure_tolerance_);
  get_parameter("publish_zero_velocity", publish_zero_velocity_);

  costmap_ros_->configure();
  costmap_thread_ = std::make_unique<amr_util::NodeThread>(costmap_ros_);

  try {
    progress_checker_type_ = amr_util::get_plugin_type_param(node, progress_checker_id_);
    progress_checker_ = progress_checker_loader_.createUniqueInstance(progress_checker_type_);
    LOG(INFO)<<"progress_checker: "<<  progress_checker_id_.c_str();

    progress_checker_->initialize(node, progress_checker_id_);
  } catch (const pluginlib::PluginlibException & ex) {
      LOG(ERROR)<<"创建progress_checker失败: "<< ex.what();
    return amr_util::CallbackReturn::FAILURE;
  }

  for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
    try {
      goal_checker_types_[i] = amr_util::get_plugin_type_param(node, goal_checker_ids_[i]);
      amr_core::GoalChecker::Ptr goal_checker =
        goal_checker_loader_.createUniqueInstance(goal_checker_types_[i]);
      LOG(INFO)<<"goal checker: "<< goal_checker_ids_[i].c_str();
      goal_checker->initialize(node, goal_checker_ids_[i], costmap_ros_);
      goal_checkers_.insert({goal_checker_ids_[i], goal_checker});
    } catch (const pluginlib::PluginlibException & ex) {
      LOG(ERROR)<<"创建goal checker失败: "<< ex.what();
      return amr_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
    goal_checker_ids_concat_ += goal_checker_ids_[i] + std::string(" ");
  }


  for (size_t i = 0; i != controller_ids_.size(); i++) {
    try {
      controller_types_[i] = amr_util::get_plugin_type_param(node, controller_ids_[i]);
      amr_core::Controller::Ptr controller =
        lp_loader_.createUniqueInstance(controller_types_[i]);
      LOG(INFO)<<"创建控制器：["<< controller_ids_[i].c_str()<<"]  类型：[" << controller_types_[i]<<"]";
      controller->configure(
        node, controller_ids_[i],
        costmap_ros_->getTfBuffer(), costmap_ros_);
      controllers_.insert({controller_ids_[i], controller});
    } catch (const pluginlib::PluginlibException & ex) {
       LOG(ERROR)<<"创建控制器失败: "<< ex.what();  
      return amr_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != controller_ids_.size(); i++) {
    controller_ids_concat_ += controller_ids_[i] + std::string(" ");
  }
  LOG(INFO) << "当前正在使用控制器 ---> " << controller_ids_concat_;

 // odom_sub_ = std::make_unique<nav_2d_utils::OdomSubscriber>(node);
  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  action_server_ = std::make_unique<ActionServer>(
    shared_from_this(),
    amr_msgs::action_names::LOCAL_CONTROL,
    std::bind(&ControllerServer::computeControl, this),
    nullptr,
    std::chrono::milliseconds(500),
    true);

  speed_limit_sub_ = create_subscription<amr_msgs::msg::SpeedLimit>(
    speed_limit_topic, rclcpp::QoS(10),
    std::bind(&ControllerServer::speedLimitCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(amr_msgs::topic_names::ODOM,rclcpp::SensorDataQoS(),std::bind(&ControllerServer::odomCallback,this,std::placeholders::_1));
  return amr_util::CallbackReturn::SUCCESS;
}

amr_util::CallbackReturn
ControllerServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{

  costmap_ros_->activate();
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->activate();
  }
  vel_publisher_->on_activate();
  action_server_->activate();

  auto node = shared_from_this();

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&ControllerServer::dynamicParametersCallback, this, _1));

  createBond();
  LOG(INFO)<<"controller_server ---> 已激活";
  return amr_util::CallbackReturn::SUCCESS;
}

amr_util::CallbackReturn
ControllerServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{

  action_server_->deactivate();
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->deactivate();
  }

  costmap_ros_->deactivate();

  publishZeroVelocity();
  vel_publisher_->on_deactivate();
  dyn_params_handler_.reset();
  destroyBond();
  LOG(INFO)<<"controller_server ---> 已停用";
  return amr_util::CallbackReturn::SUCCESS;
}

amr_util::CallbackReturn
ControllerServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{

  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->cleanup();
  }
  controllers_.clear();

  goal_checkers_.clear();

  costmap_ros_->cleanup();

  action_server_.reset();
  // odom_sub_.reset();
  costmap_thread_.reset();
  vel_publisher_.reset();
  speed_limit_sub_.reset();
  LOG(INFO)<<"controller_server ---> 已清除";
  return amr_util::CallbackReturn::SUCCESS;
}

amr_util::CallbackReturn
ControllerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  LOG(INFO)<<"controller_server ---> 已关闭";
  return amr_util::CallbackReturn::SUCCESS;
}

bool ControllerServer::findControllerId(
  const std::string & c_name,
  std::string & current_controller)
{
  if (controllers_.find(c_name) == controllers_.end()) {
    if (controllers_.size() == 1 && c_name.empty()) {
      LOG(WARNING)<<"只有当前控制器可用: "<<controller_ids_concat_;
      current_controller = controllers_.begin()->first;
    } else {
      LOG(ERROR)<<"控制器："<<c_name<<"不存在";
      return false;
    }
  } else {
    current_controller = c_name;
    LOG(INFO)<<"当前控制器：["<<current_controller<<"]";
  }

  return true;
}

bool ControllerServer::findGoalCheckerId(
  const std::string & c_name,
  std::string & current_goal_checker)
{
  if (goal_checkers_.find(c_name) == goal_checkers_.end()) {
    if (goal_checkers_.size() == 1 && c_name.empty()) {
     // LOG(WARNING)<<"只有当前目标检测器可用："<< goal_checker_ids_concat_;
      current_goal_checker = goal_checkers_.begin()->first;
    } else {
       LOG(ERROR)<<"检测器："<<c_name<<"不存在";
      return false;
    }
  } else {
    current_goal_checker = c_name;
  }

  return true;
}

void ControllerServer::computeControl()
{
  LOG(INFO)<<"执行路径跟踪控制服务...";

  try {
    std::string c_name = action_server_->get_current_goal()->controller_id;
    std::string current_controller;
    if (findControllerId(c_name, current_controller)) {
      std::lock_guard<std::mutex> lock(mutex_);
      current_controller_ = current_controller;
    } else {
      auto result = std::make_shared<Action::Result>();
      result->nav_status = Action::Result::NAV_FAILED;
      action_server_->terminate_current(result);
      return;
    }

    std::string gc_name = action_server_->get_current_goal()->goal_checker_id;
    std::string current_goal_checker;
    if (findGoalCheckerId(gc_name, current_goal_checker)) {
      std::lock_guard<std::mutex> lock(mutex_);
      current_goal_checker_ = current_goal_checker;
    } else {
      auto result = std::make_shared<Action::Result>();
      result->nav_status = Action::Result::NAV_FAILED;
      action_server_->terminate_current(result);
      return;
    }

    setPlannerPath(action_server_->get_current_goal()->global_path);
    LOG(INFO)<<"目标速度：" << action_server_->get_current_goal()->nav_speed.data;
    progress_checker_->reset();

    last_valid_cmd_time_ = now();
    double controller_frequency;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      controller_frequency = controller_frequency_;
    }
    rclcpp::WallRate loop_rate(controller_frequency);
    while (rclcpp::ok()) {
      if (action_server_ == nullptr || !action_server_->is_server_active()) {
        LOG(ERROR) <<"Action server unavailable or inactive. Stopping.";
        return;
      }

      if (action_server_->is_cancel_requested()) {
        LOG(INFO)<<"目标取消,停止机器人";
        auto result = std::make_shared<Action::Result>();
        result->nav_status = Action::Result::NAV_FAILED;
        action_server_->terminate_all(result);
        publishZeroVelocity();
        return;
      }

      rclcpp::Rate r(100);
      while (!costmap_ros_->isCurrent()) {
        r.sleep();
      }

      updateGlobalPath();

     computeAndPublishVelocity();

      if (isGoalReached()) {
        LOG(INFO)<<"目标点已到达";
        auto result = std::make_shared<Action::Result>();
        result->nav_status = Action::Result::NAV_SUCCESS;
        action_server_->succeeded_current(result);
        if (publish_zero_velocity_) {
          publishZeroVelocity();
        }
        return;
      }

      if (!loop_rate.sleep()) {
        LOG(INFO)<<"控制频率未到达设置频率"<<controller_frequency_<<"HZ";
      }
    }
  } catch (amr_core::PlannerException & e) {
    LOG(ERROR)<<e.what();
    publishZeroVelocity();
    auto result = std::make_shared<Action::Result>();
    result->nav_status = Action::Result::NAV_FAILED;
    action_server_->terminate_current(result);
    return;
  } catch (std::exception & e) {
    LOG(ERROR)<<e.what();
    publishZeroVelocity();
    auto result = std::make_shared<Action::Result>();
    result->nav_status = Action::Result::NAV_FAILED;
    action_server_->terminate_current(result);
    return;
  }


  if (publish_zero_velocity_) {
    publishZeroVelocity();
  }

}

void ControllerServer::setPlannerPath(const nav_msgs::msg::Path & path)
{
  LOG(INFO)<<"控制服务收到全局路径大小："<<path.poses.size() << "  起点：("<< path.poses.front().pose.position.x<< ", " << path.poses.front().pose.position.y << "), 终点：(" << path.poses.back().pose.position.x << ", " << path.poses.back().pose.position.y <<")";
  
  if (path.poses.empty()) {
    throw amr_core::PlannerException("Invalid path, Path is empty.");
  }
  controllers_[current_controller_]->setPlan(path);
  goal_checkers_[current_goal_checker_]->reset();
  end_pose_ = path.poses.back();
  end_pose_.header.frame_id = path.header.frame_id;
  current_path_ = path;
}

void ControllerServer::computeAndPublishVelocity()
{

  geometry_msgs::msg::PoseStamped pose;

  if (!getRobotPose(pose)) {
    throw amr_core::PlannerException("Failed to obtain robot pose");
  }
  if (!progress_checker_->check(pose)) {
    throw amr_core::PlannerException("Failed to make progress");
  }
  geometry_msgs::msg::TwistStamped cmd_vel;

  try {
    cmd_vel =
      controllers_[current_controller_]->computeVelocityCommands(
      pose,
      odom_vel_,
     goal_checkers_[current_goal_checker_].get());
    last_valid_cmd_time_ = now();
  } catch (amr_core::PlannerException & e) {
    if (failure_tolerance_ > 0 || failure_tolerance_ == -1.0) {
      LOG(WARNING)<<"速度计算异常："<<e.what();
      cmd_vel.twist.angular.x = 0;
      cmd_vel.twist.angular.y = 0;
      cmd_vel.twist.angular.z = 0;
      cmd_vel.twist.linear.x = 0;
      cmd_vel.twist.linear.y = 0;
      cmd_vel.twist.linear.z = 0;
      cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
      cmd_vel.header.stamp = now();
      if ((now() - last_valid_cmd_time_).seconds() > failure_tolerance_ &&
        failure_tolerance_ != -1.0)
      {
        throw amr_core::PlannerException("Controller patience exceeded");
      }
    } else {
      throw amr_core::PlannerException(e.what());
    }
  }

  // std::shared_ptr<Action::Feedback> feedback = std::make_shared<Action::Feedback>();
  // feedback->speed = std::hypot(cmd_vel_2d.twist.linear.x, cmd_vel_2d.twist.linear.y);

  // Find the closest pose to current pose on global path
  nav_msgs::msg::Path & current_path = current_path_;
  auto find_closest_pose_idx =
    [&pose, &current_path]() {
      size_t closest_pose_idx = 0;
      double curr_min_dist = std::numeric_limits<double>::max();
      for (size_t curr_idx = 0; curr_idx < current_path.poses.size(); ++curr_idx) {
        double curr_dist = amr_util::geometry_utils::euclidean_distance(
          pose, current_path.poses[curr_idx]);
        if (curr_dist < curr_min_dist) {
          curr_min_dist = curr_dist;
          closest_pose_idx = curr_idx;
        }
      }
      return closest_pose_idx;
    };

  // feedback->distance_to_goal =
  //   amr_util::geometry_utils::calculate_path_length(current_path_, find_closest_pose_idx());
  // action_server_->publish_feedback(feedback);

  publishVelocity(cmd_vel);
}

void ControllerServer::updateGlobalPath()
{
  if (action_server_->is_preempt_requested()) {
    LOG(INFO)<<"收到新的路径";
    auto goal = action_server_->accept_pending_goal();
    std::string current_controller;
    if (findControllerId(goal->controller_id, current_controller)) {
      current_controller_ = current_controller;
    } else {
      LOG(ERROR)<<"无控制器可用";
      action_server_->terminate_current();
      return;
    }
    std::string current_goal_checker;
    if (findGoalCheckerId(goal->goal_checker_id, current_goal_checker)) {
      current_goal_checker_ = current_goal_checker;
    } else {
      LOG(INFO)<<"无检测器可用";
      action_server_->terminate_current();
      return;
    }
    setPlannerPath(goal->global_path);
  }
}

void ControllerServer::publishVelocity(const geometry_msgs::msg::TwistStamped & velocity)
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>(velocity.twist);
  if (vel_publisher_->is_activated() && vel_publisher_->get_subscription_count() > 0) {
    vel_publisher_->publish(std::move(cmd_vel));
  }
}

void ControllerServer::publishZeroVelocity()
{
  geometry_msgs::msg::TwistStamped velocity;
  velocity.twist.angular.x = 0;
  velocity.twist.angular.y = 0;
  velocity.twist.angular.z = 0;
  velocity.twist.linear.x = 0;
  velocity.twist.linear.y = 0;
  velocity.twist.linear.z = 0;
  velocity.header.frame_id = costmap_ros_->getBaseFrameID();
  velocity.header.stamp = now();
  publishVelocity(velocity);
}

bool ControllerServer::isGoalReached()
{
  geometry_msgs::msg::PoseStamped pose;

  if (!getRobotPose(pose)) {
    return false;
  }
  return true;//TODO: 临时为了编译加的，后期要删掉
  return goal_checkers_[current_goal_checker_]->isGoalReached(
    pose.pose, end_pose_.pose,
    odom_vel_);

}

bool ControllerServer::getRobotPose(geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!costmap_ros_->getRobotPose(current_pose)) {
    return false;
  }
  pose = current_pose;
  return true;
}

void ControllerServer::speedLimitCallback(const amr_msgs::msg::SpeedLimit::SharedPtr msg)
{
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->setSpeedLimit(msg->speed_limit, msg->percentage);
  }
}

void ControllerServer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  odom_vel_.linear.x = msg->twist.twist.linear.x;
  odom_vel_.linear.y = msg->twist.twist.linear.y;
  odom_vel_.angular.z = msg->twist.twist.angular.z;
}

rcl_interfaces::msg::SetParametersResult
ControllerServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  // Separate dotted (controller-specific) params from flat params
  std::vector<rclcpp::Parameter> controller_params;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    // Forward dotted parameters to controllers
    if (name.find('.') != std::string::npos) {
      controller_params.push_back(parameter);
      continue;
    }

    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
      RCLCPP_WARN(
        get_logger(),
        "Unable to dynamically change Parameters while the controller is currently running");
      result.successful = false;
      result.reason =
        "Unable to dynamically change Parameters while the controller is currently running";
      return result;
    }

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "controller_frequency") {
        controller_frequency_ = parameter.as_double();
      } else if (name == "min_x_velocity_threshold") {
        min_x_velocity_threshold_ = parameter.as_double();
      } else if (name == "min_y_velocity_threshold") {
        min_y_velocity_threshold_ = parameter.as_double();
      } else if (name == "min_theta_velocity_threshold") {
        min_theta_velocity_threshold_ = parameter.as_double();
      } else if (name == "failure_tolerance") {
        failure_tolerance_ = parameter.as_double();
      }
    }
  }

  // Forward dotted params to all controllers (each one filters its own)
  if (!controller_params.empty()) {
    for (auto & ctrl_pair : controllers_) {
      ctrl_pair.second->dynamicParametersCallback(controller_params);
    }
  }

  result.successful = true;
  return result;
}

} 
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(amr_controller::ControllerServer)
