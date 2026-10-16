#include "amr_mpc_controller/mpc_controller.hpp"

namespace amr_mpc_controller{

void MPCController::configure(const amr_util::LifecycleNode::WeakPtr & parent,
    std::string name,const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<amr_costmap_2d::Costmap2DROS> costmap_ros){
        parent_ = parent;
        costmap_ros_ = costmap_ros;
        tf_buffer_ = tf;
        name_ = name;
        auto node = parent_.lock();
        optimizer_ = std::make_shared<mpc::Optimizer>();
        optimizer_->initialize(parent_,name_,costmap_ros_);
        amr_util::declare_parameter_if_not_declared(node, name_ + ".lookahead_dist_min", rclcpp::ParameterValue(0.1));
        amr_util::declare_parameter_if_not_declared(node, name_ + ".lookahead_dist_max", rclcpp::ParameterValue(0.5));
        amr_util::declare_parameter_if_not_declared(node, name_ + ".lookahead_dist_close_to_goal", rclcpp::ParameterValue(0.0));
        
        node->get_parameter(name_ + ".lookahead_dist_min", lookahead_dist_min_);
        node->get_parameter(name_ + ".lookahead_dist_max", lookahead_dist_max_);
        node->get_parameter(name_ + ".lookahead_dist_close_to_goal", lookahead_dist_close_to_goal_);

        local_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    }

void MPCController::activate(){
  local_path_pub_->on_activate();
}

void MPCController::deactivate(){
  local_path_pub_->on_deactivate();
}

void MPCController::cleanup(){
  local_path_pub_.reset();
}

void MPCController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  goal_pose_ = path.poses[path.poses.size() - 1].pose;
}

geometry_msgs::msg::TwistStamped MPCController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & amr_pose,
  const geometry_msgs::msg::Twist & speed,
  amr_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock(mutex_);
  nav_msgs::msg::Path transformed_plan = transformGlobalPlan(amr_pose);
  double lookahead_dist = getLookAheadDistance();

  auto carrot_pose = getLookAheadPoint(lookahead_dist,transformed_plan);

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel = optimizer_->getControl(goal_pose_,carrot_pose,amr_pose,speed);
  
  return cmd_vel;
}
double MPCController::getLookAheadDistance()
{
  double lookahead_dist = lookahead_dist_min_;
  if (!slow_down_ || closer_to_goal_)
  {
    lookahead_dist = lookahead_dist_max_;
    if (closer_to_goal_) {
      lookahead_dist = lookahead_dist_close_to_goal_;
    }
  }
  return lookahead_dist;
}

nav_msgs::msg::Path MPCController::transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose)
{
  if(global_plan_.poses.empty()){
    throw amr_core::PlannerException("Invalid path, Path is empty.");
  }
  geometry_msgs::msg::PoseStamped amr_pose;
  if(!transformPose(global_plan_.header.frame_id, pose, amr_pose))
  {
    throw amr_core::PlannerException("Transform pose failed.");
  }
  amr_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  const double max_costmap_dist = std::max(costmap->getSizeInMetersX(), costmap->getSizeInMetersY()) * 0.50;
  LOG(INFO)<<"max_costmap_dist: "<<max_costmap_dist;
  auto transformation_begin = amr_util::geometry_utils::min_by(global_plan_.poses.begin(), global_plan_.poses.end(), [&amr_pose](const geometry_msgs::msg::PoseStamped & pose)
  {
    return amr_util::geometry_utils::euclidean_distance(pose.pose, amr_pose.pose);
  });
  geometry_msgs::msg::PoseStamped final_pose;
  final_pose.header = global_plan_.header;
  final_pose.pose = global_plan_.poses[global_plan_.poses.size() - 1].pose;
  if (amr_util::geometry_utils::euclidean_distance(amr_pose, final_pose) <= lookahead_dist_close_to_goal_) {
    closer_to_goal_ = true;
  }
  else {
    closer_to_goal_ = false;
  }

  auto transformation_end = std::find_if(
    transformation_begin, end(global_plan_.poses),
    [&](const auto & global_plan_pose) {
      return amr_util::geometry_utils::euclidean_distance(amr_pose, global_plan_pose) > max_costmap_dist;
    });

    auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = amr_pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
      return transformed_pose;
    };


    nav_msgs::msg::Path transformed_plan;
    std::transform(
      transformation_begin, transformation_end,
      std::back_inserter(transformed_plan.poses),
      transformGlobalPoseToLocal);
    transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
    transformed_plan.header.stamp = amr_pose.header.stamp;

    global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
    local_path_pub_->publish(transformed_plan);

  return transformed_plan;
}

bool MPCController::transformPose(const std::string frame, const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const
{
  if(in_pose.header.frame_id == frame)
  {
    out_pose = in_pose;
    return true;
  }
  try{
    tf_buffer_->transform(in_pose, out_pose, frame, tf2::durationFromSec(0.1));
    out_pose.header.frame_id = frame;
    return true;
  }
  catch(const tf2::TransformException & e)
  {
    LOG(ERROR)<<"Transform pose failed: "<<e.what();
    return false;
  }
}

geometry_msgs::msg::PoseStamped MPCController::getLookAheadPoint(
  const double & lookahead_dist,
  const nav_msgs::msg::Path & transformed_plan)
{
  
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  }

  return *goal_pose_it;
}


void MPCController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
}

int MPCController::shareCostMap()
{
  return 0;
}

rcl_interfaces::msg::SetParametersResult
MPCController::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (auto & param : parameters) {
    const auto & name = param.get_name();

    // Only handle parameters that belong to this controller
    if (name.find(name_ + ".") != 0) {
      continue;
    }

    if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      optimizer_->updateParameter(name, param.as_double());
    } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      optimizer_->updateParameter(name, static_cast<double>(param.as_int()));
    }
  }

  return result;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(amr_mpc_controller::MPCController, amr_core::Controller)