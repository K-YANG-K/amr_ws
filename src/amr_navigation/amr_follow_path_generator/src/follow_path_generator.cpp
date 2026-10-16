#include "lifecycle_msgs/msg/state.hpp"
#include "amr_util/node_utils.hpp"
#include "amr_follow_path_generator/follow_path_generator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace amr_follow_path_generator{

FollowPathGenerator::FollowPathGenerator(){};  

FollowPathGenerator::~FollowPathGenerator(){};

void
FollowPathGenerator::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<amr_costmap_2d::Costmap2DROS> costmap_ros)
{
  tf_ = tf;
  name_ = name;
  node_ = parent;
  auto node = parent.lock();
  amr_util::declare_parameter_if_not_declared(node, "resolution", rclcpp::ParameterValue(0.05));
  amr_util::declare_parameter_if_not_declared(node, "min_sample_num", rclcpp::ParameterValue(10));
  node->get_parameter("resolution",resolution_);
  node->get_parameter("min_sample_num",min_sample_num_);

}

void
FollowPathGenerator::activate(){
     
  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&FollowPathGenerator::dynamicParametersCallback, this, _1));
}

void
FollowPathGenerator::deactivate()
{

  dyn_params_handler_.reset();
}

void
FollowPathGenerator::cleanup()
{

  dyn_params_handler_.reset();
}

nav_msgs::msg::Path FollowPathGenerator::createPlan(const std::vector<geometry_msgs::msg::PoseStamped>& way_poses){
    /*
        z = 0      正向行驶  2点一组 航向不变
        z = 5000   反向行驶（倒车）2点一组  航向旋转 180°
        z = 10000  贝塞尔曲线正向  4点一组(起点->贝1->贝2->终点)，航向不变
        z = 15000  贝塞尔曲线反向  4点一组，航向旋转 180°
    */

    std::vector<geometry_msgs::msg::PoseStamped> goal_way_poses = way_poses;
    size_t size = goal_way_poses.size();


    double z = goal_way_poses.begin()->pose.position.z;
    bool reverse = (static_cast<int>(z) == 5000 || static_cast<int>(z) == 15000);

    double path_length;
    if (size == 2) {
      path_length = amr_util::geometry_utils::euclidean_distance(goal_way_poses.back(), goal_way_poses.front());
    } else {
      path_length = 0.0;
      for (size_t i = 1; i < size; ++i) {
        path_length += amr_util::geometry_utils::euclidean_distance(goal_way_poses[i],goal_way_poses[i - 1]);
             }
    }
    int sample_num = std::max(min_sample_num_, static_cast<int>(path_length / resolution_));

    nav_msgs::msg::Path follow_path;
    if (size == 2) {
      follow_path = computeLinePath(goal_way_poses.front(), goal_way_poses.back(),
                                    sample_num, reverse, goal_way_poses.front().header);
      LOG(INFO)<<"路径类型：直线， 生成路径大小: "<<follow_path.poses.size()<<", 方向: "<<(reverse?"反向":"正向");
    } else {
      follow_path = computeBezierPath(
          goal_way_poses,
          sample_num, reverse, goal_way_poses.begin()->header);
            LOG(INFO)<<"路径类型：曲线， 生成路径大小: "<<follow_path.poses.size()<<", 方向: "<<(reverse?"反向":"正向");
    }

    return follow_path;
    
}


nav_msgs::msg::Path FollowPathGenerator::computeLinePath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & end,
    int sample_num, bool reverse,
    const std_msgs::msg::Header & header)
{
  nav_msgs::msg::Path path;
  path.header = header;

  double x0 = start.pose.position.x, y0 = start.pose.position.y;
  double x1 = end.pose.position.x, y1 = end.pose.position.y;
  double yaw = std::atan2(y1 - y0, x1 - x0);
  if (reverse) yaw += M_PI;

  for (int i = 0; i < sample_num; ++i) {
    double t = static_cast<double>(i) / (sample_num - 1);
    geometry_msgs::msg::PoseStamped pt;
    pt.header = header;
    pt.pose.position.x = x0 + (x1 - x0) * t;
    pt.pose.position.y = y0 + (y1 - y0) * t;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pt.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(pt);
  }
  return path;
}

nav_msgs::msg::Path FollowPathGenerator::computeBezierPath(
    const std::vector<geometry_msgs::msg::PoseStamped> & control_poses,
    int sample_num, bool reverse,
    const std_msgs::msg::Header & header)
{
  nav_msgs::msg::Path path;
  path.header = header;

  double x0 = control_poses[0].pose.position.x, y0 = control_poses[0].pose.position.y;
  double x1 = control_poses[1].pose.position.x, y1 = control_poses[1].pose.position.y;
  double x2 = control_poses[2].pose.position.x, y2 = control_poses[2].pose.position.y;
  double x3 = control_poses[3].pose.position.x, y3 = control_poses[3].pose.position.y;

  for (int i = 0; i < sample_num; ++i) {
    double t = static_cast<double>(i) / (sample_num - 1);
    double t2 = t * t, t3 = t2 * t;
    double mt = 1 - t, mt2 = mt * mt, mt3 = mt2 * mt;

    geometry_msgs::msg::PoseStamped pt;
    pt.header = header;
    pt.pose.position.x = mt3 * x0 + 3 * mt2 * t * x1 + 3 * mt * t2 * x2 + t3 * x3;
    pt.pose.position.y = mt3 * y0 + 3 * mt2 * t * y1 + 3 * mt * t2 * y2 + t3 * y3;

    double dx = 3 * mt2 * (x1 - x0) + 6 * mt * t * (x2 - x1) + 3 * t2 * (x3 - x2);//求一阶导数算方向
    double dy = 3 * mt2 * (y1 - y0) + 6 * mt * t * (y2 - y1) + 3 * t2 * (y3 - y2);
    double yaw = std::atan2(dy, dx);
    if (reverse) yaw += M_PI;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pt.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(pt);
  }
  return path;
}

rcl_interfaces::msg::SetParametersResult
FollowPathGenerator::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (name.find('.') != std::string::npos) {
      continue;
    }

  if (type == ParameterType::PARAMETER_DOUBLE){
        if (name == "resolution") {
        resolution_ = parameter.as_double();
        }
    }
    else if (type == ParameterType::PARAMETER_INTEGER){
        if (name == "min_sample_num_") {
        min_sample_num_ = parameter.as_int();
      } 
    }
    
  }

  result.successful = true;
  return result;
}

} 

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(amr_follow_path_generator::FollowPathGenerator, amr_core::GlobalPlanner)
