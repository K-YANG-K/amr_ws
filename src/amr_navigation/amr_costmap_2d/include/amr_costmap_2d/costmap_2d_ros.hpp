#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/polygon.h"
#include "geometry_msgs/msg/polygon_stamped.h"
#include "amr_costmap_2d/costmap_2d_publisher.hpp"
#include "amr_costmap_2d/footprint.hpp"
#include "amr_costmap_2d/clear_costmap_service.hpp"
#include "amr_costmap_2d/layered_costmap.hpp"
#include "amr_costmap_2d/layer.hpp"
#include "amr_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2/convert.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/time.h"
#include "tf2/transform_datatypes.h"
#include "glog/logging.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

namespace amr_costmap_2d
{


class Costmap2DROS : public amr_util::LifecycleNode
{
public:

  Costmap2DROS(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  explicit Costmap2DROS(const std::string & name);

  explicit Costmap2DROS(
    const std::string & name,
    const std::string & parent_namespace,
    const std::string & local_namespace);

  ~Costmap2DROS();

  amr_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;


  amr_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  amr_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  amr_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  amr_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  void on_rcl_preshutdown() override
  {
    if (is_lifecycle_follower_) {
      return;
    }

    RCLCPP_INFO(
      get_logger(), "Running amr LifecycleNode rcl preshutdown (%s)",
      this->get_name());

    runCleanups();

    destroyBond();
  }


  void start();

  void stop();

  void pause();

  void resume();

  void updateMap();

  void resetLayers();

  bool isCurrent()
  {
    return layered_costmap_->isCurrent();
  }

  bool getRobotPose(geometry_msgs::msg::PoseStamped & global_pose);

  bool transformPoseToGlobalFrame(
    const geometry_msgs::msg::PoseStamped & input_pose,
    geometry_msgs::msg::PoseStamped & transformed_pose);

  std::string getName() const
  {
    return name_;
  }

  double getTransformTolerance() const
  {
    return transform_tolerance_;
  }

  Costmap2D * getCostmap()
  {
    return layered_costmap_->getCostmap();
  }

  std::string getGlobalFrameID()
  {
    return global_frame_;
  }

  std::string getBaseFrameID()
  {
    return robot_base_frame_;
  }

  LayeredCostmap * getLayeredCostmap()
  {
    return layered_costmap_.get();
  }

  geometry_msgs::msg::Polygon getRobotFootprintPolygon()
  {
    return amr_costmap_2d::toPolygon(padded_footprint_);
  }

  std::vector<geometry_msgs::msg::Point> getRobotFootprint()
  {
    return padded_footprint_;
  }

  std::vector<geometry_msgs::msg::Point> getUnpaddedRobotFootprint()
  {
    return unpadded_footprint_;
  }

  void getOrientedFootprint(std::vector<geometry_msgs::msg::Point> & oriented_footprint);

  void setRobotFootprint(const std::vector<geometry_msgs::msg::Point> & points);

  void setRobotFootprintPolygon(const geometry_msgs::msg::Polygon::SharedPtr footprint);

  std::shared_ptr<tf2_ros::Buffer> getTfBuffer() {return tf_buffer_;}

  bool getUseRadius() {return use_radius_;}

  double getRobotRadius() {return robot_radius_;}

protected:
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
    footprint_pub_;
  std::unique_ptr<Costmap2DPublisher> costmap_publisher_{nullptr};

  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr footprint_sub_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_sub_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<amr_util::NodeThread> executor_thread_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<LayeredCostmap> layered_costmap_{nullptr};
  std::string name_;
  std::string parent_namespace_;

  void mapUpdateLoop(double frequency);
  bool map_update_thread_shutdown_{false};
  std::atomic<bool> stop_updates_{false};
  std::atomic<bool> initialized_{false};
  std::atomic<bool> stopped_{true};
  std::mutex _dynamic_parameter_mutex;
  std::unique_ptr<std::thread> map_update_thread_; 
  rclcpp::Time last_publish_{0, 0, RCL_ROS_TIME};
  rclcpp::Duration publish_cycle_{1, 0};
  pluginlib::ClassLoader<Layer> plugin_loader_{"amr_costmap_2d", "amr_costmap_2d::Layer"};

  void getParameters();
  bool always_send_full_costmap_{false};
  std::string footprint_;
  float footprint_padding_{0};
  std::string global_frame_;     
  int map_height_meters_{0};
  double map_publish_frequency_{0};
  double map_update_frequency_{0};
  int map_width_meters_{0};
  double origin_x_{0};
  double origin_y_{0};
  std::vector<std::string> default_plugins_;
  std::vector<std::string> default_types_;
  std::vector<std::string> plugin_names_;
  std::vector<std::string> plugin_types_;
  std::vector<std::string> filter_names_;
  std::vector<std::string> filter_types_;
  double resolution_{0};
  std::string robot_base_frame_;   
  double robot_radius_;
  bool rolling_window_{false};   
  bool track_unknown_space_{false};
  double transform_tolerance_{0}; 

  bool is_lifecycle_follower_{true};    

  // Derived parameters
  bool use_radius_{false};
  std::vector<geometry_msgs::msg::Point> unpadded_footprint_;
  std::vector<geometry_msgs::msg::Point> padded_footprint_;

  std::unique_ptr<ClearCostmapService> clear_costmap_service_;
  OnSetParametersCallbackHandle::SharedPtr dyn_params_handler;


  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

}