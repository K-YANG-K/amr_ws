#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "amr_util/lifecycle_node.hpp"
#include "amr_msgs/msg/costmap_filter_info.hpp"

namespace amr_map_server
{

class CostmapFilterInfoServer : public amr_util::LifecycleNode
{
public:
  /**
   * @brief Constructor for the amr_map_server::CostmapFilterInfoServer
   * @param options Additional options to control creation of the node.
   */
  explicit CostmapFilterInfoServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor for the amr_map_server::CostmapFilterInfoServer
   */
  ~CostmapFilterInfoServer();

protected:
  /**
   * @brief Creates CostmapFilterInfo publisher and forms published message from ROS parameters
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  amr_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Publishes a CostmapFilterInfo message
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  amr_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates publisher
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  amr_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Resets publisher
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  amr_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in Shutdown state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  amr_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  rclcpp_lifecycle::LifecyclePublisher<amr_msgs::msg::CostmapFilterInfo>::SharedPtr publisher_;

  amr_msgs::msg::CostmapFilterInfo msg_;
};  // CostmapFilterInfoServer

}  
