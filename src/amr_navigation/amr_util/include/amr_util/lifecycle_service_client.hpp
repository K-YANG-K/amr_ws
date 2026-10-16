#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "amr_util/service_client.hpp"
#include "amr_util/node_utils.hpp"
#include "glog/logging.h"
namespace amr_util
{


class LifecycleServiceClient
{
public:
  explicit LifecycleServiceClient(const std::string & lifecycle_node_name);
  LifecycleServiceClient(
    const std::string & lifecycle_node_name,
    rclcpp::Node::SharedPtr parent_node);


  bool change_state(
    const uint8_t transition,  
    const std::chrono::seconds timeout);

  bool change_state(std::uint8_t transition);

  uint8_t get_state(const std::chrono::seconds timeout = std::chrono::seconds(2));

protected:
  rclcpp::Node::SharedPtr node_;
  ServiceClient<lifecycle_msgs::srv::ChangeState> change_state_;
  ServiceClient<lifecycle_msgs::srv::GetState> get_state_;
};

}  

