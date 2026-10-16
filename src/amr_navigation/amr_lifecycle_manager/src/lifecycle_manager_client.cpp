#include "amr_lifecycle_manager/lifecycle_manager_client.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace amr_lifecycle_manager
{

LifecycleManagerClient::LifecycleManagerClient(
  const std::string & name,
  std::shared_ptr<rclcpp::Node> parent_node)
{
  manage_service_name_ = name + std::string("/manage_nodes");
  active_service_name_ = name + std::string("/is_active");

  node_ = parent_node;

  manager_client_ = std::make_shared<amr_util::ServiceClient<ManageAmrNodes>>(
    manage_service_name_, node_);
  is_active_client_ = std::make_shared<amr_util::ServiceClient<std_srvs::srv::Trigger>>(
    active_service_name_, node_);
}

bool
LifecycleManagerClient::startup(const std::chrono::nanoseconds timeout,
  const std::vector<std::string> & node_names)
{
  return callService(ManageAmrNodes::Request::STARTUP, timeout, node_names);
}

bool
LifecycleManagerClient::shutdown(const std::chrono::nanoseconds timeout,
  const std::vector<std::string> & node_names)
{
  return callService(ManageAmrNodes::Request::SHUTDOWN, timeout, node_names);
}

bool
LifecycleManagerClient::pause(const std::chrono::nanoseconds timeout,
  const std::vector<std::string> & node_names)
{
  return callService(ManageAmrNodes::Request::PAUSE, timeout, node_names);
}

bool
LifecycleManagerClient::resume(const std::chrono::nanoseconds timeout,
  const std::vector<std::string> & node_names)
{
  return callService(ManageAmrNodes::Request::RESUME, timeout, node_names);
}

bool
LifecycleManagerClient::reset(const std::chrono::nanoseconds timeout,
  const std::vector<std::string> & node_names)
{
  return callService(ManageAmrNodes::Request::RESET, timeout, node_names);
}

SystemStatus
LifecycleManagerClient::is_active(const std::chrono::nanoseconds timeout)
{
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto response = std::make_shared<std_srvs::srv::Trigger::Response>();


  if (!is_active_client_->wait_for_service(std::chrono::seconds(1))) {
    return SystemStatus::TIMEOUT;
  }

  LOG(INFO) << "发送 " << active_service_name_ << " 请求";

  try {
    response = is_active_client_->invoke(request, timeout);
  } catch (std::runtime_error &) {
    return SystemStatus::TIMEOUT;
  }

  if (response->success) {
    return SystemStatus::ACTIVE;
  } else {
    return SystemStatus::INACTIVE;
  }
}

bool
LifecycleManagerClient::callService(uint8_t command, const std::chrono::nanoseconds timeout,
  const std::vector<std::string> & node_names)
{
  auto request = std::make_shared<ManageAmrNodes::Request>();
  request->command = command;
  request->node_names = node_names;

  LOG(INFO) << "等待 " << manage_service_name_ << " 服务...";

  while (!manager_client_->wait_for_service(timeout)) {
    if (!rclcpp::ok()) {
      LOG(ERROR) << "等待服务时客户端被中断";
      return false;
    }
    LOG(INFO) << "等待服务就绪...";
  }

  LOG(INFO) << "发送 " << manage_service_name_ << " 请求";
  try {
    auto future_result = manager_client_->invoke(request, timeout);
    return future_result->success;
  } catch (std::runtime_error &) {
    return false;
  }
}

}  
