#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"
#include "amr_msgs/srv/manage_amr_nodes.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "amr_util/service_client.hpp"

namespace amr_lifecycle_manager
{
enum class SystemStatus {ACTIVE, INACTIVE, TIMEOUT};
class LifecycleManagerClient{
public:
    explicit LifecycleManagerClient(const std::string & name,
    std::shared_ptr<rclcpp::Node> parent_node);
    bool startup(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1),
      const std::vector<std::string> & node_names = {});
    bool shutdown(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1),
      const std::vector<std::string> & node_names = {});
    bool pause(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1),
      const std::vector<std::string> & node_names = {});
    bool resume(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1),
      const std::vector<std::string> & node_names = {});
    bool reset(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1),
      const std::vector<std::string> & node_names = {});
    SystemStatus is_active(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
protected:
    using ManageAmrNodes = amr_msgs::srv::ManageAmrNodes;
    bool callService(uint8_t command, const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1),
      const std::vector<std::string> & node_names = {});

    rclcpp::Node::SharedPtr node_;

    std::shared_ptr<amr_util::ServiceClient<ManageAmrNodes>> manager_client_;
    std::shared_ptr<amr_util::ServiceClient<std_srvs::srv::Trigger>> is_active_client_;
    std::string manage_service_name_;
    std::string active_service_name_;
    };
}