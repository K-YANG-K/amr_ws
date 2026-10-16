#include "amr_lifecycle_manager/lifecycle_manager.hpp"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

using lifecycle_msgs::msg::Transition;
using lifecycle_msgs::msg::State;
using amr_util::LifecycleServiceClient;

namespace amr_lifecycle_manager
{

LifecycleManager::LifecycleManager(const rclcpp::NodeOptions & options)
: Node("lifecycle_manager", options), diagnostics_updater_(this)
{
  LOG(INFO)<<"初始化Node--->[lifecycle_manager]";

  declare_parameter("node_names", rclcpp::PARAMETER_STRING_ARRAY);
  declare_parameter("autostart", rclcpp::ParameterValue(false));
  declare_parameter("bond_timeout", 4.0);
  declare_parameter("bond_respawn_max_duration", 10.0);
  declare_parameter("attempt_respawn_reconnection", true);

  registerRclPreshutdownCallback();
  registerRclShutdownCallback();

  node_names_ = get_parameter("node_names").as_string_array();
  get_parameter("autostart", autostart_);
  double bond_timeout_s;
  get_parameter("bond_timeout", bond_timeout_s);
  bond_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(bond_timeout_s));

  double respawn_timeout_s;
  get_parameter("bond_respawn_max_duration", respawn_timeout_s);
  bond_respawn_max_duration_ = rclcpp::Duration::from_seconds(respawn_timeout_s);

  get_parameter("attempt_respawn_reconnection", attempt_respawn_reconnection_);

  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  manager_srv_ = create_service<ManageAmrNodes>(
    amr_msgs::server_names::AMR_LIFECYCLE_MANAGER,
    std::bind(&LifecycleManager::managerCallback, this, _1, _2, _3),
    rclcpp::ServicesQoS().get_rmw_qos_profile(),
    callback_group_);

  is_active_srv_ = create_service<std_srvs::srv::Trigger>(
    amr_msgs::server_names::AMR_LIFECYCLE_STATUS,
    std::bind(&LifecycleManager::isActiveCallback, this, _1, _2, _3),
    rclcpp::ServicesQoS().get_rmw_qos_profile(),
    callback_group_);//用于查询各节点是否启动

  transition_state_map_[Transition::TRANSITION_CONFIGURE] = State::PRIMARY_STATE_INACTIVE;
  transition_state_map_[Transition::TRANSITION_CLEANUP] = State::PRIMARY_STATE_UNCONFIGURED;
  transition_state_map_[Transition::TRANSITION_ACTIVATE] = State::PRIMARY_STATE_ACTIVE;
  transition_state_map_[Transition::TRANSITION_DEACTIVATE] = State::PRIMARY_STATE_INACTIVE;
  transition_state_map_[Transition::TRANSITION_UNCONFIGURED_SHUTDOWN] =
    State::PRIMARY_STATE_FINALIZED;

  transition_label_map_[Transition::TRANSITION_CONFIGURE] = std::string("正在配置---> ");
  transition_label_map_[Transition::TRANSITION_CLEANUP] = std::string("正在清理---> ");
  transition_label_map_[Transition::TRANSITION_ACTIVATE] = std::string("正在激活---> ");
  transition_label_map_[Transition::TRANSITION_DEACTIVATE] = std::string("正在停用---> ");
  transition_label_map_[Transition::TRANSITION_UNCONFIGURED_SHUTDOWN] =
    std::string("正在关闭---> ");

  init_timer_ = this->create_wall_timer(
    0s,
    [this]() -> void {
      init_timer_->cancel();
      createLifecycleServiceClients();
      if (autostart_) {
        init_timer_ = this->create_wall_timer(
          0s,
          [this]() -> void {
            init_timer_->cancel();
            startup();
          },
          callback_group_);
      }
      auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor->add_callback_group(callback_group_, get_node_base_interface());
      service_thread_ = std::make_unique<amr_util::NodeThread>(executor);
    });
  diagnostics_updater_.setHardwareID("amr");
  diagnostics_updater_.add("amr Health", this, &LifecycleManager::CreateActiveDiagnostic);
}

LifecycleManager::~LifecycleManager()
{
  service_thread_.reset();
}

void
LifecycleManager::managerCallback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<ManageAmrNodes::Request> request,
  std::shared_ptr<ManageAmrNodes::Response> response)
{
  std::vector<std::string> target_nodes = request->node_names;
  switch (request->command) {
    case ManageAmrNodes::Request::STARTUP:
      response->success = startup(target_nodes);
      break;
    case ManageAmrNodes::Request::RESET:
      response->success = reset(false, target_nodes);
      break;
    case ManageAmrNodes::Request::SHUTDOWN:
      response->success = shutdown(target_nodes);
      break;
    case ManageAmrNodes::Request::PAUSE:
      response->success = pause(target_nodes);
      break;
    case ManageAmrNodes::Request::RESUME:
      response->success = resume(target_nodes);
      break;
  }
}

void
LifecycleManager::isActiveCallback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  response->success = system_active_;
}

void
LifecycleManager::CreateActiveDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (system_active_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "amr is active");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "amr is inactive");
  }
}

void
LifecycleManager::createLifecycleServiceClients()
{
  for (auto & node_name : node_names_) {
    node_map_[node_name] =
      std::make_shared<LifecycleServiceClient>(node_name, shared_from_this());
  }
}

void
LifecycleManager::destroyLifecycleServiceClients()
{
  LOG(INFO) << "销毁生命周期服务客户端";
  for (auto & kv : node_map_) {
    kv.second.reset();
  }
}

bool
LifecycleManager::createBondConnection(const std::string & node_name)
{
  const double timeout_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(bond_timeout_).count();
  const double timeout_s = timeout_ns / 1e9;

  if (bond_map_.find(node_name) == bond_map_.end() && bond_timeout_.count() > 0.0) {
    bond_map_[node_name] =
      std::make_shared<bond::Bond>("bond", node_name, shared_from_this());
    bond_map_[node_name]->setHeartbeatTimeout(timeout_s);
    bond_map_[node_name]->setHeartbeatPeriod(0.10);
    bond_map_[node_name]->start();
    if (
      !bond_map_[node_name]->waitUntilFormed(
        rclcpp::Duration(rclcpp::Duration::from_nanoseconds(timeout_ns / 2))))
    {
      LOG(ERROR) << "Bond 无法连接节点 " << node_name
        << "，超时 " << timeout_s
        << " 秒，该节点可能配置有误";
      return false;
    }
    LOG(INFO) << "节点 [" << node_name << "] 已建立心跳连接";
  }

  return true;
}

bool
LifecycleManager::changeStateForNode(const std::string & node_name, std::uint8_t transition)
{
  LOG(INFO) << transition_label_map_[transition] + node_name;

  if (!node_map_[node_name]->change_state(transition) ||
    !(node_map_[node_name]->get_state() == transition_state_map_[transition]))
  {
    LOG(ERROR) << "无法更改节点 " << node_name << " 的状态";
    return false;
  }

  if (transition == Transition::TRANSITION_ACTIVATE) {
    return createBondConnection(node_name);
  } else if (transition == Transition::TRANSITION_DEACTIVATE) {
    bond_map_.erase(node_name);
  }

  return true;
}

bool
LifecycleManager::changeStateForAllNodes(std::uint8_t transition, bool hard_change)
{
  if (transition == Transition::TRANSITION_CONFIGURE ||
    transition == Transition::TRANSITION_ACTIVATE)
  {
    for (auto & node_name : node_names_) {
      try {
        if (!changeStateForNode(node_name, transition) && !hard_change) {
          return false;
        }
      } catch (const std::runtime_error & e) {
        LOG(ERROR) << "无法更改节点 " << node_name
          << " 的状态，异常: " << e.what();
        return false;
      }
    }
  } else {
    std::vector<std::string>::reverse_iterator rit;
    for (rit = node_names_.rbegin(); rit != node_names_.rend(); ++rit) {
      try {
        if (!changeStateForNode(*rit, transition) && !hard_change) {
          return false;
        }
      } catch (const std::runtime_error & e) {
        LOG(ERROR) << "无法更改节点 " << *rit
          << " 的状态，异常: " << e.what();
        return false;
      }
    }
  }
  return true;
}

bool
LifecycleManager::changeStateForNodes(
  const std::vector<std::string> & node_names,
  std::uint8_t transition, bool hard_change)
{
  if (transition == Transition::TRANSITION_CONFIGURE ||
    transition == Transition::TRANSITION_ACTIVATE)
  {
    for (auto & node_name : node_names) {
      try {
        if (!changeStateForNode(node_name, transition) && !hard_change) {
          return false;
        }
      } catch (const std::runtime_error & e) {
        LOG(ERROR) << "无法更改节点 " << node_name
          << " 的状态，异常: " << e.what();
        return false;
      }
    }
  } else {
    for (auto rit = node_names.rbegin(); rit != node_names.rend(); ++rit) {
      try {
        if (!changeStateForNode(*rit, transition) && !hard_change) {
          return false;
        }
      } catch (const std::runtime_error & e) {
        LOG(ERROR) << "无法更改节点 " << *rit
          << " 的状态，异常: " << e.what();
        return false;
      }
    }
  }
  return true;
}

void
LifecycleManager::shutdownNodes(const std::vector<std::string> & node_names)
{
  LOG(INFO) << "停用、清理并关闭指定节点";
  changeStateForNodes(node_names, Transition::TRANSITION_DEACTIVATE);
  changeStateForNodes(node_names, Transition::TRANSITION_CLEANUP);
  changeStateForNodes(node_names, Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
}

void
LifecycleManager::shutdownAllNodes()
{
  LOG(INFO) << "停用、清理并关闭节点";
  changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE);
  changeStateForAllNodes(Transition::TRANSITION_CLEANUP);
  changeStateForAllNodes(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
}

bool
LifecycleManager::startup(const std::vector<std::string> & node_names)
{
  const auto & target_nodes = node_names.empty() ? node_names_ : node_names;
  std::stringstream ss;
  for (size_t i = 0; i < target_nodes.size(); ++i) {
    if (i > 0) ss << ", ";
    ss << target_nodes[i];
  }
  LOG(INFO) << "正在启动生命周期节点...";
  bool result;
  if (node_names.empty()) {
    result = changeStateForAllNodes(Transition::TRANSITION_CONFIGURE) &&
      changeStateForAllNodes(Transition::TRANSITION_ACTIVATE);
  } else {
    result = changeStateForNodes(node_names, Transition::TRANSITION_CONFIGURE) &&
      changeStateForNodes(node_names, Transition::TRANSITION_ACTIVATE);
  }
  if (!result) {
    LOG(ERROR) << "无法启动请求的节点，中止启动";
    return false;
  }
  LOG(INFO) << "已激活生命周期节点: [" << ss.str() << "]";
  system_active_ = true;
  createBondTimer();
  return true;
}

bool
LifecycleManager::shutdown(const std::vector<std::string> & node_names)
{
  system_active_ = false;
  destroyBondTimer();

  LOG(INFO) << "正在关闭受管节点...";
  const auto & shutdown_targets = node_names.empty() ? node_names_ : node_names;
  std::stringstream ss;
  for (size_t i = 0; i < shutdown_targets.size(); ++i) {
    if (i > 0) ss << ", ";
    ss << shutdown_targets[i];
  }
  if (node_names.empty()) {
    shutdownAllNodes();
    destroyLifecycleServiceClients();
  } else {
    shutdownNodes(node_names);
  }
  LOG(INFO) << "已关闭受管节点: [" << ss.str() << "]";
  return true;
}

bool
LifecycleManager::reset(bool hard_reset, const std::vector<std::string> & node_names)
{
  system_active_ = false;
  destroyBondTimer();

  LOG(INFO) << "正在重置受管节点...";
  const auto & reset_targets = node_names.empty() ? node_names_ : node_names;
  std::stringstream ss;
  for (size_t i = 0; i < reset_targets.size(); ++i) {
    if (i > 0) ss << ", ";
    ss << reset_targets[i];
  }
  bool result;
  if (node_names.empty()) {
    result = changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE, hard_reset) &&
      changeStateForAllNodes(Transition::TRANSITION_CLEANUP, hard_reset);
  } else {
    result = changeStateForNodes(node_names, Transition::TRANSITION_DEACTIVATE, hard_reset) &&
      changeStateForNodes(node_names, Transition::TRANSITION_CLEANUP, hard_reset);
  }
  if (!result && !hard_reset) {
    LOG(ERROR) << "无法重置节点，中止重置";
    return false;
  }

  LOG(INFO) << "已重置受管节点: [" << ss.str() << "]";
  return true;
}

bool
LifecycleManager::pause(const std::vector<std::string> & node_names)
{
  system_active_ = false;
  destroyBondTimer();

  LOG(INFO) << "正在暂停受管节点...";
  const auto & pause_targets = node_names.empty() ? node_names_ : node_names;
  std::stringstream ss;
  for (size_t i = 0; i < pause_targets.size(); ++i) {
    if (i > 0) ss << ", ";
    ss << pause_targets[i];
  }
  bool result = node_names.empty() ?
    changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE) :
    changeStateForNodes(node_names, Transition::TRANSITION_DEACTIVATE);
  if (!result) {
    LOG(ERROR) << "无法暂停节点，中止暂停";
    return false;
  }

  LOG(INFO) << "已暂停受管节点: [" << ss.str() << "]";
  return true;
}

bool
LifecycleManager::resume(const std::vector<std::string> & node_names)
{
  LOG(INFO) << "正在恢复受管节点...";
  const auto & resume_targets = node_names.empty() ? node_names_ : node_names;
  std::stringstream ss;
  for (size_t i = 0; i < resume_targets.size(); ++i) {
    if (i > 0) ss << ", ";
    ss << resume_targets[i];
  }

  bool result = node_names.empty() ?
    changeStateForAllNodes(Transition::TRANSITION_ACTIVATE) :
    changeStateForNodes(node_names, Transition::TRANSITION_ACTIVATE);
  if (!result) {
    LOG(ERROR) << "无法恢复节点，中止恢复";
    return false;

  }

  LOG(INFO) << "已恢复受管节点: [" << ss.str() << "]";
  system_active_ = true;
  createBondTimer();
  return true;
}

void
LifecycleManager::createBondTimer()
{
  if (bond_timeout_.count() <= 0) {
    return;
  }

  LOG(INFO) << "创建心跳定时器...";
  bond_timer_ = this->create_wall_timer(
    200ms,
    std::bind(&LifecycleManager::checkBondConnections, this),
    callback_group_);
}

void
LifecycleManager::destroyBondTimer()
{
  if (bond_timer_) {
    LOG(INFO) << "终止心跳定时器...";
    bond_timer_->cancel();
    bond_timer_.reset();
  }
}

void
LifecycleManager::onRclPreshutdown()
{
  LOG(INFO)<<"异常退出预关闭 " << this->get_name();
  destroyBondTimer();
  service_thread_.reset();

  destroyLifecycleServiceClients();

  node_names_.clear();
  node_map_.clear();
  bond_map_.clear();
}

void
LifecycleManager::registerRclPreshutdownCallback()
{
  rclcpp::Context::SharedPtr context = get_node_base_interface()->get_context();

  context->add_pre_shutdown_callback(
    std::bind(&LifecycleManager::onRclPreshutdown, this)
  );
}

void
LifecycleManager::registerRclShutdownCallback()
{
  rclcpp::Context::SharedPtr context = get_node_base_interface()->get_context();

  context->add_on_shutdown_callback(
    []() {
      LOG(INFO) << "生命周期节点已全部清理";
      std::cout.flush();
      std::_Exit(0);
    }
  );
}

void
LifecycleManager::checkBondConnections()
{
  if (!system_active_ || !rclcpp::ok() || bond_map_.empty()) {
    return;
  }

  for (auto & node_name : node_names_) {
    if (!rclcpp::ok()) {
      return;
    }

    if (bond_map_[node_name]->isBroken()) {
      LOG(INFO) << "未收到节点 " + node_name + ".";

      LOG(ERROR) << "严重故障：节点 " << node_name
        << " 心跳超时 " << static_cast<int>(bond_timeout_.count())
        << " ms 已宕机，正在关闭相关节点";
      reset(true);  
      bond_map_.clear();

      if (attempt_respawn_reconnection_) {
        bond_respawn_timer_ = this->create_wall_timer(
          1s,
          std::bind(&LifecycleManager::checkBondRespawnConnection, this),
          callback_group_);
      }
      return;
    }
  }
}

void
LifecycleManager::checkBondRespawnConnection()
{

  if (bond_respawn_start_time_.nanoseconds() == 0) {
    bond_respawn_start_time_ = now();
  }

  if (system_active_ || !rclcpp::ok() || node_names_.empty()) {
    bond_respawn_start_time_ = rclcpp::Time(0);
    bond_respawn_timer_.reset();
    return;
  }


  int live_servers = 0;
  const int max_live_servers = node_names_.size();
  for (auto & node_name : node_names_) {
    if (!rclcpp::ok()) {
      return;
    }

    try {
      node_map_[node_name]->get_state();  
      live_servers++;
    } catch (...) {
      break;
    }
  }

  if (live_servers == max_live_servers) {
    LOG(INFO) << "成功重新建立连接，正在恢复...";
    bond_respawn_start_time_ = rclcpp::Time(0);
    bond_respawn_timer_.reset();
    startup();
  } else if (now() - bond_respawn_start_time_ >= bond_respawn_max_duration_) {
    LOG(INFO) << "无法重新建立连接，已超时";
    bond_respawn_start_time_ = rclcpp::Time(0);
    bond_respawn_timer_.reset();
  }
}

}  

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(amr_lifecycle_manager::LifecycleManager)
