#pragma once

#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/srv/list_parameters.hpp"
#include "glog/logging.h"
namespace amr_util{
template<typename NodeT>
void declare_parameter_if_not_declared(
  NodeT node,
  const std::string & param_name,
  const rclcpp::ParameterValue & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
  rcl_interfaces::msg::ParameterDescriptor())
{
  if (!node->has_parameter(param_name)) {
    node->declare_parameter(param_name, default_value, parameter_descriptor);
  }
}

template<typename NodeT>
void declare_parameter_if_not_declared(
  NodeT node,
  const std::string & param_name,
  const rclcpp::ParameterType & param_type,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
  rcl_interfaces::msg::ParameterDescriptor())
{
  if (!node->has_parameter(param_name)) {
    node->declare_parameter(param_name, param_type, parameter_descriptor);
  }
}

template<typename NodeT>
void declare_parameter_if_not_declared(
  NodeT node,
  const std::string & param_name,
  const std::vector<std::string> & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
  rcl_interfaces::msg::ParameterDescriptor())
{
  if (!node->has_parameter(param_name)) {
    node->declare_parameter(param_name, default_value, parameter_descriptor);
  }
}

template<typename NodeT1, typename NodeT2>
void copy_all_parameters(const NodeT1 & parent, const NodeT2 & child)
{
  using Parameters = std::vector<rclcpp::Parameter>;
  std::vector<std::string> param_names = parent->list_parameters({}, 0).names;
  Parameters params = parent->get_parameters(param_names);
  for (Parameters::const_iterator iter = params.begin(); iter != params.end(); ++iter) {
    if (!child->has_parameter(iter->get_name())) {
      child->declare_parameter(iter->get_name(), iter->get_parameter_value());
    }
  }
}
template<typename NodeT>
std::string get_plugin_type_param(
  NodeT node,
  const std::string & plugin_name)
{
  declare_parameter_if_not_declared(node, plugin_name + ".plugin", rclcpp::PARAMETER_STRING);
  std::string plugin_type;
  try {
    if (!node->get_parameter(plugin_name + ".plugin", plugin_type)) {
      LOG(FATAL) << "无法获取 " << plugin_name << " 的 plugin 参数值";
    }
  } catch (rclcpp::exceptions::ParameterUninitializedException & ex) {
    LOG(FATAL) << "未定义 " << plugin_name << " 的 plugin 参数";
  }

  return plugin_type;
}
std::string generate_internal_node_name(const std::string & prefix = "");
rclcpp::Node::SharedPtr generate_internal_node(const std::string & prefix = "");
std::string sanitize_node_name(const std::string & potential_node_name);
std::string add_namespaces(const std::string & top_ns, const std::string & sub_ns = "");
std::string time_to_string(size_t len);
}