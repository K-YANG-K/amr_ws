cmake_minimum_required(VERSION 3.5)
project(amr_planner)
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/Atlas/amr_ws/3rd/Glog")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(amr_util REQUIRED)
find_package(amr_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(amr_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(amr_core REQUIRED)
find_package(glog REQUIRED)
include_directories(
  include
)

set(executable_name planner_server)
set(library_name ${executable_name}_core)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  visualization_msgs
  amr_util
  amr_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2_ros
  amr_costmap_2d
  pluginlib
  amr_core
  glog
)

add_library(${library_name} SHARED
  src/planner_server.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_link_libraries(${library_name}
  glog::glog
)

add_executable(${executable_name}
  src/main.cpp
)

target_link_libraries(${executable_name} ${library_name} glog::glog)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

rclcpp_components_register_nodes(${library_name} "amr_planner::PlannerServer")

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)


ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
