cmake_minimum_required(VERSION 3.5)
project(amr_mpc_controller)
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/Atlas/amr_ws/3rd/Glog")
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/Atlas/amr_ws/3rd/NLopt")
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
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(amr_core REQUIRED)
find_package(amr_costmap_2d REQUIRED)
find_package(glog REQUIRED)
find_package(NLopt REQUIRED)

include_directories(
  include
)

set(executable_name mpc_controller)
set(library_name ${executable_name})

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
  tf2_geometry_msgs
  pluginlib
  amr_core
  amr_costmap_2d
  glog
)

add_library(${library_name} SHARED
  src/mpc_controller.cpp
  src/optimizer.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_link_libraries(${library_name}
  glog::glog
  NLopt::nlopt
)


pluginlib_export_plugin_description_file(amr_core mpc_controller_plugin.xml)


install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES mpc_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()