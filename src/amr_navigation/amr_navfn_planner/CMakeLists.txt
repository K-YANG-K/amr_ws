cmake_minimum_required(VERSION 3.5)
project(amr_navfn_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(amr_util REQUIRED)
find_package(amr_core REQUIRED)
find_package(amr_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(amr_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)


include_directories(
  include
)

set(library_name amr_navfn_planner)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  visualization_msgs
  amr_util
  amr_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2_ros
  amr_costmap_2d
  amr_core
  pluginlib
)

add_library(${library_name} SHARED
  src/navfn_planner.cpp
  src/navfn.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

pluginlib_export_plugin_description_file(amr_core global_planner_plugin.xml)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES global_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)



ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
