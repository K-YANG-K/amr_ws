cmake_minimum_required(VERSION 3.5)
project(amr_bt_navigator)
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/Atlas/amr_ws/3rd/Glog")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(amr_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(amr_util REQUIRED)
find_package(bond REQUIRED)
find_package(bondcpp REQUIRED)
find_package(glog REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(amr_behavior_tree REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)
include_directories(
  include
)

set(executable_name amr_bt_navigator)

add_executable(${executable_name}
  src/main.cpp
)

set(library_name ${executable_name}_core)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  nav_msgs
  amr_msgs
  behaviortree_cpp_v3
  std_srvs
  amr_util
  tf2_ros
  bond
  bondcpp
  glog
  pluginlib
  amr_behavior_tree
  example_interfaces
  rclcpp_components
)

add_library(${library_name} SHARED
  src/amr_bt_navigator.cpp
  src/navigators/amr_nav_mode_a.cpp
  src/navigators/amr_nav_mode_b.cpp
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_link_libraries(${library_name} glog::glog)

rclcpp_components_register_nodes(${library_name} "amr_bt_navigator::AmrBtNavigator")

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

install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
