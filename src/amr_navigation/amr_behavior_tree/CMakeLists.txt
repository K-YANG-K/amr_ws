cmake_minimum_required(VERSION 3.5)
project(amr_behavior_tree)
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/Atlas/amr_ws/3rd/Glog")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(amr_msgs REQUIRED)
find_package(amr_util REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(glog REQUIRED)

include_directories(include)
set(library_name ${PROJECT_NAME})
set(dependencies
    rclcpp_action
    rclcpp_lifecycle
    amr_msgs
    amr_util
    tf2_ros
    std_srvs
    std_msgs
    behaviortree_cpp_v3
    nav_msgs
    builtin_interfaces
    sensor_msgs
    tf2_geometry_msgs
    glog
)
add_library(${library_name} SHARED
  src/behavior_tree_engine.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_link_libraries(${library_name}
  glog::glog
)

add_library(generate_follow_path_action_bt_node SHARED plugins/action/generate_follow_path_action.cpp)
list(APPEND plugin_libs generate_follow_path_action_bt_node)

add_library(local_control_action_bt_node SHARED plugins/action/local_control_action.cpp)
list(APPEND plugin_libs local_control_action_bt_node)

add_library(plan_global_path_action_bt_node SHARED plugins/action/plan_global_path_action.cpp)
list(APPEND plugin_libs plan_global_path_action_bt_node)

add_library(pipeline_sequence_bt_node SHARED plugins/control/pipeline_sequence.cpp)
list(APPEND plugin_libs pipeline_sequence_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_link_libraries(${bt_plugin} glog::glog)
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${library_name}
                ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES amr_tree_nodes.xml DESTINATION share/${PROJECT_NAME})


ament_export_include_directories(
  include
)

ament_export_libraries(
  ${library_name}
  ${plugin_libs}
)

ament_export_dependencies(${dependencies})

ament_package()