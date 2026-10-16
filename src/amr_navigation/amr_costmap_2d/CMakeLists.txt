cmake_minimum_required(VERSION 3.5)
project(amr_costmap_2d)
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/Atlas/amr_ws/3rd/Glog")
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(map_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(amr_msgs REQUIRED)
find_package(amr_util)
find_package(amr_voxel_grid REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(glog REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
find_package(Eigen3 3.3 REQUIRED)

add_library(amr_costmap_2d_core SHARED
  src/array_parser.cpp
  src/costmap_2d.cpp
  src/layer.cpp
  src/layered_costmap.cpp
  src/costmap_2d_ros.cpp
  src/costmap_2d_publisher.cpp
  src/costmap_math.cpp
  src/footprint.cpp
  src/costmap_layer.cpp
  src/observation_buffer.cpp
  src/clear_costmap_service.cpp
  src/footprint_collision_checker.cpp
  plugins/costmap_filters/costmap_filter.cpp
)
add_library(${PROJECT_NAME}::amr_costmap_2d_core ALIAS amr_costmap_2d_core)

target_include_directories(amr_costmap_2d_core
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

set(dependencies
  geometry_msgs
  laser_geometry
  map_msgs
  message_filters
  amr_msgs
  amr_util
  amr_voxel_grid
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
  visualization_msgs
  angles
  glog
)

ament_target_dependencies(amr_costmap_2d_core
  ${dependencies}
)
target_link_libraries(amr_costmap_2d_core Eigen3::Eigen glog::glog)

add_library(layers SHARED
  plugins/inflation_layer.cpp
  plugins/static_layer.cpp
  plugins/obstacle_layer.cpp
  src/observation_buffer.cpp
  plugins/voxel_layer.cpp
  plugins/range_sensor_layer.cpp
  plugins/denoise_layer.cpp
)
add_library(${PROJECT_NAME}::layers ALIAS layers)
ament_target_dependencies(layers
  ${dependencies}
)
target_link_libraries(layers
  ${PROJECT_NAME}::amr_costmap_2d_core
  glog::glog
)

add_library(filters SHARED
  plugins/costmap_filters/keepout_filter.cpp
  plugins/costmap_filters/speed_filter.cpp
  plugins/costmap_filters/binary_filter.cpp
)
add_library(${PROJECT_NAME}::filters ALIAS filters)


ament_target_dependencies(filters
  ${dependencies}
)
target_link_libraries(filters
  ${PROJECT_NAME}::amr_costmap_2d_core glog::glog
)

add_library(amr_costmap_2d_client SHARED
  src/footprint_subscriber.cpp
  src/costmap_subscriber.cpp
  src/costmap_topic_collision_checker.cpp
)
add_library(${PROJECT_NAME}::amr_costmap_2d_client ALIAS amr_costmap_2d_client)

ament_target_dependencies(amr_costmap_2d_client
  ${dependencies}
)

target_link_libraries(amr_costmap_2d_client
  ${PROJECT_NAME}::amr_costmap_2d_core glog::glog
)

add_executable(amr_costmap_2d_markers src/costmap_2d_markers.cpp)
target_link_libraries(amr_costmap_2d_markers
  ${PROJECT_NAME}::amr_costmap_2d_core glog::glog
)

ament_target_dependencies(amr_costmap_2d_markers
  ${dependencies}
)

add_executable(amr_costmap_2d_cloud src/costmap_2d_cloud.cpp)
target_link_libraries(amr_costmap_2d_cloud
  ${PROJECT_NAME}::amr_costmap_2d_core glog::glog
)

add_executable(amr_costmap_2d src/costmap_2d_node.cpp)
ament_target_dependencies(amr_costmap_2d
  ${dependencies}
)

target_link_libraries(amr_costmap_2d
  ${PROJECT_NAME}::amr_costmap_2d_core
  ${PROJECT_NAME}::layers
  ${PROJECT_NAME}::filters
  glog::glog
)

install(TARGETS
  layers
  filters
  amr_costmap_2d_core
  amr_costmap_2d_client
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS amr_costmap_2d_markers amr_costmap_2d_cloud amr_costmap_2d
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(amr_costmap_2d costmap_plugins.xml)
ament_package()
