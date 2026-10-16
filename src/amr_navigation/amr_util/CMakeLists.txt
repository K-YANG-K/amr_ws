cmake_minimum_required(VERSION 3.5)
project(amr_util)
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/Atlas/amr_ws/3rd/Glog")



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(bond REQUIRED)
find_package(bondcpp REQUIRED)
find_package(glog REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
set(add_dependencies
    rclcpp
    lifecycle_msgs
    rclcpp_lifecycle
    bondcpp
    bond
    glog
    tf2_ros
    tf2 
    tf2_geometry_msgs
    geometry_msgs
    rclcpp_action
)
include_directories(include)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(library_name ${PROJECT_NAME}_core)

add_library(${library_name} SHARED

  src/lifecycle_node.cpp
  src/amr_glog.cpp
  src/node_thread.cpp
  src/lifecycle_service_client.cpp
  src/node_utils.cpp
  src/robot_utils.cpp

)

ament_target_dependencies(${library_name}
  rclcpp
  lifecycle_msgs
  rclcpp_lifecycle
  bondcpp
  glog
  geometry_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  rclcpp_action
)

target_link_libraries(${library_name}
  glog::glog
)

install(DIRECTORY include/
        DESTINATION include/
)

install(TARGETS
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)



ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${add_dependencies})

ament_package()
