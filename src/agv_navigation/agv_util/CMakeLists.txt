cmake_minimum_required(VERSION 3.5)
project(agv_util)

# Add 3rd/Glog to CMAKE_PREFIX_PATH for find_package
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/tianxing/agv_ws/3rd/Glog")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(bond REQUIRED)
find_package(bondcpp REQUIRED)
find_package(glog REQUIRED)

set(add_dependencies
    rclcpp
    lifecycle_msgs
    rclcpp_lifecycle
    bondcpp
    bond
    glog
)
include_directories(include)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(library_name ${PROJECT_NAME}_lib)

add_library(${library_name} SHARED

  src/lifecycle_node.cpp
  src/agv_glog.cpp

)

ament_target_dependencies(${library_name}
  rclcpp
  lifecycle_msgs
  rclcpp_lifecycle
  bondcpp
  glog
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
ament_export_dependencies(${dependencies})

ament_package()
