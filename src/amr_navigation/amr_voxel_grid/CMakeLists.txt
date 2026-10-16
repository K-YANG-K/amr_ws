cmake_minimum_required(VERSION 3.5)
project(amr_voxel_grid)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include)

add_library(voxel_grid SHARED
  src/voxel_grid.cpp
)

set(dependencies
  rclcpp
)

ament_target_dependencies(voxel_grid
  ${dependencies}
)

install(TARGETS voxel_grid
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)


ament_export_dependencies(rclcpp)
ament_export_include_directories(include)
ament_export_libraries(voxel_grid)

ament_package()
