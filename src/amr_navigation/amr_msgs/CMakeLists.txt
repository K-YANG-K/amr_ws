cmake_minimum_required(VERSION 3.5)
project(amr_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED) 
find_package(action_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
include_directories(include)
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Costmap.msg"
    "msg/CostmapFilterInfo.msg"
    "msg/CostmapMetaData.msg"
    "msg/EdgeCost.msg"
    "msg/SpeedLimit.msg"
    "msg/VoxelGrid.msg"

    "srv/ManageAmrNodes.srv"
    "srv/GetCostmap.srv"
    "srv/IsPathValid.srv"
    "srv/ClearCostmapExceptRegion.srv"
    "srv/ClearCostmapAroundRobot.srv"
    "srv/ClearEntireCostmap.srv"
    "srv/DynamicEdges.srv"
    "srv/ManageLifecycleNodes.srv"
    "srv/LoadMap.srv"
    "srv/SaveMap.srv"
    "srv/SetInitialPose.srv"
    "srv/SetRouteGraph.srv"

    "action/AmrNavModeA.action"
    "action/AmrNavModeB.action"
    "action/GenerateFollowPath.action"
    "action/PlanGlobalPath.action"
    "action/LocalControl.action"
    "action/LocalPlan.action"
    
DEPENDENCIES builtin_interfaces geometry_msgs std_msgs nav_msgs
)

install(DIRECTORY include/
  DESTINATION include/
)
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
  