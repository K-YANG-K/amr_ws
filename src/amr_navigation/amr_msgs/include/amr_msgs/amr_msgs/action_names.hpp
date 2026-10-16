/*
    说明：action的服务名统一引用该文件下的别名
    命名规则：/节点名/服务名
*/
#pragma once

namespace amr_msgs {
namespace action_names {
    inline constexpr const char* AMR_NAV_MODE_A = "/amr_bt_navigator/amr_nav_mode_a";//导航A服务，详见AmrNavModeA.action

    inline constexpr const char* AMR_NAV_MODE_B = "/amr_bt_navigator/amr_nav_mode_b";//导航B服务，详见AmrNavModeB.action

    inline constexpr const char* GENERATE_FOLLOW_PATH = "/amr_follow_path_generator/generate_follow_path";//导航A路径生成服务

    inline constexpr const char* PLAN_GLOBAL_PATH = "/amr_global_path_palnner/plan_global_path";//导航B路径生成服务

    inline constexpr const char* LOCAL_CONTROL = "/amr_controller/local_control" ;//导航A路径跟踪控制服务

    inline constexpr const char* LOCAL_PLAN = "/amr_controller/local_plan" ;//导航B路径跟踪控制服务

}  
}  