/*
    说明：topic的话题名统一引用该文件下的别名
    命名规则：/节点名/话题名
*/
#pragma once


namespace amr_msgs {
namespace topic_names {
    inline constexpr const char* GLOBAL_PATH = "/planner_server/global_path";//导航生成的路径
    inline constexpr const char* ODOM = "/amr_controller/odom";//里程计消息
}
}