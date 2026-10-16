/*
    说明：service的服务名统一引用该文件下的别名
    命名规则：/节点名/服务名
*/
#pragma once


namespace amr_msgs {
namespace server_names {
    inline constexpr const char* AMR_LIFECYCLE_MANAGER = "/amr_lifecycle_manager/manage_amr_nodes";//节点生命管理服务，详见ManageAmrNodes.srv
    
    inline constexpr const char* AMR_LIFECYCLE_STATUS = "/amr_lifecycle_manager/amr_node_is_active";//节点是否活跃

    inline constexpr const char* AMR_IS_PATH_VALID = "/amr_planner/is_path_valid";//检查路径合法性
}
}