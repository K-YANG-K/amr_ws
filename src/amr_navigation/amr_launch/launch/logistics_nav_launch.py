import os 
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction , SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile


def generate_launch_description():

    amr_launch_dir = get_package_share_directory('amr_launch')
    launch_dir = os.path.join(amr_launch_dir, 'launch')
    params_dir = os.path.join(amr_launch_dir, 'params')
    urdf_dir = os.path.join(amr_launch_dir, 'urdf')
    maps_dir = os.path.join(amr_launch_dir, 'maps')
                
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]


    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    maps_file = LaunchConfiguration('map')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')

    lifecycle_nodes = ['map_server',
                       'controller_server',
                       'planner_server',
                       'amr_bt_navigator',
                       ]

    configured_params = ParameterFile(
        params_file,
        allow_substs=True)


    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true') 

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(params_dir,'logistics_nav_params.yaml'))
    #print(os.path.join(amr_launch_dir, 'params', 'logistics_nav_params.yaml'))
    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='amr_container')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            maps_dir, 'warehouse.yaml'),
        description='Full path to map file to load')

    urdf_path = os.path.join(urdf_dir, 'logistics_omni.urdf')
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}],
        remappings=remappings)

    start_container_cmd = Node(
        name='amr_container',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[{'use_sim_time': use_sim_time,}],
        arguments=['--ros-args', '--params-file', params_file],
        remappings=remappings,
        output='screen')

    mpc_params = ParameterFile(
        os.path.join(params_dir,'mpc_params.yaml'),
        allow_substs=True
    )

    load_composable_nodes = LoadComposableNodes(
        target_container=container_name_full,
        composable_node_descriptions=[

        ComposableNode(
            package='amr_bt_navigator',
            plugin='amr_bt_navigator::AmrBtNavigator',
            name='amr_bt_navigator',
            parameters=[configured_params,
                        {'use_sim_time': use_sim_time,}],
            remappings=remappings),

        ComposableNode(
            package='amr_map_server',
            plugin='amr_map_server::MapServer',
            name='map_server',
            parameters=[configured_params,
                        {'yaml_filename': maps_file}],
            remappings=remappings),

        ComposableNode(
            package='amr_controller',
            plugin='amr_controller::ControllerServer',
            name='controller_server',
            parameters=[configured_params,mpc_params],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),

        ComposableNode(
            package='amr_planner',
            plugin='amr_planner::PlannerServer',
            name='planner_server',
            parameters=[configured_params],
            remappings=remappings),
        

        ComposableNode(
            package='amr_lifecycle_manager',
            plugin='amr_lifecycle_manager::LifecycleManager',
            name='amr_lifecycle_manager',
            parameters=[{'use_sim_time': use_sim_time,
                            'autostart': autostart,
                            'node_names': lifecycle_nodes}]),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_container_name_cmd)

    ld.add_action(start_container_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(load_composable_nodes)

    return ld
