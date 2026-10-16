# 仿真启动文件：Gazebo + 全向底盘 + 真值定位 + logistics 导航栈

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # ── 路径 ────────────────────────────────────────────────────
    amr_launch_dir = get_package_share_directory('amr_launch')
    launch_dir = os.path.join(amr_launch_dir, 'launch')

    # ── 启动参数 ────────────────────────────────────────────────
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    robot_name = LaunchConfiguration('robot_name')
    autostart = LaunchConfiguration('autostart')

    # 机器人初始位姿
    pose = {
        'x': LaunchConfiguration('x_pose', default='-2.00'),
        'y': LaunchConfiguration('y_pose', default='-0.50'),
        'z': LaunchConfiguration('z_pose', default='0.01'),
        'R': LaunchConfiguration('roll', default='0.00'),
        'P': LaunchConfiguration('pitch', default='0.00'),
        'Y': LaunchConfiguration('yaw', default='0.00'),
    }

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # ── 参数声明 ────────────────────────────────────────────────
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='',
        description='顶层命名空间')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='使用 Gazebo 仿真时钟')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator', default_value='True',
        description='是否启动 Gazebo 仿真器')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True',
        description='是否启动 RViz')

    declare_headless_cmd = DeclareLaunchArgument(
        'headless', default_value='False',
        description='是否无头模式（不启动 gzclient）')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'worlds', 'turtlebot3_world.world'),
        description='Gazebo .world 文件路径')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(amr_launch_dir, 'maps', 'turtlebot3_world.yaml'),
        description='地图 yaml 文件路径')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(amr_launch_dir, 'params', 'logistics_nav_params.yaml'),
        description='导航参数文件路径')

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name', default_value='logistics_omni',
        description='机器人模型名称')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='自动启动导航栈')

    # ── Gazebo 仿真器 ────────────────────────────────────────────
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression(
            [use_simulator, ' and not ', headless])),
        cmd=['gzclient'],
        output='screen')

    # ── 机器人模型（Gazebo spawn，URDF 文件）────────────────────
    urdf_file = os.path.join(amr_launch_dir, 'urdf', 'logistics_omni.urdf')

    start_gazebo_spawner_cmd = Node(
        condition=IfCondition(use_simulator),
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-file', urdf_file,
            '-robot_namespace', namespace,
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']])

    # ── 真值定位（替代 AMCL）────────────────────────────────────
    ground_truth_localization_cmd = Node(
        condition=IfCondition(use_simulator),
        package='amr_launch',
        executable='ground_truth_localization.py',
        name='ground_truth_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_model_name': robot_name}])

    # ── RViz ────────────────────────────────────────────────────
    rviz_config_file = os.path.join(amr_launch_dir, 'rviz', 'logistics_nav.rviz')
    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}])

    # ── 导航栈（logistics_nav_launch）────────────────────────────
    logistics_nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'logistics_nav_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'map': map_yaml_file,
        }.items())

    # ── 组装 LaunchDescription ──────────────────────────────────
    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_autostart_cmd)

    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_gazebo_spawner_cmd)
    ld.add_action(ground_truth_localization_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(logistics_nav_cmd)

    return ld
