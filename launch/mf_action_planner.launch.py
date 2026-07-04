#!/usr/bin/env python3
"""
mf_action_planner 完整启动文件

启动顺序：
1. rosbridge_server (rosbridge_websocket) — WebSocket 桥接
2. mf_buffer_node                         — 暂存区（缓存 /mf_action_seq）
3. 自动打开 mf_manager.html 网页         — 可视化交互界面
4. dfs_planner_node                      — DFS 路径规划节点
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """生成完整的启动配置"""

    # ============================
    # 参数声明
    # ============================
    declared_arguments = [
        DeclareLaunchArgument(
            'open_browser',
            default_value='true',
            description='是否自动打开网页界面',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('mf_action_planner'),
                'config',
                'para.yaml',
            ]),
            description='参数文件路径',
        ),
    ]

    # ============================
    # 1. rosbridge_server — WebSocket 桥接
    # ============================
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_main',
        parameters=[{
            'port': 9090,
            'default_call_service_timeout': 5.0,
            'call_services_in_new_thread': True,
            'send_action_goals_in_new_thread': True,
        }],
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen'
    )

    # ============================
    # 2. 自动打开网页 (可选)
    # ============================
    web_file = PathJoinSubstitution([
        FindPackageShare('mf_action_planner'),
        'web',
        'mf_manager.html',
    ])

    open_browser_action = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('open_browser')),
        cmd=['xdg-open', web_file],
        shell=False,
        name='open_mf_manager_html',
    )

    # ============================
    # 2. mf_buffer_node — 暂存区
    # ============================
    buffer_node = Node(
        package='mf_action_planner',
        executable='mf_buffer_node',
        name='mf_buffer_node',
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
    )

    # ============================
    # 3. dfs_planner_node — DFS 路径规划
    # ============================
    dfs_planner_node = Node(
        package='mf_action_planner',
        executable='dfs_planner_node',
        name='dfs_planner_node',
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    # ============================
    # 事件处理：等 rosbridge 启动后再打开网页
    # ============================
    delayed_browser_open = RegisterEventHandler(
        OnProcessStart(
            target_action=rosbridge_node,
            on_start=[
                TimerAction(
                    period=1.0,
                    actions=[open_browser_action],
                ),
            ],
        ),
    )

    # ============================
    # 构建 LaunchDescription
    # ============================
    ld = LaunchDescription(declared_arguments)
    ld.add_action(rosbridge_node)
    ld.add_action(buffer_node)
    ld.add_action(delayed_browser_open)
    ld.add_action(dfs_planner_node)

    return ld
