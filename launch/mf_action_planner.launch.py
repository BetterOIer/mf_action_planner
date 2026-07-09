#!/usr/bin/env python3
"""
mf_action_planner 完整启动文件

启动顺序：
1. rosbridge_server (rosbridge_websocket) — WebSocket 桥接
2. monitor_node                          — HTTP 中继（Topic 限频 + Action 代理，端口 8765）
3. mf_buffer_node                        — 暂存区（缓存 /mf_action_seq）
4. 自动打开 mf_manager.html 网页        — 可视化交互界面
5. dfs_planner_node                      — DFS 路径规划节点
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
            default_value='false',
            description='是否自动打开网页界面',
        ),
        DeclareLaunchArgument(
            'start_rosbridge',
            default_value='false',
            description='是否启动 rosbridge_websocket；总控页面启动时通常保持 false',
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
        condition=IfCondition(LaunchConfiguration('start_rosbridge')),
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
    # 2. 静态网页服务与自动打开网页 (可选)
    # ============================
    web_dir = PathJoinSubstitution([
        FindPackageShare('mf_action_planner'),
        'web',
    ])
    web_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '7890', '--bind', '0.0.0.0', '--directory', web_dir],
        shell=False,
        name='mf_action_planner_web',
        output='screen',
    )

    open_browser_action = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('open_browser')),
        cmd=['xdg-open', 'http://localhost:7890/mf_manager.html'],
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
        arguments=['--ros-args', '--log-level', 'INFO'],
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
    # 4. monitor_node — HTTP 中继（Topic 限频 + Action 代理）
    # ============================
    monitor_node = Node(
        package='mf_action_planner',
        executable='monitor_node',
        name='monitor_node',
        arguments=['--ros-args', '--log-level', 'INFO'],
        output='screen',
    )

    # ============================
    # 事件处理：等 rosbridge 启动后再打开网页
    # ============================
    delayed_browser_open = RegisterEventHandler(
        OnProcessStart(
            target_action=web_server,
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
    ld.add_action(monitor_node)
    ld.add_action(buffer_node)
    ld.add_action(web_server)
    ld.add_action(delayed_browser_open)
    ld.add_action(dfs_planner_node)

    return ld
