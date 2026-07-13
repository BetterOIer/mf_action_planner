#!/usr/bin/env python3
"""
mf_action_planner 完整启动文件

启动顺序：
1. monitor_node                          — HTTP 中继（Topic 限频 + Action 代理，端口 8765）
2. mf_buffer_node                        — 暂存区（缓存 /mf_action_seq）
3. dfs_planner_node                      — DFS 路径规划节点
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('mf_action_planner'),
                'config',
                'para.yaml',
            ]),
            description='参数文件路径',
        ),
    ]

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
    # 构建 LaunchDescription
    # ============================
    ld = LaunchDescription(declared_arguments)
    ld.add_action(monitor_node)
    ld.add_action(buffer_node)
    ld.add_action(dfs_planner_node)

    return ld
