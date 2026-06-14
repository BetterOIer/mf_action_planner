#!/usr/bin/env python3
"""
DFS路径规划节点 - 简化版

功能：
- 订阅 kfs_data 话题获取网格数据
- 收到数据后依次以 (0,0), (0,1), (0,2) 为起点规划路径并发布
"""

import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from core.dfs import DFSPlanner

class DFSPlannerNode(Node):
    """DFS路径规划节点"""

    def __init__(self):
        super().__init__('dfs_planner')

        # 从参数服务器读取参数
        self.declare_parameter('grid_rows', 4)
        self.declare_parameter('grid_cols', 3)

        self.GRID_ROWS = int(self.get_parameter('grid_rows').value)
        self.GRID_COLS = int(self.get_parameter('grid_cols').value)

        self.kfs_grid = np.zeros((self.GRID_ROWS, self.GRID_COLS), dtype=int)
        self.kfs_grid_height_blue = np.array([
            [000, 000, 000],
            [400, 200, 400],
            [600, 400, 200],
            [400, 600, 400],
            [200, 400, 200],
            [000, 000, 000]
        ], dtype=int)
        self.kfs_grid_height_red = np.array([
            [000, 000, 000],
            [400, 200, 400],
            [200, 400, 600],
            [400, 600, 400],
            [200, 400, 200],
            [000, 000, 000]
        ], dtype=int)

        kfs_topic = self.declare_parameter('mf_r2_data', '/mf_r2_data').value
        path_topic = self.declare_parameter('planning_path', '/planning/path').value
        path_rviz_topic = self.declare_parameter('path_on_rviz', '/path_on_rviz').value

        self.current_team = 'red'   # 默认红队
        self.current_method = 1     # 默认 uni

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.kfs_data_sub = self.create_subscription(
            String, kfs_topic, self.kfs_data_callback, qos_profile
        )

        # Path 发布器
        path_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.path_pub = self.create_publisher(String, path_topic, path_qos)
        self.path_rviz_pub = self.create_publisher(Path, path_rviz_topic, path_qos)

        # Web 路径摘要发布器（供 mf_manager.html 订阅）
        self.paths_for_web_pub = self.create_publisher(
            String, '/planning/paths_for_web', path_qos
        )

        self.get_logger().info('DFS Planner Node started (Simplified)')

    def kfs_data_callback(self, msg):
        """处理 /mf_r2_data 消息 — 包含 grid + team + method"""
        try:
            payload = json.loads(msg.data)
            grid = np.array(payload['grid'])

            if grid.shape != (self.GRID_ROWS, self.GRID_COLS):
                self.get_logger().warn(f'Invalid grid dimensions: {grid.shape}')
                return

            self.kfs_grid = grid
            self.current_team = payload.get('team', 'red')
            self.current_method = int(payload.get('method', 0))

            self.get_logger().info(
                f'Received mf_r2_data: team={self.current_team}, '
                f'method={self.current_method}\n{grid}'
            )
            self._plan_all_starts()

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing mf_r2_data: {e}')

    def _plan_all_starts(self):
        """依次以 (0,0), (0,1), (0,2) 为起点规划，发布找到的路径"""
        start_y = [0, 1, 2]

        # 根据队伍选择高度图
        if self.current_team == 'blue':
            height_map = self.kfs_grid_height_blue
        else:
            height_map = self.kfs_grid_height_red

        planner = DFSPlanner(
            self.GRID_COLS, self.GRID_ROWS,
            self.kfs_grid,
            height_map,
            2,
            method=self.current_method,
            logger=self.get_logger(),
        )
        self.get_logger().info(f'=== Planning from start_y={start_y}===')
        path = planner.plan_path(start_y)

        # path 结构: [[], [], [], [], []] — kfs2=0..4 各一个 bucket
        for k in range(5):
            bucket = path[k]
            n = min(3, len(bucket))
            self.get_logger().info(
                f'--- kfs2={k}: {len(bucket)} paths, showing top {n} ---'
            )
            for i in range(n):
                p, cost, kfs2_cnt, kfs1_list = bucket[i]
                self.get_logger().info(
                    f'  #{i+1}: cost={cost}, kfs1_affected={len(kfs1_list)}, '
                    f'steps={len(p)}'
                )
                for idx, step in enumerate(p):
                    self.get_logger().info(
                        f'    [{step[0]}, {step[1]}, {step[2]}, '
                        f'{step[3]}, {step[4]}, {step[5]}, {step[6]}, {step[7]}]'
                    )

        # 发布路径摘要到 Web 界面（kfs2=0..3 各取前10条）
        self._publish_paths_for_web(path)

    def _publish_paths_for_web(self, path):
        """将 kfs2=0..4 各前10条路径发布到 /planning/paths_for_web"""
        web_data = {'buckets': []}

        for k in range(5):  # kfs2 = 0, 1, 2, 3, 4
            bucket = path[k]
            top_n = min(10, len(bucket))
            bucket_entry = {
                'kfs2_count': k,
                'total': len(bucket),
                'paths': [],
            }
            for i in range(top_n):
                p, cost, kfs2_cnt, kfs1_list = bucket[i]
                path_entry = {
                    'index': i + 1,
                    'cost': cost,
                    'kfs2_count': kfs2_cnt,
                    'kfs1_affected': len(kfs1_list),
                    'steps': len(p),
                    'detail': [
                        [int(s[0]), float(s[1]), float(s[2]),
                         float(s[3]), float(s[4]),
                         float(s[5]), float(s[6]), float(s[7])]
                        for s in p
                    ],
                }
                bucket_entry['paths'].append(path_entry)
            web_data['buckets'].append(bucket_entry)

        msg = String()
        msg.data = json.dumps(web_data, ensure_ascii=False)
        self.paths_for_web_pub.publish(msg)
        self.get_logger().info(
            f'Published paths_for_web: '
            f'{ {k: len(path[k]) for k in range(5)} }'
        )

def main(args=None):
    rclpy.init(args=args)
    node = DFSPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
