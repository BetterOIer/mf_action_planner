# MF Action Planner

基于 ROS2 的梅林区路径与操作规划器，集成 Web 可视化交互界面。

仓库地址：[https://github.com/BetterOIer/mf_action_planner.git](https://github.com/BetterOIer/mf_action_planner.git)

## 功能概述

- **Web 网格编辑**：在 6×3 网格上可视化放置 KFS1（蓝色）、KFS2（红色）、假 KFS（灰色）标记
- **DFS 路径规划**：以入口区三列为起点，深度优先搜索所有可行路径，按 KFS2 抓取数（0~4）分桶排序
- **路径可视化**：选中路径后在网格上以绿实线（move）+ 黄虚线（fetch）绘制，支持展开查看每步详情
- **一键发布**：选中路径后发送至 `/mf_action_seq`，含 10 秒自动发送机制
- **手动路径编辑**：拖拽式交互绘制/修改自定义路径，支持移动和抓取两种动作类型，实时预览动作序列与高度/朝向参数
- **队伍与底盘切换**：红/蓝队切换自动匹配合适的高度地图，单向/全向底盘切换传入规划器
- **KFS 布局分析**：遍历所有合法 KFS 摆放，对每种布局做 DFS 路径规划，找出最差 30 种布局并生成 LaTeX 可视化文档

## 快速启动

```bash
# 1. 构建
colcon build --packages-select mf_action_planner

# 2. 环境
source install/setup.bash

# 3. 一键启动（rosbridge + 规划器 + 浏览器）
ros2 launch mf_action_planner mf_action_planner.launch.py
```

浏览器会自动打开 `mf_manager.html`。若未自动打开，手动访问文件即可。

操作指南请参见[User Guide](doc/UserGuide.md)。

## 代价模型

路径代价由**高度差耗时**和**抓取耗时**两部分组成（浮点数，均以秒为单位）：

| 动作 | 代价 |
|------|------|
| 平层移动（Δh=0） | 1.0s |
| 上 +200mm 台阶 | 2.5s |
| 上 +400mm 台阶 | 4.0s |
| 下 -200mm 台阶 | 3.5s |
| 下 -400mm 台阶 | 5.0s |
| 抓取 KFS2（与上一步同向） | 4.0s |
| 抓取 KFS2（与上一步异向，有转向） | 5.0s |

代价常量定义在 `core/dfs.py` 的 `_DH_COST`、`_FETCH_COST_NO_TURN` 和 `_FETCH_COST_TURN` 中，旧版 `move_cost` / `turn_cost` / `fetch_kfs2_cost` 参数已废弃。

### 路径排序

每个 KFS2 抓取数（0~4）的 bucket 内按以下优先级升序排列：

1. **路径代价**（越低越好）
2. **受影响的 KFS1 数量**（越少越好）
3. **路径中的转向次数**（越少越好）——转向定义为相邻步间朝向 `arg4` 发生变化

### 移动模型

- **单向模式 (method=1)**：**无独立转向步**。抓取指令自带新朝向（`arg4`），后续所有 move 指令沿用在抓取指令上设定的朝向，直到下一次抓取改变朝向。
- **全向模式 (method=0)**：所有 move 和抓取指令朝向恒为 0。

两种模式均不产生原地转向步骤。路径发布时直接套用以上规则得到朝向序列，无需另外的转向动作。

## 架构与数据流

```
mf_manager.html ──新建/修改──→ path_editor.html  (sessionStorage 传 grid+team+method+path)
       │                            │
       │                            │  拖拽绘制 / 微调路径
       │                            │
       │←────── 返回 ────────────────┘  (sessionStorage 回传 grid+team+method)
       │
       ├── /mf_r2_data ────────────────→ dfs_planner_node  (grid + team + method)
       │                                        │
       │                                        ├── DFSPlanner.plan_path()───→dfs.py
       │                                        │   ├── 红队 → kfs_grid_height_red
       │                                        │   └── 蓝队 → kfs_grid_height_blue
       │                                        │
       │←─ /planning/paths_for_web ←────────────┤  (各桶前10条路径摘要)
       │
       ├── /mf_action_seq ←── mf_manager（选路径）
       └── /mf_action_seq ←── path_editor（自定义路径）
```

---

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/mf_r2_data` | `String` (JSON) | Web→ROS | `{grid, team, method, timestamp}`，grid 已上下左右翻转 |
| `/planning/paths_for_web` | `String` (JSON) | ROS→Web | 各 KFS2 桶（0~4）的前 10 条路径 |
| `/mf_action_seq` | `Float32MultiArray` | Web→ROS | 展平路径，`layout.dim` 编码步数和字段数 |

### /mf_r2_data — 网格与配置

Web 端发布，`dfs_planner_node` 订阅。QoS：`RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(depth=1)`，新节点启动即可收到最新状态。

**JSON 结构：**

```json
{
    "grid": [[0,0,0],[0,2,0],[0,0,1],[1,0,0],[0,0,0],[0,0,0]],
    "team": "red",
    "method": 0,
    "timestamp": 1718400000000
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `grid` | `int[6][3]` | 0=空, 1=KFS1, 2=KFS2, 3=假KFS；`grid[0][0]`=(0,0) |
| `team` | `string` | `"red"` \| `"blue"` |
| `method` | `int` | `1`=uni(单向), `0`=omni(全向) |

**Python 订阅示例：**

```python
import json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String


class MyPlanner(Node):
    def __init__(self):
        super().__init__('my_planner')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(String, '/mf_r2_data', self.cb, qos)

    def cb(self, msg: String):
        data = json.loads(msg.data)
        grid = np.array(data['grid'], dtype=int)   # shape (6, 3)
        team = data['team']                         # "red" | "blue"
        method = data['method']                     # 0 | 1
        self.get_logger().info(f'team={team} method={method}\n{grid}')
```

### /planning/paths_for_web — 路径摘要

`dfs_planner_node` 发布，Web 端订阅。QoS 同上。

**JSON 结构：**

```json
{
    "buckets": [
        {
            "kfs2_count": 0,
            "total": 42,
            "paths": [
                {
                    "index": 1,
                    "cost": 30.0,
                    "kfs2_count": 0,
                    "kfs1_affected": 2,
                    "steps": 5,
                    "detail": [
                        [0, 0, 1, 0, 0.0, 0, 0, 0],
                        [0, 1, 1, 0, 0.0, 0, 0, 0],
                        [1, 2, 0, 200, 0.0, 0, 0, 0],
                        [0, 2, 0, 0, 0.0, 0, 0, 0],
                        [1, 3, 0, 200, 0.0, 0, 0, 0]
                    ]
                }
            ]
        }
    ]
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `buckets[].kfs2_count` | `int` | KFS2 抓取数量（0~4） |
| `buckets[].total` | `int` | 该桶路径总数 |
| `paths[].cost` | `float` | 路径代价（秒） |
| `paths[].kfs1_affected` | `int` | 受影响的 KFS1 数量 |
| `paths[].steps` | `int` | 总步数 |
| `paths[].detail[][]` | `float[8]` | 每步的 8 参数数组（见 `/mf_action_seq` 说明） |

**Python 订阅示例：**

```python
def cb(self, msg: String):
    data = json.loads(msg.data)
    for bucket in data['buckets']:
        k = bucket['kfs2_count']
        self.get_logger().info(f'KFS2={k}: {bucket["total"]} 条')
        for p in bucket['paths']:
            self.get_logger().info(
                f'  #{p["index"]}: cost={p["cost"]} '
                f'kfs1_affected={p["kfs1_affected"]} steps={p["steps"]}'
            )
```

### /mf_action_seq — 行动序列

Web 端发布，机器人执行端订阅。使用 `layout.dim` 编码多步结构，`data` 为所有步的展平数组。

**`Float32MultiArray` 结构：**

```
layout.dim[0]:  label="step",  size=N（步数）,  stride=8
layout.dim[1]:  label="field", size=8（字段数）, stride=1
data:           [step0_arg0, step0_arg1, ..., step0_arg7,
                 step1_arg0, step1_arg1, ..., step1_arg7,
                 ...
                 stepN_arg0, ..., stepN_arg7]
```

**每步 8 个 float 含义：**

| 索引 | 名称 | 含义 |
|------|------|------|
| `[0]` | arg0 | `0`=move, `1`=fetch |
| `[1]` | arg1 | 目标 X（网格索引，0~5） |
| `[2]` | arg2 | 目标 Y（网格索引，0~2） |
| `[3]` | arg3 | move 时为 Z 坐标（mm）；fetch 时为高度差 Δh（目标减当前，mm） |
| `[4]` | arg4 | 朝向（弧度，0=↑，逆时针为正）。单向模式：move 步沿用最近一次抓取步的朝向，抓取步自带目标朝向。全向模式：恒为 0 |
| `[5]`~`[7]` | — | 保留，暂为 0 |

**Python 订阅示例：**

```python
from std_msgs.msg import Float32MultiArray


def cb(self, msg: Float32MultiArray):
    num_steps = msg.layout.dim[0].size
    self.get_logger().info(f'收到 {num_steps} 步行动序列')

    for i in range(num_steps):
        off = i * 8
        arg0 = int(msg.data[off])
        arg1 = int(msg.data[off + 1])   # X（网格索引）
        arg2 = int(msg.data[off + 2])   # Y（网格索引）
        arg3 = msg.data[off + 3]        # Z 或 Δh
        arg4 = msg.data[off + 4]        # 朝向（弧度）

        if arg0 == 0:
            self.get_logger().info(
                f'  [{i}] move  → ({arg1}, {arg2}, z={arg3}, yaw={arg4:.2f})'
            )
        else:
            self.get_logger().info(
                f'  [{i}] fetch → ({arg1}, {arg2}, Δh={arg3}, yaw={arg4:.2f})'
            )
```

> **注意：** arg1/arg2 为网格索引，订阅者需自行转换为实际地图坐标。每次收到的是完整多步路径，应按序逐条执行。

## 关键文件

| 文件 | 职责 |
|------|------|
| `web/mf_manager.html` | Web 界面：网格编辑、队伍/底盘切换、路径浏览与发布、跳转编辑器入口 |
| `web/path_editor.html` | Web 界面：拖拽式手动路径绘制/修改，含移动和抓取工具，实时动作序列预览 |
| `web/roslib.min.js` | rosbridge WebSocket 客户端库 |
| `app/dfs_planner_node.py` | ROS2 节点：订阅 `/mf_r2_data`，驱动 DFS 规划，发布路径摘要 |
| `core/dfs.py` | DFS 搜索核心算法：路径评估、代价计算（高度差+抓取）、排序 |
| `launch/mf_action_planner.launch.py` | 启动 rosbridge + 规划器 + 浏览器 |
| `config/para.yaml` | 参数配置（话题名、默认值、web_pub_top_n 等；代价常量写死在 `core/dfs.py` 中） |
| `test/kfs_placement_analysis.py` | 离线分析工具：遍历所有 KFS 摆放 → DFS 规划 → 排名 → 生成 LaTeX 文档 |

## 坐标系约定

以武馆区右侧第一个台阶前方格子为 `(0, 0)`。网格为 6 行 × 3 列，X 轴指向出口区（0→5），Y 轴平行于台阶（0→2）。弧度 0 方向朝上，逆时针为正。

```
蓝区：
+-------------+-------------+-------------+-----------------+
| (5, 2, 000) | (5, 1, 000) | (5, 0, 000) |    ←出口区坐标
+=============+=============+=============+=================+
║ (4, 2, 200) | (4, 1, 400) | (4, 0, 200) ║
+-------------+-------------+-------------+
║ (3, 2, 400) | (3, 1, 600) | (3, 0, 400) ║
+-------------+-------------+-------------+    ←梅林区坐标
║ (2, 2, 200) | (2, 1, 400) | (2, 0, 600) ║
+-------------+-------------+-------------+
║ (1, 2, 400) | (1, 1, 200) | (1, 0, 400) ║
+=============+=============+=============+=================+
| (0, 2, 000) | (0, 1, 000) | (0, 0, 000) |    ←入口区坐标
+-------------+-------------+-------------+-----------------+

红区：
+-------------+-------------+-------------+-----------------+
| (5, 2, 000) | (5, 1, 000) | (5, 0, 000) |    ←出口区坐标
+=============+=============+=============+=================+
║ (4, 2, 200) | (4, 1, 400) | (4, 0, 200) ║
+-------------+-------------+-------------+
║ (3, 2, 400) | (3, 1, 600) | (3, 0, 400) ║
+-------------+-------------+-------------+    ←梅林区坐标
║ (2, 2, 600) | (2, 1, 400) | (2, 0, 200) ║
+-------------+-------------+-------------+
║ (1, 2, 400) | (1, 1, 200) | (1, 0, 400) ║
+=============+=============+=============+=================+
| (0, 2, 000) | (0, 1, 000) | (0, 0, 000) |    ←入口区坐标
+-------------+-------------+-------------+-----------------+

↑为弧度0方向，逆时针为正，范围[-π, π]
```

Web 界面显示时行 0 在底部、行 5 在顶部；发布 `/mf_r2_data` 前自动翻转，使 `grid[0][0]` = `(0,0)`。

## 注意事项

1. **高度图**：红蓝两队台阶高度不同，切换队伍后规划器自动选用对应高度图，路径会重新规划。
2. **代价模型**：基于台阶高度差的浮点数代价（平层 1.0s，±200mm 含 0.5s 小数，±400mm 为整数秒）。由于路径起点和终点高度均为 0，±200mm 次数必为偶数，总代价恒为 `.0` 或 `.5`。旧版 `move_cost` / `turn_cost` 参数已废弃。
3. **自动发送**：每次收到新路径数据后 10 秒自动发送 KFS2=2 的第一条（fallback KFS2=3），手动选择其他路径或点击取消可中断。
4. **rosbridge**：依赖 `rosbridge_suite`，确保已安装（`sudo apt install ros-<distro>-rosbridge-suite`）。
5. **`/mf_action_seq`**：发布的是完整多步路径，订阅者需按 `layout.dim[0].size` 逐步执行。

