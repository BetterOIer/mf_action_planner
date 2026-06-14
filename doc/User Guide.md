# MF Action Planner 用户指南

本文档对项目的三个核心模块进行功能介绍和关键代码讲解。

---

## 1. dfs_planner_node.py — 规划器节点

### 功能

ROS2 节点，是整个系统的中枢。订阅 Web 端发来的 `/mf_r2_data`，解析网格、队伍、底盘类型后驱动 DFS 搜索，并将结果发布回 Web 和下游。

### 关键代码

**订阅与解析：**

```python
kfs_topic = self.declare_parameter('mf_r2_data', '/mf_r2_data').value
self.kfs_data_sub = self.create_subscription(String, kfs_topic, ...)
```

收到消息后提取三个字段：

```python
self.kfs_grid = np.array(payload['grid'])          # 6×3 网格
self.current_team = payload.get('team', 'red')      # "red" | "blue"
self.current_method = int(payload.get('method', 0)) # 0=uni, 1=omni
```

**按队伍选高度图：**

```python
if self.current_team == 'blue':
    height_map = self.kfs_grid_height_blue   # 蓝区台阶高度
else:
    height_map = self.kfs_grid_height_red    # 红区台阶高度
```

红蓝两区台阶高度分布不同（见 README 地图），选错高度图会导致抓取动作的高度差计算错误。

**传入规划器：**

```python
planner = DFSPlanner(
    self.GRID_COLS, self.GRID_ROWS,  # 3 列 × 4 行（不含首末排）
    self.kfs_grid,                    # 当前网格
    height_map,                       # 高度图
    2,                                # kfs2_preferred：优先取2个
    method=self.current_method,       # uni / omni
    logger=self.get_logger(),
)
path = planner.plan_path([0, 1, 2])  # 从入口三列分别出发
```

**发布路径摘要到 Web：**

`_publish_paths_for_web()` 遍历 `path[0]` 到 `path[4]`（五个 KFS2 桶），每桶取前 10 条，转为 JSON 发到 `/planning/paths_for_web`。

---

## 2. mf_manager.html — Web 管理界面

### 功能

单页面应用，通过 rosbridge_websocket 与 ROS2 通信。包含网格编辑器、队伍/底盘切换栏、路径浏览列表、路径可视化覆盖层、自动发送倒计时。

### 关键代码

**ROS 连接与话题：**

```javascript
let ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });

// 发布：grid + team + method
const r2Publisher = new ROSLIB.Topic({
    ros, name: '/mf_r2_data', messageType: 'std_msgs/String'
});

// 订阅：路径摘要
const pathsForWebSub = new ROSLIB.Topic({
    ros, name: '/planning/paths_for_web', messageType: 'std_msgs/String'
});

// 发布：行动序列
const actionSeqPublisher = new ROSLIB.Topic({
    ros, name: '/mf_action_seq', messageType: 'std_msgs/Float32MultiArray'
});
```

**发布网格数据（含翻转）：**

```javascript
function publishR2Data() {
    // 上下左右翻转，使 grid[0][0] = (0,0)
    const flipped = grid.slice().reverse().map(row => [...row].reverse());
    const data = { grid: flipped, team: currentTeam, method: currentMethod, ... };
    r2Publisher.publish(new ROSLIB.Message({ data: JSON.stringify(data) }));
}
```

调用时机：放置/清除标记、清空、随机放置、切换队伍/底盘。

**路径在网格上的绘制（Canvas 覆盖层）：**

```javascript
function drawPathOnGrid(detail) {
    // 1. 绿色实线：连接所有 move 点
    // 2. 黄色虚线：每个 fetch 连向其前面最近的 move
    // 3. 绿色实心圆：move 端点
    // 4. 黄色空心圆：fetch 端点（最后画，避免被同坐标 move 实心圆覆盖）
}
```

坐标映射：网格坐标 `(x,y)` → 像素 `(px,py)`：
```
col = 2 - y,  row = 5 - x
px = PAD + col * (CELL + GAP) + CELL/2
py = PAD + row * (CELL + GAP) + CELL/2
```

**发布行动序列：**

```javascript
function publishPath(detail, cost, kfs1) {
    // 将所有步的 8 个 float 展平
    const flat = [];
    detail.forEach(s => { for (let i=0; i<8; i++) flat.push(Number(s[i])); });

    const msg = new ROSLIB.Message({
        layout: {
            dim: [{ label:'step', size:numSteps, stride:8 },
                  { label:'field', size:8, stride:1 }],
            data_offset: 0
        },
        data: flat
    });
    actionSeqPublisher.publish(msg);
}
```

`layout.dim[0].size` 告知订阅者步数，每 8 个 float 为一步。

**自动发送机制：**

收到新路径数据 → 优先选 KFS2=2 第一条（fallback KFS2=3）→ 10 秒倒计时 → 归零自动发送。手动选路径、点取消或立即发送均可中断。

---

## 3. dfs.py — DFS 搜索核心

### 功能

深度优先遍历 6×3 网格的所有可行路径，评估每条路径的代价、KFS2 抓取数、影响的 KFS1 数量，按桶分类排序。

### 关键代码

**构造参数：**

```python
class DFSPlanner:
    def __init__(self, grid_cols, grid_rows, grid, kfs_grid_height,
                 kfs2_preferred, method=0, logger=None):
        self.DIRECTIONS = [(1,0,0), (0,1,π/2), (0,-1,-π/2)]
        self.method = method          # 0=uni, 1=omni（预留）
        self.path = [[],[],[],[],[]]  # 5 个桶，按 kfs2 数量分组
```

`kfs2_preferred=2` 表示优选抓取 2 个 KFS2。

**DFS 搜索入口：**

```python
def plan_path(self, sty):   # sty = [0, 1, 2]（入口三列）
    for y in sty:
        for i in range(5):  # kfs2_indicated = 0..4
            self._dfs(0, y, 0)  # 从 (0, y) 出发，初始朝向 0
    return self.path
```

**单步搜索：**

```python
def _dfs(self, posX, posY, yaw):
    if posX == 5:  # 到达出口行 → 记录路径并评估
        self.cnt_path.append([0, posX, posY, 0, yaw, ...])
        self._evaluate_path_uni()  # 或 _omni
        self.cnt_path.pop()
        return

    self.visited[posX][posY] = True
    self.cnt_path.append([0, posX, posY, 0, yaw, ...])

    for direction in self.DIRECTIONS:
        newX, newY = posX + dx, posY + dy
        if 在界内 and 未访问 and 不是假KFS:
            self._dfs(newX, newY, direction[2])  # 递归

    self.visited[posX][posY] = False
    self.cnt_path.pop()
```

**路径评估 `_evaluate_path_uni`：**

1. 检查首排 KFS2 是否已取（未取则路径无效）
2. 必经之路上的 KFS2（直接踩到的）
3. 朝向恰好有的 KFS2（正前方一格）
4. 转向能拿到的 KFS2（四个方向检查）
5. 必经之路上的 KFS1（记录受影响数量）
6. 可行性检查：实际取到的 KFS2 数量 = `kfs2_indicated`
7. 构建输出路径：append move 步 + 插入 fetch 步（含高度差 `Δh`）
8. 计算代价 = move_cost × N + turn_cost × T + fetch_cost × F

**路径排序：**

```python
def _add_path(self, path, cost, kfs2_needed, kfs1_affected):
    bucket = self.path[kfs2_needed]  # 按 kfs2 数量入桶
    bucket.append((path, cost, kfs2_needed, kfs1_affected))
    # 桶内按 (cost, kfs1数量) 升序插入
```

### 输出格式

每条路径为 `[arg0..arg7]` 的列表。`arg0=0` 为 move，`arg1/arg2` 为网格索引（非实际坐标），`arg3` 为 Z 坐标或高度差，`arg4` 为弧度朝向。订阅者需自行转换为实际地图坐标。

### 可行性规则

- 首排有 KFS2 必须取
- 穿透全部 4 个 KFS2 的路径无效（R2 放不下）
- 拿到少于 2 个 KFS2 的路径不考虑
- 经过假 KFS（值为 3）的格子时跳过
