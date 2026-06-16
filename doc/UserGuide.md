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

单页面应用，通过 rosbridge_websocket 与 ROS2 通信。包含网格编辑器、队伍/底盘切换栏、路径浏览列表、路径可视化覆盖层、自动发送倒计时。左侧包含两个面板：工具栏（KFS1/KFS2/K假/清空/随机）和独立的发送面板。

### 界面布局

```
┌──────────┐ ┌────────────────────┐ ┌────┬────┐
│  KFS1    │ │                    │ │自定│KFS2│
│  KFS2    │ │   3×6 布局网格      │ │义路│=0  │
│  K假     │ │   + Canvas 叠加     │ │线  │    │
│  清空    │ │                    │ ├────┼────┤
│  随机    │ │  [红队/蓝队]       │ │新建│KFS2│
└──────────┘ │  [单向/全向]       │ │修改│=1  │
┌──────────┐ └────────────────────┘ ├────┼────┤
│  发送    │                        │KFS2│KFS2│
└──────────┘                        │=3  │=4  │
                                    └────┴────┘
```

### 新增功能：路径编辑器入口

原"自定义路线"面板替换为**新建**和**修改**两个按钮：

- **新建**：将当前 KFS 布局、队伍、底盘信息通过 `sessionStorage` 传给 `path_editor.html`，从空白开始绘制
- **修改**：在新建的基础上额外传入当前选中路径的 `detail` 数据，在已有路径上微调

两个按钮在无选中路径时**修改**按钮禁用，选中路径后同时启用。从编辑器返回时自动恢复 KFS 布局、队伍和底盘设置。

### 发送面板

发送按钮独立为左侧下方面板，与工具栏面板保持 12px 间距。选中路径后启用，点击将路径展平为 `Float32MultiArray` 发布到 `/mf_action_seq`。

### 关键代码（与原始版相同部分省略）

**页面间数据传递：**

```javascript
function navigateToEditor(mode) {
    sessionStorage.setItem('mf_path_editor_data', JSON.stringify({
        grid, team: currentTeam, method: currentMethod, mode,
        pathInfo: mode === 'modify' ? { detail, cost, steps, ... } : undefined
    }));
    window.location.href = 'path_editor.html';
}
```

**从编辑器返回时恢复状态：**

```javascript
const data = JSON.parse(sessionStorage.getItem('mf_return_data'));
if (data) { grid = data.grid; currentTeam = data.team; ... publishR2Data(); }
```

### 其他改动

- ROS 连接后不再自动随机摆放 KFS，网格初始为空白
- 路径选中/取消选中联动修改按钮的启用/禁用状态

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

---

## 4. path_editor.html — 手动路径编辑器

### 功能

独立页面，支持通过拖拽交互手动绘制或修改机器人路径。从 `mf_manager.html` 的新建/修改按钮跳转进入。包含移动和抓取两种工具，提供实时动作序列预览和 ROS 发布能力。

### 界面布局

```
┌──────┐   ┌──────────────────┐   ┌──────────────┐
│ 移动 │   │                  │   │ 动作序列      │
│ 抓取 │   │   3×6 网格       │   │ #1 move      │
│ 清空 │   │   (不可编辑KFS)  │   │ (0,1) z=0    │
└──────┘   │                  │   │   yaw=0.00   │
┌──────┐   │  [红队/蓝队]     │   │ #2 move      │
│ 发送 │   │  [单向/全向]     │   │ (1,1) z=400  │
└──────┘   └──────────────────┘   │   yaw=0.00   │
                                  │ ...          │
                                  └──────────────┘
```

左侧双面板：上方编辑工具（移动/抓取/清空），下方发送按钮。右侧路径查看器实时显示动作序列，含每步的坐标、Z/Δh、朝向。

### 两种模式

| 模式 | 入口 | 初始状态 |
|------|------|----------|
| 新建 | mf_manager "新建"按钮 | 空白路径，从 entry 区 (x=0) 开始绘制 |
| 修改 | mf_manager "修改"按钮 | 加载已有路径，在此基础上调整 |

### 移动工具（绿色）

按住并拖拽模式，模仿地铁线路图游戏的操作逻辑。

**从端点延伸（0 或 1 连接）：**
1. 按下 entry 区 `(0,n)` 或路径 tail 端点
2. 相邻可达位置高亮（左/前/右三个方向），显示为绿色虚线命中圈
3. 拖入高亮圈 → 路径延伸，起点更新，高亮刷新
4. 自始至终绘制当前起点到光标的虚线预览

**撤销（端点）：**
- 离开当前起点命中圈 → `dragPhase = left`
- 重新进入当前起点命中圈 → 撤销该步（含附着的 fetch），回退到前驱
- 连续撤销可一路回到 entry

**从中间节点调整（2 连接）：**
1. 按下路径中间某节点
2. 计算前驱后继 ∩ 后驱前驱的公共可达位置，高亮
3. 拖入公共位置 → 节点坐标替换为新高亮格

**可达性约束：** 假 KFS 标记格始终不可达；高亮格自动过滤已有节点避免重叠。

### 抓取工具（黄色）

**创建 fetch：**
1. 按下任意绿色 move 节点
2. 相邻可达位置高亮为黄色命中圈
3. 拖入高亮圈 → 创建 fetch 并立即完成（无需等到松手）

**删除 fetch：**
1. 按下已有 fetch 终点
2. 拖动离开其命中圈再次进入 → fetch 被删除

**可达性约束：** 假 KFS 格不可达；同一位置不可重复创建 fetch。

### 清空工具（灰色）

点击清除全部自定义路径和 entry 点，恢复空白状态。

### 路径数据模型

```
moveNodes: [{id, x, y, z, yaw, prevId, nextId, fetchIds[]}]
fetchNodes: [{id, x, y, z, yaw, parentMoveId}]
```

- Move 节点组成双向链表（prevId/nextId），形成路径主干
- Fetch 节点挂在 move 节点上（parentMoveId），为叶子
- Entry 点作为链首 move 节点（prevId=null, x=0）存入路径

### 动作序列生成 `buildActionSequence()`

遍历 move 链，为每步计算实际参数：

| 参数 | move | fetch |
|------|------|-------|
| `[0]` action | 0 | 1 |
| `[1],[2]` (x,y) | 目标格子 | 目标格子 |
| `[3]` z/Δh | Z = `getHeight(x,y)` | Δh = 目标高度 - 当前高度 |
| `[4]` yaw | 全向=0，单向=`atan2(Δy,Δx)` | 始终=`atan2(Δy,Δx)` |

高度数据来自红/蓝队高度数组（已做上下左右翻转对齐屏幕坐标系）。

### 路径查看器

右侧面板实时展示当前路径的动作序列，格式与 `mf_manager` 一致：
- 绿色标签 `move` — 坐标 + z + yaw
- 黄色标签 `fetch` — 坐标 + Δh + yaw

路径每步变化（拖拽推进/撤销/创建 fetch 等）后自动刷新。

### 发送工具

点击"发送"按钮将当前自定义路径发布到 `/mf_action_seq`。发布格式与 `mf_manager` 完全一致（`Float32MultiArray` + `layout.dim`）。按钮仅在 ROS 连接且路径非空时启用。

### 与 mf_manager 的数据流

```
mf_manager ──sessionStorage──→ path_editor
  mf_path_editor_data: {grid, team, method, mode, pathInfo?}

path_editor ──sessionStorage──→ mf_manager
  mf_return_data: {grid, team, method}
```

- 进入编辑器：传入 KFS 布局（只读展示）、队伍/底盘信息（背景色）
- 返回管理页：回传 KFS 布局、队伍/底盘信息，mf_manager 自动恢复并发布 `/mf_r2_data`

### 高度数组

```javascript
// 红队（已翻转对齐屏幕坐标系）
const HEIGHT_RED = [
    [0,   0,   0],    // screen row 0 (x=5, 出口区)
    [200, 400, 200],  // screen row 1 (x=4)
    [400, 600, 400],  // screen row 2 (x=3)
    [200, 400, 600],  // screen row 3 (x=2)
    [400, 200, 400],  // screen row 4 (x=1)
    [0,   0,   0],    // screen row 5 (x=0, 入口区)
];
// 蓝队同上，row 3 为 [600, 400, 200]
```

### 坐标映射（与 mf_manager 一致）

```
col = 2 - y,  row = 5 - x
px = PAD + col * (CELL + GAP) + CELL/2
py = PAD + row * (CELL + GAP) + CELL/2
```

命中半径 `HIT_RADIUS = 35px`（约 1/3 格宽），进入该圆范围即触发命中。
