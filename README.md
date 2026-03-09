# Triple Map Manager - KFS系统

一个基于ROS2的三重地图管理系统，支持Web界面交互式KFS标记放置和可视化。

## 🚀 Quick Start（快速开始）

### 1️⃣ Clone 下载

```bash
cd ~/ros2_ws/src
git clone https://github.com/fynngwu/triple_map_manager.git
```

### 2️⃣ 安装依赖

```bash
# ROS2 依赖（以 Humble 为例）
sudo apt install ros-humble-rclpy ros-humble-nav-msgs ros-humble-std-msgs \
  ros-humble-visualization-msgs ros-humble-geometry-msgs \
  ros-humble-rosbridge-suite

# Python 依赖
pip3 install "numpy<2" opencv-python pyyaml

# Web界面额外依赖（可选，控制台版本不需要）
sudo apt install xdg-utils firefox -y
```

### 3️⃣ 构建包

```bash
cd ~/ros2_ws
colcon build --packages-select triple_map_manager --symlink-install
```

### 4️⃣ 启动

**推荐：控制台版本（无需Web界面，适合WSL2）**
```bash
source install/setup.bash
ros2 launch triple_map_manager kfs_console.launch.py
```

**完整版本（带Web界面）**
```bash
source install/setup.bash
ros2 launch triple_map_manager kfs_direct.launch.py
# 浏览器访问 http://localhost:9090
```

---

## 📋 项目概述

### 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                    triple_map_manager                        │
├─────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐       │
│  │   Map 1      │  │   Map 2      │  │   Map 3      │       │
│  │  (基础地图)   │  │  (KFS网格)   │  │  (路径规划)   │       │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘       │
│         │                 │                 │               │
│         └─────────────────┼─────────────────┘               │
│                           ▼                                 │
│              ┌─────────────────────┐                        │
│              │   Web Interface     │                        │
│              │   (JavaScript/HTML) │                        │
│              └─────────────────────┘                        │
└─────────────────────────────────────────────────────────────┘
```

### 当前实现方式

- **Map 1**: 基础障碍物地图（OccupancyGrid）
- **Map 2**: KFS标记网格地图（4行×3列固定网格）
- **Map 3**: 路径规划地图
- **Web界面**: 基于rosbridge的JavaScript前端（非原生Qt）

---

## 🔧 架构改进计划

### 当前问题

1. **界面不友好**: 使用Web/JavaScript界面而非原生Qt，启动复杂，依赖浏览器
2. **硬编码坐标**: 地图使用绝对坐标系统，需要手动指定每个点的精确坐标
3. **缺乏灵活性**: 无法通过配置文件简单定义地图区域

### 建议改进方案

#### 1. 原生Qt界面（高优先级）

建议开发基于PyQt5/6的原生ROS2 RQT插件：

```python
# 建议的RQT插件结构
rqt_kfs_manager/
├── plugin.xml
├── setup.py
├── src/
│   └── rqt_kfs_manager/
│       ├── __init__.py
│       ├── kfs_manager_plugin.py  # 主插件类
│       ├── kfs_grid_widget.py     # 网格可视化组件
│       └── kfs_control_panel.py   # 控制面板
```

**优势**:
- ✅ 原生集成RViz2
- ✅ 无需浏览器
- ✅ 更稳定，更适合现场部署
- ✅ 更好的ROS2话题可视化

#### 2. 相对坐标配置系统（高优先级）

建议改为YAML配置文件定义地图，支持相对坐标：

```yaml
# config/map_config.yaml 建议格式
map_definition:
  # 起点位置（左下角或中心点）
  origin:
    x: 0.0
    y: 0.0
    frame_id: "map"

  # 区域定义 - 使用长宽而非绝对坐标
  zones:
    kfs_zone:
      type: "grid"
      # 从原点偏移
      offset_x: 2.0
      offset_y: 1.0
      # 网格尺寸
      rows: 4
      cols: 3
      cell_width: 1.0   # 每个格子1米宽
      cell_height: 1.0  # 每个格子1米高
      # 自动计算：总宽 = cols * cell_width, 总高 = rows * cell_height

    obstacle_zone:
      type: "polygon"
      offset_x: 0.0
      offset_y: 0.0
      points:
        - {x: 0, y: 0}
        - {x: 10, y: 0}
        - {x: 10, y: 5}
        - {x: 0, y: 5}

# 旧系统对比：
# 当前需要：手动计算每个格子的精确坐标
# 建议系统：只需定义起点 + 长宽，自动生成网格
```

**配置文件示例**:

```yaml
# config/grid_config.yaml
grid:
  # 方式1：从起点定义
  origin: {x: 0.0, y: 0.0}
  cell_size: {width: 1.0, height: 1.0}
  dimensions: {rows: 4, cols: 3}
  # 自动计算每个cell的坐标

  # 方式2：预定义特殊位置（可选）
  special_cells:
    - row: 0
      col: 0
      label: "start_zone"
      allowed_types: ["kfs1"]
    - row: 3
      col: 2
      label: "end_zone"
      allowed_types: ["kfs2", "fake"]
```

#### 3. 渐进式改进计划

| 阶段 | 优先级 | 改进项 | 预计工作量 | 状态 |
|------|--------|--------|-----------|------|
| 1 | 🔴 高 | YAML配置文件支持相对坐标 | 2-3天 | 📋 计划中 |
| 2 | 🔴 高 | 地图配置热重载 | 1-2天 | 📋 计划中 |
| 3 | 🟡 中 | Qt基础界面（显示网格状态） | 3-4天 | 📋 计划中 |
| 4 | 🟡 中 | Qt交互界面（点击放置KFS） | 2-3天 | 📋 计划中 |
| 5 | 🟢 低 | 移除Web界面依赖 | 1天 | 📋 计划中 |

---

## 📦 依赖详情

**依赖库说明**：

- **ros-humble-rclpy**：ROS2 Python 客户端库，用于编写 ROS2 节点和服务
- **ros-humble-nav-msgs**：导航相关消息类型（如 OccupancyGrid），用于地图数据
- **ros-humble-std-msgs**：标准消息类型（String, Int32等）
- **ros-humble-visualization-msgs**：可视化消息（Marker, MarkerArray），用于在 RViz 中显示标记
- **ros-humble-geometry-msgs**：几何消息类型（Point, Pose等）
- **ros-humble-rosbridge-suite**：WebSocket 桥接套件，让 Web 界面与 ROS2 通信
- **python3-numpy**：Python 数值计算库，用于数组操作
- **python3-opencv**：OpenCV 计算机视觉库，用于图像处理
- **python3-yaml**：YAML 解析库，用于读取配置文件

---

## 📡 发布的话题

### 消息类型

本包发布的主要话题：

| 话题名称 | 消息类型 | 用途 |
|---------|---------|------|
| `/map1` | `nav_msgs/OccupancyGrid` | 第一个 occupancy grid 地图 |
| `/map2` | `nav_msgs/OccupancyGrid` | 第二个 occupancy grid 地图（用于 KFS 网格标记） |
| `/map3` | `nav_msgs/OccupancyGrid` | 第三个 occupancy grid 地图 |
| `/kfs_grid_data` | `std_msgs/String` | KFS 网格数据话题（由 Web 界面发布） |
| `/map2_kfs_markers` | `visualization_msgs/MarkerArray` | KFS 标记可视化 |

### OccupancyGrid 使用方式

**nav_msgs/OccupancyGrid** 消息类型用于表示占用栅格地图，常用于导航、SLAM等场景。

#### 常用操作：

```bash
# 1. 查看地图话题信息
ros2 topic info /map1

# 2. 查看地图消息内容
ros2 topic echo /map1 --once

# 3. 实时监听地图更新
ros2 topic echo /map1

# 4. 查看地图类型和数据类型
ros2 interface show nav_msgs/msg/OccupancyGrid
```

#### 在代码中使用：

```python
from nav_msgs.msg import OccupancyGrid

def map_callback(msg: OccupancyGrid):
    # 地图分辨率（米/像素）
    resolution = msg.info.resolution
    
    # 地图原点
    origin_x = msg.info.origin.position.x
    origin_y = msg.info.origin.position.y
    
    # 地图尺寸
    width = msg.info.width    # 像素宽度
    height = msg.info.height  # 像素高度
    
    # 占用数据：-1=未知, 0-100=占用概率
    data = msg.data  # 一维数组
```

### KFS Grid Data 消息格式

**`/kfs_grid_data`** 话题发布 `std_msgs/String` 类型消息，内容为 JSON 格式：

```json
{
  "grid": [
    [0, 0, 0],
    [1, 0, 2],
    [0, 3, 0],
    [0, 1, 0]
  ],
  "timestamp": 1698234567890
}
```

#### 消息含义：

- **`grid`**：4×3 二维数组，表示 KFS 网格的标记布局
  - `0` = 空单元格
  - `1` = KFS1（蓝色标记）
  - `2` = KFS2（红色标记）
  - `3` = KFS Fake（灰色标记）
- **`timestamp`**：时间戳（毫秒）

#### 网格布局：

```
        Col 0    Col 1    Col 2
Row 0   [0,0]    [0,1]    [0,2]
Row 1   [1,0]    [1,1]    [1,2]
Row 2   [2,0]    [2,1]    [2,2]
Row 3   [3,0]    [3,1]    [3,2]
```

#### 使用示例：

```bash
# 监听 KFS 网格数据
ros2 topic echo /kfs_grid_data

# 查看消息类型
ros2 topic info /kfs_grid_data
ros2 interface show std_msgs/msg/String
```

### 配置文件

所有地图配置和障碍物定义在 `config/` 目录中，详见 [配置教程](doc/CONFIG_TUTORIAL.md)。

---

## 🎮 Launch 文件详情

本包提供了多个 launch 文件，位于 `launch/` 目录：

| Launch 文件 | 路径 | 说明 |
|------------|------|------|
| `kfs_console.launch.py` | `launch/kfs_console.launch.py` | 控制台版本，无需 Web 界面 |
| `kfs_direct.launch.py` | `launch/kfs_direct.launch.py` | 完整版本，包含 Web 界面和 rosbridge |

### 使用控制台版本（推荐）

```bash
source install/setup.bash
ros2 launch triple_map_manager kfs_console.launch.py
```

系统会自动完成以下操作：
1. **随机放置 KFS 标记**：3个 KFS1（蓝色）、4个 KFS2（红色）、1个 KFS Fake（灰色）
2. **发布到 ROS2**：自动发布 `/kfs_grid_data` 话题
3. **显示网格状态**：在终端中显示 ASCII 艺术网格布局
4. **保持运行**：节点保持活跃状态，持续监听 ROS2 系统

**优势**：
- ✅ 零配置，自动随机放置
- ✅ 适合 WSL2 环境
- ✅ 无需用户交互
- ✅ 自动遵循放置规则

**放置规则**：
- KFS1: 只能放在左/右列（第0列或第2列），最多3个
- KFS2: 不能放在第0行，最多4个
- KFS Fake: 不能放在第0行，最多1个

**查看结果**：在 RViz 中查看可视化标记

### 使用完整版本（带 Web 界面）

```bash
#下载依赖
sudo apt install xdg-utils firefox -y
pip3 install "numpy<2"
source install/setup.bash
ros2 launch triple_map_manager kfs_direct.launch.py
```

此版本包含：
- ✅ 所有控制台版本的功能
- ✅ Web 界面（通过 rosbridge）
- ✅ 交互式 KFS 标记放置
- ✅ 实时可视化更新

**Web 界面访问**：启动后访问 `http://localhost:9090` 打开 Web 界面

---

详细的项目信息（系统架构、文件结构、技术特性等）请参阅 [项目详情](doc/PROJECT_DETAILS.md)。
