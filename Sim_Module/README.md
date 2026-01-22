# 🤖 FinalProject - 基于双层LLM的机器人控制系统

基于双层 LLM 架构的机器人控制系统，采用 MCP (Model Context Protocol) 模块化设计，使用 **ROS2 话题**进行通信。

## 📑 目录

- [项目概述](#-项目概述)
- [快速开始](#-快速开始)
- [Interactive_Module - 交互界面](#interactive_module---交互界面)
- [LLM_Module - 双层LLM核心](#llm_module---双层llm核心)
- [Robot_Module - MCP工具注册中心](#robot_module---mcp工具注册中心)
- [Sim_Module - 2D仿真环境](#sim_module---2d仿真环境)
- [Test_Module - 追击功能](#test_module---追击功能)
- [VLM_Module - 视觉语言模型](#vlm_module---视觉语言模型)
- [Yolo_Module - YOLO目标检测](#yolo_module---yolo目标检测)
- [ROS2通讯](#-ros2通讯)
- [依赖安装](#-依赖安装)

---

## 🎯 项目概述

### 核心特性

- **双层 LLM 架构**: 任务规划 + 执行控制分离
- **MCP 模块化设计**: 参考 RoboOS，工具注册标准化
- **ROS2 通讯**: 使用 ROS2 话题，标准化通信
- **自然语言控制**: 支持中文指令控制机器人
- **2D 仿真**: 基于 Pygame 的轻量级仿真环境
- **追击功能**: 自动追击敌人，PID控制精确定位
- **视觉检测**: VLM 颜色检测与动作映射
- **懒加载设计**: 自动初始化，零配置

### 系统架构

```
用户输入: "追击敌人" 或 "前进1米然后左转90度"
    ↓
┌─────────────────────────────────────────────┐
│ ① Interactive_Module (交互界面)             │
│    - 接收用户自然语言指令                     │
│    - 协调 LLM 和 Robot_Module                │
│    - 显示执行结果                             │
└──────────────┬──────────────────────────────┘
               ↓
┌─────────────────────────────────────────────┐
│ ② LLM_Module - 上层LLM (任务规划)           │
│    输入: 用户指令 + 可用技能列表              │
│    输出: 子任务序列                           │
└──────────────┬──────────────────────────────┘
               ↓
┌─────────────────────────────────────────────┐
│ ② LLM_Module - 下层LLM (执行控制)           │
│    输入: 单个子任务描述                       │
│    输出: 工具调用                             │
└──────────────┬──────────────────────────────┘
               ↓
┌─────────────────────────────────────────────┐
│ ③ Robot_Module (MCP 工具注册中心)           │
│    ├─ skill.py: FastMCP 服务器              │
│    ├─ module/base.py: 移动/旋转/停止技能    │
│    ├─ module/chase.py: 追击技能             │
│    └─ module/vision.py: 视觉检测技能        │
└──────────────┬──────────────────────────────┘
               ↓
      ROS2 Topic (/robot/command, /robot/enemies, /robot/state)
         ↓ 发布 JSON 消息
┌─────────────────────────────────────────────┐
│ ④ Sim_Module (2D 仿真器)                    │
│    ├─ 订阅 ROS2 话题                         │
│    ├─ 机器人运动可视化                       │
│    ├─ 敌人管理 (spawn/move/remove)           │
│    └─ 状态发布 (robot/enemy positions)       │
└─────────────────────────────────────────────┘
```

### 项目结构

```
FinalProject/
├── README.md              # 本文档
├── requirements.txt       # Python 依赖
├── .env                   # API 密钥配置
│
├── Interactive_Module/    # 交互界面
│   └── interactive.py     # CLI 交互主程序
│
├── LLM_Module/            # 双层LLM核心
│   ├── llm_core.py        # LLMAgent: 规划 + 执行
│   └── prompts/           # YAML 提示词模板
│       └── planning_prompt_2d.yaml
│
├── Robot_Module/          # MCP 工具注册中心
│   ├── skill.py           # FastMCP 服务器入口
│   └── module/            # 功能模块
│       ├── base.py        # 底盘控制 (move/turn/stop)
│       ├── chase.py       # 追击功能 (chase_enemy)
│       └── vision.py      # 视觉检测 (detect_color)
│
├── Sim_Module/            # 仿真模块
│   └── sim2d/
│       └── simulator.py   # 主仿真器
│
├── Test_Module/           # 测试模块
│   ├── chase_core.py      # 追击算法核心
│   ├── enemy_manager.py   # 敌人管理器
│   └── chase_simulator.py # 追击仿真器 (简化版)
│
├── VLM_Module/            # 视觉语言模型
│   ├── vlm_core.py        # 本地 VLM (Ollama)
│   ├── vlm_core_remote.py # 远程 VLM (API)
│   └── prompts/           # VLM 提示词
│
├── Yolo_Module/           # YOLO目标检测
│   ├── target_detector.py # 目标检测器
│   ├── screen_capture.py  # 屏幕捕获
│   └── coordinate_mapper.py # 坐标映射
│
└── ros_topic_comm.py      # ROS2 通讯模块
```

---

## 🚀 快速开始

### 环境要求

- Python 3.10+
- Linux (推荐 Ubuntu 22.04)
- ROS2 Humble (可选，用于调试)

### 安装

```bash
# 1. 进入项目目录
cd /home/xcj/work/testfinal/FinalProject

# 2. 安装 Python 依赖
pip install -r requirements.txt

# 3. 配置 API Key
export Test_API_KEY=your_api_key_here

# 或创建 .env 文件
echo "Test_API_KEY=your_api_key_here" > .env
```

### 运行

```bash
# 方式1: 一键启动（推荐）
./start_robot_system.sh

# 方式2: 手动启动
# 终端1: 启动仿真器
python3 Sim_Module/sim2d/simulator.py

# 终端2: 启动交互程序
python3 Interactive_Module/interactive.py
```

### 交互示例

```bash
💬 请输入指令: 前进1米然后左转90度

[上层LLM] 任务规划:
  步骤1: 前进1米
  步骤2: 左转90度

[下层LLM] 执行控制:
  调用工具: move_forward(distance=1.0) ✅
  调用工具: turn(angle=90.0) ✅

📊 [完成] 2/2 个任务成功
```

---


====================================
各模块详细文档
====================================

## Interactive_Module - 交互界面
# Interactive_Module - 交互界面模块

用户交互界面的核心模块，提供 CLI 命令行交互功能，协调 LLM 和 Robot_Module。

## 📁 模块结构

```
Interactive_Module/
├── README.md              # 本文档
└── interactive.py         # CLI 交互主程序
```

## 🎯 功能特性

- **自然语言交互**: 支持中文指令输入

====================================
Sim_Module 模块详细文档
====================================

# Sim_Module - 2D 仿真模块

基于 Pygame 的 2D 机器人仿真环境，提供可视化界面和物理模拟。

## 📁 模块结构

```
Sim_Module/
├── README.md              # 本文档
└── sim2d/                 # 2D 仿真
    └── simulator.py       # 主仿真器
```

## 🎯 功能特性

- **2D 可视化**: 基于 Pygame 的图形界面
- **机器人模拟**: 移动、旋转、物理运动
- **敌人管理**: 生成、移动、删除敌人
- **ROS2 通讯**: 订阅动作命令，发布状态
- **实时更新**: 60 FPS 平滑动画
- **中文支持**: 完整的中文字体支持

## 🔧 核心组件

### simulator.py - 主仿真器

**主要类：**

```python
class Robot:
    """机器人类"""
    def __init__(self, x, y):
        """
        Args:
            x, y: 初始位置
        """
        self.x = x              # 当前 X 坐标
        self.y = y              # 当前 Y 坐标
        self.angle = 0.0        # 当前角度（度）
        self.speed = 0.3        # 移动速度（m/s）
        self.angular_speed = 0.5 # 旋转角速度（rad/s）

    def update(self):
        """更新机器人状态（平滑动画）"""

    def draw(self, screen):
        """绘制机器人"""

    def execute_action(self, action):
        """执行动作命令"""


class Simulator:
    """仿真器主类"""
    def __init__(self):
        """初始化仿真器"""

    def handle_events(self):
        """处理事件（鼠标、键盘）"""

    def update(self):
        """更新状态"""

    def draw(self):
        """绘制场景"""

    def run(self):
        """运行仿真器主循环"""
```

**仿真器配置：**

```python
WIDTH = 800          # 屏幕宽度（像素）
HEIGHT = 600         # 屏幕高度（像素）
FPS = 60             # 帧率
PIXELS_PER_METER = 100  # 1米 = 100像素
```

## 🎮 操作说明

### 键盘控制

- **ESC**: 退出仿真器
- **鼠标左键**: 在点击位置生成敌人（部分模式）

### 机器人运动

**前进 (move_forward):**
```json
{
  "action": "move_forward",
  "parameters": {
    "distance": 1.0,    // 距离（米）
    "speed": 0.3        // 速度（m/s）
  }
}
```

**后退 (move_backward):**
```json
{
  "action": "move_backward",
  "parameters": {
    "distance": 1.0,
    "speed": 0.3
  }
}
```

**旋转 (turn):**
```json
{
  "action": "turn",
  "parameters": {
    "angle": 90.0,           // 角度（度）
    "angular_speed": 0.5     // 角速度（rad/s）
  }
}
```

**停止 (stop):**
```json
{
  "action": "stop",
  "parameters": {}
}
```

## 🔄 ROS2 通讯

### 订阅的话题

| 话题名称 | 消息类型 | 用途 |
|---------|---------|------|
| `/robot/command` | String | 接收动作命令 |

### 发布的话题

| 话题名称 | 消息类型 | 用途 |
|---------|---------|------|
| `/robot/state` | String | 发布机器人状态 |
| `/robot/enemies` | String | 发布敌人位置 |

### 消息格式

**机器人状态:**
```json
{
  "x": 400.0,
  "y": 300.0,
  "angle": 45.0
}
```

**敌人位置:**
```json
[
  {"id": "1", "x": 600, "y": 200},
  {"id": "2", "x": 200, "y": 400}
]
```

## 🚀 快速开始

### 1. 启动仿真器

```bash
# 终端1: 启动仿真器
python3 Sim_Module/sim2d/simulator.py
```

### 2. 查看可视化

仿真器窗口会显示：
- 网格背景（1米网格）
- 机器人（三角形箭头）
- 敌人（圆形）
- 状态信息（位置、角度）

### 3. 发送命令

```bash
# 终端2: 启动交互程序
python3 Interactive_Module/interactive.py

# 输入命令
前进1米
左转90度
```

## 📊 物理模拟

### 坐标系统

```
(0, 0) ────────────────── (800, 0)
  │                         │
  │      (400, 300)         │
  │          ●             │
  │       Robot            │
  │                         │
(0, 600) ────────────────── (600, 600)
```

- 原点 (0, 0) 在左上角
- X 轴向右增长
- Y 轴向下增长
- 角度 0° 指向右，90° 指向下

### 运动学模型

**直线运动:**
```
新位置 = 旧位置 + 方向向量 × 距离

x_new = x_old + cos(angle) × distance
y_new = y_old + sin(angle) × distance
```

**旋转运动:**
```
角度更新采用平滑插值

当前角度 = 当前角度 + (目标角度 - 当前角度) × 速度因子
```

**时间计算:**
```
前进时间 = 距离 / 速度
旋转时间 = 角度 / 角速度
```

## 🎨 可视化元素

### 机器人

- **形状**: 三角形箭头
- **颜色**: 蓝色
- **方向**: 箭头指向当前朝向
- **大小**: 半径 20 像素

### 敌人

- **形状**: 圆形
- **颜色**: 红色
- **ID**: 显示在敌人上方
- **大小**: 半径 15 像素

### 网格

- **间距**: 100 像素（1米）
- **颜色**: 浅灰色
- **线宽**: 1 像素

### 信息面板

显示内容：
```
机器人: (400.0, 300.0)  角度: 0.0°
敌人数量: 2  选中: 1
[鼠标]生成 [C]清除 [L]连线 [ESC]退出
```

## 🔧 配置参数

### 修改屏幕尺寸

```python
# simulator.py
WIDTH = 1000   # 修改宽度
HEIGHT = 800   # 修改高度
```

### 修改比例尺

```python
# simulator.py
PIXELS_PER_METER = 100  # 1米 = 100像素（可修改为 50、200 等）
```

### 修改帧率

```python
# simulator.py
FPS = 60  # 可修改为 30、120 等
```

### 修改机器人参数

```python
# 在 Robot.__init__ 中
self.speed = 0.3          # 默认速度
self.angular_speed = 0.5  # 默认角速度
```

## 🐛 调试技巧

### 1. 查看机器人状态

```python
# 在 update() 中添加
print(f"Robot: x={self.robot.x:.1f}, y={self.robot.y:.1f}, angle={self.robot.angle:.1f}°")
```

### 2. 查看动作命令

```python
# 在 execute_action() 中添加
print(f"[Simulator] 接收命令: {action}")
```

### 3. 查看 ROS2 话题

```bash
# 查看机器人状态
ros2 topic echo /robot/state

# 查看敌人位置
ros2 topic echo /robot/enemies

# 查看动作命令
ros2 topic echo /robot/command
```

### 4. 降低帧率调试

```python
FPS = 10  # 降低帧率，便于观察
```

## 💡 扩展建议

### 添加障碍物

```python
class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

    def draw(self, screen):
        pygame.draw.circle(screen, (128, 128, 128), (self.x, self.y), self.radius)
```

### 添加轨迹显示

```python
class Robot:
    def __init__(self, x, y):
        self.trail = []  # 轨迹点

    def update(self):
        # 记录位置
        self.trail.append((self.x, self.y))
        if len(self.trail) > 100:  # 最多保留100个点
            self.trail.pop(0)

    def draw(self, screen):
        # 绘制轨迹
        if len(self.trail) > 1:
            pygame.draw.lines(screen, (100, 100, 255), False, self.trail, 2)
```

### 添加传感器模拟

```python
class Robot:
    def get_sensor_reading(self, obstacles):
        """获取距离传感器读数"""
        readings = []
        for angle in [-45, 0, 45]:  # 左、中、右
            ray_angle = math.radians(self.angle + angle)
            distance = self.cast_ray(ray_angle, obstacles)
            readings.append(distance)
        return readings
```

## 🔗 相关模块

- `ros_topic_comm.py` - ROS2 通讯模块
- `Robot_Module/module/base.py` - 底盘控制
- `Test_Module/chase_simulator.py` - 追击仿真器

## 📝 依赖

```
pygame>=2.5.0  # 2D 可视化
rclpy          # ROS2 Python 客户端库
```

## 🎯 性能优化

- 使用 `spin_once(timeout_sec=0.001)` 非阻塞 ROS 回调
- 限制敌人数量（建议 < 50）
- 减少不必要的绘制操作
- 使用对象池避免频繁创建/销毁

---

**可视化，易调试！** 🎮
