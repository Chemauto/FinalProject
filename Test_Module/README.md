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
Test_Module 模块详细文档
====================================

# Test_Module - 追击功能模块

本模块包含机器人追击功能的核心算法和测试仿真器。

## 📁 模块结构

```
Test_Module/
├── README.md              # 本文档
├── chase_core.py          # 追击算法核心
├── enemy_manager.py       # 敌人管理器
└── chase_simulator.py     # 追击仿真器（简化版）
```

## 🎯 功能特性

- **自动追击**: 计算最近敌人并自动追击
- **PID控制**: 距离越近步长越小，精确定位
- **角度计算**: 自动计算旋转角度和方向
- **实时更新**: 通过 ROS2 话题获取实时位置
- **自动清除**: 追击完成后自动清除敌人

## 📦 核心组件

### 1. chase_core.py - 追击算法核心

提供追击控制算法，包括角度计算、距离计算、PID步长控制等。

**主要类：**
```python
class ChaseController:
    """追击控制器"""

    def calculate_target_angle(robot_x, robot_y, target_x, target_y):
        """计算目标角度（度）"""

    def calculate_angle_difference(current_angle, target_angle):
        """计算角度差（-180° 到 180°）"""

    def calculate_distance(robot_x, robot_y, target_x, target_y):
        """计算直线距离（像素）"""

    async def chase_step(robot_pos, target, progress_callback=None):
        """执行单步追击（旋转 + 前进）"""

    async def chase_target(target, progress_callback=None):
        """完整追击流程，直到到达目标"""
```

**关键参数：**
```python
robot_getter: Callable         # 获取机器人位置的函数
move_forward_fn: Callable      # 前进函数
turn_fn: Callable              # 旋转函数
max_steps: int = 10            # 最大步数限制
step_distance: float = 0.5     # 每步移动距离（米）
arrival_threshold: float = 20  # 到达阈值（像素）
```

**使用示例：**
```python
from Test_Module.chase_core import ChaseController

# 创建控制器
controller = ChaseController()

# 计算追击角度
target_angle = controller.calculate_target_angle(400, 300, 600, 200)
angle_diff = controller.calculate_angle_difference(0, target_angle)

# 计算距离
distance = controller.calculate_distance(400, 300, 600, 200)

print(f"目标角度: {target_angle}°, 需要旋转: {angle_diff}°, 距离: {distance}px")
```

### 2. enemy_manager.py - 敌人管理器

管理敌人的生成、移动、删除和绘制。

**主要类：**
```python
class Enemy:
    """敌人类"""
    def __init__(self, enemy_id, x, y, move_mode="static"):
        """
        Args:
            enemy_id: 敌人ID
            x, y: 初始位置
            move_mode: 移动模式 ("static", "random", "chase")
        """

    def update(robot_pos):
        """更新敌人位置"""

    def draw(screen):
        """绘制敌人"""


class EnemyManager:
    """敌人管理器"""

    def spawn_enemy(x=None, y=None, move_mode="static"):
        """生成新敌人"""

    def remove_enemy(enemy_id):
        """删除指定敌人"""

    def get_all_enemies():
        """获取所有敌人列表"""

    def get_nearest_enemy(robot_pos):
        """获取最近的敌人"""

    def clear_all():
        """清除所有敌人"""

    def update(robot_pos):
        """更新所有敌人"""

    def draw_all(screen):
        """绘制所有敌人"""

    def draw_line_to_robot(screen, robot_pos, enemy_id, color):
        """绘制从机器人到敌人的连线"""
```

**使用示例：**
```python
from Test_Module.enemy_manager import EnemyManager

# 创建敌人管理器
manager = EnemyManager(bounds=(800, 600))

# 生成静态敌人
enemy1 = manager.spawn_enemy(x=600, y=200, move_mode="static")
enemy2 = manager.spawn_enemy(x=200, y=400, move_mode="random")

# 获取最近敌人
robot_pos = {'x': 400, 'y': 300}
nearest = manager.get_nearest_enemy(robot_pos)
print(f"最近的敌人: {nearest.id} at ({nearest.x}, {nearest.y})")

# 删除敌人
manager.remove_enemy(enemy1.id)

# 绘制（在 Pygame 循环中）
manager.update(robot_pos)
manager.draw_all(screen)
```

### 3. chase_simulator.py - 追击仿真器

简化版 2D 仿真器，专门用于测试追击功能。

**主要类：**
```python
class ChaseSimulator:
    """追击仿真器"""

    def __init__(self):
        """初始化仿真器"""

    def handle_events():
        """处理事件（鼠标、键盘）"""

    def spawn_enemy_at(x, y):
        """在指定位置生成敌人"""

    def clear_enemies():
        """清除所有敌人"""

    def toggle_chase_line():
        """切换追击线显示"""

    def update():
        """更新状态（机器人、敌人、ROS通讯）"""

    def draw():
        """绘制场景"""

    def run():
        """运行仿真器主循环"""
```

**操作说明：**
- **鼠标左键**: 在点击位置生成敌人
- **按 C**: 清除所有敌人
- **按 L**: 切换追击线显示
- **按 ESC**: 退出

**ROS2 通讯：**
仿真器通过以下 ROS2 话题与追击模块通信：
- `/robot/command` - 接收动作命令
- `/robot/state` - 发布机器人状态
- `/robot/enemies` - 发布敌人位置
- `/robot/enemy_remove` - 接收敌人清除命令

**启动方式：**
```bash
python3 Test_Module/chase_simulator.py
```

## 🚀 快速开始

### 1. 启动追击仿真器

```bash
# 终端1: 启动仿真器
python3 Test_Module/chase_simulator.py
```

### 2. 在交互程序中使用追击功能

```bash
# 终端2: 启动交互程序
python3 Interactive_Module/interactive.py

# 输入命令
追击敌人
```

### 3. 直接使用追击算法

```python
import asyncio
from Test_Module.chase_core import ChaseController
from ros_topic_comm import get_robot_state, get_shared_queue

async def chase_example():
    # 获取 ROS 队列
    queue = get_shared_queue()

    # 创建控制器
    controller = ChaseController(
        robot_getter=get_robot_state,
        move_forward_fn=lambda dist, speed: queue.put({
            'action': 'move_forward',
            'parameters': {'distance': dist, 'speed': speed}
        }),
        turn_fn=lambda angle, speed: queue.put({
            'action': 'turn',
            'parameters': {'angle': angle, 'angular_speed': speed}
        })
    )

    # 追击目标
    target = {'x': 600, 'y': 200}
    result = await controller.chase_target(target)
    print(result)

asyncio.run(chase_example())
```

## 🔧 参数配置

### ChaseController 参数

```python
max_steps = 10              # 最大步数（防止无限循环）
step_distance = 0.5         # 默认步长（米）
arrival_threshold = 20.0    # 到达阈值（像素）
```

调整建议：
- **提高精度**: 减小 `arrival_threshold`（如 5.0）
- **提高速度**: 增大 `step_distance`（如 1.0）
- **防止超时**: 增大 `max_steps`（如 20）

### Robot_Module 参数

在 `Robot_Module/module/chase.py` 中：
```python
MAX_STEP_DISTANCE = 0.5     # 最大步长（米）
ARRIVAL_THRESHOLD = 5.0     # 到达阈值（像素）
```

## 📊 追击算法详解

### 1. 角度计算

使用 `atan2` 计算目标方向：
```python
dx = target_x - robot_x
dy = target_y - robot_y
target_angle = math.degrees(math.atan2(dy, dx))
```

### 2. 角度差计算

标准化到 [-180°, 180°]：
```python
angle_diff = target_angle - current_angle
angle_diff = (angle_diff + 180) % 360 - 180
```

- `angle_diff > 0`: 左转
- `angle_diff < 0`: 右转
- `angle_diff = 0`: 已对准

### 3. PID 步长控制

```python
def calculate_step_distance(current_distance_pixels):
    distance_meters = current_distance_pixels / 100.0
    step_distance = min(distance_meters * 0.8, MAX_STEP_DISTANCE)
    step_distance = max(step_distance, 0.1)  # 最小 0.1 米
    return step_distance
```

特性：
- 距离越远 → 步长越大（最大 0.5 米）
- 距离越近 → 步长越小（最小 0.1 米）
- 避免超调，精确定位

### 4. 追击流程

```
1. 获取机器人位置
2. 获取敌人位置列表
3. 找到最近的敌人
4. 计算目标角度和角度差
5. 旋转到目标方向
6. 循环前进直到到达：
   - 获取当前距离
   - 使用 PID 计算步长
   - 前进
   - 更新位置
   - 检查是否到达（< 5 像素）
7. 清除敌人
```

## 🐛 调试技巧

### 1. 查看追击日志

```bash
# 追击时会输出详细日志
[chase.chase_enemy] 机器人位置: (400.0, 300.0), 角度: 0.0°
[chase.chase_enemy] 追击目标: 1 at (612, 114)
[chase.chase_enemy] 距离: 282.0 像素 (2.82 米)
[chase.chase_enemy] 目标角度: 41.3°, 需要旋转: 41.3°
[chase.chase_enemy] 步骤2.1: 当前距离 282.0 像素
[chase.chase_enemy]  前进 0.50 米 (PID控制)
```

### 2. 监控 ROS2 话题

```bash
# 查看敌人位置
ros2 topic echo /robot/enemies

# 查看机器人状态
ros2 topic echo /robot/state

# 查看动作命令
ros2 topic echo /robot/command
```

### 3. 常见问题

**问题1**: 找不到敌人
- 检查仿真器是否启动
- 检查是否已生成敌人（鼠标点击）
- 检查 `/robot/enemies` 话题是否有数据

**问题2**: 机器人不移动
- 检查 `/robot/command` 话题是否发布命令
- 检查仿真器是否订阅了 `/robot/command`

**问题3**: 追击不精确
- 减小 `ARRIVAL_THRESHOLD`（如 5.0 → 3.0）
- 减小 `MAX_STEP_DISTANCE`（如 0.5 → 0.3）
- 增大 `max_steps`（如 10 → 20）

## 📝 依赖

```
pygame>=2.5.0         # 2D 仿真可视化
numpy>=1.26.0         # 数值计算
rclpy                 # ROS2 Python 客户端库
```

## 🔗 相关模块

- `Robot_Module/module/chase.py` - 追击功能 MCP 工具
- `ros_topic_comm.py` - ROS2 通讯模块
- `Sim_Module/sim2d/simulator.py` - 主仿真器

---

**追击算法，精确制导！** 🎯
