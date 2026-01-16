# Sim_Module - 仿真模块

## 概述

Sim_Module 提供**2D 机器人仿真环境**，基于 Pygame 实现可视化，通过 **ROS2 话题**接收来自 Robot_Module 的动作指令并执行仿真。

### 核心功能

- **2D 仿真**: 基于 Pygame 的轻量级 2D 机器人仿真
- **实时可视化**: 显示机器人位置、朝向和运动轨迹
- **ROS2 通讯**: 通过 ROS2 话题接收动作指令
- **差速驱动**: 模拟差速驱动机器人运动
- **自动初始化**: ROS队列延迟初始化，确保连接建立

## 文件结构

```
Sim_Module/
├── sim2d/
│   ├── simulator.py      # 2D 仿真器主程序
│   └── README.md         # 本文件
└── README.md
```

## 支持的动作

| 动作 | 参数 | 描述 |
|------|------|------|
| `move_forward` | `distance`, `speed` | 向前移动指定距离 |
| `move_backward` | `distance`, `speed` | 向后移动指定距离 |
| `turn` | `angle`, `angular_speed` | 原地旋转指定角度 |
| `stop` | 无 | 立即停止运动 |

## 数据流

```
Robot_Module (工具函数)
    ↓ 返回动作指令 JSON
    {"action": "move_forward", "parameters": {"distance": 1.0, "speed": 0.3}}
    ↓
ROS2 Topic (/robot/command)
    ↓ 发布消息
Sim2DRobot (仿真器)
    ↓ 订阅并读取消息
action = action_queue.get_nowait()
    ↓ 解析动作
if action['action'] == 'move_forward':
    robot.execute_action(action)
    ↓ 执行仿真
更新机器人状态 + Pygame 可视化
```

## 核心类: Robot

### 机器人状态

```python
# 位置 (x, y) - 单位: 像素
robot.x = 400
robot.y = 300

# 朝向 (angle) - 单位: 度
robot.angle = 0.0  # 初始朝向右

# 机器人尺寸
ROBOT_SIZE = 25    # 25像素半径
```

## 核心方法

### 1. `execute_action()` - 执行动作

```python
def execute_action(self, action):
    """执行动作指令

    Args:
        action: 动作指令字典
            {
                'action': 'move_forward',
                'parameters': {'distance': 1.0, 'speed': 0.3}
            }
    """
    action_type = action.get('action')
    params = action.get('parameters', {})

    if action_type == 'move_forward':
        self._move('front', params)
    elif action_type == 'move_backward':
        self._move('back', params)
    elif action_type == 'turn':
        self._turn(params)
    elif action_type == 'stop':
        self._stop()
```

### 2. 动作执行

**移动**:
```python
def _move(self, direction, params):
    """移动机器人"""
    distance = params.get('distance', 1.0)
    speed = params.get('speed', 0.3)

    # 计算移动时间
    duration = distance / speed if speed > 0 else 0

    # 更新位置
    dx = distance * math.cos(math.radians(self.angle))
    dy = distance * math.sin(math.radians(self.angle))

    if direction == 'front':
        self.x += dx
        self.y += dy
    else:
        self.x -= dx
        self.y -= dy
```

**旋转**:
```python
def _turn(self, params):
    """旋转机器人"""
    angle = params.get('angle', 0.0)
    self.angle += angle
    self.angle %= 360  # 规范化到 0-360
```

## ROS2 通讯

### 初始化流程

```python
from ros_topic_comm import get_shared_queue

# 1. 获取共享队列
action_queue = get_shared_queue()

# 2. 设置订阅者（自动延迟0.2秒确保连接）
action_queue.setup_subscriber()
# [ActionPublisher] ROS话题发布器已创建: /robot/command
# [等待0.2秒确保ROS连接建立]

# 3. 订阅 /robot/command 话题
# [ActionSubscriber] 订阅者已创建: /robot/command
```

**重要**: ROS队列初始化时会添加0.2秒延迟，确保ROS2连接完全建立。这解决了第一次命令发送时仿真器未就绪的问题。

### 话题格式

- **话题名称**: `/robot/command`
- **消息类型**: `std_msgs/String`
- **消息格式**: JSON 字符串

```python
# 消息示例
{"action": "move_forward", "parameters": {"distance": 1.0, "speed": 0.3}}
{"action": "move_backward", "parameters": {"distance": 1.0, "speed": 0.3}}
{"action": "turn", "parameters": {"angle": 90.0, "angular_speed": 0.5}}
{"action": "stop", "parameters": {}}
```

### 调试命令

```bash
# 查看话题列表
ros2 topic list

# 查看话题消息
ros2 topic echo /robot/command

# 查看话题信息
ros2 topic info /robot/command

# 查看节点图
ros2 node list
ros2 topic list
```

## 启动仿真器

### 方法1：通过主脚本

```bash
./start_robot_system.sh

# 选择选项 1：自动启动（包含仿真器）
# 或选项 2：手动启动
```

### 方法2：直接启动

```bash
cd Sim_Module/sim2d
python3 simulator.py
```

## 仿真界面

```
┌────────────────────────────────────┐
│          2D Robot Simulator        │
│                                    │
│         (0,0)                      │
│           ──────────────→ x        │
│           │                       │
│           │ y                     │
│           ↓                       │
│                                    │
│            ●  ← 机器人             │
│           ↗                       │
│                                    │
└────────────────────────────────────┘
```

## 输出示例

```
[ActionSubscriber] 订阅者已创建: /robot/command
[Simulator] 2D仿真器已启动
[Simulator] 等待动作指令...

[ActionSubscriber] 接收命令: {'action': 'move_forward', 'parameters': {'distance': 1.0, 'speed': 0.3}}
[Simulator] 执行动作: {'action': 'move_forward', 'parameters': {'distance': 1.0, 'speed': 0.3}}
[Simulator] 移动: front, 距离=1.0m, 速度=0.3m/s
... ✅ 完成!

[ActionSubscriber] 接收命令: {'action': 'turn', 'parameters': {'angle': 90.0, 'angular_speed': 0.5}}
[Simulator] 执行动作: {'action': 'turn', 'parameters': {'angle': 90.0, 'angular_speed': 0.5}}
[Simulator] 旋转: angle=90.0°, angular_speed=0.5rad/s
... ✅ 完成!
```

## 依赖

```bash
pygame>=2.5.0       # 2D 图形渲染
rclpy               # ROS2 Python 客户端库
std_msgs            # ROS2 标准消息类型
```

## 设计特点

1. **轻量级**: 基于 Pygame 的简单仿真
2. **ROS2集成**: 标准ROS2话题通讯
3. **实时可视化**: 实时显示机器人状态
4. **差速驱动**: 模拟真实机器人运动
5. **延迟初始化**: 自动处理ROS连接建立

## 仿真参数

### 机器人参数

```python
ROBOT_SIZE = 25          # 机器人半径（像素）
ROBOT_COLOR = (0, 128, 255)  # 机器人颜色（蓝色）
```

### 环境参数

```python
SCREEN_WIDTH = 800       # 屏幕宽度（像素）
SCREEN_HEIGHT = 600      # 屏幕高度（像素）
BG_COLOR = (240, 240, 240)  # 背景颜色（浅灰）
```

### 运动参数

```python
# 默认速度
DEFAULT_SPEED = 0.3      # 默认移动速度（米/秒）
DEFAULT_ANGULAR_SPEED = 0.5  # 默认角速度（弧度/秒）

# 比例尺
PIXELS_PER_METER = 50    # 1米 = 50像素
```

## 故障排除

### 问题1：仿真器不响应

**症状**: 发送命令后仿真器没有反应

**原因**: ROS2连接未建立

**解决**:
- 已修复：在 `ros_topic_comm.py` 中添加0.2秒延迟
- 确保先启动仿真器再发送命令

### 问题2：找不到ROS2话题

**症状**: `ros2 topic list` 看不到 `/robot/command`

**解决**:
```bash
# 确保仿真器正在运行
ps aux | grep simulator.py

# 重启仿真器
```

## 相关文档

- [主项目 README](../README.md)
- [Robot_Module README](../Robot_Module/README.md)
- [LLM_Module README](../LLM_Module/README.md)
- [Interactive_Module README](../Interactive_Module/README.md)
- [ros_topic_comm.py](../ros_topic_comm.py) - ROS2 通讯模块

---

**轻量仿真，实时反馈！** 🚀
