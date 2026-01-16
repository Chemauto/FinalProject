# Sim_Module - 仿真模块

## 概述

Sim_Module 提供**2D 机器人仿真环境**，基于 Pygame 实现可视化，通过 **ROS2 话题**接收来自 Robot_Module 的动作指令并执行仿真。

### 核心功能

- **2D 仿真**: 基于 Pygame 的轻量级 2D 机器人仿真
- **实时可视化**: 显示机器人位置、朝向和运动轨迹
- **ROS2 通讯**: 通过 ROS2 话题接收动作指令
- **差速驱动**: 模拟差速驱动机器人运动

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

### 初始化

```python
from ros_topic_comm import get_shared_queue

action_queue = get_shared_queue()
action_queue.setup_subscriber()
```

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
        self.target_x = self.x
        self.target_y = self.y
```

### 2. `update()` - 更新状态

```python
def update(self):
    """更新机器人位置（平滑移动到目标位置）"""
    # 平滑旋转
    if abs(self.angle - self.target_angle) > 1:
        diff = (self.target_angle - self.angle + 180) % 360 - 180
        self.angle += diff * 0.1
        self.angle %= 360

    # 平滑移动
    if abs(self.x - self.target_x) > 1 or abs(self.y - self.target_y) > 1:
        self.is_moving = True
        self.x += (self.target_x - self.x) * 0.05
        self.y += (self.target_y - self.y) * 0.05
    else:
        self.is_moving = False
```

### 3. `draw()` - 绘制场景

```python
def draw(self, screen):
    """绘制仿真场景"""
    # 绘制机器人主体（圆形）
    pygame.draw.circle(screen, ROBOT_BODY_COLOR, (int(self.x), int(self.y)), ROBOT_SIZE)

    # 绘制方向指示器（三角形）
    angle_rad = math.radians(self.angle)
    front_x = self.x + ROBOT_SIZE * math.cos(angle_rad)
    front_y = self.y - ROBOT_SIZE * math.sin(angle_rad)
    pygame.draw.circle(screen, ROBOT_CABIN_COLOR, (int(front_x), int(front_y)), ROBOT_SIZE // 3)
```

## 使用示例

### 直接运行仿真器

```bash
python3 Sim_Module/sim2d/simulator.py
```

### 与系统集成

```bash
# 使用启动脚本
./start_robot_system.sh
# 选择 1（自动启动）
```

### 调试 ROS2 话题

```bash
# 查看话题列表
ros2 topic list

# 查看话题消息
ros2 topic echo /robot/command

# 查看话题信息
ros2 topic info /robot/command
```

## 仿真界面

### 窗口

```
┌────────────────────────────────────────┐
│       2D Robot Simulator              │
│  ┌──────────────────────────────────┐ │
│  │  背景: 白色                        │ │
│  │  网格: 浅灰色线条                  │ │
│  │                                  │ │
│  │         ┃──>                      │ │
│  │         ┃  机器人 (蓝色圆形)        │ │
│  │                                  │ │
│  └──────────────────────────────────┘ │
│                                        │
│  Pos: (400, 300)  Angle: 0°           │
└────────────────────────────────────────┘
```

### 信息显示

- **Pos**: 机器人位置 (x, y) 像素
- **Angle**: 机器人朝向 (度)
- **Command**: 最后执行的命令

## 配置参数

```python
# 窗口大小
WIDTH, HEIGHT = 800, 600

# 机器人参数
ROBOT_SIZE = 25           # 机器人半径（像素）
GRID_SPACING = 50         # 网格间距（像素）

# 像素-米转换比例
# 1米 = 100像素

# 颜色
WHITE = (255, 255, 255)           # 背景色（白色）
GRID_COLOR = (220, 220, 220)      # 网格颜色
ROBOT_BODY_COLOR = (60, 120, 180) # 机器人颜色（蓝色）
ROBOT_CABIN_COLOR = (255, 200, 0) # 方向指示颜色（黄色）
TEXT_COLOR = (50, 50, 50)         # 文字颜色（深灰）
```

## 通信机制

### ROS2 话题格式

**话题名称**: `/robot/command`
**消息类型**: `std_msgs/String`
**消息格式**: JSON字符串

```python
# 动作指令格式
action = {
    'action': 'move_forward',  # 动作类型
    'parameters': {             # 动作参数
        'distance': 1.0,
        'speed': 0.3
    }
}
```

### 支持的动作类型

```python
ACTIONS = {
    'move_forward': '前进',
    'move_backward': '后退',
    'turn': '旋转',
    'stop': '停止'
}
```

## 依赖

```
pygame>=2.5.0    # Pygame 2D 可视化
rclpy            # ROS2 Python 客户端库
```

## 键盘控制

| 按键 | 功能 |
|------|------|
| `ESC` | 退出仿真器 |
| 关闭窗口 | 退出仿真器 |

## 设计特点

1. **轻量级**: 纯 Pygame 实现，无复杂依赖
2. **实时性**: 60 FPS 流畅动画
3. **ROS2 通讯**: 使用标准 ROS2 话题
4. **可视化**: 清晰显示机器人状态
5. **易于扩展**: 可添加更多机器人类型

## 性能优化

1. **FPS 控制**: 使用 `clock.tick(60)` 保持稳定帧率
2. **非阻塞读取**: 使用 `get_nowait()` 避免阻塞
3. **平滑动画**: 逐步更新位置和朝向

## 相关文档

- [主项目 README](../README.md)
- [Robot_Module README](../Robot_Module/README.md)
- [ros_topic_comm.py](../ros_topic_comm.py) - ROS2 通讯模块

---

**仿真，可视化！** 🚀
