# Sim_Module - 仿真模块

## 概述

Sim_Module 提供**2D 机器人仿真环境**，基于 Pygame 实现可视化，通过 multiprocessing.Queue 接收来自 Robot_Module 的动作指令并执行仿真。

### 核心功能

- **2D 仿真**: 基于 Pygame 的轻量级 2D 机器人仿真
- **实时可视化**: 显示机器人位置、朝向和运动轨迹
- **队列通信**: 通过 multiprocessing.Queue 接收动作指令
- **差速驱动**: 模拟差速驱动机器人运动

## 文件结构

```
Sim_Module/
├── sim2d/
│   ├── simulator.py      # 2D 仿真器主程序
│   └── README.md         # 本文件
└── README.md
```

## 核心类: Sim2DRobot

### 初始化

```python
from sim2d.simulator import Sim2DRobot

# 创建仿真器
robot = Sim2DRobot(action_queue=None)
```

### 参数说明

```python
def __init__(self, action_queue=None):
    """
    初始化2D机器人仿真器

    Args:
        action_queue: multiprocessing.Queue，用于接收动作指令
                      如果为 None，则使用共享队列
    """
```

### 机器人状态

```python
# 位置 (x, y) - 单位: 像素
self.position = [400, 300]  # 窗口中心

# 朝向 (theta) - 单位: 弧度
self.orientation = 0.0  # 初始朝向右

# 机器人尺寸
self.robot_size = 40    # 40x40 像素
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
multiprocessing.Queue (进程间通信)
    ↓
Sim2DRobot.run()
    ↓ 读取队列
action = action_queue.get()
    ↓ 解析动作
if action['action'] == 'move_forward':
    self.move_forward(**action['parameters'])
    ↓ 执行仿真
更新机器人状态 + Pygame 可视化
```

## 核心方法

### 1. `run()` - 主循环

```python
def run(self):
    """启动仿真器主循环"""
    pygame.init()
    screen = pygame.display.set_mode((800, 600))
    pygame.display.set_caption("2D Robot Simulator")

    clock = pygame.time.Clock()
    running = True

    while running:
        # 1. 处理 Pygame 事件
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

        # 2. 从队列读取动作指令
        try:
            action = self.action_queue.get_nowait()
            self.execute_action(action)
        except:
            pass

        # 3. 更新机器人状态
        self.update()

        # 4. 绘制场景
        self.draw(screen)

        # 5. 刷新显示
        pygame.display.flip()
        clock.tick(60)  # 60 FPS

    pygame.quit()
```

### 2. `execute_action()` - 执行动作

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
        self.move_forward(**params)
    elif action_type == 'move_backward':
        self.move_backward(**params)
    elif action_type == 'turn':
        self.turn(**params)
    elif action_type == 'stop':
        self.stop()
```

### 3. `move_forward()` - 前进

```python
def move_forward(self, distance=1.0, speed=0.3):
    """向前移动

    Args:
        distance: 移动距离（米）
        speed: 移动速度（米/秒）
    """
    # 计算目标位置
    dx = distance * self.pixels_per_meter * math.cos(self.orientation)
    dy = distance * self.pixels_per_meter * math.sin(self.orientation)

    target_x = self.position[0] + dx
    target_y = self.position[1] + dy

    # 执行动画移动
    steps = int(distance / speed * 60)  # 60 FPS
    for _ in range(steps):
        self.position[0] += dx / steps
        self.position[1] += dy / steps
        pygame.time.delay(1000 // 60)

    self.position = [target_x, target_y]
```

### 4. `turn()` - 旋转

```python
def turn(self, angle=90.0, angular_speed=0.5):
    """原地旋转

    Args:
        angle: 旋转角度（度），正值为左转
        angular_speed: 角速度（弧度/秒）
    """
    angle_rad = math.radians(angle)

    # 计算目标朝向
    target_orientation = self.orientation + angle_rad

    # 执行动画旋转
    duration = abs(angle_rad) / angular_speed
    steps = int(duration * 60)  # 60 FPS

    for _ in range(steps):
        self.orientation += angle_rad / steps
        pygame.time.delay(1000 // 60)

    self.orientation = target_orientation
```

### 5. `draw()` - 绘制场景

```python
def draw(self, screen):
    """绘制仿真场景

    Args:
        screen: Pygame 屏幕
    """
    # 1. 填充背景
    screen.fill((240, 240, 240))

    # 2. 绘制网格
    self.draw_grid(screen)

    # 3. 绘制机器人
    self.draw_robot(screen)

    # 4. 绘制信息
    self.draw_info(screen)
```

## 使用示例

### 直接运行仿真器

```bash
python3 Sim_Module/sim2d/simulator.py
```

### 与系统集成

```bash
# 终端1: 启动仿真器
python3 Sim_Module/sim2d/simulator.py

# 终端2: 启动交互界面
python3 Interactive_Module/interactive.py
```

### 独立测试队列

```python
import sys
sys.path.insert(0, '/home/robot/work/FinalProject')

from multiprocessing import Queue
from Sim_Module.sim2d.simulator import Sim2DRobot
import json

# 创建队列和仿真器
action_queue = Queue()
robot = Sim2DRobot(action_queue)

# 发送动作指令
action = {
    'action': 'move_forward',
    'parameters': {'distance': 1.0, 'speed': 0.3}
}
action_queue.put(action)

# 运行仿真器
robot.run()
```

## 仿真界面

### 窗口

```
┌────────────────────────────────────────┐
│       2D Robot Simulator              │
│  ┌──────────────────────────────────┐ │
│  │  背景: 浅灰色                      │ │
│  │  网格: 浅灰色线条                  │ │
│  │                                  │ │
│  │         ┃──>                      │ │
│  │         ┃  机器人 (蓝色方块)        │ │
│  │         ┃                         │ │
│  │                                  │ │
│  └──────────────────────────────────┘ │
│                                        │
│  Position: (420, 315)                 │
│  Orientation: 90.0°                   │
└────────────────────────────────────────┘
```

### 信息显示

- **Position**: 机器人位置 (x, y) 像素
- **Orientation**: 机器人朝向 (度)

## 配置参数

```python
# 窗口大小
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 600

# 机器人参数
ROBOT_SIZE = 40           # 机器人尺寸（像素）
PIXELS_PER_METER = 50     # 像素-米转换比例

# 颜色
COLOR_BG = (240, 240, 240)        # 背景色（浅灰）
COLOR_GRID = (200, 200, 200)      # 网格颜色
COLOR_ROBOT = (0, 120, 215)       # 机器人颜色（蓝色）
COLOR_DIRECTION = (255, 0, 0)     # 朝向指示颜色（红色）
```

## 通信机制

### 队列格式

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

## 共享队列

使用 `shared_queue.py` 实现跨进程通信：

```python
# 文件: /tmp/robot_finalproject/commands.jsonl
{"action": "move_forward", "parameters": {"distance": 1.0, "speed": 0.3}}
{"action": "turn", "parameters": {"angle": 90.0, "angular_speed": 0.5}}
```

## 依赖

```
pygame>=2.5.0    # Pygame 2D 可视化
```

## 键盘控制

| 按键 | 功能 |
|------|------|
| `ESC` | 退出仿真器 |
| 关闭窗口 | 退出仿真器 |

## 设计特点

1. **轻量级**: 纯 Pygame 实现，无复杂依赖
2. **实时性**: 60 FPS 流畅动画
3. **队列驱动**: 异步接收动作指令
4. **可视化**: 清晰显示机器人状态
5. **易于扩展**: 可添加更多机器人类型

## 扩展仿真器

### 添加障碍物

```python
def __init__(self, action_queue=None):
    # 添加障碍物列表
    self.obstacles = [
        {'position': (500, 300), 'size': (50, 50)},
        {'position': (600, 400), 'size': (40, 40)}
    ]

def draw(self, screen):
    # 绘制障碍物
    for obs in self.obstacles:
        pygame.draw.rect(screen, (100, 100, 100), obs)
```

### 添加轨迹绘制

```python
def __init__(self, action_queue=None):
    self.trajectory = []  # 轨迹点列表

def move_forward(self, distance=1.0, speed=0.3):
    # 记录轨迹点
    self.trajectory.append(tuple(self.position))

def draw(self, screen):
    # 绘制轨迹
    if len(self.trajectory) > 1:
        pygame.draw.lines(screen, (0, 200, 0), False, self.trajectory, 2)
```

## 性能优化

1. **FPS 控制**: 使用 `clock.tick(60)` 保持稳定帧率
2. **异步读取**: 使用 `get_nowait()` 避免阻塞
3. **增量更新**: 逐步更新位置和朝向，平滑动画

## 相关文档

- [主项目 README](../README.md)
- [Robot_Module README](../Robot_Module/README.md)
- [Interactive_Module README](../Interactive_Module/README.md)

---

**仿真，可视化！** 🚀
