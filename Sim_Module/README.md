# Sim_Module - 仿真模块

基于 Pygame 的 2D 机器人仿真环境，提供可视化界面和追击功能测试。

## 📁 模块结构

```
Sim_Module/
├── README.md              # 本文档
├── enemy_manager.py       # 敌人管理器
└── sim2d/
    └── simulator.py       # 主仿真器（含追击功能）
```

## 🎯 功能特性

- **2D 可视化**: 基于 Pygame 的图形界面
- **机器人模拟**: 移动、旋转、物理运动
- **敌人管理**: 生成、移动、删除敌人
- **追击功能**: 自动追击最近的敌人
- **ROS2 通讯**: 订阅动作命令，发布状态
- **实时更新**: 60 FPS 平滑动画
- **中文支持**: 完整的中文字体支持

## 🔧 核心组件

### enemy_manager.py - 敌人管理器

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

    def update(self):
        """更新机器人状态（平滑动画）"""

    def draw(self, screen):
        """绘制机器人"""

    def execute_action(self, action):
        """执行动作命令"""


class ChaseSimulator:
    """追击仿真器主类"""
    def __init__(self):
        """初始化仿真器"""

    def handle_events(self):
        """处理事件（鼠标、键盘）"""

    def spawn_enemy_at(x, y):
        """在指定位置生成敌人"""

    def clear_enemies(self):
        """清除所有敌人"""

    def toggle_chase_line(self):
        """切换追击线显示"""

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

| 按键 | 功能 |
|-----|------|
| **鼠标左键** | 在点击位置生成敌人 |
| **C** | 清除所有敌人 |
| **L** | 切换追击线显示 |
| **ESC** | 退出仿真器 |

### 机器人运动

**前进 (move_forward):**
```json
{
  "action": "move_forward",
  "parameters": {
    "distance": 1.0,
    "speed": 0.3
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
    "angle": 90.0,
    "angular_speed": 0.5
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
| `/robot/enemy_remove` | String | 接收敌人清除命令 |

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
python3 Sim_Module/sim2d/simulator.py
```

### 2. 查看可视化

仿真器窗口会显示：
- 网格背景（1米网格）
- 机器人（圆形，带方向指示器）
- 敌人（红色圆形）
- 状态信息（位置、角度）
- 追击线（可选）

### 3. 测试追击功能

```bash
# 终端2: 启动交互程序
python3 Interactive_Module/interactive.py

# 输入命令
追击敌人
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
- 角度 0° 指向右（东），90° 指向上（北）

### 运动学模型

**直线运动:**
```
新位置 = 旧位置 + 方向向量 × 距离

x_new = x_old + cos(angle) × distance
y_new = y_old - sin(angle) × distance  # 注意屏幕坐标 y 向下
```

**旋转运动:**
```
角度更新采用平滑插值

当前角度 = 当前角度 + (目标角度 - 当前角度) × 速度因子
```

## 🎨 可视化元素

### 机器人

- **主体**: 蓝色圆形（半径 25 像素）
- **方向指示器**: 黄色小圆点，指向当前朝向
- **平滑移动**: 插值动画，60 FPS

### 敌人

- **形状**: 红色圆形（半径 15 像素）
- **ID**: 显示在敌人上方
- **追击线**: 蓝色连线，指向机器人

### 网格

- **间距**: 50 像素（0.5米）
- **颜色**: 浅灰色
- **线宽**: 1 像素

### 信息面板

显示内容：
```
机器人: (400.0, 300.0)  角度: 0.0°
敌人数量: 2  选中: 1
[鼠标]生成 [C]清除 [L]连线 [ESC]退出
```

## 🐛 调试技巧

### 1. 查看机器人状态

```python
# 在 update() 中添加
print(f"Robot: x={self.robot.x:.1f}, y={self.robot.y:.1f}, angle={self.robot.angle:.1f}°")
```

### 2. 查看 ROS2 话题

```bash
# 查看机器人状态
ros2 topic echo /robot/state

# 查看敌人位置
ros2 topic echo /robot/enemies

# 查看动作命令
ros2 topic echo /robot/command
```

### 3. 降低帧率调试

```python
FPS = 10  # 降低帧率，便于观察
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
# 1米 = 100像素（在移动计算中）
distance_pixels = distance * 100
```

### 修改机器人参数

```python
# Robot 类中
self.speed = 0.3          # 默认速度（由命令参数控制）
self.angular_speed = 0.5  # 默认角速度（由命令参数控制）
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
        if len(self.trail) > 100:
            self.trail.pop(0)

    def draw(self, screen):
        # 绘制轨迹
        if len(self.trail) > 1:
            pygame.draw.lines(screen, (100, 100, 255), False, self.trail, 2)
```

## 🔗 相关模块

- `ros_topic_comm.py` - ROS2 通讯模块
- `Robot_Module/module/chase.py` - 追击功能
- `Robot_Module/module/base.py` - 底盘控制

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

**仿真器，追击功能测试！** 🎮
