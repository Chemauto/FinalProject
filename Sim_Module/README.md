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
