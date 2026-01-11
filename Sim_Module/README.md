# Sim_Module - 仿真模块

## 概述

Sim_Module 提供了**多种机器人仿真环境**，支持从简单的 2D 验证到高精度的 3D 物理仿真。该模块完全独立，不依赖通信层，便于测试和开发。

### 核心功能

- **2D 仿真**: 快速逻辑验证和算法测试
- **MuJoCo 3D**: 高性能物理仿真，适合强化学习
- **多通信方式**: 支持 ROS2 和 Dora 两种通信协议
- **可视化**: 实时显示机器人状态和环境

### 仿真器对比

| 特性 | 2D Pygame | MuJoCo 3D |
|-----|-----------|-----------|
| 用途 | 快速验证、逻辑测试 | 高精度物理仿真、强化学习 |
| 性能 | 极快，实时性高 | 快速，比 Gazebo 快 5-10 倍 |
| 视觉效果 | 简单 2D 俯视图 | 真实 3D 渲染 |
| 物理引擎 | 简单运动学 | 完整动力学仿真 |
| 适用机器人 | 差速机器人 | 四足、机械臂等复杂机器人 |
| 资源消耗 | 极低 | 中等 |

## 目录结构

```
Sim_Module/
├── __init__.py
├── ros2_2d/                    # ROS2 版 2D 仿真
│   └── simulator.py            # Pygame 可视化 + ROS2 话题
├── dora_2d/                    # Dora 版 2D 仿真
│   └── simulator.py            # Pygame 可视化 + Dora 数据流
└── mujoco/                     # MuJoCo 3D 物理仿真
    ├── mujoco_simulator.py     # MuJoCo 仿真器主程序
    ├── install_mujoco.sh       # MuJoCo 安装脚本
    ├── start_mujoco_sim.sh     # MuJoCo 启动脚本
    └── README_MUJOCO.md        # MuJoCo 详细文档
```

## 2D 仿真器

### ROS2 版本 (`ros2_2d/simulator.py`)

**差速机器人 2D 仿真器**，使用 ROS2 话题通信。

#### 特性

- Pygame 可视化界面
- 订阅 `/cmd_vel` 话题 (geometry_msgs/Twist)
- 实时显示机器人位置、朝向和运动轨迹
- 网格背景便于距离估算

#### 使用方式

```bash
# 方式 1: 直接运行
cd Sim_Module/ros2_2d
python3 simulator.py

# 方式 2: 通过启动脚本
cd Middle_Module/ROS
./start_ros2_mcp.sh --sim 2d
```

#### 机器人显示

- **蓝色圆形**: 机器人主体
- **黄色圆点**: 朝向指示器
- **网格间距**: 50 像素 (约 1 米)

#### 控制示例

```bash
# 发布 ROS2 命令
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}, angular: {z: 0.5}}"
```

### Dora 版本 (`dora_2d/simulator.py`)

**Dora 数据流版本的 2D 仿真器**，功能与 ROS2 版相同。

#### 使用方式

```bash
cd Middle_Module/Dora
dora up
dora start dora-interactive-mcp.yaml --attach
```

## MuJoCo 3D 仿真

### 概述

**MuJoCo (Multi-Joint dynamics with Contact)** 是一个高性能物理仿真引擎，特别适合强化学习和复杂机器人仿真。

### 特性

- 高精度物理仿真 (接触、摩擦、动力学)
- 支持 Unitree Go2 四足机器人 URDF 模型
- 实时 3D 可视化
- ROS2 `/cmd_vel` 话题控制
- 比 Gazebo 快 5-10 倍

### 快速开始

#### 方式 1: 手动启动

```bash
cd Sim_Module/mujoco

# 首次使用需要安装 MuJoCo
./install_mujoco.sh

# 启动仿真
./start_mujoco_sim.sh
```

#### 方式 2: 通过启动脚本

```bash
cd Middle_Module/ROS
./start_ros2_mcp.sh --sim mujoco
```

### MuJoCo 仿真器功能

- **机器人模型**: Unitree Go2 四足机器人
- **控制接口**: ROS2 `/cmd_vel` 话题
- **可视化**: MuJoCo 原生查看器
- **物理特性**: 完整动力学、接触仿真

### 详细文档

MuJoCo 仿真的详细配置和使用请参考: [README_MUJOCO.md](./mujoco/README_MUJOCO.md)

## 设计原则

### 1. 独立性

仿真器完全独立，不依赖通信层：
- 可以单独运行和测试
- 易于移植到其他平台
- 便于单元测试

### 2. 可互换性

同一个仿真逻辑支持多种通信方式：
- ROS2 版本: 通过 `/cmd_vel` 话题控制
- Dora 版本: 通过 Dora 数据流控制
- 核心仿真代码完全相同

### 3. 轻量级 vs 高精度

- **2D 仿真**: 用于快速验证逻辑，实时性极高
- **MuJoCo**: 用于高精度物理仿真，适合强化学习训练

## 依赖

### 2D 仿真依赖

```bash
# Python 包
pip install pygame>=2.5.0

# ROS2 包
sudo apt install ros-humble-geometry-msgs
```

### MuJoCo 仿真依赖

```bash
# 运行安装脚本
cd Sim_Module/mujoco
./install_mujoco.sh

# 或手动安装
pip install mujoco
```

## 机器人状态

### 2D 仿真机器人状态

```python
{
    'x': 400.0,              # X 坐标 (像素)
    'y': 300.0,              # Y 坐标 (像素)
    'angle': 0.0,            # 朝向角度 (度，0=东，90=北)
    'target_x': 400.0,       # 目标 X 坐标
    'target_y': 300.0,       # 目标 Y 坐标
    'target_angle': 0.0,     # 目标角度
    'speed': 0.0,            # 当前速度
    'angular_speed': 0.0,    # 当前角速度
    'is_moving': False,      # 是否在运动
    'last_command': ""       # 最后一条命令
}
```

### MuJoCo 仿真机器人状态

```python
{
    'position': [x, y, z],       # 3D 位置
    'orientation': [w, x, y, z], # 四元数姿态
    'linear_velocity': [vx, vy, vz],
    'angular_velocity': [wx, wy, wz],
    'joint_positions': [...],    # 关节位置
    'joint_velocities': [...]    # 关节速度
}
```

## 使用示例

### 2D 仿真手动控制

```bash
# 终端 1: 启动仿真器
cd Sim_Module/ros2_2d
python3 simulator.py

# 终端 2: 控制机器人
source /opt/ros/humble/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.0}}" --once
```

### 通过 Python 控制仿真

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimController(Node):
    def __init__(self):
        super().__init__('sim_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def move_forward(self, speed=0.3):
        msg = Twist()
        msg.linear.x = speed
        self.publisher.publish(msg)

    def stop(self):
        msg = Twist()
        self.publisher.publish(msg)

# 使用
rclpy.init()
controller = SimController()
controller.move_forward(0.3)
rclpy.spin_once(controller, timeout_sec=1.0)
controller.stop()
```

## 性能说明

### 2D 仿真

- **帧率**: 60 FPS
- **延迟**: <10ms
- **CPU 占用**: <5%
- **适用场景**: 快速逻辑验证、算法测试

### MuJoCo 仿真

- **帧率**: 50-100 FPS (取决于模型复杂度)
- **物理精度**: 毫秒级
- **与 Gazebo 对比**: 速度快 5-10 倍
- **适用场景**: 强化学习、高精度物理仿真

## 扩展仿真模块

### 添加新的 2D 机器人

在 `ros2_2d/simulator.py` 中修改 `Robot` 类:

```python
class CustomRobot(Robot):
    def __init__(self, x, y):
        super().__init__(x, y)
        # 添加自定义属性
        self.arm_angle = 0

    def draw(self, screen):
        super().draw(screen)
        # 绘制自定义部件
        pass
```

### 添加新的 MuJoCo 机器人

1. 准备 URDF 模型文件
2. 在 `mujoco_simulator.py` 中加载模型
3. 配置关节和驱动器
4. 测试仿真效果

## 故障排除

### 2D 仿真器无法启动

1. 检查 Pygame 是否安装: `python3 -c "import pygame"`
2. 检查 ROS2 环境: `source /opt/ros/humble/setup.bash`
3. 检查端口占用: `lsof -i :8000`

### MuJoCo 仿真器无法启动

1. 检查 MuJoCo 安装: `python3 -c "import mujoco"`
2. 检查 OpenGL 支持: `glxinfo | grep "OpenGL version"`
3. 尝试 headless 模式 (无可视化)

### 仿真器卡顿

1. 关闭其他占用 GPU 的程序
2. 降低仿真频率
3. 减小可视化窗口

## 相关文档

- [MuJoCo 详细文档](./mujoco/README_MUJOCO.md) - MuJoCo 仿真详细说明
- [Middle_Module README](../Middle_Module/README.md) - 通信层模块
- [Robot_Module README](../Robot_Module/README.md) - 机器人模块
- [主项目 README](../README.md) - 项目总览

