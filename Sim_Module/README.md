# Sim_Module - 仿真模块

## 功能

提供各种机器人仿真环境：
- **2D 仿真**: 快速验证和逻辑测试
- **Gazebo 3D**: 物理仿真和 Sim2Real

## 目录结构

```
Sim_Module/
├── __init__.py
├── ros2_2d/              # ROS2 版本的 2D 仿真
│   └── simulator.py      # Pygame 可视化，订阅 /cmd_vel
├── dora_2d/              # Dora 版本的 2D 仿真
│   └── simulator.py      # Pygame 可视化，订阅 Dora dataflow
└── gazebo/               # Gazebo 3D 仿真
    ├── go2_sim_launch.py
    ├── Go2_Gazebo_Description/
    │   ├── launch/
    │   ├── urdf/
    │   └── config/
    ├── start_gazebo_simple.sh
    ├── install_gazebo.sh
    └── GAZEBO_QUICKSTART.md
```

## 2D 仿真器

### ROS2 版本 (`ros2_2d/simulator.py`)

使用 ROS2 话题通信：
- 订阅: `/cmd_vel` (geometry_msgs/Twist)
- 显示机器人位置和运动

### Dora 版本 (`dora_2d/simulator.py`)

使用 Dora dataflow 通信：
- 订阅 Dora 数据流
- 显示机器人位置和运动

## Gazebo 3D 仿真

### 快速开始

```bash
cd Sim_Module/gazebo
./start_gazebo_simple.sh
```

### 安装 Gazebo

```bash
cd Sim_Module/gazebo
./install_gazebo.sh
```

### 机器人模型

Go2 机器人的 URDF 模型位于 `Go2_Gazebo_Description/` 目录。

## 设计原则

- **独立性**: 仿真器完全独立，不依赖通信层
- **可互换**: 同一个仿真可以支持 ROS2 和 Dora 两种通信方式
- **轻量级**: 2D 仿真用于快速测试，Gazebo 用于物理仿真
