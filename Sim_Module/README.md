# Sim_Module - 仿真模块

## 功能

提供各种机器人仿真环境：
- **2D 仿真**: 快速验证和逻辑测试
- **MuJoCo 3D**: 高性能物理仿真和强化学习

## 目录结构

```
Sim_Module/
├── __init__.py
├── ros2_2d/              # ROS2 版本的 2D 仿真
│   └── simulator.py      # Pygame 可视化，订阅 /cmd_vel
├── dora_2d/              # Dora 版本的 2D 仿真
│   └── simulator.py      # Pygame 可视化，订阅 Dora dataflow
└── mujoco/               # MuJoCo 3D 物理仿真
    ├── mujoco_simulator.py    # MuJoCo 仿真器主程序
    ├── install_mujoco.sh      # MuJoCo 安装脚本
    ├── start_mujoco_sim.sh    # MuJoCo 启动脚本
    └── README_MUJOCO.md       # MuJoCo 详细文档
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

## MuJoCo 3D 仿真

### 快速开始

```bash
cd Sim_Module/mujoco
./install_mujoco.sh      # 首次使用需要安装
./start_mujoco_sim.sh    # 启动 MuJoCo 仿真
```

### 通过 ROS2 MCP 系统启动

```bash
cd /home/xcj/work/FinalProject/Middle_Module/ROS
./start_ros2_mcp.sh --sim mujoco
```

### 详细文档

请查看 [MuJoCo 仿真模块文档](./mujoco/README_MUJOCO.md)

## 设计原则

- **独立性**: 仿真器完全独立，不依赖通信层
- **可互换**: 同一个仿真可以支持 ROS2 和 Dora 两种通信方式
- **轻量级**: 2D 仿真用于快速测试，MuJoCo 用于高精度物理仿真
- **高性能**: MuJoCo 比 Gazebo 快 5-10 倍，更适合强化学习

