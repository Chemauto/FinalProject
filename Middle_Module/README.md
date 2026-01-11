# Middle_Module - 中间通信层

## 功能

统一的机器人通信接口，支持多种通信协议：
- **ROS2**: 基于 ROS2 的机器人通信
- **Dora**: 基于 Dora 的数据流通信

## 设计原则

- **代码尽可能少**: 只负责通信，不包含业务逻辑
- **通用性**: 通过配置适配不同机器人
- **独立性**: 仿真器代码在 `Sim_Module`

## 目录结构

```
Middle_Module/
├── __init__.py
├── ROS/                     # ROS2 通信
│   ├── ros2_interactive_mcp.py    # LLM交互界面
│   ├── ros2_robot_controller.py   # 通用ROS2桥接
│   └── start_ros2_mcp.sh          # 启动脚本
└── Dora/                    # Dora 通信
    ├── dora-interactive-mcp.yaml   # Dora配置
    ├── input_ui.py                 # 输入UI
    └── requirements.txt            # Dora依赖
```

## ROS2 通信

### ros2_interactive_mcp.py

LLM 交互界面，连接 LLM_Module 和 ROS2：
- 使用 `LLM_Module.LLMAgent` 进行任务规划
- 使用 `MCP_Module` 获取机器人技能
- 发布命令到 `/robot_command` 话题

### ros2_robot_controller.py

通用 ROS2 桥接器，从 `Robot_Module` 读取配置：
```bash
python3 ros2_robot_controller.py --robot Sim_2D
python3 ros2_robot_controller.py --robot Go2_Quadruped
```

根据 `Robot_Module/{robot}/robot_config.yaml` 动态创建 ROS2 话题发布者。

## Dora 通信

- `dora-interactive-mcp.yaml`: Dora 数据流配置
- `input_ui.py`: Dora 输入界面

## 快速开始

### ROS2 (2D 仿真)

```bash
cd Middle_Module/ROS
./start_ros2_mcp.sh --sim 2d
```

### ROS2 (Gazebo 3D 仿真)

```bash
cd Middle_Module/ROS
./start_ros2_mcp.sh --sim gazebo
```

### ROS2 (真实机器人)

```bash
cd Middle_Module/ROS
./start_ros2_mcp.sh --sim real
```
