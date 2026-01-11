# Middle_Module - 中间通信层

## 概述

Middle_Module 提供了**统一的机器人通信接口**，支持多种通信协议。该模块作为纯通信层，不包含业务逻辑，负责将上层指令传递给底层执行单元。

### 核心功能

- **多协议支持**: 支持 ROS2、Dora 等多种通信方式
- **通用 ROS2 桥接**: 根据配置文件动态创建 ROS2 话题
- **LLM 集成**: 连接 LLM_Module 与底层通信层
- **模块化设计**: 通信协议可独立开发和测试

### 设计原则

1. **代码最少化**: 只负责通信，不包含业务逻辑
2. **配置驱动**: 通过配置文件适配不同机器人
3. **独立性**: 仿真器代码在 `Sim_Module`，真实机器人代码在 `Real_Module`
4. **可扩展**: 易于添加新的通信协议

## 目录结构

```
Middle_Module/
├── __init__.py
├── ROS/                             # ROS2 通信
│   ├── ros2_interactive_mcp.py      # LLM 交互界面 (主程序)
│   ├── ros2_robot_controller.py     # 通用 ROS2 桥接器
│   └── start_ros2_mcp.sh            # 一键启动脚本
└── Dora/                            # Dora 通信
    ├── dora-interactive-mcp.yaml    # Dora 数据流配置
    ├── input_ui.py                  # Dora 输入 UI
    └── requirements.txt             # Dora 依赖
```

## ROS2 通信

### ros2_interactive_mcp.py

**LLM 交互界面**，是整个系统的主入口点。

#### 功能

- 使用 `LLM_Module.LLMAgent` 进行双层 LLM 任务规划和执行
- 使用 `MCP_Module` 获取机器人技能定义
- 发布命令到 `/robot_command` ROS2 话题
- 提供交互式命令行界面

#### 使用方式

```bash
cd Middle_Module/ROS
python3 ros2_interactive_mcp.py
```

#### 交互示例

```
🤖 ROS2 Interactive MCP - 双层LLM架构
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  - 上层LLM: 任务规划 (qwen-plus)
  - 下层LLM: 任务执行 (qwen-plus)
  - 技能来源: Robot_Module/Sim_2D/skills/
  - 执行层: ROS2 Humble (2D Simulation)
══════════════════════════════════════════

💡 请输入指令 (或 'quit' 退出): 前进1米然后左转90度
```

### ros2_robot_controller.py

**通用 ROS2 桥接器**，从 `Robot_Module` 读取配置并动态创建话题。

#### 功能特性

- **配置驱动**: 根据 `robot_config.yaml` 动态创建话题
- **多机器人支持**: 通过 `--robot` 参数指定机器人
- **话题映射**: 支持多种 ROS2 消息类型 (Twist, String, JointState 等)

#### 使用方式

```bash
# 控制指定机器人
python3 ros2_robot_controller.py --robot Sim_2D
python3 ros2_robot_controller.py --robot Go2_Quadruped
```

#### 工作流程

1. 从 `Robot_Module/{robot}/robot_config.yaml` 加载配置
2. 创建订阅者监听 `/robot_command` 话题
3. 根据配置动态创建话题发布者
4. 接收 JSON 格式命令并转换为 ROS2 消息

#### 支持的消息类型

| 消息类型 | ROS2 Topic | 用途 |
|---------|-----------|------|
| `geometry_msgs/Twist` | `/cmd_vel` | 速度控制 (线速度、角速度) |
| `std_msgs/String` | `/robot_command` | 通用命令接收 |
| `sensor_msgs/JointState` | `/joint_states` | 关节状态控制 |
| `std_msgs/Float64` | `/gripper` | 夹爪控制 |

### start_ros2_mcp.sh

**一键启动脚本**，简化系统启动流程。

#### 使用方式

```bash
cd Middle_Module/ROS

# 启动 2D 仿真
./start_ros2_mcp.sh --sim 2d

# 启动 MuJoCo 3D 仿真
./start_ros2_mcp.sh --sim mujoco

# 使用真实机器人 (需硬件支持)
./start_ros2_mcp.sh --sim real
```

#### 脚本功能

1. 设置代理取消 (避免网络问题)
2. 启动对应仿真器 (Sim_Module)
3. 启动 ROS2 控制器
4. 启动 LLM 交互界面
5. 退出时自动清理进程

## Dora 通信

### 概述

Dora 是一个高性能数据流框架，适用于实时机器人应用。

### 文件说明

#### dora-interactive-mcp.yaml

Dora 数据流配置文件，定义节点和连接关系。

#### input_ui.py

Dora 输入界面，提供用户交互入口。

#### requirements.txt

Dora 相关依赖:
```
dora-rs
pyarrow
pyyaml
python-dotenv
pygame
```

### 使用方式

```bash
# 安装 Dora 依赖
pip install dora-rs pyarrow

# 启动 Dora 数据流
cd Middle_Module/Dora
dora up
dora start dora-interactive-mcp.yaml --attach
```

## ROS2 话题说明

### 订阅话题 (输入)

| 话题名 | 消息类型 | 描述 |
|-------|---------|------|
| `/robot_command` | `std_msgs/String` | 接收 LLM/MCP 的 JSON 格式命令 |

### 发布话题 (输出)

| 话题名 | 消息类型 | 描述 |
|-------|---------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 速度命令 (线速度 + 角速度) |
| `/gripper` | `std_msgs/Float64` | 夹爪位置 (0.0-1.0) |
| `/joint_states` | `sensor_msgs/JointState` | 关节状态 |

### 命令格式

`/robot_command` 接收的 JSON 格式命令:

```json
{
  "action": "navigate",
  "parameters": {
    "direction": "front",
    "distance": "1.0m"
  }
}
```

支持的动作类型:
- `navigate`: 导航移动
- `turn_left`: 左转
- `turn_right`: 右转
- `stop`: 停止
- `pick`: 抓取
- `place`: 放置

## 与 Robot_Module 集成

Middle_Module 通过读取 `Robot_Module/{robot}/robot_config.yaml` 动态配置通信参数。

### 配置示例

```yaml
# Robot_Module/Sim_2D/robot_config.yaml
robot:
  name: "Sim2D"
  type: "differential_drive"

communication:
  - ROS2

ros2:
  subscribe:
    command_topic: "/robot_command"
  publish:
    cmd_vel: "/cmd_vel"
```

控制器会自动读取此配置并创建对应的 ROS2 话题。

## 快速开始

### 前置要求

- Python 3.10+
- ROS2 Humble 已安装
- 项目依赖已安装

### 启动流程

#### 方式 1: 2D 仿真 (推荐新手)

```bash
cd Middle_Module/ROS
./start_ros2_mcp.sh --sim 2d
```

这将启动:
- Pygame 2D 仿真窗口
- ROS2 控制器
- LLM 交互界面

#### 方式 2: MuJoCo 3D 仿真

```bash
cd Middle_Module/ROS
./start_ros2_mcp.sh --sim mujoco
```

这将启动:
- MuJoCo 3D 物理仿真
- ROS2 控制器
- LLM 交互界面

#### 方式 3: Dora 数据流

```bash
pip install dora-rs pyarrow
cd Middle_Module/Dora
dora up
dora start dora-interactive-mcp.yaml --attach
```

## 依赖

### ROS2 依赖

```bash
sudo apt install ros-humble-rclpy \
                 ros-humble-geometry-msgs \
                 ros-humble-sensor-msgs \
                 ros-humble-std-msgs
```

### Python 依赖

```
rclpy                 # ROS2 Python 客户端
python-dotenv         # 环境变量管理
pyyaml                # YAML 配置解析
```

### Dora 依赖 (可选)

```
dora-rs               # Dora 数据流框架
pyarrow               # 数据序列化
```

## 开发指南

### 添加新的通信协议

1. 在 `Middle_Module/` 下创建新目录 (如 `MyProtocol/`)
2. 实现通信逻辑 (尽量简洁)
3. 编写配置文件 (如 `myprotocol_config.yaml`)
4. 在主 README 中添加使用说明

### 添加新的 ROS2 消息类型

在 `ros2_robot_controller.py` 中添加:

```python
# 在 _setup_publishers() 中
if 'new_topic' in publish_config:
    topic = publish_config['new_topic']
    self.topic_publishers['new_topic'] = self.create_publisher(
        CustomMsg, topic, 10
    )

# 在 command_callback() 中处理
if action == 'new_action':
    # 处理逻辑
    pass
```

## 故障排除

### ROS2 节点无法连接

1. 检查 ROS2 环境: `source /opt/ros/humble/setup.bash`
2. 检查话题: `ros2 topic list`
3. 检查节点: `ros2 node list`

### 配置文件未加载

1. 确认 `robot_config.yaml` 存在
2. 检查 YAML 语法是否正确
3. 查看控制台错误信息

### 仿真器未启动

1. 检查是否安装了必要依赖 (pygame, mujoco)
2. 确认没有端口冲突
3. 尝试手动启动仿真器

## 相关文档

- [LLM_Module README](../LLM_Module/README.md) - LLM 模块
- [MCP_Module README](../MCP_Module/README.md) - MCP 中间件
- [Robot_Module README](../Robot_Module/README.md) - 机器人模块
- [Sim_Module README](../Sim_Module/README.md) - 仿真模块
- [主项目 README](../README.md) - 项目总览
