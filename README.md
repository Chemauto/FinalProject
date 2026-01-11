# 🤖 Robot Control with MCP + Dual-Layer LLM (Modular Architecture)

基于 Model Context Protocol (MCP) 的机器人控制系统，采用双层LLM架构与完全模块化设计。

## 🎯 核心特性

- **双层 LLM 架构**:
  - 上层 LLM (任务规划): 将复杂指令分解为子任务序列
  - 下层 LLM (执行控制): 将子任务转换为具体技能调用
- **7个核心模块**: 每个模块职责清晰,易于维护和扩展
- **完全模块化**: 添加新机器人只需创建新文件夹,无需修改核心代码
- **通用 ROS2 桥接**: 根据配置文件动态创建 ROS2 话题，支持任意机器人
- **多通信协议**: 支持 ROS2、Dora 等多种机器人通信方式
- **VLM集成**: 支持视觉语言模型进行环境感知

## 📁 项目结构 (7个核心模块)

```
FinalProject/
├── README.md
│
├── LLM_Module/                 # ① 大语言模型模块 (双层LLM)
│   ├── llm_core.py            # LLMAgent 类
│   └── prompts/               # YAML 提示词
│
├── VLM_Module/                 # ② 视觉语言模型模块
│   ├── vlm_core.py            # VLMCore 类
│   └── prompts/               # VLM 提示词
│
├── Middle_Module/              # ③ 中间通信层 (纯通信，代码最少)
│   ├── ROS/                   # ROS2 通信
│   │   ├── ros2_interactive_mcp.py
│   │   ├── ros2_robot_controller.py   # 通用 ROS2 桥接
│   │   └── start_ros2_mcp.sh
│   └── Dora/                  # Dora 通信
│       ├── dora-interactive-mcp.yaml
│       ├── input_ui.py
│       └── requirements.txt
│
├── MCP_Module/                 # ④ MCP 中间件 (技能注册和调用)
│   └── mcp_bridge.py          # MCPBridge + SkillRegistry
│
├── Sim_Module/                 # ⑤ 仿真模块 (所有仿真代码)
│   ├── ros2_2d/               # ROS2 版 2D 仿真
│   │   └── simulator.py
│   ├── dora_2d/               # Dora 版 2D 仿真
│   │   └── simulator.py
│   └── mujoco/                # MuJoCo 3D 物理仿真
│       ├── mujoco_simulator.py
│       ├── install_mujoco.sh
│       └── start_mujoco_sim.sh
│
├── Real_Module/                # ⑥ 真实机器人模块
│   └── __init__.py            # 硬件驱动接口 (待实现)
│
└── Robot_Module/               # ⑦ 机器人模块 (配置和技能)
    ├── Go2_Quadruped/         # Go2 四足机器人
    │   ├── robot_config.yaml  # ROS2 话题映射
    │   └── skills/            # 技能实现
    └── Sim_2D/                # 2D 仿真机器人
        ├── robot_config.yaml
        └── skills/
```

## 🔄 双层LLM工作流程

```
用户输入: "向左移动1米,然后旋转90度"
    ↓
┌─────────────────────────────────────────┐
│      LLM Module - 上层LLM (任务规划)      │
│  Input: 用户指令 + planning_prompt       │
│  Output: 子任务序列                       │
└──────────────┬──────────────────────────┘
               ↓
┌─────────────────────────────────────────┐
│      LLM Module - 下层LLM (执行控制)      │
│  Input: 单个子任务                        │
│  Output: 技能调用                          │
└──────────────┬──────────────────────────┘
               ↓
┌─────────────────────────────────────────┐
│      MCP Module - 技能注册与调用          │
│  从 Robot_Module 加载技能                │
└──────────────┬──────────────────────────┘
               ↓
┌─────────────────────────────────────────┐
│   Robot_Module/Sim_2D/skills/            │
│   执行技能函数，返回 action + parameters  │
└──────────────┬──────────────────────────┘
               ↓
┌─────────────────────────────────────────┐
│   Middle_Module/ROS/                     │
│   读取 robot_config.yaml                  │
│   根据配置发布到对应 ROS2 话题             │
└──────────────┬──────────────────────────┘
               ↓
      🤖 Sim_Module (仿真环境)
```

## 🚀 快速开始

### 方式 1: ROS2 (2D 仿真)

最基础的模式,用于快速验证逻辑。

```bash
cd Middle_Module/ROS
./start_ros2_mcp.sh --sim 2d
```

### 方式 2: ROS2 (Gazebo 3D 仿真)

```bash
cd Middle_Module/ROS
./start_ros2_mcp.sh --sim mujoco
```

### 方式 3: Dora (仿真测试)

```bash
pip install dora-rs pyarrow pyyaml python-dotenv pygame
cd Middle_Module/Dora
dora up
dora start dora-interactive-mcp.yaml --attach
```

## 🔧 添加新机器人 (只需4步!)

### 步骤1: 创建机器人文件夹

```bash
mkdir -p Robot_Module/MyNewRobot/skills
```

### 步骤2: 创建配置文件

`Robot_Module/MyNewRobot/robot_config.yaml`:
```yaml
robot:
  name: "MyRobot"
  type: "custom"

communication:
  - ROS2

ros2:
  subscribe:
    command_topic: "/robot_command"
  publish:
    cmd_vel: "/cmd_vel"
```

### 步骤3: 创建技能文件

`Robot_Module/MyNewRobot/skills/myrobot_skills.py`:
```python
def skill_move_forward(distance: float = 1.0, speed: float = 0.3):
    """向前移动"""
    return {
        'action': 'navigate',
        'parameters': {'direction': 'front', 'distance': f'{distance}m'}
    }
```

### 步骤4: 创建技能模块导出

`Robot_Module/MyNewRobot/skills/__init__.py`:
```python
from .myrobot_skills import *
```

完成! 系统会自动加载新机器人。

## 📦 模块说明

### ① LLM_Module - 大语言模型模块

**职责**:
- 任务规划: 用户指令 → 子任务序列
- 执行控制: 子任务 → 技能调用

**核心文件**:
- `llm_core.py`: LLMAgent类实现
- `prompts/`: YAML格式提示词

### ② VLM_Module - 视觉语言模型模块

**职责**:
- 环境感知: 图像 → 环境描述
- 障碍物检测: 图像 → 障碍物列表

**状态**: 待实现

### ③ Middle_Module - 中间通信层

**职责**:
- 统一通信接口
- 代码尽可能少
- 通用 ROS2 桥接

**子模块**:
- `ROS/`: ROS2通信
  - `ros2_interactive_mcp.py`: MCP交互
  - `ros2_robot_controller.py`: **通用 ROS2 桥接**
  - `start_ros2_mcp.sh`: 启动脚本
- `Dora/`: Dora通信

### ④ MCP_Module - MCP 中间件

**职责**:
- 从 Robot_Module 加载和注册技能
- 提供 MCP 工具定义
- 执行技能

**核心文件**:
- `mcp_bridge.py`: MCPBridge + SkillRegistry

### ⑤ Sim_Module - 仿真模块

**职责**:
- 提供各种仿真环境
- 支持多种通信方式

**子模块**:
- `ros2_2d/`: ROS2 版 2D 仿真
- `dora_2d/`: Dora 版 2D 仿真
- `gazebo/`: Gazebo 3D 仿真

### ⑥ Real_Module - 真实机器人模块

**职责**:
- 硬件驱动接口
- 传感器数据采集

**状态**: 待实现

### ⑦ Robot_Module - 机器人模块 (核心!)

**职责**:
- 定义机器人配置
- 实现机器人技能
- 配置 ROS2 话题映射

**结构**:
```
Robot_Module/RobotName/
├── robot_config.yaml       # 机器人配置和 ROS2 话题映射
└── skills/                  # 技能实现
    ├── __init__.py
    └── robotname_skills.py
```

## 🎯 使用示例

### 使用 MCP Bridge

```python
from MCP_Module import create_mcp_bridge

# 创建桥接并加载机器人
bridge = create_mcp_bridge(['Sim_2D', 'Go2_Quadruped'])

# 查看可用技能
skills = bridge.get_available_skills()

# 执行技能
result = bridge.execute_skill('move_forward', distance=1.0)
```

### 使用 LLM Agent

```python
from LLM_Module import LLMAgent

llm = LLMAgent(api_key="your_api_key")

# 规划任务
tasks = llm.plan_tasks("向前走2米,然后左转90度", tools=[])

# 执行流程
results = llm.run_pipeline("向前走2米", tools=[], execute_tool_fn=fn)
```

## 🔗 模块依赖关系

```
用户输入
  ↓
LLM_Module (双层LLM处理)
  ↓
MCP_Module (技能管理)
  ↓
Robot_Module (技能定义 + ROS2话题映射)
  ↓
Middle_Module (通信层 - 根据配置发布话题)
  ↓
Sim_Module / Real_Module (执行)
```

## 📚 扩展指南

- [LLM_Module README](LLM_Module/README.md)
- [VLM_Module README](VLM_Module/README.md)
- [Middle_Module README](Middle_Module/README.md)
- [MCP_Module README](MCP_Module/README.md)
- [Sim_Module README](Sim_Module/README.md)
- [Real_Module README](Real_Module/README.md)
- [Robot_Module README](Robot_Module/README.md)

---

**开始使用**: 选择一种仿真方式，按照上面的指令启动。

**项目已完全模块化,添加新机器人只需4步!**
