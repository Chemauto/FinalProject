# 🤖 FinalProject - 基于双层LLM的机器人控制系统

基于双层 LLM 架构的机器人控制系统，采用 MCP (Model Context Protocol) 模块化设计。

## 🎯 核心特性

- **双层 LLM 架构**: 任务规划 + 执行控制分离
- **MCP 模块化设计**: 参考 RoboOS，工具注册标准化
- **简化通信**: multiprocessing.Queue，无 ROS2 依赖
- **自然语言控制**: 支持中文指令控制机器人
- **2D 仿真**: 基于 Pygame 的轻量级仿真环境

## 🔄 系统架构与数据流

```
用户输入: "前进1米然后左转90度"
    ↓
┌─────────────────────────────────────────────┐
│ ① Interactive_Module (交互界面)             │
│    - 接收用户自然语言指令                     │
│    - 协调 LLM 和 Robot_Module                │
│    - 显示执行结果                             │
└──────────────┬──────────────────────────────┘
               ↓
┌─────────────────────────────────────────────┐
│ ② LLM_Module - 上层LLM (任务规划)           │
│    输入: 用户指令 + 可用技能列表              │
│    输出: 子任务序列 ( [{"task": "前进1米"}, {"task": "左转90度"}] ) │
└──────────────┬──────────────────────────────┘
               ↓
┌─────────────────────────────────────────────┐
│ ② LLM_Module - 下层LLM (执行控制)           │
│    输入: 单个子任务描述                       │
│    输出: 工具调用 (move_forward(distance=1.0)) │
└──────────────┬──────────────────────────────┘
               ↓
┌─────────────────────────────────────────────┐
│ ③ Robot_Module (MCP 工具注册中心)           │
│    ├─ skill.py: FastMCP 服务器              │
│    ├─ module/base.py: 移动/旋转/停止技能    │
│    └─ module/example.py: 示例模板           │
│    输入: 工具调用请求                         │
│    输出: 动作指令 JSON ({"action": "move_forward", ...}) │
└──────────────┬──────────────────────────────┘
               ↓
      multiprocessing.Queue (进程间通信)
               ↓
┌─────────────────────────────────────────────┐
│ ④ Sim_Module (2D 仿真器)                    │
│    输入: 动作指令 JSON                       │
│    输出: 机器人运动可视化 + 状态更新          │
└─────────────────────────────────────────────┘
```

## 📁 项目结构

```
FinalProject/
├── README.md
│
├── LLM_Module/            # 双层LLM核心
│   ├── llm_core.py        # LLMAgent: 规划 + 执行
│   └── prompts/           # YAML 提示词模板
│
├── VLM_Module/            # 视觉语言模型 (预留)
│   └── vlm_core.py        # VLMCore: 图像分析
│
├── Robot_Module/          # MCP 工具注册中心
│   ├── skill.py           # FastMCP 服务器入口
│   └── module/            # 功能模块
│       ├── base.py        # 底盘控制
│       └── example.py     # 示例模板
│
├── Interactive_Module/    # 交互界面
│   └── interactive.py     # CLI 交互主程序
│
└── Sim_Module/            # 仿真模块
    └── sim2d/
        └── simulator.py   # 2D Pygame 仿真器
```

## 🚀 快速开始

### 环境要求

- Python 3.10+
- Linux (推荐 Ubuntu 22.04)

### 安装

```bash
# 1. 克隆项目
cd /home/robot/work/FinalProject

# 2. 安装依赖
pip install -r requirements.txt

# 3. 配置 API Key
export Test_API_KEY=your_api_key_here
```

### 运行

```bash
# 一键启动 (推荐)
./start_robot_system.sh

# 或手动启动
# 终端1: python3 Sim_Module/sim2d/simulator.py
# 终端2: python3 Interactive_Module/interactive.py
```

### 交互示例

```
💬 请输入指令: 前进1米然后左转90度

[上层LLM] 任务规划:
  步骤1: 前进1米
  步骤2: 左转90度

[下层LLM] 执行控制:
  调用工具: move_forward(distance=1.0) ✅
  调用工具: turn(angle=90.0) ✅

📊 [完成] 2/2 个任务成功
```

## 🔧 添加新工具

1. **创建模块文件**
```bash
cd Robot_Module/module
cp example.py your_module.py
```

2. **编写工具函数**
```python
async def your_tool(param: float) -> str:
    """工具描述

    Args:
        param: 参数描述
    """
    action = {'action': 'your_action', 'parameters': {'param': param}}
    return json.dumps(action)
```

3. **注册到 skill.py**
```python
# skill.py
from module.your_module import register_tools as register_your_tools

def register_all_modules():
    register_your_tools(mcp, _tool_registry, _tool_metadata)
```

## 📦 模块说明

| 模块 | 职责 | 核心文件 |
|------|------|----------|
| **Interactive_Module** | 交互界面 | `interactive.py` |
| **LLM_Module** | 双层LLM: 规划+执行 | `llm_core.py` |
| **Robot_Module** | MCP工具注册中心 | `skill.py` |
| **Sim_Module** | 2D仿真环境 | `sim2d/simulator.py` |
| **VLM_Module** | 视觉感知 (预留) | `vlm_core.py` |

详细说明请查看各模块的 README.md。

## 📝 依赖

```
openai>=1.0.0         # OpenAI API (兼容 Qwen/Dashscope)
fastmcp>=0.1.0        # MCP 服务器框架
pygame>=2.5.0         # 2D 仿真可视化
pyyaml>=6.0           # YAML 配置解析
numpy>=1.26.0         # 数值计算
python-dotenv>=1.0.0  # 环境变量管理
```

## 📚 相关文档

- [LLM_Module README](LLM_Module/README.md)
- [VLM_Module README](VLM_Module/README.md)
- [Robot_Module README](Robot_Module/README.md)
- [Interactive_Module README](Interactive_Module/README.md)
- [Sim_Module README](Sim_Module/README.md)

---

**模块化，易扩展！** 🚀
