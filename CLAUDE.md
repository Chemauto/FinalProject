# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 在此仓库中工作时提供指导。

## 系统概述

这是一个基于 MCP (Model Context Protocol) 模块化架构的**双层 LLM 机器人控制系统**，使用 ROS2 话题通信。系统包含：

- **LLM_Module**：双层 LLM（任务规划 + 执行控制）
- **Robot_Module**：MCP 工具注册中心，带有懒加载的 ROS2 队列
- **VLM_Module**：视觉语言模型，用于颜色检测和动作映射
- **Interactive_Module**：CLI 交互界面，协调 LLM 和机器人模块
- **Sim_Module**：2D Pygame 仿真器，用于可视化

### 架构流程

```
用户输入 → Interactive_Module → LLM_Module (规划) → LLM_Module (执行)
                                                        ↓
                                                Robot_Module (工具)
                                                        ↓
                                            ROS2 话题 (/robot/command)
                                                        ↓
                                                Sim_Module (仿真器)
```

## 常用命令

### 运行系统

```bash
# 一键启动（推荐）
./start_robot_system.sh

# 手动启动 - 终端1：仿真器
python3 Sim_Module/sim2d/simulator.py

# 手动启动 - 终端2：交互界面
python3 Interactive_Module/interactive.py
```

### 环境配置

```bash
# 安装依赖
pip install -r requirements.txt

# 设置 API 密钥（LLM/VLM 必需）
export Test_API_KEY=your_api_key_here

# 或使用 .env 文件（Interactive_Module 会自动加载）
echo "Test_API_KEY=your_api_key_here" > .env
```

### ROS2 调试（可选）

```bash
# 列出话题
ros2 topic list

# 查看话题消息
ros2 topic echo /robot/command

# 查看话题信息
ros2 topic info /robot/command

# 可视化节点图
ros2 run rqt_graph rqt_graph
```

## 架构详解

### 双层 LLM 设计

`LLM_Module/llm_core.py` 实现了两层架构：

1. **上层（规划）**：使用 `LLM_Module/prompts/planning_prompt_2d.yaml` 中的提示词，将复杂的用户输入分解为子任务序列
2. **下层（执行）**：通过 OpenAI function-calling API 调用 Robot_Module 工具来执行各个子任务

核心方法：`run_pipeline(user_input, tools, execute_tool_fn)` 协调整个流程。

### MCP 工具注册

`Robot_Module/skill.py` 是 FastMCP 服务器的入口点。工具在 `Robot_Module/module/` 中注册：

- **base.py**：底盘控制（move_forward, move_backward, turn, stop）
- **vision.py**：使用 VLM 进行颜色检测（detect_color_and_act）

添加新工具：
1. 在模块文件中创建函数，使用 `@mcp.tool()` 装饰器并编写完整的 docstring
2. 在 `register_tools()` 函数中添加
3. 在 `skill.py` 中调用 `register_your_tools(mcp)`

### ROS2 通信

`ros_topic_comm.py` 提供了围绕 ROS2 话题的 `SharedCommandQueue` 封装：

- **发布者**：Robot_Module 以 JSON 格式发送动作命令
- **订阅者**：Sim_Module 接收并执行命令
- **队列**：线程安全的 `queue.Queue()` 缓冲动作

动作格式：
```json
{
  "action": "move_forward",
  "parameters": {"distance": 1.0, "speed": 0.3}
}
```

### 懒加载模式

Robot_Module 的所有模块都通过 `_get_action_queue()` 和 `_get_vlm_core()` 使用懒加载，将 ROS2/VLM 的初始化推迟到首次使用时，避免启动时的依赖问题。

### 动态提示词加载

`Interactive_Module/interactive.py` 动态填充规划提示词：
- `robot_config`：机器人类型和功能描述
- `available_skills`：格式化的可用工具列表

这使得规划 LLM 能够理解当前系统能力，无需硬编码。

### VLM 集成

提供两种 VLM 实现：
- **本地**：`VLM_Module/vlm_core.py` - 使用 Ollama 客户端运行本地模型
- **远程**：`VLM_Module/vlm_core_remote.py` - 使用 OpenAI 兼容 API（Qwen VL）

两者都基于检测到的颜色返回标准化的动作映射。

## 重要文件位置

- **规划提示词**：`LLM_Module/prompts/planning_prompt_2d.yaml`
- **VLM 提示词**：`VLM_Module/prompts/perceive_environment.yaml`
- **入口点**：`Interactive_Module/interactive.py`
- **MCP 服务器**：`Robot_Module/skill.py`
- **仿真器**：`Sim_Module/sim2d/simulator.py`
- **ROS2 通信**：`ros_topic_comm.py`

## 关键配置

- **API 密钥**：通过 `Test_API_KEY` 环境变量或 `.env` 文件设置
- **LLM 模型**：在 `LLM_Module/llm_core.py` 中配置（默认：qwen3-32b）
- **VLM 模型**：在相应的 VLM 模块文件中配置
- **ROS2 话题**：`/robot/command`（std_msgs/String，JSON 载荷）

## 测试

测试单个模块：
```bash
# 测试 ROS2 通信
python3 ros_topic_comm.py

# 测试 VLM（需要 Ollama 或 API 密钥）
python3 VLM_Module/vlm_core.py
python3 VLM_Module/vlm_core_remote.py

# 测试 MCP 工具（运行 MCP 服务器）
python3 Robot_Module/skill.py
```
