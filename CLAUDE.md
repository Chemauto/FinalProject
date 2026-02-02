# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 在此仓库中工作时提供指导。

## 系统概述

这是一个基于 MCP (Model Context Protocol) 模块化架构的**双层 LLM 机器人控制系统**，使用 ROS2 话题通信。系统包含：

- **LLM_Module**：双层 LLM（任务规划 + 执行控制）
- **Robot_Module**：MCP 工具注册中心，带有懒加载的 ROS2 队列
- **VLM_Module**：视觉语言模型，用于颜色检测和动作映射
- **Interactive_Module**：CLI 交互界面，协调 LLM 和机器人模块
- **Sim_Module**：2D Pygame 仿真器，用于可视化和追击功能
- **Yolo_Module**：YOLO 目标检测模块

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
ros2 topic echo /robot/enemies

# 查看话题信息
ros2 topic info /robot/command

# 可视化节点图
ros2 run rqt_graph rqt_graph
```

## 架构详解

### 双层 LLM 设计

`LLM_Module/llm_core.py` 实现了两层架构：

1. **上层（规划）**：使用 `LLM_Module/prompts/planning_prompt_2d.yaml` 中的提示词，将复杂的用户输入分解为子任务序列
2. **下层（执行）**：通过 OpenAI function-calling API 调用 Robot_Module 工具来执行各个子任务，支持 `previous_result` 传递

核心方法：`run_pipeline(user_input, tools, execute_tool_fn)` 协调整个流程。

### MCP 工具注册

`Robot_Module/skill.py` 是 FastMCP 服务器的入口点。工具在 `Robot_Module/module/` 中注册：

- **base.py**：底盘控制（move_forward, move_backward, turn, stop）
- **chase.py**：追击功能（get_enemy_positions, chase_enemy）- **2个MCP工具**
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

ROS2 话题列表：
- `/robot/command` - 动作命令（Robot_Module → Sim_Module）
- `/robot/state` - 机器人状态（Sim_Module → Robot_Module）
- `/robot/enemies` - 敌人位置（Sim_Module → Robot_Module）
- `/robot/enemy_remove` - 清除敌人命令（Robot_Module → Sim_Module）

### 懒加载模式

Robot_Module 的所有模块都通过 `_get_action_queue()` 和 `_get_vlm_core()` 使用懒加载，将 ROS2/VLM 的初始化推迟到首次使用时，避免启动时的依赖问题。

### 动态提示词加载

`Interactive_Module/interactive.py` 动态填充规划提示词：
- `robot_config`：机器人类型和功能描述
- `available_skills`：格式化的可用工具列表

这使得规划 LLM 能够理解当前系统能力，无需硬编码。

### 追击功能

`Robot_Module/module/chase.py` 实现了完整的追击功能，包含 **2个MCP工具**：

1. **get_enemy_positions()** - 获取敌人位置
   - 初始化ROS2订阅器
   - 等待连接建立（0.5秒订阅器初始化 + 1.0秒额外等待）
   - 刷新回调50次确保接收消息
   - 返回敌人位置JSON字符串

2. **chase_enemy(enemy_positions)** - 追击最近的敌人
   - 接收敌人位置JSON字符串
   - 找到最近的敌人
   - 计算角度并旋转
   - 前进（PID控制，自适应角度校正）
   - 到达后清除敌人
   - 等待仿真器更新位置

**追击算法特性**：
- **角度计算**：使用 atan2 计算目标方向，标准化到 [-180°, 180°]
- **自适应校正**：检测距离连续增大3次时，重新计算角度并旋转
- **PID控制**：步长 = 距离 * 0.8，最大1.0米，最小0.1米
- **到达阈值**：5像素

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
- **追击模块**：`Robot_Module/module/chase.py`
- **仿真器**：`Sim_Module/sim2d/simulator.py`
- **敌人管理器**：`Sim_Module/enemy_manager.py`
- **ROS2 通信**：`ros_topic_comm.py`
- **数据流文档**：`process.md`

## 关键配置

- **API 密钥**：通过 `Test_API_KEY` 环境变量或 `.env` 文件设置
- **LLM 模型**：在 `LLM_Module/llm_core.py` 中配置（默认：qwen3-32b）
- **VLM 模型**：在相应的 VLM 模块文件中配置
- **ROS2 话题**：
  - `/robot/command`（std_msgs/String，JSON 载荷）
  - `/robot/state`、`/robot/enemies`、`/robot/enemy_remove`

## 仿真器配置

- **屏幕尺寸**：800x600 像素（在 `simulator.py` 中修改 WIDTH, HEIGHT）
- **比例尺**：1米 = 100像素
- **帧率**：60 FPS
- **敌人位置发布频率**：每3秒（180帧）
- **机器人更新**：平滑插值，每帧5%移动

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

## 追击功能使用

1. **启动仿真器**：
   ```bash
   python3 Sim_Module/sim2d/simulator.py
   ```
   - 鼠标左键：生成敌人
   - 按 C：清除所有敌人
   - 按 L：切换追击线显示
   - 按 ESC：退出

2. **启动交互程序**：
   ```bash
   python3 Interactive_Module/interactive.py
   ```

3. **输入命令**：
   ```
   追击敌人
   ```

系统会自动：
1. 获取敌人位置（步骤1）
2. 追击最近的敌人（步骤2）
3. 清除已追击的敌人
4. 等待仿真器更新位置
5. 可以继续追击下一个敌人

## 常见问题

### Q: 追击时机器人不动？

A: 确保仿真器在运行并且已经生成敌人。检查是否看到日志：
- `[Simulator] 发布敌人位置: [...]`
- `[chase.get_enemy_positions] 获取到 N 个敌人`

### Q: 获取不到敌人位置？

A: ROS2 订阅者需要时间建立连接。确保：
1. 仿真器先启动并已发布敌人位置
2. 等待几秒后再输入"追击敌人"命令
3. 查看日志中的 `[EnemyPositionsSubscriber] 已更新缓存`

### Q: 追击后第二个敌人还是追到第一个的位置？

A: 这是已修复的问题。现在追击完成后会：
1. 发送 `remove_enemy` 命令给仿真器
2. 仿真器移除敌人并立即发布更新后的位置
3. 等待1秒确保消息传递
4. 下次追击会获取新的位置

### Q: 机器人在接近目标时来回移动？

A: 已实现自适应角度校正：
- 检测距离连续增大3次
- 重新计算角度并旋转
- 如果角度已对准但仍偏离，减小步长

## 数据流参考

详细的数据流、输入输出关系和时序图请参考：**`process.md`**

该文档包含：
- 整体架构数据流图
- 追击功能详细流程
- ROS2 话题发布-订阅关系
- 角度计算和自适应校正算法
- 时序图和缓存机制


特别重要的：不要尝试新建太多md文件