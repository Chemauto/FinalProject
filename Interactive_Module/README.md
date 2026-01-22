# 🤖 FinalProject - 基于双层LLM的机器人控制系统

基于双层 LLM 架构的机器人控制系统，采用 MCP (Model Context Protocol) 模块化设计，使用 **ROS2 话题**进行通信。

## 📑 目录

- [项目概述](#-项目概述)
- [快速开始](#-快速开始)
- [Interactive_Module - 交互界面](#interactive_module---交互界面)
- [LLM_Module - 双层LLM核心](#llm_module---双层llm核心)
- [Robot_Module - MCP工具注册中心](#robot_module---mcp工具注册中心)
- [Sim_Module - 2D仿真环境](#sim_module---2d仿真环境)
- [Test_Module - 追击功能](#test_module---追击功能)
- [VLM_Module - 视觉语言模型](#vlm_module---视觉语言模型)
- [Yolo_Module - YOLO目标检测](#yolo_module---yolo目标检测)
- [ROS2通讯](#-ros2通讯)
- [依赖安装](#-依赖安装)

---

## 🎯 项目概述

### 核心特性

- **双层 LLM 架构**: 任务规划 + 执行控制分离
- **MCP 模块化设计**: 参考 RoboOS，工具注册标准化
- **ROS2 通讯**: 使用 ROS2 话题，标准化通信
- **自然语言控制**: 支持中文指令控制机器人
- **2D 仿真**: 基于 Pygame 的轻量级仿真环境
- **追击功能**: 自动追击敌人，PID控制精确定位
- **视觉检测**: VLM 颜色检测与动作映射
- **懒加载设计**: 自动初始化，零配置

### 系统架构

```
用户输入: "追击敌人" 或 "前进1米然后左转90度"
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
│    输出: 子任务序列                           │
└──────────────┬──────────────────────────────┘
               ↓
┌─────────────────────────────────────────────┐
│ ② LLM_Module - 下层LLM (执行控制)           │
│    输入: 单个子任务描述                       │
│    输出: 工具调用                             │
└──────────────┬──────────────────────────────┘
               ↓
┌─────────────────────────────────────────────┐
│ ③ Robot_Module (MCP 工具注册中心)           │
│    ├─ skill.py: FastMCP 服务器              │
│    ├─ module/base.py: 移动/旋转/停止技能    │
│    ├─ module/chase.py: 追击技能             │
│    └─ module/vision.py: 视觉检测技能        │
└──────────────┬──────────────────────────────┘
               ↓
      ROS2 Topic (/robot/command, /robot/enemies, /robot/state)
         ↓ 发布 JSON 消息
┌─────────────────────────────────────────────┐
│ ④ Sim_Module (2D 仿真器)                    │
│    ├─ 订阅 ROS2 话题                         │
│    ├─ 机器人运动可视化                       │
│    ├─ 敌人管理 (spawn/move/remove)           │
│    └─ 状态发布 (robot/enemy positions)       │
└─────────────────────────────────────────────┘
```

### 项目结构

```
FinalProject/
├── README.md              # 本文档
├── requirements.txt       # Python 依赖
├── .env                   # API 密钥配置
│
├── Interactive_Module/    # 交互界面
│   └── interactive.py     # CLI 交互主程序
│
├── LLM_Module/            # 双层LLM核心
│   ├── llm_core.py        # LLMAgent: 规划 + 执行
│   └── prompts/           # YAML 提示词模板
│       └── planning_prompt_2d.yaml
│
├── Robot_Module/          # MCP 工具注册中心
│   ├── skill.py           # FastMCP 服务器入口
│   └── module/            # 功能模块
│       ├── base.py        # 底盘控制 (move/turn/stop)
│       ├── chase.py       # 追击功能 (chase_enemy)
│       └── vision.py      # 视觉检测 (detect_color)
│
├── Sim_Module/            # 仿真模块
│   └── sim2d/
│       └── simulator.py   # 主仿真器
│
├── Test_Module/           # 测试模块
│   ├── chase_core.py      # 追击算法核心
│   ├── enemy_manager.py   # 敌人管理器
│   └── chase_simulator.py # 追击仿真器 (简化版)
│
├── VLM_Module/            # 视觉语言模型
│   ├── vlm_core.py        # 本地 VLM (Ollama)
│   ├── vlm_core_remote.py # 远程 VLM (API)
│   └── prompts/           # VLM 提示词
│
├── Yolo_Module/           # YOLO目标检测
│   ├── target_detector.py # 目标检测器
│   ├── screen_capture.py  # 屏幕捕获
│   └── coordinate_mapper.py # 坐标映射
│
└── ros_topic_comm.py      # ROS2 通讯模块
```

---

## 🚀 快速开始

### 环境要求

- Python 3.10+
- Linux (推荐 Ubuntu 22.04)
- ROS2 Humble (可选，用于调试)

### 安装

```bash
# 1. 进入项目目录
cd /home/xcj/work/testfinal/FinalProject

# 2. 安装 Python 依赖
pip install -r requirements.txt

# 3. 配置 API Key
export Test_API_KEY=your_api_key_here

# 或创建 .env 文件
echo "Test_API_KEY=your_api_key_here" > .env
```

### 运行

```bash
# 方式1: 一键启动（推荐）
./start_robot_system.sh

# 方式2: 手动启动
# 终端1: 启动仿真器
python3 Sim_Module/sim2d/simulator.py

# 终端2: 启动交互程序
python3 Interactive_Module/interactive.py
```

### 交互示例

```bash
💬 请输入指令: 前进1米然后左转90度

[上层LLM] 任务规划:
  步骤1: 前进1米
  步骤2: 左转90度

[下层LLM] 执行控制:
  调用工具: move_forward(distance=1.0) ✅
  调用工具: turn(angle=90.0) ✅

📊 [完成] 2/2 个任务成功
```

---


====================================
各模块详细文档
====================================

## Interactive_Module - 交互界面
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
Interactive_Module 模块详细文档
====================================

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
- **双层 LLM 协调**: 自动协调规划 LLM 和执行 LLM
- **动态提示词加载**: 根据可用技能动态生成提示词
- **实时结果反馈**: 显示任务规划和执行过程
- **环境变量支持**: 自动加载 .env 文件
- **错误处理**: 友好的错误提示信息

## 🔧 核心组件

### interactive.py - CLI 交互主程序

**主要功能：**

1. **环境初始化**
   - 加载 .env 文件
   - 设置 API 密钥
   - 初始化 ROS2 通讯

2. **MCP 工具加载**
   - 启动 FastMCP 服务器
   - 注册所有 Robot_Module 工具
   - 获取可用工具列表

3. **动态提示词生成**
   - 读取规划提示词模板
   - 填充机器人配置
   - 填充可用技能列表

4. **双层 LLM 流程**
   - 上层 LLM: 任务规划
   - 下层 LLM: 执行控制
   - 结果汇总和反馈

**使用方式：**

```bash
# 直接启动
python3 Interactive_Module/interactive.py

# 或使用绝对路径
cd /home/xcj/work/testfinal/FinalProject
python3 Interactive_Module/interactive.py
```

**交互示例：**

```bash
💬 请输入指令 (quit/exit 退出): 前进1米然后左转90度

==========================================
[上层 LLM] 任务规划
==========================================
  步骤1: 前进1米
  步骤2: 左转90度

整体任务概述: 先转向再前进

==========================================
[下层 LLM] 执行控制
==========================================
  ✓ 任务 1/2: 前进1米
    → 调用工具: move_forward(distance=1.0, speed=0.3)
    → 执行成功

  ✓ 任务 2/2: 左转90度
    → 调用工具: turn(angle=90.0, angular_speed=0.5)
    → 执行成功

==========================================
📊 [完成] 2/2 个任务成功
==========================================
```

## 📝 配置说明

### 环境变量配置

创建 `.env` 文件（项目根目录）：

```bash
# API 密钥（必需）
Test_API_KEY=your_api_key_here

# LLM 模型配置（可选）
LLM_MODEL=qwen3-32b
LLM_BASE_URL=https://your-api-endpoint

# VLM 模型配置（可选）
VLM_MODEL=qwen-vl-plus
```

### 提示词配置

规划提示词位于 `LLM_Module/prompts/planning_prompt_2d.yaml`：

```yaml
system_prompt: |
  你是一个机器人任务规划助手。你的职责是将用户的复杂指令分解为可执行的子任务序列。

  机器人配置:
  {robot_config}

  注意:
  - 输出必须是有效的JSON格式
  - 只分解与可用技能相关的指令
  - 如果输入无效或不属于机器人操作，返回空任务列表
```

## 🔌 API 接口

### LLMAgent Pipeline

```python
async def run_pipeline(
    user_input: str,          # 用户输入
    tools: list,              # 可用工具列表
    execute_tool_fn: callable # 工具执行函数
) -> dict:
    """
    双层 LLM 流水线

    Returns:
        {
            'success': bool,
            'tasks': list,
            'results': list,
            'summary': str
        }
    """
```

### 工具执行函数

```python
async def execute_tool(tool_name: str, parameters: dict) -> str:
    """
    执行单个工具

    Args:
        tool_name: 工具名称（如 'move_forward'）
        parameters: 参数字典（如 {'distance': 1.0}）

    Returns:
        执行结果 JSON 字符串
    """
```

## 💡 使用示例

### 示例 1: 基础运动控制

```bash
💬 请输入指令: 前进2米，然后后退1米

[上层 LLM] 任务规划:
  步骤1: 前进2米
  步骤2: 后退1米

[下层 LLM] 执行控制:
  ✓ 调用工具: move_forward(distance=2.0, speed=0.3)
  ✓ 调用工具: move_backward(distance=1.0, speed=0.3)

📊 [完成] 2/2 个任务成功
```

### 示例 2: 追击敌人

```bash
💬 请输入指令: 追击最近的敌人

[上层 LLM] 任务规划:
  步骤1: 追击最近的敌人

[下层 LLM] 执行控制:
  ✓ 调用工具: chase_enemy()
  [追击] 找到敌人 1 at (612, 114)
  [追击] 距离: 282.0 像素 (2.82 米)
  [追击] ✓ 已到达目标！

📊 [完成] 1/1 个任务成功
```

### 示例 3: 复杂任务

```bash
💬 请输入指令: 先左转45度，前进1.5米，然后右转90度

[上层 LLM] 任务规划:
  步骤1: 左转45度
  步骤2: 前进1.5米
  步骤3: 右转90度

[下层 LLM] 执行控制:
  ✓ 调用工具: turn(angle=-45.0, angular_speed=0.5)
  ✓ 调用工具: move_forward(distance=1.5, speed=0.3)
  ✓ 调用工具: turn(angle=90.0, angular_speed=0.5)

📊 [完成] 3/3 个任务成功
```

## 🐛 调试技巧

### 1. 查看 LLM 提示词

在 `interactive.py` 中添加打印：

```python
print("=== 规划提示词 ===")
print(planning_prompt)
print("==================")
```

### 2. 查看工具调用

```python
async def execute_tool(tool_name, parameters):
    print(f"[DEBUG] 调用工具: {tool_name}({parameters})")
    result = await mcp_call(tool_name, parameters)
    print(f"[DEBUG] 工具返回: {result}")
    return result
```

### 3. 测试单个工具

```python
# 在交互程序中输入
move_forward(distance=1.0, speed=0.3)
```

### 常见问题

**Q1: 提示 "API Key 未配置"**
- 检查 `.env` 文件是否存在
- 检查 `Test_API_KEY` 是否设置

**Q2: 工具调用失败**
- 检查 Robot_Module 是否正常启动
- 检查 ROS2 通讯是否正常

**Q3: LLM 返回无效 JSON**
- 检查提示词格式
- 检查 LLM 模型是否支持

## 🔗 相关模块

- `LLM_Module/llm_core.py` - 双层 LLM 核心
- `Robot_Module/skill.py` - MCP 工具注册
- `ros_topic_comm.py` - ROS2 通讯

## 📝 依赖

```
openai>=1.0.0         # OpenAI API
python-dotenv>=1.0.0  # 环境变量管理
pyyaml>=6.0           # YAML 配置解析
```

---

**用户友好的交互界面！** 💬





# Interactive_Module - 交互界面

## 概述

Interactive_Module 是系统的**用户交互入口**，负责接收用户自然语言指令，协调 LLM_Module 和 Robot_Module 完成任务执行，并显示结果。

### 核心功能

- **交互界面**: 提供命令行交互界面 (CLI)
- **模块协调**: 连接 LLM_Module 和 Robot_Module
- **动态提示词**: 自动生成包含可用技能列表的提示词
- **路径参数支持**: 智能处理带路径参数的用户指令
- **结果展示**: 格式化显示任务规划和执行结果
- **自动初始化**: ROS2 通讯层自动管理，无需手动配置

## 文件结构

```
Interactive_Module/
├── interactive.py        # 交互界面主程序
└── README.md
```

## 数据流

```
用户输入
    ↓
┌─────────────────────────────────────────┐
│ Interactive_Module.interactive.py      │
│                                         │
│ 1. 接收用户自然语言指令                  │
│ 2. 注册 Robot_Module 工具（自动）       │
│ 3. 动态加载并填充提示词                  │
│ 4. 调用 LLM_Module 双层LLM               │
│ 5. 执行 MCP 工具函数                     │
│ 6. 显示执行结果                          │
└──────────────┬──────────────────────────┘
               ↓
        ┌──────┴──────┐
        ↓             ↓
    LLM_Module    Robot_Module
    (规划+执行)    (工具调用 + ROS2发布)
        ↓             ↓
    ROS2 Topic (/robot/command)
        ↓
    Sim_Module (仿真执行)
```

## 主函数流程

```python
def main():
    """主函数"""
    # 1. 注册所有 Robot_Module 的工具函数
    register_all_modules()

    # 2. 检查 API Key
    api_key = os.getenv('Test_API_KEY')

    # 3. 从 Robot_Module 获取工具定义
    tools = get_tool_definitions()

    # 4. 获取提示词路径
    prompt_path = "LLM_Module/prompts/planning_prompt_2d.yaml"

    # 5. 初始化 LLM Agent
    llm_agent = LLMAgent(api_key=api_key, prompt_path=str(prompt_path))

    # 6. 动态加载并填充提示词
    dynamic_prompt = load_dynamic_prompt(prompt_path, tools)
    llm_agent.planning_prompt_template = dynamic_prompt

    # 7. 主循环 - 接收用户输入
    while True:
        user_input = input("💬 请输入指令: ").strip()

        # 8. 执行双层 LLM 流程
        results = llm_agent.run_pipeline(
            user_input=user_input,
            tools=tools,
            execute_tool_fn=execute_tool
        )

        # 9. 显示结果
        if results:
            success_count = sum(1 for r in results if r.get("success"))
            print(f"📊 [完成] {success_count}/{len(results)} 个任务成功")
```

## 输入指令格式

### 格式1：基础指令
```
前进1米
左转90度
停止
后退0.5米
```

### 格式2：组合指令
```
前进1米，然后左转90度
先右转45度，再前进2米
左转90度，后退1米，停止
```

### 格式3：带图片路径的指令（VLM）
```
检测颜色并移动
根据 /home/robot/work/FinalProject/VLM_Module/assets/green.png 检测颜色并执行动作
前进1米，然后根据 /path/to/red.png 检测颜色
```

### 格式4：复杂组合指令
```
前进1米，左转90度，再根据 /home/robot/work/FinalProject/VLM_Module/assets/blue.png 检测颜色并执行相应动作
```

## 工具执行函数

```python
def execute_tool(function_name: str, function_args: dict) -> dict:
    """执行 Robot_Module 中的工具函数

    由 LLM_Module 的下层 LLM 调用

    Args:
        function_name: 工具函数名称
        function_args: 工具函数参数字典

    Returns:
        执行结果字典 {"result": ..., "delay": ...}
    """
    # 1. 获取工具函数
    skill_func = get_skill_function(function_name)

    # 2. 异步执行工具函数
    result = asyncio.run(skill_func(**function_args))

    # 3. 估算执行时间
    delay = calculate_delay(function_name, function_args)

    return {"result": result, "delay": delay}
```

## 动态提示词加载

```python
def load_dynamic_prompt(prompt_path: str, tools: List[Dict]) -> str:
    """动态加载并填充提示词

    将可用工具列表填充到提示词模板中

    Args:
        prompt_path: 提示词文件路径
        tools: 工具定义列表

    Returns:
        填充后的提示词字符串
    """
    with open(prompt_path, 'r', encoding='utf-8') as f:
        prompt_template = f.read()

    # 生成可用技能列表
    available_skills = "\n".join([
        f"- {tool['function']['name']}: {tool['function']['description'][:50]}"
        for tool in tools
    ])

    # 填充模板
    return prompt_template.format(
        available_skills=available_skills,
        robot_config="2D差速驱动机器人"
    )
```

## 执行时间估算

```python
def calculate_delay(function_name: str, function_args: dict) -> float:
    """估算工具执行时间

    Args:
        function_name: 工具函数名称
        function_args: 工具参数

    Returns:
        估算的执行时间（秒）
    """
    if function_name in ['move_forward', 'move_backward']:
        distance = function_args.get('distance', 1.0)
        speed = function_args.get('speed', 0.3)
        delay = distance / speed if speed > 0 else 0
    elif function_name == 'turn':
        angle = abs(function_args.get('angle', 90.0))
        angular_speed = function_args.get('angular_speed', 0.5)
        delay = (angle / 180.0 * 3.14159) / angular_speed if angular_speed > 0 else 0
    elif function_name == 'detect_color_and_act':
        delay = 3.3  # VLM识别+动作执行约3.3秒
    else:
        delay = 0

    return delay
```

## 使用示例

### 基础使用

```bash
./start_robot_system.sh

# 选择自动启动或手动启动后
💬 请输入指令: 前进1米
```

### 完整输出示例

```
💬 请输入指令: 前进1米，然后根据 /home/robot/work/FinalProject/VLM_Module/assets/green.png 检测颜色

████████████████████████████████████████████████████████████
📥 [用户输入] 前进1米，然后根据 /home/robot/work/FinalProject/VLM_Module/assets/green.png 检测颜色
████████████████████████████████████████████████████████████

============================================================
🧠 [上层LLM] 任务规划中...
============================================================
✅ [规划完成] 共分解为 2 个子任务
📋 [任务概述] 前进后检测颜色并执行动作

子任务序列：
  步骤 1: 向前移动1.0米 (移动)
  步骤 2: 根据 /home/robot/work/FinalProject/VLM_Module/assets/green.png 检测颜色并执行相应动作 (视觉检测)

████████████████████████████████████████████████████████████
🚀 [开始执行] 按顺序执行子任务
████████████████████████████████████████████████████████████

【步骤 1/2】
──────────────────────────────────────────────────
⚙️  [执行中] 向前移动1.0米
──────────────────────────────────────────────────
🔧 [工具调用] move_forward({'distance': 1.0})
[base.move_forward] 前进 1.0m, 速度 0.3m/s
⏳ [等待] 执行时间: 3.3秒... ✅ 完成!

【步骤 2/2】
──────────────────────────────────────────────────
⚙️  [执行中] 根据 /home/robot/work/FinalProject/VLM_Module/assets/green.png 检测颜色并执行相应动作
──────────────────────────────────────────────────
🔧 [工具调用] detect_color_and_act({'image_path': '/home/robot/work/FinalProject/VLM_Module/assets/green.png'})
[VLM] 识别颜色: green
[vision] 检测到绿色，执行后退
⏳ [等待] 执行时间: 3.3秒... ✅ 完成!

████████████████████████████████████████████████████████████
✅ [执行完成] 任务总结
████████████████████████████████████████████████████████████
  1. 向前移动1.0米 - ✅ 成功
  2. 检测颜色并执行相应动作 - ✅ 成功

📊 [完成] 2/2 个任务成功

💬 请输入指令:
```

## 退出系统

```bash
💬 请输入指令: quit
# 或
💬 请输入指令: exit
# 或
💬 请输入指令: q

👋 再见!
```

## 环境变量配置

```bash
# .env 文件
Test_API_KEY=your_api_key_here
```

```python
# interactive.py 中读取
from dotenv import load_dotenv
load_dotenv()

api_key = os.getenv('Test_API_KEY')
```

## 设计特点

1. **简洁交互**: 清晰的命令行界面
2. **智能路径处理**: 自动提取和传递文件路径参数
3. **详细反馈**: 显示完整的执行过程
4. **错误处理**: 优雅处理失败任务
5. **动态提示词**: 根据可用工具自动生成提示词
6. **时间估算**: 准确估算工具执行时间

## 依赖

```bash
python-dotenv>=1.0.0 # 环境变量管理
asyncio               # 异步执行支持
```

## 相关文档

- [主项目 README](../README.md)
- [LLM_Module README](../LLM_Module/README.md)
- [Robot_Module README](../Robot_Module/README.md)
- [Sim_Module README](../Sim_Module/README.md)
- [VLM_Module README](../VLM_Module/README.md)

---

**简单交互，强大功能！** 🚀