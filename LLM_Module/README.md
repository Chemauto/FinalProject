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
LLM_Module 模块详细文档
====================================

# LLM_Module - 双层 LLM 核心模块

实现双层 LLM 架构，包含任务规划和任务执行的通用逻辑。

## 📁 模块结构

```
LLM_Module/
├── README.md                    # 本文档
├── llm_core.py                  # 双层 LLM 核心
└── prompts/                     # 提示词模板
    └── planning_prompt_2d.yaml  # 2D 机器人规划提示词
```

## 🎯 功能特性

- **双层 LLM 架构**: 规划层 + 执行层分离
- **动态提示词加载**: 从 YAML 文件加载提示词模板
- **工具调用支持**: 支持 Function Calling
- **任务分解**: 将复杂指令分解为子任务序列
- **错误处理**: JSON 解析错误和 LLM 调用失败处理
- **灵活配置**: 支持自定义模型和 API 端点

## 🧠 双层 LLM 架构

### 架构图

```
用户输入: "前进1米然后左转90度"
    ↓
┌─────────────────────────────────────┐
│ 上层 LLM (任务规划)                  │
│ - 输入: 用户指令 + 可用工具列表       │
│ - 输出: 子任务序列                   │
│ - 提示词: planning_prompt_2d.yaml   │
└──────────────┬──────────────────────┘
               ↓
    子任务序列:
    [
      {"step": 1, "task": "前进1米", "type": "移动"},
      {"step": 2, "task": "左转90度", "type": "转向"}
    ]
               ↓
┌─────────────────────────────────────┐
│ 下层 LLM (任务执行)                  │
│ - 输入: 单个子任务描述               │
│ - 输出: 工具调用                     │
│ - 方法: Function Calling            │
└──────────────┬──────────────────────┘
               ↓
    工具调用:
    [
      {"tool": "move_forward", "parameters": {"distance": 1.0}},
      {"tool": "turn", "parameters": {"angle": 90.0}}
    ]
```

## 🔧 核心组件

### llm_core.py - 双层 LLM 核心

**LLMAgent 类：**

```python
class LLMAgent:
    """双层 LLM 代理"""

    def __init__(self, api_key: str, base_url: str, prompt_path: str):
        """
        初始化 LLM 代理

        Args:
            api_key: OpenAI API 密钥
            base_url: API 基础 URL
            prompt_path: 规划提示词 YAML 文件路径
        """

    def load_prompt(self, prompt_path: str) -> str:
        """从 YAML 文件加载规划提示词"""

    def plan_tasks(self, user_input: str, tools: List[Dict]) -> List[Dict]:
        """
        上层 LLM：将用户输入分解为子任务序列

        Args:
            user_input: 用户输入的指令
            tools: 可用工具列表

        Returns:
            子任务列表 [{"step": 1, "task": "...", "type": "..."}]
        """

    def execute_task(self, task: str, tools: List[Dict], execute_fn: Callable) -> Any:
        """
        下层 LLM：执行单个子任务

        Args:
            task: 子任务描述
            tools: 可用工具列表
            execute_fn: 工具执行函数

        Returns:
            执行结果
        """

    async def run_pipeline(self, user_input: str, tools: List[Dict], execute_fn: Callable) -> Dict:
        """
        完整流程：规划 + 执行

        Args:
            user_input: 用户输入
            tools: 可用工具列表
            execute_fn: 工具执行函数

        Returns:
            {
                'success': bool,
                'tasks': List[Dict],
                'results': List[Any],
                'summary': str
            }
        """
```

### prompts/planning_prompt_2d.yaml - 规划提示词

**提示词结构：**

```yaml
system_prompt: |
  你是一个机器人任务规划助手。你的职责是将用户的复杂指令分解为可执行的子任务序列。

  机器人配置:
  {robot_config}

  注意:
  - 输出必须是有效的JSON格式
  - 只分解与可用技能相关的指令
  - 如果输入无效或不属于机器人操作，返回空任务列表

prompt: |
  你是一个机器人任务规划助手。你的职责是将用户的复杂指令分解为简单的、顺序执行的子任务。

  可用技能:
  {available_skills}

  只处理与上述可用技能相关的指令。如果输入无效、无法理解或不属于机器人操作范围，返回空任务列表。

  重要规则：
  - 如果用户指令中包含图片文件路径（如 .png, .jpg），在子任务描述中必须保留完整的路径信息
  - 特别是调用 detect_color_and_act 时，必须明确指定图片路径
  - 示例："根据 /home/path/to/image.png 检测颜色" 应该保留完整路径
  - **高级复合工具不要过度分解**：某些工具（如 chase_enemy、detect_color_and_act）已经包含完整的子任务序列，应该作为单个任务执行
  - 例如："追击敌人"应该映射到 chase_enemy 工具（单个任务），而不是分解为"旋转+前进"（多个任务）

  输出格式（JSON）：
  {{{{
    "tasks": [
      {{{{ "step": 1, "task": "子任务描述1", "type": "动作类型"}}}},
      {{{{ "step": 2, "task": "子任务描述2", "type": "动作类型"}}}}
    ],
    "summary": "整体任务概述"
  }}}}

  用户输入：{{user_input}}

  请将上述指令分解为子任务序列，每个子任务应该简洁明确，可以直接被下层LLM执行。
```

## 📝 使用示例

### 示例 1: 基础使用

```python
from LLM_Module.llm_core import LLMAgent

# 初始化 LLM 代理
agent = LLMAgent(
    api_key="your_api_key",
    base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
    prompt_path="LLM_Module/prompts/planning_prompt_2d.yaml"
)

# 定义可用工具
tools = [
    {
        "name": "move_forward",
        "description": "前进指定距离",
        "parameters": {"distance": "float", "speed": "float"}
    },
    {
        "name": "turn",
        "description": "旋转指定角度",
        "parameters": {"angle": "float", "angular_speed": "float"}
    }
]

# 定义工具执行函数
async def execute_tool(tool_name, parameters):
    print(f"执行工具: {tool_name}({parameters})")
    return {"success": True}

# 运行完整流程
result = await agent.run_pipeline(
    user_input="前进1米然后左转90度",
    tools=tools,
    execute_fn=execute_tool
)

print(result)
# {
#     'success': True,
#     'tasks': [
#         {'step': 1, 'task': '前进1米', 'type': '移动'},
#         {'step': 2, 'task': '左转90度', 'type': '转向'}
#     ],
#     'results': [...],
#     'summary': '先转向再前进'
# }
```

### 示例 2: 单独使用规划功能

```python
# 只进行任务规划
tasks = agent.plan_tasks(
    user_input="前进1米然后左转90度",
    tools=tools
)

print(tasks)
# [
#     {'step': 1, 'task': '前进1米', 'type': '移动'},
#     {'step': 2, 'task': '左转90度', 'type': '转向'}
# ]
```

### 示例 3: 单独使用执行功能

```python
# 执行单个任务
result = agent.execute_task(
    task="前进1米",
    tools=tools,
    execute_fn=execute_tool
)
```

## 🔌 API 参考

### plan_tasks()

**功能**: 将用户输入分解为子任务序列

**参数**:
- `user_input` (str): 用户输入的指令
- `tools` (List[Dict]): 可用工具列表

**返回**:
- `List[Dict]`: 子任务列表

**示例输出**:
```json
[
  {"step": 1, "task": "前进1米", "type": "移动"},
  {"step": 2, "task": "左转90度", "type": "转向"}
]
```

### execute_task()

**功能**: 执行单个子任务

**参数**:
- `task` (str): 子任务描述
- `tools` (List[Dict]): 可用工具列表
- `execute_fn` (Callable): 工具执行函数

**返回**:
- `Any`: 工具执行结果

### run_pipeline()

**功能**: 完整流程（规划 + 执行）

**参数**:
- `user_input` (str): 用户输入
- `tools` (List[Dict]): 可用工具列表
- `execute_fn` (Callable): 工具执行函数

**返回**:
- `Dict`: 包含 success, tasks, results, summary

## ⚙️ 配置说明

### 模型配置

在 `llm_core.py` 中修改：

```python
class LLMAgent:
    def __init__(self, ...):
        self.model = "qwen3-32b"  # 可修改为其他模型
```

支持的模型：
- `qwen3-32b` - 通义千问 3 32B（推荐）
- `qwen3-72b` - 通义千问 3 72B
- `qwen-turbo` - 通义千问 Turbo
- 其他兼容 OpenAI API 的模型

### API 端点配置

```python
agent = LLMAgent(
    base_url="https://dashscope.aliyuncs.com/compatible-mode/v1"  # 阿里云
    # 或
    base_url="https://api.openai.com/v1"  # OpenAI
)
```

### 提示词配置

编辑 `prompts/planning_prompt_2d.yaml`：

```yaml
prompt: |
  可用技能:
  {available_skills}

  # 添加自定义规则
  - 规则1: ...
  - 规则2: ...
```

## 💡 设计理念

### 为什么使用双层 LLM？

1. **职责分离**
   - 上层 LLM: 理解用户意图，进行高层规划
   - 下层 LLM: 将具体任务映射到工具调用

2. **提高准确性**
   - 规划 LLM 可以看到所有可用工具，做出全局最优规划
   - 执行 LLM 只需要关注单个任务，减少错误

3. **易于扩展**
   - 添加新工具只需更新可用工具列表
   - 提示词模板化，便于维护

### Function Calling vs 提示词工程

- **Function Calling**: 下层 LLM 使用，确保工具调用格式正确
- **提示词工程**: 上层 LLM 使用，灵活控制任务分解逻辑

## 🐛 调试技巧

### 1. 查看规划结果

```python
tasks = agent.plan_tasks(user_input, tools)
print("规划结果:", json.dumps(tasks, indent=2, ensure_ascii=False))
```

### 2. 查看工具调用

```python
async def debug_execute_fn(tool_name, parameters):
    print(f"[DEBUG] 调用工具: {tool_name}")
    print(f"[DEBUG] 参数: {parameters}")
    result = await actual_execute(tool_name, parameters)
    print(f"[DEBUG] 结果: {result}")
    return result
```

### 3. 测试提示词

```python
# 加载提示词
prompt = agent.load_prompt("prompts/planning_prompt_2d.yaml")

# 填充变量
formatted_prompt = prompt.format(
    available_skills="\n".join([f"- {t['name']}: {t['description']}" for t in tools]),
    user_input="前进1米"
)

print(formatted_prompt)
```

## 🔗 相关模块

- `Interactive_Module/interactive.py` - 交互界面
- `Robot_Module/skill.py` - MCP 工具注册

## 📝 依赖

```
openai>=1.0.0  # OpenAI API
pyyaml>=6.0    # YAML 配置解析
```

---

**智能规划，精确执行！** 🧠



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