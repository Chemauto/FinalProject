# LLM_Module - 双层LLM核心

## 概述

LLM_Module 实现了**双层 LLM 架构**，用于机器人的智能任务规划和执行控制。该模块将自然语言指令转换为可执行的机器人技能调用。

### 核心功能

- **任务规划 (上层 LLM)**: 将复杂用户指令分解为有序的子任务序列
- **执行控制 (下层 LLM)**: 将子任务转换为具体的技能函数调用
- **工具调用支持**: 自动选择和调用合适的机器人技能
- **错误恢复**: 规划失败时自动回退到单任务执行模式

## 架构与数据流

```
用户输入: "前进1米然后左转90度"
    ↓
┌─────────────────────────────────────────┐
│ 上层LLM (plan_tasks)                    │
│ 输入: 用户指令 + 可用技能列表            │
│ 输出: 子任务序列                        │
│   [                                    │
│     {"task": "前进1米", "type": "移动"},│
│     {"task": "左转90度", "type": "旋转"} │
│   ]                                    │
└──────────────┬──────────────────────────┘
               ↓
┌─────────────────────────────────────────┐
│ 下层LLM (execute_single_task)           │
│ 输入: 单个子任务描述                     │
│ 输出: 工具调用                          │
│   {                                    │
│     "function": "move_forward",        │
│     "arguments": {"distance": 1.0}     │
│   }                                    │
└──────────────┬──────────────────────────┘
               ↓
      execute_tool_fn() 回调
               ↓
      Robot_Module (MCP 工具)
```

## 文件结构

```
LLM_Module/
├── __init__.py           # 模块导出
├── llm_core.py           # LLMAgent 核心类实现
└── prompts/              # YAML 格式提示词模板
    └── planning_prompt_2d.yaml  # 2D 机器人任务规划提示
```

## 核心类: LLMAgent

### 初始化

```python
from LLM_Module import LLMAgent
import os

# 使用默认配置
api_key = os.getenv('Test_API_KEY')
llm = LLMAgent(api_key=api_key)

# 使用自定义提示词文件
llm = LLMAgent(
    api_key=api_key,
    prompt_path="LLM_Module/prompts/planning_prompt_2d.yaml"
)
```

### 配置

```python
# API 配置
api_key: str              # API 密钥
base_url: str = "https://dashscope.aliyuncs.com/compatible-mode/v1"  # API 端点
model: str = "qwen-plus"  # 模型名称

# 提示词配置
prompt_path: str = None   # 提示词文件路径（YAML）
planning_prompt_template: str = None  # 规划提示词模板（可动态设置）
```

## 核心方法

### 1. `plan_tasks(user_input, tools)` - 任务规划

**上层 LLM**：将用户输入分解为子任务序列。

```python
tasks = llm.plan_tasks("向前走2米，然后左转90度", tools)

# 返回格式:
# [
#     {"step": 1, "task": "向前走2米", "type": "移动"},
#     {"step": 2, "task": "左转90度", "type": "旋转"}
# ]
```

**数据流**:
```
用户输入
    ↓
填充提示词模板 ({user_input}, {available_skills})
    ↓
调用 LLM API
    ↓
解析 JSON 响应
    ↓
返回子任务序列
```

**输出示例**:
```
============================================================================
🧠 [上层LLM] 任务规划中...
============================================================================
✅ [规划完成] 共分解为 2 个子任务
📋 [任务概述] 用户需要先向前移动2米，然后向左旋转90度

子任务序列：
  步骤 1: 向前走2米 (移动)
  步骤 2: 左转90度 (旋转)
```

### 2. `execute_single_task(task_description, tools, execute_tool_fn)` - 执行单个任务

**下层 LLM**：执行单个子任务，通过工具调用实际技能。

```python
def execute_tool_fn(function_name: str, function_args: dict) -> dict:
    """执行工具函数的回调"""
    # 调用 Robot_Module 中的技能
    from Robot_Module.skill import get_skill_function
    skill_func = get_skill_function(function_name)
    result = asyncio.run(skill_func(**function_args))
    return {"result": result}

result = llm.execute_single_task(
    "向前走2米",
    tools=tools,  # 工具定义列表
    execute_tool_fn=execute_tool_fn
)

# 返回格式:
# {
#     "success": True,
#     "action": "move_forward",
#     "task": "向前走2米",
#     "result": {...}
# }
```

**数据流**:
```
子任务描述
    ↓
填充提示词
    ↓
调用 LLM API (function calling)
    ↓
获取工具调用 (function_name, arguments)
    ↓
execute_tool_fn(function_name, **arguments)
    ↓
Robot_Module 执行技能
    ↓
返回执行结果
```

**输出示例**:
```
──────────────────────────────────────────────────
⚙️  [执行中] 向前走2米
──────────────────────────────────────────────────
🔧 [工具调用] move_forward({'distance': 2.0, 'speed': 0.3})
[base.move_forward] 前进 2.0m, 速度 0.3m/s
⏳ [等待] 执行时间: 6.7秒... ✅ 完成!
```

### 3. `run_pipeline(user_input, tools, execute_tool_fn)` - 完整流程

运行完整的双层 LLM 流程：规划 → 执行 → 总结。

```python
results = llm.run_pipeline(
    "向前走2米，然后左转90度",
    tools=tools,  # 工具定义列表
    execute_tool_fn=execute_tool_fn
)

# 返回格式:
# [
#     {"success": True, "task": "向前走2米", "result": {...}},
#     {"success": True, "task": "左转90度", "result": {...}}
# ]
```

**完整输出示例**:
```
////////////////////////////////////////////////////////////
📥 [用户输入] 向前走2米，然后左转90度
////////////////////////////////////////////////////////////

============================================================================
🧠 [上层LLM] 任务规划中...
============================================================================
✅ [规划完成] 共分解为 2 个子任务
...

////////////////////////////////////////////////////////////
🚀 [开始执行] 按顺序执行子任务
////////////////////////////////////////////////////////////

【步骤 1/2】
──────────────────────────────────────────────────
⚙️  [执行中] 向前走2米
──────────────────────────────────────────────────
🔧 [工具调用] move_forward({'distance': 2.0, 'speed': 0.3})
⏳ [等待] 执行时间: 6.7秒... ✅ 完成!

【步骤 2/2】
──────────────────────────────────────────────────
⚙️  [执行中] 左转90度
──────────────────────────────────────────────────
🔧 [工具调用] turn({'angle': 90.0, 'angular_speed': 0.5})
⏳ [等待] 执行时间: 1.8秒... ✅ 完成!

////////////////////////////////////////////////////////////
✅ [执行完成] 任务总结
////////////////////////////////////////////////////////////
  1. 向前走2米 - ✅ 成功
  2. 左转90度 - ✅ 成功
```

## 提示词系统

### 提示词格式

提示词使用 YAML 格式，支持变量占位符：

```yaml
# prompts/planning_prompt_2d.yaml
prompt: |
  你是一个机器人任务规划助手。
  请将用户的指令分解为简单的子任务序列。

  可用技能:
  {available_skills}

  用户输入：{user_input}

  请返回 JSON 格式的任务列表，包含：
  - tasks: 子任务数组
  - summary: 任务概述

  每个子任务包含：
  - step: 步骤编号
  - task: 任务描述
  - type: 任务类型（移动、旋转、停止等）

  输出示例：
  {{{{
    "tasks": [
      {{{{ "step": 1, "task": "左转90度", "type": "转向"}}}},
      {{{{ "step": 2, "task": "向前移动1米", "type": "移动"}}}}
    ],
    "summary": "先转向再前进"
  }}}}
```

### 变量占位符

- `{user_input}`: 用户输入的指令（在 LLM 调用时填充）
- `{available_skills}`: 可用技能列表（在初始化时动态生成）
- `{robot_config}`: 机器人配置信息（可选）

### 花括号转义

由于 YAML 解析器会将 `{{` 解析为 `{`，所以在 YAML 中需要使用 `{{{{` 来得到最终的 `{{`：

```
YAML 文件: {{{{  →  YAML 解析: {{  →  Python format(): {  →  LLM 接收: {
```

## 工具定义格式

工具定义使用 **OpenAI Function Calling** 格式：

```json
{
  "type": "function",
  "function": {
    "name": "move_forward",
    "description": "向前移动指定距离",
    "parameters": {
      "type": "object",
      "properties": {
        "distance": {
          "type": "number",
          "description": "移动距离（米），默认1.0米"
        },
        "speed": {
          "type": "number",
          "description": "移动速度（米/秒），默认0.3米/秒"
        }
      },
      "required": []
    }
  }
}
```

## 错误处理

### 1. 规划失败

```
❌ [规划失败] JSON解析错误
[回退] 将作为单个任务处理
```

**处理机制**:
- 捕获 JSON 解析异常
- 自动回退到单任务执行模式
- 将整个用户输入作为单个任务处理

### 2. 执行失败

```
⚠️ [警告] 步骤 1 失败，但继续执行后续任务
```

**处理机制**:
- 记录错误但继续执行
- 在最终总结中标记失败任务

### 3. 无效指令

```
[跳过] 无效指令或无法识别的操作
```

**处理机制**:
- 上层 LLM 返回空任务列表
- 显示提示信息并跳过

## 使用示例

### 基础使用

```python
from LLM_Module import LLMAgent
import os

# 初始化
llm = LLMAgent(api_key=os.getenv('Test_API_KEY'))

# 只进行任务规划
tasks = llm.plan_tasks("向前走2米，左转90度，后退1米", tools=[])
for task in tasks:
    print(f"步骤 {task['step']}: {task['task']}")
```

### 完整集成

```python
from LLM_Module import LLMAgent
from Robot_Module.skill import get_skill_function, register_all_modules
import asyncio
import os

# 注册工具
register_all_modules()
tools = get_tool_definitions()

# 初始化 LLM
llm = LLMAgent(api_key=os.getenv('Test_API_KEY'))

# 定义执行函数
def execute_tool(function_name: str, function_args: dict) -> dict:
    skill_func = get_skill_function(function_name)
    result = asyncio.run(skill_func(**function_args))

    # 估算执行时间
    if function_name in ['move_forward', 'move_backward']:
        distance = function_args.get('distance', 1.0)
        speed = function_args.get('speed', 0.3)
        delay = distance / speed if speed > 0 else 0
    elif function_name == 'turn':
        angle = abs(function_args.get('angle', 90.0))
        angular_speed = function_args.get('angular_speed', 0.5)
        delay = (angle / 180.0 * 3.14159) / angular_speed if angular_speed > 0 else 0
    else:
        delay = 0

    return {"result": result, "delay": delay}

# 运行完整流程
results = llm.run_pipeline(
    "先前进1米然后右转45度",
    tools=tools,
    execute_tool_fn=execute_tool
)

# 检查结果
for result in results:
    if result['success']:
        print(f"✅ {result['task']} 完成")
    else:
        print(f"❌ {result['task']} 失败: {result.get('error')}")
```

## 依赖

```
openai>=1.0.0        # OpenAI API 客户端 (兼容 Dashscope)
pyyaml>=6.0          # YAML 配置解析
python-dotenv>=1.0.0 # 环境变量管理
```

## 设计特点

1. **分层解耦**: 任务规划与执行控制分离，职责清晰
2. **错误恢复**: 多层错误处理，保证系统稳定性
3. **灵活扩展**: 支持自定义提示词和模型
4. **详细日志**: 完整的执行过程输出，便于调试
5. **工具集成**: 原生支持 OpenAI 函数调用 (Function Calling)
6. **动态提示词**: 根据可用工具自动生成提示词

## 相关文档

- [主项目 README](../README.md)
- [Robot_Module README](../Robot_Module/README.md)
- [Interactive_Module README](../Interactive_Module/README.md)
- [VLM_Module README](../VLM_Module/README.md)

---

**智能规划，精准执行！** 🚀
