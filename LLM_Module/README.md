# LLM_Module - 大语言模型模块

## 概述

LLM_Module 实现了**双层 LLM 架构**，用于机器人的智能任务规划和执行控制。该模块将自然语言指令转换为可执行的机器人技能调用。

### 核心功能

- **任务规划 (上层 LLM)**: 将复杂用户指令分解为有序的子任务序列
- **执行控制 (下层 LLM)**: 将子任务转换为具体的技能函数调用
- **工具调用支持**: 自动选择和调用合适的机器人技能
- **错误恢复**: 规划失败时自动回退到单任务执行模式

### 支持的模型

- **Qwen-Plus** (默认): 通过阿里云 Dashscope API
- 兼容 OpenAI API 格式的其他模型

## 文件结构

```
LLM_Module/
├── __init__.py           # 模块导出
├── llm_core.py           # LLMAgent 核心类实现
└── prompts/              # YAML 格式提示词模板
    ├── planning_prompt_2d.yaml       # 2D 机器人任务规划提示
    ├── planning_prompt_go2.yaml      # Go2 四足机器人任务规划提示
    └── execution_prompt_go2.yaml     # Go2 四足机器人执行控制提示
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
    base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
    prompt_path="LLM_Module/prompts/planning_prompt_2d.yaml"
)
```

### 方法说明

#### 1. `plan_tasks(user_input, tools)` - 任务规划

**上层 LLM**：将用户输入分解为子任务序列。

```python
tasks = llm.plan_tasks("向前走2米，然后左转90度", tools=[])

# 返回格式:
# [
#     {"step": 1, "task": "向前走2米", "type": "移动"},
#     {"step": 2, "task": "左转90度", "type": "旋转"}
# ]
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

#### 2. `execute_single_task(task_description, tools, execute_tool_fn)` - 执行单个任务

**下层 LLM**：执行单个子任务，通过工具调用实际技能。

```python
def execute_tool_fn(function_name: str, function_args: dict) -> dict:
    """执行工具函数的回调"""
    # 调用实际技能并返回结果
    return bridge.execute_skill(function_name, **function_args)

result = llm.execute_single_task(
    "向前走2米",
    tools=bridge.get_mcp_tools_definition(),
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

**输出示例**:
```
──────────────────────────────────────────────────
⚙️  [执行中] 向前走2米
──────────────────────────────────────────────────
🔧 [工具调用] move_forward({'distance': 2.0, 'speed': 0.3})
⏳ [等待] 执行时间: 6.7秒... ✅ 完成!
```

#### 3. `run_pipeline(user_input, tools, execute_tool_fn)` - 完整流程

运行完整的双层 LLM 流程：规划 → 执行 → 总结。

```python
results = llm.run_pipeline(
    "向前走2米，然后左转90度",
    tools=bridge.get_mcp_tools_definition(),
    execute_tool_fn=execute_tool_fn
)
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
🔧 [工具调用] rotate({'angle': 90.0, 'angular_speed': 0.5})
⏳ [等待] 执行时间: 1.8秒... ✅ 完成!

////////////////////////////////////////////////////////////
✅ [执行完成] 任务总结
////////////////////////////////////////////////////////////
  1. 向前走2米 - ✅ 成功
  2. 左转90度 - ✅ 成功
```

## 提示词格式

提示词使用 YAML 格式，支持变量占位符：

```yaml
# prompts/planning_prompt_2d.yaml
prompt: |
  你是一个机器人任务规划助手。
  请将用户的指令分解为简单的子任务序列。

  用户指令：{user_input}

  请返回 JSON 格式的任务列表，包含：
  - tasks: 子任务数组
  - summary: 任务概述

  每个子任务包含：
  - step: 步骤编号
  - task: 任务描述
  - type: 任务类型（移动、旋转、停止等）
```

**变量占位符**:
- `{user_input}`: 用户输入的指令

## 错误处理

模块包含完善的错误处理机制：

1. **规划失败**: 自动回退到单任务执行模式
   ```
   ❌ [规划失败] JSON解析错误
   [回退] 将作为单个任务处理
   ```

2. **执行失败**: 记录错误但继续执行后续任务
   ```
   ⚠️  [警告] 步骤 1 失败，但继续执行后续任务
   ```

3. **无效指令**: 跳过无法识别的操作
   ```
   [跳过] 无效指令或无法识别的操作
   ```

## 配置说明

### API 配置

在项目根目录的 `.env` 文件中配置：

```bash
# 阿里云 Dashscope API Key (Qwen-Plus)
Test_API_KEY=sk-xxxxxxxxxxxxxxxxxxxxxx
```

### 自定义 Base URL

```python
# 使用其他兼容 OpenAI API 的服务
llm = LLMAgent(
    api_key="your_key",
    base_url="https://your-api-endpoint.com/v1"
)
```

## 使用示例

### 完整示例: 与 MCP Bridge 集成

```python
from LLM_Module import LLMAgent
from MCP_Module import create_mcp_bridge
import os

# 初始化
api_key = os.getenv('Test_API_KEY')
llm = LLMAgent(api_key=api_key)
bridge = create_mcp_bridge(['Sim_2D'])

# 定义执行函数
def execute_tool(function_name: str, function_args: dict) -> dict:
    return bridge.execute_skill(function_name, **function_args)

# 运行完整流程
results = llm.run_pipeline(
    "先前进1米然后右转45度",
    tools=bridge.get_mcp_tools_definition(),
    execute_tool_fn=execute_tool
)

# 检查结果
for result in results:
    if result['success']:
        print(f"✅ {result['task']} 完成")
    else:
        print(f"❌ {result['task']} 失败: {result.get('error')}")
```

### 仅规划任务

```python
# 只进行任务规划，不执行
tasks = llm.plan_tasks("向前走2米，左转90度，后退1米", tools=[])

for task in tasks:
    print(f"步骤 {task['step']}: {task['task']}")
```

### 执行单个任务

```python
# 直接执行单个任务
result = llm.execute_single_task(
    "停止机器人",
    tools=bridge.get_mcp_tools_definition(),
    execute_tool_fn=execute_tool
)
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

## 相关文档

- [VLM_Module README](../VLM_Module/README.md) - 视觉语言模型模块
- [MCP_Module README](../MCP_Module/README.md) - MCP 中间件模块
- [主项目 README](../README.md) - 项目总览
