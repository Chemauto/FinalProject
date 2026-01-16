# LLM_Module - 双层LLM核心

## 概述

LLM_Module 实现了**双层 LLM 架构**，用于机器人的智能任务规划和执行控制。该模块将自然语言指令转换为可执行的机器人技能调用。

### 核心功能

- **任务规划 (上层 LLM)**: 将复杂用户指令分解为有序的子任务序列
- **执行控制 (下层 LLM)**: 将子任务转换为具体的技能函数调用
- **工具调用支持**: 自动选择和调用合适的机器人技能
- **路径参数提取**: 智能提取用户输入中的文件路径并传递给工具
- **错误恢复**: 规划失败时自动回退到单任务执行模式

## 架构与数据流

```
用户输入: "前进1米，然后根据 /path/to/image.png 检测颜色"
    ↓
┌─────────────────────────────────────────┐
│ 上层LLM (plan_tasks)                    │
│ 1. 保留文件路径信息                      │
│ 2. 分解为子任务序列                      │
│   [                                    │
│     {"task": "前进1米", ...},          │
│     {"task": "根据 /path/to/image.png 检测颜色", ...} │
│   ]                                    │
└──────────────┬──────────────────────────┘
               ↓
┌─────────────────────────────────────────┐
│ 下层LLM (execute_single_task)           │
│ 1. 提取文件路径参数                      │
│ 2. 调用相应工具                           │
│   {                                    │
│     "function": "detect_color_and_act",│
│     "arguments": {"image_path": "/path/to/image.png"} │
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
model: str = "qwen3-32b"  # 模型名称

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

**路径提取功能**:
```
用户输入: "根据 /home/robot/image.png 检测颜色"
    ↓
上层LLM保留完整路径
    ↓
子任务: {"task": "根据 /home/robot/image.png 检测颜色并执行相应动作"}
```

### 2. `execute_single_task(task_description, tools, execute_tool_fn)` - 执行单个任务

**下层 LLM**：执行单个子任务，通过工具调用实际技能。

```python
result = llm.execute_single_task(
    "根据 /path/to/image.png 检测颜色",
    tools=tools,
    execute_tool_fn=execute_tool_fn
)

# LLM会自动提取路径并调用:
# detect_color_and_act(image_path='/path/to/image.png')
```

**智能路径提取**:
```
子任务描述: "根据 /home/robot/image.png 检测颜色并执行相应动作"
    ↓
下层LLM分析任务描述
    ↓
提取文件路径: /home/robot/image.png
    ↓
构造工具调用: detect_color_and_act(image_path='/home/robot/image.png')
```

### 3. `run_pipeline(user_input, tools, execute_tool_fn)` - 完整流程

运行完整的双层 LLM 流程：规划 → 执行 → 总结。

```python
results = llm.run_pipeline(
    "前进1米，然后根据 /path/to/green.png 检测颜色",
    tools=tools,
    execute_tool_fn=execute_tool_fn
)
```

## 提示词系统

### 规划提示词 (planning_prompt_2d.yaml)

**关键特性**:
- **路径保留规则**: 明确指示保留文件路径信息
- **示例驱动**: 提供带路径的示例
- **格式化输出**: JSON格式的任务列表

```yaml
重要规则：
- 如果用户指令中包含图片文件路径（如 .png, .jpg），在子任务描述中必须保留完整的路径信息
- 特别是调用 detect_color_and_act 时，必须明确指定图片路径

示例（带图片路径）:
输入: "前进1米，然后根据 /home/robot/image.png 检测颜色并执行动作"
输出:
{{
  "tasks": [
    {{{{ "step": 1, "task": "向前移动1米", "type": "移动"}}}},
    {{{{ "step": 2, "task": "根据 /home/robot/image.png 检测颜色并执行相应动作", "type": "视觉检测"}}}}
  ],
  "summary": "前进后检测颜色并执行动作"
}}}
```

### 执行提示词 (llm_core.py)

**关键特性**:
- **路径提取规则**: 指导LLM从任务描述中提取路径
- **参数传递规范**: 确保路径参数正确传递给工具

```python
system_prompt = """你是一个机器人控制助手。

重要规则：
1. 如果任务描述中包含文件路径（特别是图片路径 .png, .jpg），必须将其作为参数传入
2. 调用 detect_color_and_act 时，如果任务中有路径，必须设置 image_path 参数
3. 示例：任务"根据 /home/path/image.png 检测颜色"应该调用 detect_color_and_act(image_path='/home/path/image.png')
"""
```

## 使用示例

### 基础使用

```python
from LLM_Module import LLMAgent
import os

llm = LLMAgent(api_key=os.getenv('Test_API_KEY'))

# 任务规划
tasks = llm.plan_tasks("前进1米，然后左转90度", tools=[])
```

### 支持路径参数的指令

```python
# 示例1：指定图片路径
results = llm.run_pipeline(
    "前进1米，然后根据 /home/robot/work/FinalProject/VLM_Module/assets/green.png 检测颜色",
    tools=tools,
    execute_tool_fn=execute_tool_fn
)

# 示例2：组合多个动作
results = llm.run_pipeline(
    "前进1米，左转90度，再根据 /path/to/blue.png 检测颜色",
    tools=tools,
    execute_tool_fn=execute_tool_fn
)
```

## 输出示例

### 带路径参数的完整流程

```
////////////////////////////////////////////////////////////
📥 [用户输入] 前进1米，然后根据 /path/to/green.png 检测颜色
////////////////////////////////////////////////////////////

============================================================
🧠 [上层LLM] 任务规划中...
============================================================
✅ [规划完成] 共分解为 2 个子任务
📋 [任务概述] 前进后检测颜色并执行动作

子任务序列：
  步骤 1: 向前移动1.0米 (移动)
  步骤 2: 根据 /path/to/green.png 检测颜色并执行相应动作 (视觉检测)

////////////////////////////////////////////////////////////
🚀 [开始执行] 按顺序执行子任务
////////////////////////////////////////////////////////////

【步骤 1/2】
⚙️  [执行中] 向前移动1.0米
🔧 [工具调用] move_forward({'distance': 1.0})
⏳ [等待] 执行时间: 3.3秒... ✅ 完成!

【步骤 2/2】
⚙️  [执行中] 根据 /path/to/green.png 检测颜色并执行相应动作
🔧 [工具调用] detect_color_and_act({'image_path': '/path/to/green.png'})
[VLM] 识别颜色: green
[vision] 检测到绿色，执行后退
⏳ [等待] 执行时间: 3.3秒... ✅ 完成!
```

## 输入指令格式

### 格式1：基础指令
```
前进1米
左转90度
停止
```

### 格式2：带图片路径的指令
```
根据 /home/robot/work/FinalProject/VLM_Module/assets/green.png 检测颜色并移动
检测 /path/to/red.png 的颜色
```

### 格式3：组合指令
```
前进1米，然后根据 /home/robot/work/FinalProject/VLM_Module/assets/blue.png 检测颜色
左转90度，再根据 /path/to/yellow.png 执行相应动作
```

## 路径参数提取机制

### 工作原理

```
用户输入包含路径
    ↓
┌─────────────────────────────────────────┐
│ 上层LLM规划阶段                         │
│ - 识别文件扩展名 (.png, .jpg)          │
│ - 保留完整路径                          │
│ - 在子任务描述中包含路径                │
└──────────────┬──────────────────────────┘
               ↓
┌─────────────────────────────────────────┐
│ 下层LLM执行阶段                         │
│ - 分析子任务描述                        │
│ - 提取文件路径                          │
│ - 构造工具调用参数                      │
│ - detect_color_and_act(image_path=...)  │
└──────────────┬──────────────────────────┘
               ↓
        VLM使用指定路径识别颜色
```

### 提示词关键点

1. **规划阶段提示词**:
   - "如果用户指令中包含图片文件路径，在子任务描述中必须保留完整的路径信息"
   - 提供带路径的示例

2. **执行阶段提示词**:
   - "如果任务描述中包含文件路径，必须将其作为参数传入"
   - "调用 detect_color_and_act 时，如果任务中有路径，必须设置 image_path 参数"

## 错误处理

### 1. 规划失败
```
❌ [规划失败] JSON解析错误
[回退] 将作为单个任务处理
```

### 2. 执行失败
```
⚠️ [警告] 步骤 1 失败，但继续执行后续任务
```

### 3. 无效指令
```
[跳过] 无效指令或无法识别的操作
```

## 依赖

```
openai>=1.0.0        # OpenAI API 客户端 (兼容 Dashscope)
pyyaml>=6.0          # YAML 配置解析
python-dotenv>=1.0.0 # 环境变量管理
```

## 设计特点

1. **分层解耦**: 任务规划与执行控制分离，职责清晰
2. **路径感知**: 智能提取和传递文件路径参数
3. **错误恢复**: 多层错误处理，保证系统稳定性
4. **灵活扩展**: 支持自定义提示词和模型
5. **详细日志**: 完整的执行过程输出，便于调试
6. **工具集成**: 原生支持 OpenAI 函数调用

## 相关文档

- [主项目 README](../README.md)
- [Robot_Module README](../Robot_Module/README.md)
- [Interactive_Module README](../Interactive_Module/README.md)
- [VLM_Module README](../VLM_Module/README.md)

---

**智能规划，精准执行！** 🚀
