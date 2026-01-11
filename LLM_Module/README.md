# LLM_Module - 大语言模型模块

## 功能

双层 LLM 架构用于机器人任务规划和执行：
- **上层 LLM (任务规划)**: 将用户指令分解为子任务序列
- **下层 LLM (执行控制)**: 将子任务转换为具体技能调用

## 文件结构

```
LLM_Module/
├── __init__.py
├── llm_core.py          # LLMAgent 类实现
└── prompts/             # YAML 格式提示词
    ├── planning_prompt_go2.yaml
    ├── planning_prompt_2d.yaml
    ├── execution_prompt_go2.yaml
    └── robot_control.yaml
```

## 核心类

### LLMAgent

```python
from LLM_Module import LLMAgent

llm = LLMAgent(api_key="your_api_key", prompt_path="path/to/prompt.yaml")

# 任务规划
tasks = llm.plan_tasks("向前走2米，然后左转90度", tools=[])

# 执行单个任务
result = llm.execute_single_task("左转90度", tools=[], execute_tool_fn=fn)

# 完整流程
results = llm.run_pipeline("向前走2米，然后左转90度", tools=[], execute_tool_fn=fn)
```

## 提示词格式

提示词使用 YAML 格式，包含：
- `system_prompt`: 系统提示
- `template`: 消息模板，支持 `{variable}` 占位符

## 依赖

- openai
- pyyaml
- python-dotenv
