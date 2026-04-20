# Planner_Module

LLM 规划层。负责将用户意图分解为任务序列，并通过低层 LLM 选择工具执行。

## 核心接口

```python
from Planner_Module import Planner, TaskExecutor

# 高层规划
planner = Planner(client=openai_client, parameter_calculator=calculator)
tasks, meta = planner.plan_tasks(
    user_input="导航到6,0,0",
    agent_thought="用户想去目标点",
    tools=action_tool_definitions,
    visual_context="...",
    scene_facts={...},
    object_facts={...},
)

# 低层执行
executor = TaskExecutor(client=openai_client)
result = executor.execute_single_task(
    task_info=tasks[0],
    tools=action_tool_definitions,
    execute_tool_fn=tool_callback,
)
```

## 目录结构

```text
Planner_Module/
  __init__.py       对外暴露 Planner, TaskExecutor, TaskIntent
  planner.py        高层任务规划（CoT 推理 + 规则覆盖）
  executor.py       低层任务执行（LLM 工具选择）
  schema.py         TaskIntent dataclass
  prompts/
    highlevel_prompt.yaml  高层规划提示词
    lowlevel_prompt.yaml   低层执行提示词
```

## 各文件职责

### `planner.py` — Planner

高层任务规划器，通过 CoT 推理链生成任务序列。

**主要方法：**

- `plan_tasks(user_input, agent_thought, tools, ..., on_event=None)` — 核心规划入口
  - 使用 YAML prompt 模板格式化输入
  - 调用 LLM 生成任务序列 JSON
  - 应用规则覆盖（box-assisted / climbable-obstacle）
  - 失败时回退到规则链或默认任务
  - `on_event` 回调输出规划事件（`llm_planning`、`plan_done`、`plan_error`、`rule_override`）

- `annotate_tasks(tasks, object_facts)` — 参数标注
  - 通过依赖注入的 `parameter_calculator`（`Data_Module/params.py`）完成
  - 如果没有 object_facts 或 parameter_calculator，直接返回原任务

**规则覆盖逻辑：**

| 场景 | 条件 | 覆盖任务链 |
|------|------|-----------|
| box-assisted | 两侧被阻挡 + 有可移动箱子 + 箱子可做台阶 | `push_box -> climb_align -> climb -> nav_climb` |
| climbable-obstacle | 两侧有平台 + 一侧可攀爬 + 无箱子 | `way_select -> nav_climb` |

当 LLM 规划结果不符合场景几何时，规则覆盖会强制修正。规则覆盖事件通过 `on_event("rule_override", ...)` 回调输出（`on_event=None` 时回退为 `print()`）。

**依赖注入：**

```python
planner = Planner(
    client=openai_client,
    parameter_calculator=ParameterCalculator(),  # 可选，注入参数计算器
)
```

Planner 不直接导入 `Data_Module`，而是通过构造函数接收 `parameter_calculator`。

### `executor.py` — TaskExecutor

低层任务执行器，根据单个任务描述选择并调用工具。

**主要方法：**

- `execute_single_task(task_info, tools, execute_tool_fn, ...)` — 执行单个任务
  - 如果任务已有 `calculated_parameters`，直接调用工具（跳过 LLM）
  - 否则调用 LLM + function calling 选择工具
  - 通过 `execute_tool_fn` 回调实际执行工具（不直接导入 Robot_Module）

**回调模式：**

```python
def my_tool_executor(function_name: str, function_args: dict) -> dict:
    # 实际执行工具逻辑
    ...

result = executor.execute_single_task(task, tools, my_tool_executor)
```

TaskExecutor 不导入任何上层模块，完全通过回调工作。

### `schema.py` — TaskIntent

```python
@dataclass
class TaskIntent:
    step: int                              # 步骤编号
    task: str                              # 任务描述
    type: str = "未分类"                    # 任务类型
    function: str = "待LLM决定"             # 建议的工具函数名
    reason: str = "未提供规划依据"           # 规划依据
    parameter_context: dict = {}           # 参数上下文
    calculated_parameters: dict = {}       # 已计算参数
```

### `prompts/`

- `highlevel_prompt.yaml` — 高层规划 prompt，包含 `system_prompt` 和 `prompt` 模板
  - 模板变量：`{available_skills}`, `{user_input}`, `{agent_thought}`, `{visual_context}`, `{scene_facts}`, `{object_facts}`
- `lowlevel_prompt.yaml` — 低层执行 prompt，包含 `system_prompt` 和 `user_prompt` 模板
  - 模板变量：`{task_description}`, `{task_type}`, `{suggested_function}`, `{parameter_context}` 等

## 与其他模块的关系

- 被 `Robot_Module/tools.py` 调用（创建 Planner/TaskExecutor 实例）
- 被 `Excu_Module/pipeline.py` 调用（`run_pipeline()` 编排规划 + 执行循环）
- 通过依赖注入使用 `Data_Module/params.py`（ParameterCalculator）
- 不导入 `Excu_Module`、`Robot_Module`、`Hardware_Module`
- 完全无状态，所有外部依赖通过构造函数或回调注入
