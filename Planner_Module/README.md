# Planner_Module

LLM 规划层。负责将用户意图分解为任务序列，并在必要时用规则覆盖修正已知场景。

## 核心接口

```python
from Planner_Module import Planner

planner = Planner(client=openai_client, parameter_calculator=calculator)
tasks, meta = planner.plan_tasks(
    user_input="导航到6,0,0",
    agent_thought="用户想去目标点",
    tools=action_tool_definitions,
    visual_context="...",
    scene_facts={...},
    object_facts={...},
)
```

## 目录结构

```text
Planner_Module/
  __init__.py       对外暴露 Planner, TaskIntent
  planner.py        高层任务规划入口（LLM 调用 + on_event 回调）
  parsing.py        规划 JSON 解析和任务标准化
  rule_overrides.py 规则覆盖与规则回退
  schema.py         TaskIntent dataclass
  prompts/
    highlevel_prompt.yaml  高层规划提示词
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

### `parsing.py`

规划响应解析工具。负责去除 Markdown code fence、解析 JSON、标准化 task 字段。

### `rule_overrides.py`

规则覆盖和规则回退。当前包含 box-assisted 与 climbable-obstacle 两类场景。

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

## 与其他模块的关系

- 被 `Robot_Module/pipeline_factory.py` 调用（创建 Planner 实例）
- 被 `Excu_Module/pipeline.py` 调用（`run_pipeline()` 编排规划 + 执行循环）
- 通过依赖注入使用 `Data_Module/params.py`（ParameterCalculator）
- 不导入 `Excu_Module`、`Robot_Module`、`Hardware_Module`
- 完全无状态，所有外部依赖通过构造函数或回调注入
