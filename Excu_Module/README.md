# Excu_Module

通用执行层。不包含任何项目特定代码（如 Bishe），只提供统一执行基础设施。

把"具体技能实现"和"通用执行管理"分开：

- `Robot_Module/tasks/<Project>/*.py` — 只负责技能实现
- `Excu_Module` — 负责统一执行、等待、验收

## 目录结构

```text
Excu_Module/
  __init__.py       对外暴露公共接口
  pipeline.py       执行流水线编排（plan -> execute -> assess 循环）
  executor.py       串联执行前状态、命令下发、执行后状态、validation
  runtime.py        执行协议（file/udp/ros）、命令下发
  state.py          统一读取硬件状态，提供校验方法
  skill_base.py     技能基类（SkillBase）
  skill_registry.py 全局技能注册 + 可插拔钩子注册
  schema.py         ExecutionFeedback dataclass
```

## 各文件职责

### `pipeline.py` — 执行流水线

编排 plan -> parameter -> execute 的完整流水线。

- `run_pipeline(user_input, planner, task_executor, execute_tool_fn, ...)` — 核心编排入口
  - 调用 `planner.plan_tasks()` 规划任务序列
  - 调用 `planner.annotate_tasks()` 填充参数
  - 逐任务调用 `task_executor.execute_single_task()` 执行
  - 支持重规划循环（`max_replans` 参数）
  - 返回执行结果列表

- `assess_result(result)` — 评估单步执行结果
  - 检查 `validation.verified` 和 `validation.meets_requirements`
  - 回退检查 `feedback.signal == SUCCESS`

所有依赖通过参数注入，不导入 Robot_Module、Data_Module 等。

### `skill_registry.py` — 全局技能注册

负责全局技能注册和可插拔钩子：

- `register_skill(skill)` / `get_skill(name)` / `all_skills()`
- `register_context_hook(hook)` — 注册参数计算上下文钩子
- `build_planning_context(object_facts, tasks)` — 调用所有上下文钩子
- `register_rule_planner(planner_fn)` — 注册规则规划器
- `try_rule_planners(user_input, scene_facts, object_facts)` — 尝试规则规划
- `register_lowlevel_prompt(prompt_path)` / `get_lowlevel_prompt_path()`
- `register_highlevel_prompt(prompt_path)` / `get_highlevel_prompt_path()`
- `register_navigation_model_uses(codes)` / `get_navigation_model_uses()`

这些钩子由 `Robot_Module/tasks/__init__.py` 在加载技能模块时注册。

### `skill_base.py` — SkillBase 技能基类

技能基类定义：

- `name` — 技能名
- `execute()` — 执行入口
- `check_completion()` — 轮询完成判定
- `validate()` — 事后校验
- `calculate_parameters()` — 参数计算
- `normalize_tool_arguments()` — 参数修正
- `register_tool()` — MCP 工具注册

### `runtime.py` — 执行协议

负责：

- `file / udp / ros` 三种执行后端
- `model_use / velocity / goal / start` 命令下发
- 通用常量 `MODEL_USE_IDLE = 0`
- `goal_command` 解析
- ROS2 命令发布：`/go2/skill_command`、`/go2/cmd_vel`、`/go2/goal_pose`
- 不包含任何项目特定常量

### `state.py` — 状态读取与校验

负责：

- 通过 `Hardware_Module.get_state()` 读取统一状态
- 提取 `robot_pose / goal / model_use / skill / timestamp / box_pose`
- 0.5 秒轮询完成判定（`wait_for_skill_completion`）
- 导航到达判定（`wait_for_navigation_completion`）— 导航码通过 `get_navigation_model_uses()` 获取
- 通用事后校验（通过 skill registry 委托）

### `executor.py` — 执行编排

负责：

- 执行前读取实时状态（含箱子位置）
- 下发命令
- 0.5 秒轮询等待，满足条件提前停止
- 超时后 stop 命令
- 执行后再次读取状态
- 轮询判定 + 事后校验双保险
- 构造 `execution_feedback`
- 写入 `validation`

### `schema.py` — ExecutionFeedback

```python
@dataclass
class ExecutionFeedback:
    success: bool
    action: str
    task: str
    task_type: str = ""
    feedback: dict = {}
    result: dict = {}
    assessment_message: str = ""
```

## 执行流

```text
Robot_Module/tasks/bishe/*.py
-> Excu_Module/executor.wait_skill_feedback()
   -> Hardware_Module.get_state() 读取执行前状态
   -> runtime.apply_envtest_command() 下发命令
   -> state.wait_for_skill_completion() 0.5s轮询等待
      -> 每轮：取状态 -> 检查技能完成条件 -> 满足则提前停止
   -> runtime.stop_envtest_skill() 停止
   -> Hardware_Module.get_state() 读取执行后状态
   -> state.validate_*() 做验收
   -> 返回 execution_feedback
```

## 成功判定

动作成功不是只看 `signal == SUCCESS`，真正关键的是：

- `validation.verified` — 是否获取到真实执行数据
- `validation.meets_requirements` — 动作是否满足任务要求

没有实时状态时：
- 规划仍可能继续
- 但执行直接失败

## 与其他模块的关系

### 与 Hardware_Module

`Excu_Module` 不直接假设某个状态文件格式。它只依赖 `Hardware_Module.get_state()` 返回的统一状态。

### 与 Robot_Module

- `Robot_Module` 注册技能，提供任务技能实现，注册可插拔钩子
- `Excu_Module` 负责执行管理和结果验收，不包含任何项目特定代码

### 与 Planner_Module

- `pipeline.py` 编排 Planner 和 TaskExecutor 的调用
- 不直接导入 Planner_Module，通过参数注入

### 与 Data_Module

- `Excu_Module` 不导入 `Data_Module`
- 数据获取由上层（Robot_Module/tools.py）完成
