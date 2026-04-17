# Excu_Module

`Excu_Module` 是当前项目的通用执行层。它不包含任何项目特定代码（如 Bishe），只提供通用执行基础设施。

它的目标是把"具体技能实现"和"通用执行管理"分开：

- `Robot_Module/module/Action/Task/<Project>/*.py`
  只负责技能实现
- `Excu_Module`
  负责统一执行、等待、验收

## 当前结构

```text
Excu_Module/
  __init__.py       对外暴露的公共接口
  runtime.py        执行协议、通用常量、命令下发
  state.py          统一读取 Comm state，提供校验方法
  executor.py       串联执行前状态、命令下发、执行后状态、validation
  skill_base.py     技能基类（SkillBase）
  skill_registry.py 全局技能注册 + 可插拔钩子注册
```

## 当前职责

### `skill_registry.py`

负责全局技能注册和可插拔钩子：

- `register_skill(skill)` / `get_skill(name)` / `all_skills()`
- `register_context_hook(hook)` — 注册参数计算上下文钩子
- `build_planning_context(object_facts, tasks)` — 调用所有上下文钩子
- `register_rule_planner(planner_fn)` — 注册规则规划器
- `try_rule_planners(user_input, scene_facts, object_facts)` — 尝试规则规划
- `register_lowlevel_prompt(prompt_path)` — 注册低层 prompt 路径
- `get_lowlevel_prompt_path()` — 获取注册的低层 prompt 路径
- `register_highlevel_prompt(prompt_path)` — 注册高层 prompt 路径
- `get_highlevel_prompt_path()` — 获取注册的高层 prompt 路径
- `register_navigation_model_uses(codes)` — 注册导航 model_use 集合
- `get_navigation_model_uses()` — 获取导航 model_use 集合

这些钩子由 `Robot_Module/module/Action/skills.py` 在加载技能模块之前注册。

### `skill_base.py`

技能基类 `SkillBase`，定义：

- `name` — 技能名
- `execute()` — 执行入口
- `check_completion()` — 轮询完成判定
- `validate()` — 事后校验
- `calculate_parameters()` — 参数计算（由 `ParameterCalculator` 调用）
- `normalize_tool_arguments()` — 参数修正（由 `LowLevelExecutor` 调用）
- `register_tool()` — MCP 工具注册

### `runtime.py`

负责：

- `file / udp / ros` 执行后端
- `model_use / velocity / goal / start` 命令下发
- 通用常量 `MODEL_USE_IDLE = 0`
- `goal_command` 解析
- 不再包含任何项目特定常量（如 Bishe 的速度/超时参数）

### `state.py`

负责：

- 通过 `Comm_Module.get_state()` 读取统一状态
- 提取 `robot_pose / goal / model_use / skill / timestamp / box_pose`
- 0.5 秒轮询完成判定（`wait_for_skill_completion`）
- 导航到达判定（`wait_for_navigation_completion`）— 导航码通过 `get_navigation_model_uses()` 获取
- 各技能的轮询判定函数（通过 skill registry 委托）
- 通用事后校验（通过 skill registry 委托）

### `executor.py`

负责：

- 执行前读取实时状态（含箱子位置）
- 下发命令
- 0.5 秒轮询等待（`wait_for_skill_completion`），满足条件提前停止
- 超时后 stop 命令
- 执行后再次读取状态
- 轮询判定 + 事后校验双保险
- 构造 `execution_feedback`
- 写入 `validation`
- 导航码通过 `get_navigation_model_uses()` 判断

## 当前执行流

```text
<Project>/*.py
-> Excu_Module/executor.wait_skill_feedback()
   -> Comm_Module.get_state() 读取执行前状态（含箱子位置）
   -> runtime.apply_envtest_command() 下发命令
   -> state.wait_for_skill_completion() 0.5s轮询等待
      -> 每轮：取状态 -> 检查技能完成条件 -> 满足则提前停止
   -> runtime.stop_envtest_skill() 停止
   -> Comm_Module.get_state() 读取执行后状态
   -> state.validate_*() 做验收（双保险）
   -> 返回 execution_feedback
```

## 当前成功判定

当前动作成功不是只看：

- `status`
- `signal == SUCCESS`

真正关键的是：

- `validation.verified`
- `validation.meets_requirements`

没有实时状态时，当前行为是：

- 规划仍可能继续
- 但执行直接失败

## 与其他模块的关系

### 与 Comm_Module

`Excu_Module` 不直接假设某个状态文件格式。
它只依赖 `Comm_Module` 返回的统一状态。

### 与 Robot_Module

当前边界是：

- `Robot_Module`
  注册技能，提供任务技能实现，注册可插拔钩子
- `Excu_Module`
  负责执行管理和结果验收，不包含任何项目特定代码

### 与 LLM_Module

- `LLM_Module` 通过 `skill_registry` 的钩子获取项目特定逻辑
- `Excu_Module` 不导入 `LLM_Module`

## 与 Bishe 的解耦

`Excu_Module` 不再包含任何 Bishe 特定代码：

- 不再从 `_bishe_helpers` re-export 常量
- `NAVIGATION_MODEL_USES` 通过 `register_navigation_model_uses()` 注册
- 项目特定的上下文、规则规划、prompt 通过可插拔钩子注入
- `runtime.py` 只定义通用常量 `MODEL_USE_IDLE = 0`
