# Excu_Module

`Excu_Module` 是当前项目的通用执行层。

它的目标是把“具体技能实现”和“通用执行管理”分开：

- `Robot_Module/module/Action/Task/Bishe/*.py`
  只负责技能实现
- `Excu_Module`
  负责统一执行、等待、验收

## 当前结构

```text
Excu_Module/
  runtime.py    执行协议、基础常量、命令下发
  state.py      统一读取 Comm state，提供校验方法
  executor.py   串联执行前状态、命令下发、执行后状态、validation
```

## 当前职责

### `runtime.py`

负责：

- `file / udp / ros` 执行后端
- `model_use / velocity / goal / start` 命令下发
- 速度、超时、buffer 等基础参数
- `goal_command` 解析

### `state.py`

负责：

- 通过 `Comm_Module.get_state()` 读取统一状态
- 提取 `robot_pose / goal / model_use / skill / timestamp`
- 导航到达判定
- `walk / way_select / climb / push_box` 的通用状态校验

### `executor.py`

负责：

- 执行前读取实时状态
- 下发命令
- 等待执行
- 执行后再次读取状态
- 构造 `execution_feedback`
- 写入 `validation`

## 当前执行流

```text
Bishe/*.py
-> Excu_Module/executor.wait_skill_feedback()
   -> Comm_Module.get_state() 读取执行前状态
   -> runtime.apply_envtest_command() 下发命令
   -> 等待执行
   -> Comm_Module.get_state() 读取执行后状态
   -> state.validate_*() 做验收
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

例如 `walk` 会返回：

```text
walk 无法开始执行: 缺少实时状态
```

## 与 Comm_Module 的关系

`Excu_Module` 不直接假设某个状态文件格式。
它只依赖 `Comm_Module` 返回的统一状态。

当前执行层主要用这些标准字段：

- `observation.agent_position`
- `observation.environment.obstacles`
- `runtime.timestamp`
- `runtime.snapshot`
- `runtime.skill`
- `runtime.model_use`
- `runtime.goal`
- `runtime.start`

## 与 Robot_Module 的关系

当前边界是：

- `Robot_Module`
  注册技能，提供任务技能实现
- `Excu_Module`
  负责执行管理和结果验收

后续如果增加新的动作任务，优先保持这个边界不变。
