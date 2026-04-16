# FinalProject Dataflow

## 总览

```text
用户输入
-> Interactive_Module/interactive.py
   -> 顶层 agent 判断：直接回复 / vlm_observe / robot_act

vlm_observe
-> Robot_Module/module/Vision/Task/Bishe/vlm_observe.py
-> VLM_Module/vlm_core.py
-> 返回 visual_context + scene_facts

robot_act
-> Robot_Module/agent_tools.py
   -> Comm_Module 同步 live data 到 object_facts
   -> load_object_facts()
   -> VLMCore.merge_scene_facts()
   -> LLM_Module/llm_core.py
      -> HighLevelPlanner.plan_tasks()
      -> ParameterCalculator.annotate_tasks()
      -> LowLevelExecutor.execute_single_task()
      -> Robot_Module/module/Action/Task/Bishe/*.py
      -> Excu_Module/executor.py
      -> Comm_Module.get_state()
      -> 真实状态校验(validation)
```

## 1. 入口层

- 文本入口是 `Interactive_Module/interactive.py`
- 工具入口只有两个：
  - `vlm_observe`
  - `robot_act`

顶层 agent 只决定三件事：

- 直接回复
- 先看环境
- 开始任务规划和执行

## 2. 顶层对话循环

`interactive.py` 的 `run_agent_turn()` 会在一次用户输入里循环处理模型回复和工具调用。

渲染顺序是实时的，不是最后一次性输出：

1. Thinking 面板
2. `vlm_observe` 面板
3. 第二次 Thinking 面板
4. `robot_act` 流式日志
5. `robot_act` 摘要
6. Assistant 面板

`robot_act` 内部的 stdout 会被 `Robot_Module/agent_tools.py` 的 `_StreamingBuffer` 转成流式终端输出。

## 3. 视觉链路

视觉调用路径：

```text
interactive.py
-> Robot_Module agent tool: vlm_observe
-> Robot_Module/module/Vision/Task/Bishe/vlm_observe.py
-> VLM_Module/vlm_core.py
```

图片来源优先级：

- `/tmp/envtest_front_camera.png`
- 本机摄像头
- `VLM_Module/assets/2.png`

VLM 产出两类数据：

- `visual_context`
  给终端显示，也可回传给顶层 agent。
- `scene_facts`
  给 `robot_act` 规划阶段使用。

## 4. 状态链路

统一状态入口只有一个：

```python
from Comm_Module import get_state
state = get_state()
```

当前状态链路：

```text
Comm_Module/Task/Sim/get_data.py
-> 返回原始数据
-> Comm_Module/Task/Sim/Data.py 定义 schema
-> Comm_Module/Status/get_state.py 自动解析
-> 上层得到统一 state
```

执行层只关心统一状态，不再自己耦合某个具体 JSON 文件结构。

当前执行会重点用到这些状态字段：

- `observation.agent_position`
- `observation.environment.obstacles`
- `runtime.timestamp`
- `runtime.snapshot`
- `runtime.skill`
- `runtime.model_use`
- `runtime.goal`
- `runtime.start`

## 5. object_facts 链路

`robot_act` 启动时会先尝试同步 live EnvTest 数据到 `config/object_facts.json`。

同步路径：

```text
Robot_Module/agent_tools.py
-> Comm_Module.Task.sync_object_facts_from_live_data()
-> Comm_Module/Task/Sim/get_data.py
-> 更新 object_facts.json
```

之后才进入规划：

```text
load_object_facts()
-> merge_scene_facts()
-> LLM planning
```

这保证规划优先使用最新几何信息，而不是残留旧场景。

## 6. 高层规划

高层规划入口在 `LLM_Module/llm_core.py`。

输入包括：

- `user_input`
- `visual_context`
- `scene_facts`
- `object_facts`

输出是抽象任务列表 `tasks`。每个任务至少包含：

- `step`
- `task`
- `type`
- `function`
- `reason`

高层规划负责“做什么”，不负责“最终参数是多少”。

## 7. 参数计算

参数计算由 `LLM_Module/parameter_calculator.py` 负责。

它会为每个 task 补：

- `parameter_context`
- `calculated_parameters`

当前的设计目标是：

- 高层决定技能序列
- 参数计算把抽象任务补成可直接执行的技能参数
- 技能文件只负责把这些参数交给执行层

## 8. 低层执行

低层执行入口在 `LLM_Module/llm_lowlevel.py`。

分两种情况：

1. 已经有 `function + calculated_parameters`
   直接执行技能。
2. 参数还不完整
   让低层 LLM 决定最终工具调用。

真正执行技能时，`agent_tools.py` 会通过 `_execute_registered_tool()` 调用已注册的动作技能函数。

## 9. 动作技能层

动作注册路径：

```text
Robot_Module/module/Action/skills.py
-> 选择当前 action task
-> 注册 Robot_Module/module/Action/Task/Bishe/*.py
```

`Action/Task/Bishe` 下面现在只保留 7 个技能文件：

- `walk.py`
- `navigation.py`
- `nav_climb.py`
- `climb_align.py`
- `climb.py`
- `push_box.py`
- `way_select.py`

这些文件只做两件事：

- 定义该技能的输入和少量任务内逻辑
- 调用 `Excu_Module`

## 10. 统一执行层

当前真正的执行控制都在 `Excu_Module`：

- `runtime.py`
  执行协议、环境变量、命令下发。
- `state.py`
  统一读取 `Comm_Module` 状态并做通用验收。
- `executor.py`
  把“发命令 -> 等待 -> 校验”串起来。

执行流是：

```text
skill file
-> Excu_Module/executor.py
   -> 读取执行前状态
   -> 下发命令
   -> 等待执行
   -> 再读执行后状态
   -> 做 validation
   -> 返回 execution_feedback
```

## 11. 成功判定

成功判定现在分三层：

1. 技能函数返回结构化结果
2. `execution_feedback.validation`
3. `LLM_Module/llm_core.py` 的最终结果评估

关键规则：

- 不能只看 `signal == SUCCESS`
- 必须看 `validation.verified`
- 必须看 `validation.meets_requirements`

如果没有实时状态：

- 可以继续规划
- 但执行阶段会失败

## 12. 一个完整例子

用户输入：`往前走1米`

```text
用户输入
-> interactive.py 顶层 agent
-> Thinking: 先观察环境
-> vlm_observe
-> Thinking: 开始动作执行
-> robot_act
   -> Comm_Module 同步 live data
   -> 高层规划得到 [walk]
   -> parameter_calculator 算出 distance=1.0
   -> 调用 Action/Task/Bishe/walk.py
   -> walk.py 调用 Excu_Module/executor.py
   -> Excu_Module 读取 Comm state
   -> 下发命令
   -> 再次读取 Comm state
   -> 校验是否真的前进了足够距离
   -> 返回 validation
-> llm_core 根据 validation 判定是否继续
-> Assistant 输出最终回复
```

## 一句话总结

现在的数据流是：

`Interactive` 负责对话，`VLM` 负责观察，`LLM` 负责规划和参数，`Robot` 负责注册技能，`Excu` 负责执行和验收，`Comm` 负责统一状态。
