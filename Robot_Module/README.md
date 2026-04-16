# Robot_Module

## 作用

`Robot_Module` 是技能注册层和任务技能实现层。
它负责：

- 注册上层 agent 工具
- 注册动作技能和视觉技能
- 放置具体任务技能实现

它不再承担通用执行管理；通用执行已经移到 `Excu_Module`。

当前 `Action` 和 `Vision` 是平级模块，不存在额外的 `Perception` 包裹层。

## 当前结构

- [agent_tools.py](agent_tools.py)
  顶层注册中心，提供 `vlm_observe` 和 `robot_act`
  使用 `_StreamingBuffer` 捕获 `LLM_Module` 的 stdout 日志
- [module/Action/skills.py](module/Action/skills.py)
  动作任务开关和注册入口
- [module/Action/Task/Bishe](module/Action/Task/Bishe)
  当前 `bishe` 动作任务实现，下面只保留 7 个技能文件
- [module/Vision/skills.py](module/Vision/skills.py)
  视觉任务开关和注册入口
- [module/Vision/Task/Bishe](module/Vision/Task/Bishe)
  当前 `bishe` 视觉任务实现
- [module/Action/example.py](module/Action/example.py)
  示例模块

## 对外暴露方式

最外层智能体视角下，主要使用 2 个高层工具：

- `vlm_observe`
- `robot_act`

其中：

- `robot_act` 是顶层 Action tool
- `vlm_observe` 是顶层 Vision tool

`robot_act` 内部继续复用动作技能链。`vlm_observe` 当前会调用 Vision 下层技能 `vlm`，并用 `Comm_Module` 补一份结构化 `env_state`。

## 当前注册结构

当前注册链路是：

```text
Robot_Module/agent_tools.py
-> register_tools(mcp)
   -> register_action_tools(mcp)
   -> Robot_Module/module/Action/skills.py
   -> Robot_Module/module/Action/Task/Bishe/*.py
   -> register_vision_tools(mcp)
   -> Robot_Module/module/Vision/skills.py
   -> Robot_Module/module/Vision/Task/Bishe/vlm_observe.py  (skill: vlm)
   -> vlm_observe  (top-level Vision tool)
   -> robot_act
```

也就是说：

- `Action` 和 `Vision` 是并列注册
- `agent_tools.py` 不再把 `Vision` 当成特殊嵌套层
- `agent_tools.py` 通过一个统一的 `register_tools()` 一次性注册 `Action + Vision + robot_act`

顶层 `agent_tools.py` 当前会注册：

- `vlm_observe`
- `robot_act`

内部动作技能仍然会单独注册到工具表中，供 `robot_act` 内部调用。

## 内部动作技能

当前 `Action/Task/Bishe` 提供 7 个动作技能：

- `walk`
- `navigation`
- `nav_climb`
- `climb_align`
- `climb`
- `push_box`
- `way_select`

## 内部视觉技能

当前 `Vision/Task/Bishe` 的下层视觉技能是：

- `vlm`

也就是说当前结构是：

- 顶层 Vision tool：`vlm_observe`
- 下层 Vision skill：`vlm`
- 顶层 Action tool：`robot_act`
- 下层 Action skills：7 个动作技能

`vlm_observe` 当前返回三层结果：

- `visual_context`
  纯视觉语义
- `scene_facts`
  纯视觉抽取出的粗粒度场景事实
- `env_state`
  `VLM + Comm_Module` 合并后的最终环境理解

`robot_act` 在进入 `LLM_Module` 之前，还会进一步组装固定格式的 `planner_context`，其中至少包含：

- `robot_state`
- `envtest_alignment`
- `constraints`
- `objects`
- `scene_facts`

这些文件当前只负责：

- 定义技能函数
- 必要时做少量任务内目标计算
- 调用 `Excu_Module`
- 在文件内自己 `register_tool(mcp)`

## 与 Excu_Module 的边界

`Robot_Module` 不再负责：

- 统一执行后端
- 执行等待
- 统一状态校验
- 导航到达判定

这些都已经放到 `Excu_Module`：

- `Excu_Module/runtime.py`
- `Excu_Module/state.py`
- `Excu_Module/executor.py`

## 当前工作流

`robot_act` 内部的动作部分现在是：

```text
LLM_Module 产出 function + parameters
-> agent_tools.py 调用已注册动作技能
-> Bishe/*.py 组装技能请求
-> Excu_Module 执行和验收
-> 返回 execution_feedback + validation
```

## 日志输出

- `Excu_Module/runtime.py` 通过 `print(..., file=sys.stderr)` 输出 `[go2.skill]`、`[go2.speech]` 等日志
- 交互模式下，`interactive.py` 会抑制这些 stderr 噪音
- `agent_tools.py` 的 `_StreamingBuffer` 捕获 `llm_core.py` / `llm_lowlevel.py` 的 stdout 输出，通过 `log_callback` 转发

## 当前执行方式

- 通过 FastMCP 注册工具
- 技能执行结果统一封装为结构化反馈
- `Excu_Module` 默认直接写 IsaacLab EnvTest 控制文件
- 可选执行后端：`file` / `udp` / `ros`
- 无论后端是什么，最终动作成功都应尽量依赖真实状态校验，而不是本地写死 `SUCCESS`
- `way_select` 默认按侧向 `walk` 控制；如需切到导航策略可设置：
  - `export FINALPROJECT_WAY_SELECT_POLICY=navigation`
- `push_box.target_position` 接收具体 `[x, y, z]` 坐标（由 `parameter_calculator` 计算），不再是 `"auto"`
- `push_box` 校验基于箱子位置（不是机器人位置），到达容许误差 0.1m

## 返回结果约定

动作技能当前会返回两层结果：

1. `execution_feedback`
   - `signal`
   - `message`
   - `validation`
2. `execution_result`
   - 执行模式
   - backend
   - 参数
   - 校验细节

其中最关键的是：

- `execution_feedback.validation.verified`
- `execution_feedback.validation.meets_requirements`

上层 `LLM_Module` 现在会用这两个字段做最终成功判定。

## 本地运行

直接启动交互入口：

```bash
cd /home/robot/work/FinalProject/Interactive_Module
python interactive.py
```

## 与上层的关系

通常由 [Interactive_Module/interactive.py](../Interactive_Module/interactive.py) 间接调用：

- 顶层智能体决定直接回复、调用 `vlm_observe`，或调用 `robot_act`
- `robot_act` 内部复用 `LLM_Module` 的规划、参数计算和低层执行
- `Robot_Module` 负责注册视觉或动作技能
- `Excu_Module` 负责产出真实状态校验结果
