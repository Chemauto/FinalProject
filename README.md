# FinalProject

面向四足机器人的 LLM agent 项目。当前目标是把“对话、任务规划、参数计算、动作执行、状态验收”拆成清晰模块，便于后续从 `Bishe` 扩展到其他任务后端。

## 当前架构

最外层只暴露两个高层工具：

- `vlm_observe`
- `robot_act`

内部职责拆成 6 层：

- `Interactive_Module`
  顶层 TUI，对话、工具调度、渐进式渲染。

在 `Robot_Module` 内部，`Vision` 和 `Action` 是并列模块，不存在 `Perception` 再包一层 `Vision` 的结构。
- `VLM_Module`
  读取图片并输出结构化视觉语义，不负责真值状态。
- `LLM_Module`
  负责任务规划、子任务分解、参数计算、低层工具调用。
- `Robot_Module`
  负责技能注册和任务技能实现，不负责通用执行管理。
- `Excu_Module`
  负责统一执行、等待反馈、状态校验。
- `Comm_Module`
  负责统一状态入口，屏蔽具体数据来源。

## 当前主链路

```text
用户输入
-> Interactive_Module/interactive.py
   -> 顶层 LLM 判断：直接回复 / vlm_observe / robot_act

vlm_observe
-> Robot_Module/agent_tools.py 顶层 Vision tool
-> Robot_Module/module/Vision/Task/Bishe/vlm_observe.py 下层 vision skill: vlm
-> VLM_Module/vlm_core.py
-> Comm_Module.get_state()
-> 返回 visual_context + scene_facts + env_state

robot_act
-> Robot_Module/agent_tools.py
   -> Comm_Module 尝试同步 live data 到 object_facts
   -> 组装固定格式 planner_context
   -> LLM_Module/llm_core.py 高层规划
   -> LLM_Module/parameter_calculator.py 参数计算
   -> LLM_Module/llm_lowlevel.py 低层执行
   -> Robot_Module/module/Action/Task/Bishe/*.py 具体技能实现
   -> Excu_Module 统一执行和验收
   -> Comm_Module.get_state() 提供真实状态
```

## 当前目录职责

- `Comm_Module`
  通用状态层。`Task/<Type>/Data.py` 定义状态格式，`Task/<Type>/get_data.py` 获取真实原始数据，`Status/get_state.py` 自动解析统一状态。
- `Excu_Module`
  通用执行层。负责命令下发、执行等待、导航到达判定、动作前后状态校验。
- `LLM_Module`
  规划与执行代理层。把用户任务拆成技能序列，再把参数填入技能函数。
- `Robot_Module`
  注册层和任务技能层。`Action/Task/Bishe` 下面只保留 7 个技能实现文件，`Vision/Task/Bishe` 下面保留视觉技能实现。
- `VLM_Module`
  图像来源、视觉模型调用、结构化视觉输出。
- `Interactive_Module`
  最外层 CLI/TUI 入口。

## Action 当前结构

动作技能当前固定为 7 个：

- `walk`
- `navigation`
- `nav_climb`
- `climb_align`
- `climb`
- `push_box`
- `way_select`

当前分层是：

```text
Robot_Module/module/Action/skills.py
-> 选择 task，例如 bishe
-> 注册 Robot_Module/module/Action/Task/Bishe/*.py

Bishe/*.py
-> 只负责具体技能实现
-> 只负责把参数映射成执行请求
-> 必要时做少量任务内目标计算

Excu_Module
-> 真正负责执行和状态校验
```

`Action` 和 `Vision` 当前都是由 `Robot_Module/agent_tools.py` 并列注册的。
其中：

- 顶层工具是 `robot_act` 和 `vlm_observe`
- 下层 Action skills 当前是 `walk / navigation / ...`
- 下层 Vision skills 当前是 `vlm`

## 状态原则

当前遵循：

- `VLM` 负责理解场景的大致样子
- `Comm_Module` 负责提供真实状态和具体数值
- `LLM` 同时参考两者做规划
- `Excu_Module` 只用真实状态做执行验收
- 给规划大模型的状态输入必须是固定 JSON 结构，而不是零散自然语言

统一状态入口是：

```python
from Comm_Module import get_state
state = get_state()
```

当前标准状态至少包含：

- `connected`
- `task_type`
- `observation`
- `runtime`

其中 `runtime` 里会带：

- `timestamp`
- `snapshot`
- `skill`
- `model_use`
- `goal`
- `start`
- `scene_objects`

执行层只依赖这份统一状态，不再自己直接耦合某个具体状态文件格式。

## 执行原则

当前统一执行入口在 `Excu_Module`：

- `Excu_Module/runtime.py`
  执行协议、环境变量、命令下发、基础常量。
- `Excu_Module/state.py`
  从 `Comm_Module` 读取实时状态，并做通用校验。
- `Excu_Module/executor.py`
  串联“发命令 -> 等待 -> 做验收”。

关键原则：

- 规划可以在缺少 live 数据时继续。
- 动作执行不能假成功。
- 没有实时状态时，动作直接失败。
- `LLM_Module` 最终看的是 `validation.verified` 和 `validation.meets_requirements`，不是只看 `SUCCESS` 字符串。

## 最短运行方式

1. 启动 IsaacLab EnvTest player

```bash
cd /home/xcj/work/IsaacLab/IsaacLabBisShe
python NewTools/envtest_model_use_player.py --scene_id 3
```

2. 启动交互入口

```bash
cd /home/xcj/work/FinalProject/Interactive_Module
python interactive.py
```

3. 输入任务，例如：

```text
往前走1米
```

## 当前限制

- 当前只内置 `sim` 状态后端。
- 当前只内置 `bishe` 动作任务和视觉任务。
- 没开仿真时，VLM 可能退回默认图片，这不代表实时现场。
- 没有实时状态时，`robot_act` 仍可能完成规划，但执行阶段会失败。

## 相关文档

- `Dataflow.md`
- `CLAUDE.md`
- `Comm_Module/README.md`
- `Excu_Module/README.md`
- `LLM_Module/README.md`
- `Robot_Module/README.md`
- `VLM_Module/README.md`
- `Interactive_Module/README.md`
