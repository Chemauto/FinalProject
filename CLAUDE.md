# CLAUDE.md

本文件面向继续维护本仓库的协作者，描述当前真实分层、关键约束和需要保持的行为边界。

## 1. 当前定位

FinalProject 不是单一的“LLM 调工具”项目，而是 6 层协作：

1. `Interactive_Module`
   对话和终端展示。
2. `VLM_Module`
   环境观测。
3. `LLM_Module`
   任务规划、参数计算、低层调度。
4. `Robot_Module`
   技能注册和任务技能实现。
5. `Excu_Module`
   通用执行和统一验收。
6. `Comm_Module`
   统一状态抽象。

最外层只暴露两个高层工具：

- `vlm_observe`
- `robot_act`

当前对应关系：

- `vlm_observe`
  是顶层 Vision tool
- `robot_act`
  是顶层 Action tool
- Vision 下层当前只有一个技能：`vlm`
- Action 下层当前有 7 个技能

## 2. 当前端到端流程

```text
user_input
-> Interactive_Module/interactive.py
   -> 顶层 LLM 判断：直接回复 / vlm_observe / robot_act

robot_act
-> Robot_Module/agent_tools.py
   -> Comm_Module 同步 live data
   -> load_object_facts()
   -> merge_scene_facts()
   -> HighLevelPlanner.plan_tasks()
   -> ParameterCalculator.annotate_tasks()
   -> LowLevelExecutor.execute_single_task()
   -> Robot_Module/module/Action/Task/Bishe/*.py
   -> Excu_Module/executor.py
   -> Comm_Module.get_state()
   -> validation
```

必须守住的原则：

- 先同步，再规划。
- 高层负责任务序列，不负责底层执行协议。
- 参数计算负责把抽象任务补成具体参数。
- 技能文件只负责具体技能实现，不负责通用执行编排。
- 通用执行统一进 `Excu_Module`。
- 通用状态统一走 `Comm_Module`。
- 动作成功不能靠本地写死文案，必须尽量基于真实状态。

## 3. 当前目录职责

### `Interactive_Module`

- `interactive.py`
  顶层 agent、工具调度、渐进式渲染。

### `Comm_Module`

- `Status/get_state.py`
  通用状态入口。
- `Task/Sim/Data.py`
  定义 `sim` 后端状态格式。
- `Task/Sim/get_data.py`
  获取真实原始数据，同步 `object_facts.json`。

当前规则：

- `Data.py` 只定义格式。
- `get_data.py` 只取原始数据。
- `Status/get_state.py` 自动解析统一状态。

### `Excu_Module`

- `runtime.py`
  执行后端、命令下发、基础常量。
- `state.py`
  从 `Comm_Module` 读取统一状态，提供通用校验方法和轮询完成判定。
- `executor.py`
  串联"发命令 -> 轮询等待(0.5s/次) -> 提前停止或超时 -> 验收"。

这里是当前动作执行的公共层，后续新增任务时优先复用这里，而不是回到 `Robot_Module` 再堆公共逻辑。

### `LLM_Module`

- `llm_core.py`
  高层规划和主执行流水线。
- `parameter_calculator.py`
  参数计算。
- `llm_lowlevel.py`
  低层工具调用。
- `object_facts_loader.py`
  规范化 `object_facts.json`。

### `Robot_Module`

- `agent_tools.py`
  顶层注册中心，注册 `vlm_observe` 和 `robot_act`。
- `module/Action/skills.py`
  动作任务开关和注册。
- `module/Action/Task/Bishe/*.py`
  7 个具体动作技能文件。
- `module/Vision/skills.py`
  视觉任务开关和注册。
- `module/Vision/Task/Bishe/vlm_observe.py`
  当前 Vision 下层技能实现文件，注册的 skill 名字是 `vlm`。

这里要注意：

- `module/Action` 和 `module/Vision` 是平级模块
- 不要再引入 `Perception -> Vision` 这种额外层级概念
- 当前注册方式是 `Action` 和 `Vision` 并列挂到 `agent_tools.py`
- `agent_tools.py` 里的 `register_tools()` 会统一注册 `Action + Vision + robot_act`
- 顶层 `vlm_observe` 会调用下层 Vision skill `vlm`

### `VLM_Module`

- `vlm_core.py`
  结构化视觉输出，只负责视觉语义。
- `image_source.py`
  图片来源优先级控制。

这里要注意：

- `VLM` 只负责“大概看起来是什么”
- 真值状态、具体数值、槽位对象统一来自 `Comm_Module`
- `vlm_observe` 当前会额外返回 `env_state`，它是 `VLM + Comm_Module` 合并后的环境理解

## 4. Action 目录的硬约束

当前 `Robot_Module/module/Action/Task` 下面只应该有：

- `__init__.py`
- `Bishe/`

`Bishe/` 下面只应该保留 7 个技能实现文件：

- `walk.py`
- `navigation.py`
- `nav_climb.py`
- `climb_align.py`
- `climb.py`
- `push_box.py`
- `way_select.py`

不要再把公共执行逻辑放回 `Action/Task`。
原来那类 `runtime.py / executor.py / status.py / scene.py / demo.py` 现在都应该视为 `Excu_Module` 或任务内局部逻辑的职责。

## 5. 状态契约

统一状态入口：

```python
from Comm_Module import get_state
state = get_state()
```

当前标准状态至少有：

- `connected`
- `task_type`
- `observation`
- `runtime`

当前执行链依赖这些字段：

- `observation.agent_position`
- `observation.environment.scene_id`
- `observation.environment.obstacles`
- `runtime.timestamp`
- `runtime.snapshot`
- `runtime.skill`
- `runtime.model_use`
- `runtime.goal`
- `runtime.start`
- `runtime.scene_objects`

后续如果新增后端，优先保持这份统一字段不变。

## 6. object_facts 约束

默认路径是 `config/object_facts.json`。

它的角色是：

- 给规划提供高可信几何信息
- 保存运行时同步结果
- 保存用户输入覆盖值

当前最关键字段：

- `navigation_goal`
- `robot_pose`
- `constraints`
- `objects`
- `runtime_state`

关键约束：

- 若 `objects` 残留旧场景，规划会明显偏离真实场景。
- 所以 `robot_act` 启动时必须优先尝试同步 live data。

## 7. 规划和参数计算边界

高层规划只负责：

- 选择技能序列
- 说明原因
- 决定任务先后顺序

参数计算负责：

- 补齐坐标
- 补齐方向
- 补齐距离或高度
- 补齐技能所需具体参数
- `push_box` 的 `target_position` 会算出具体 `[x, y, z]` 坐标，不再是 `"auto"`

技能文件负责：

- 接收参数
- 做少量任务内逻辑
- 调用 `Excu_Module`

不要让高层 prompt 直接生成执行协议细节，也不要让技能文件重新变成公共执行层。

## 8. 成功判定规则

当前成功判定不是只看：

- `status == success`
- `signal == SUCCESS`

真正关键的是：

- `execution_feedback.validation.verified`
- `execution_feedback.validation.meets_requirements`

`LLM_Module/llm_core.py` 已经按这两个字段决定是否继续下一步。

### 执行策略

当前所有技能（walk / climb / push_box / way_select / navigation / nav_climb / climb_align）统一采用 **0.5 秒轮询**：

```text
下发命令
  ↓
while 0.5s 轮询:
  取状态 → 检查该技能的完成条件
  满足 → 提前停止，返回 SUCCESS
  超时 → 返回 FAILURE
  ↓
stop 命令
  ↓
最终 before/after 校验（双保险）
```

轮询频率可通过 `FINALPROJECT_STATUS_POLL_SEC` 环境变量调整（默认 0.5 秒）。

### 各技能完成条件

| 技能 | 完成条件 |
|---|---|
| `walk` | 机器人平面位移 >= 要求距离，方向正确 |
| `climb` | 机器人 z 抬升 >= 要求高度 |
| `push_box` | 箱子距目标坐标 <= 0.1m（到达容许误差） |
| `way_select` | 机器人横向位移 >= 要求距离，方向正确 |
| `navigation` | 机器人距目标 <= 到达容许误差，位置稳定 |
| `nav_climb` | 同 navigation |
| `climb_align` | 同 climb |

### push_box 校验规则

- **主校验**：箱子距目标坐标 <= 0.1m（`FINALPROJECT_PUSH_BOX_ARRIVAL_TOL_M`）
- **后备**（取不到箱子位置时）：机器人平面位移 >= 0.3m
- 目标坐标由 `parameter_calculator._build_adjacent_ground_position()` 计算，只改变 x 方向，y 保持箱子当前位置
- 不再用最小位移 0.1m 作为成功门槛（0.1m 是测量噪声级别）

## 9. 当前最容易踩坑的点

- 没开仿真时，VLM 可能退回默认图片，这不代表真实场景。
- 没有实时状态时，规划可以继续，但执行必须失败。
- 不要再文档或代码里引用已删除的 `Robot_Module/skill.py`。
- 不要再文档或代码里引用已删除的 `Robot_Module/module/Action/navigation.py`。
- 若新增动作任务，不要把公共执行逻辑重新塞进 `Robot_Module/module/Action/Task/<TaskName>`。
- `push_box` 的 `target_position` 必须是具体坐标，不要退回 `"auto"`。
- `push_box` 校验基于箱子位置（不是机器人位置），0.1m 是到达容许误差（不是最小位移要求）。
- 位置测量误差约 0.1m，所以最小位移阈值不能低于噪声级别。

## 10. 推荐扩展方式

新增状态后端：

- `Comm_Module/Task/<Type>/Data.py`
- `Comm_Module/Task/<Type>/get_data.py`
- 在 `Comm_Module/Task/__init__.py` 注册

新增动作任务：

- `Robot_Module/module/Action/Task/<TaskName>/`
- 在 `Robot_Module/module/Action/skills.py` 注册

新增视觉任务：

- `Robot_Module/module/Vision/Task/<TaskName>/`
- 在 `Robot_Module/module/Vision/skills.py` 注册

新增通用执行能力：

- 优先改 `Excu_Module`

一句话：

`Task` 放任务实现，`Excu` 放通用执行，`Comm` 放通用状态。
