# CLAUDE.md

本文件面向继续维护本仓库的协作者，描述当前真实分层、关键约束和需要保持的行为边界。

## 1. 当前定位

FinalProject 不是单一的"LLM 调工具"项目，而是 6 层协作：

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

## 2. 当前解耦状态

换项目只改 `Robot_Module` + `Comm_Module`，**零修改** `Excu_Module`、`LLM_Module`、`VLM_Module`。

`Excu_Module` 不包含任何项目特定代码。项目特定逻辑通过以下方式注入：

- `register_navigation_model_uses(codes)` — 注册导航技能的 model_use 码（当前 `{4, 5}`）
- `register_skill(skill)` — 每个技能文件注册 SkillBase 实例到全局注册表

注册时机在 `Robot_Module/module/Action/skills.py` 的 `register_tools()` 中：
1. 先调用 `register_navigation_model_uses({4, 5})`
2. 再逐个加载技能模块，每个模块在 `register_tools(mcp)` 内部调用 `register_skill(skill)`

`LLM_Module` 不导入任何项目特定模块。

## 3. 当前端到端流程

```text
user_input
-> Interactive_Module/interactive.py
   -> 顶层 LLM 判断：直接回复 / vlm_observe / robot_act

robot_act
-> Robot_Module/agent_tools.py
   -> Comm_Module 同步 live data
   -> load_object_facts()
   -> merge_scene_facts()
   -> LLMAgent.plan_tasks()
      -> LLM 生成任务序列
   -> ParameterCalculator.annotate_tasks()
   -> LowLevelExecutor.execute_single_task()
      -> load_prompts()
   -> Robot_Module/module/Action/Task/Bishe/*.py
   -> Excu_Module/executor.py
      -> 导航技能: wait_for_navigation_completion() (0.5s 轮询)
      -> 非导航技能: sleep(execution_time_sec) + before/after 状态对比
   -> Comm_Module.get_state()
```

必须守住的原则：

- 先同步，再规划。
- 高层负责任务序列，不负责底层执行协议。
- 技能文件只负责具体技能实现，不负责通用执行编排。
- 通用执行统一进 `Excu_Module`。
- 通用状态统一走 `Comm_Module`。
- `Excu_Module` 和 `LLM_Module` 不能导入任何项目特定模块（如 Bishe）。

## 4. 当前目录职责

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
  执行后端、命令下发、通用常量（`MODEL_USE_IDLE=0`）。
- `state.py`
  从 `Comm_Module` 读取统一状态，提供导航到达轮询判定。
- `executor.py`
  串联"发命令 -> 等待 -> 停止 -> 校验"。导航技能用轮询检测到达，非导航技能用超时等待。
- `skill_registry.py`
  全局技能注册（`register_skill` / `get_skill` / `all_skills`）+ 导航 model_use 码注册。
- `skill_base.py`
  技能基类。每个技能继承 `SkillBase`，实现 `name` + `execute()`。基类提供 `build_result()` 公共方法。

这里是当前动作执行的公共层，后续新增任务时优先复用这里，而不是回到 `Robot_Module` 再堆公共逻辑。

### `LLM_Module`

- `llm_core.py`
  高层规划和主执行流水线（`LLMAgent`，别名 `HighLevelPlanner`）。
- `parameter_calculator.py`
  参数计算（当前为 pass-through，保留扩展口）。
- `llm_lowlevel.py`
  低层工具调用，直接读取 `prompts/lowlevel_prompt.yaml`。
- `object_facts_loader.py`
  规范化 `object_facts.json`。约束通用化，不硬编码项目特定字段。

### `Robot_Module`

- `agent_tools.py`
  顶层注册中心，注册 `vlm_observe` 和 `robot_act`。
- `module/Action/skills.py`
  动作任务开关和注册。加载前注册 `navigation_model_uses`，然后逐个加载技能模块。
- `module/Action/Task/Bishe/*.py`
  7 个具体动作技能文件。每个文件包含一个 `SkillBase` 子类 + `register_tools(mcp)` 函数。
- `module/Vision/skills.py`
  视觉任务开关和注册。
- `module/Vision/Task/Bishe/vlm_observe.py`
  当前 Vision 下层技能实现文件，注册的 skill 名字是 `vlm`。直接调 VLM_Module API，不硬编码项目特定逻辑。

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

- `VLM` 只负责"大概看起来是什么"
- 真值状态、具体数值、槽位对象统一来自 `Comm_Module`
- `vlm_observe` 会额外返回 `env_state`，它是 `VLM + Comm_Module` 合并后的环境理解

## 5. Action 目录的硬约束

当前 `Robot_Module/module/Action/Task` 下面只应该有：

- `__init__.py`
- `Bishe/`

`Bishe/` 下面只有 7 个技能文件：

- `walk.py` / `navigation.py` / `nav_climb.py` / `climb_align.py` / `climb.py` / `push_box.py` / `way_select.py`

每个技能文件结构统一：

```python
from Excu_Module.skill_base import SkillBase

class XxxSkill(SkillBase):
    MODEL_USE = ...

    @property
    def name(self) -> str: return "xxx"

    async def execute(self, ..., **kw):
        # 参数校验 → 调 wait_skill_feedback 或 execute_goal_navigation_skill
        return self.build_result(feedback, ...)

def register_tools(mcp):
    from Excu_Module.skill_registry import register_skill
    skill = XxxSkill()
    register_skill(skill)

    @mcp.tool()
    async def xxx(...) -> str:
        result = await skill.execute(...)
        return json.dumps(result, ensure_ascii=False)
    return {"xxx": xxx}
```

不要再把公共执行逻辑放回 `Action/Task`。

## 6. 状态契约

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

## 7. object_facts 约束

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
- `constraints` 字段是通用键值直通，不硬编码项目特定约束名。

## 8. 规划和参数计算边界

高层规划只负责：

- 选择技能序列
- 说明原因
- 决定任务先后顺序

技能文件负责：

- 接收参数
- 做少量任务内逻辑
- 调用 `Excu_Module`

不要让高层 prompt 直接生成执行协议细节，也不要让技能文件重新变成公共执行层。

## 9. 成功判定规则

当前成功判定看：

- `execution_feedback.validation.verified`
- `execution_feedback.validation.meets_requirements`

`LLM_Module/llm_core.py` 按这两个字段决定是否继续下一步。

### 执行策略

**导航技能**（model_use=4, 5）：0.5 秒轮询检测到达

```text
下发 goal + start=1
  ↓
while 0.5s 轮询:
  取状态 → 计算距目标距离
  距离 <= 0.15m 且位置稳定 3 次 → SUCCESS
  start=False 或 model_use 变化 → FAILURE
  超时 → FAILURE
  ↓
stop 命令
```

**非导航技能**（model_use=1, 2, 3）：超时等待 + 事后状态对比

```text
下发 velocity/goal + start=1
  ↓
sleep(execution_time_sec)
  ↓
stop 命令
  ↓
取 after_state → 和 before_state 对比 → 构建校验结果
```

轮询频率可通过 `FINALPROJECT_STATUS_POLL_SEC` 环境变量调整（默认 0.5 秒）。

### 各技能执行参数

| 技能 | model_use | 命令类型 | 等待方式 |
|---|---|---|---|
| `walk` | 1 | velocity `[vx, 0, 0]` | 超时等待 |
| `climb` | 2 | velocity `[vx, 0, 0]` | 超时等待 |
| `push_box` | 3 | goal `[x, y, z]` | 超时等待 |
| `navigation` | 4 | goal `[x, y, z]` | 轮询检测到达 |
| `nav_climb` | 5 | goal `[x, y, z]` | 轮询检测到达 |
| `climb_align` | 4 | goal `[x, y, z]` | 轮询检测到达 |
| `way_select` | 1 或 4 | velocity 或 goal | 看模式 |

## 10. 当前最容易踩坑的点

- 没开仿真时，VLM 可能退回默认图片，这不代表真实场景。
- 没有实时状态时，规划可以继续，但执行必须失败。
- 若新增动作任务，不要把公共执行逻辑重新塞进 `Robot_Module/module/Action/Task/<TaskName>`。
- `Excu_Module` 和 `LLM_Module` 不能导入任何项目特定模块。
- `NAVIGATION_MODEL_USES` 必须通过 `register_navigation_model_uses()` 在 `skills.py` 中注册。
- 每个技能的 `register_tools()` 必须调用 `register_skill(skill)` 把实例注册到全局注册表。
- 不要在 `Excu_Module` 或 `LLM_Module` 中硬编码导航 model_use 码。

## 11. 推荐扩展方式

新增状态后端：

- `Comm_Module/Task/<Type>/Data.py`
- `Comm_Module/Task/<Type>/get_data.py`
- 在 `Comm_Module/Task/__init__.py` 注册

新增动作任务（参见第 12 节详细步骤）：

- `Robot_Module/module/Action/Task/<TaskName>/`
- 在 `Robot_Module/module/Action/skills.py` 注册

新增视觉任务：

- `Robot_Module/module/Vision/Task/<TaskName>/`
- 在 `Robot_Module/module/Vision/skills.py` 注册

新增通用执行能力：

- 优先改 `Excu_Module`

一句话：

`Task` 放任务实现，`Excu` 放通用执行，`Comm` 放通用状态。

## 12. 新增任务项目完整步骤

如果要换一个新项目（例如从 Bishe 换成搜救任务 Rescue），需要修改的文件全部在 `Robot_Module` 和 `Comm_Module` 内，`Excu_Module`、`LLM_Module`、`VLM_Module` 零修改。

### 第 1 步：创建技能文件

在 `Robot_Module/module/Action/Task/` 下新建项目目录：

```text
Robot_Module/module/Action/Task/Rescue/
  __init__.py
  search.py          # 搜寻技能
  grab.py            # 抓取技能
  ...
```

每个技能文件继承 `Excu_Module.skill_base.SkillBase`：

- `name` — 技能名（必须）
- `execute()` — 执行入口（必须）
- `check_completion()` — 轮询完成判定（可选，默认超时等待）
- `validate()` — 事后校验（可选，默认 None）

技能文件提供 `register_tools(mcp)` 函数，内部创建实例、注册到全局、定义 MCP tool。

### 第 2 步：注册到 skills.py

在 `Robot_Module/module/Action/skills.py` 中：

1. 添加模块列表：

```python
_TASK_SKILL_MODULES["rescue"] = [
    "Robot_Module.module.Action.Task.Rescue.search",
    "Robot_Module.module.Action.Task.Rescue.grab",
]
```

2. 如果有导航类技能，在 `register_tools()` 中调用 `register_navigation_model_uses({...})`。

### 第 3 步：更新 Comm_Module（如需新后端）

如果新项目用不同的状态源：

```text
Comm_Module/Task/Rescue/
  Data.py       # 定义状态格式
  get_data.py   # 获取原始数据
```

在 `Comm_Module/Task/__init__.py` 注册。

### 完整文件清单

| 文件 | 操作 |
|-----|------|
| `Robot_Module/module/Action/Task/<Project>/*.py` | 新建技能文件 |
| `Robot_Module/module/Action/skills.py` | 添加 task 模块列表 + 导航码 |
| `Comm_Module/Task/<Type>/` | 新建（如需新后端） |
| `Excu_Module/*` | **不改** |
| `LLM_Module/*` | **不改** |
| `VLM_Module/*` | **不改** |
| `Interactive_Module/*` | **不改** |
