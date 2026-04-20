# Robot_Module

技能注册层和任务技能实现层。基于 FastMCP 注册工具，分任务类型分发技能。

## 核心接口

```python
from Robot_Module import register_all, get_tool_definitions

# 注册所有工具
register_all()

# 获取工具定义
agent_tools = get_tool_definitions({"vlm_observe", "robot_act"})
action_tools = get_tool_definitions({"walk", "navigation", ...})
```

## 目录结构

```text
Robot_Module/
  __init__.py       对外暴露 register_all, mcp, get_tool_definitions 等
  tools.py          轻量 MCP 工具注册中心
  tasks/
    __init__.py     任务分发注册表
    bishe/
      __init__.py
      walk.py       行走技能
      navigation.py 导航技能
      nav_climb.py  导航攀爬技能
      climb_align.py 攀爬对正技能
      climb.py      攀爬技能
      push_box.py   推箱子技能
      way_select.py 路线选择技能
  vision/
    __init__.py
    vlm_observe.py  视觉观察技能
```

## 各文件职责

### `tools.py` — 工具注册中心

轻量 MCP 注册中心，替代原有的 `agent_tools.py`：

- `mcp` — FastMCP 实例
- `register_all()` — 注册所有技能模块（task tools + vision tools + vlm_observe + robot_act）
- `get_skill_function(name)` — 按名获取已注册技能函数
- `get_tool_definitions(allowed_names)` — 获取 MCP 工具定义列表
- `get_agent_tool_definitions()` — 获取 agent 层工具（vlm_observe, robot_act）
- `get_action_tool_definitions()` — 获取动作技能工具
- `vlm_observe` — 顶层 Vision tool（MCP 注册）
- `robot_act` — 顶层 Action tool（MCP 注册）
- `_run_robot_act_pipeline(..., on_event=None)` — robot_act 内部实现，`on_event` 透传到 `run_pipeline()`

### `tasks/__init__.py` — 任务分发注册表

管理任务类型到技能模块的映射：

```python
_TASK_REGISTRY = {
    "bishe": [
        "Robot_Module.tasks.bishe.walk",
        "Robot_Module.tasks.bishe.navigation",
        ...
    ],
}
```

- `register_tools(mcp, task=None)` — 加载指定任务类型的所有技能模块并注册到 MCP
- 注册可插拔钩子（如 `register_navigation_model_uses({4, 5})`）

### `tasks/bishe/*.py` — Bishe 任务技能

7 个动作技能实现文件：

| 技能 | 功能 | 命令类型 | model_use |
|------|------|---------|-----------|
| `walk` | 行走（前后左右四方向） | velocity `[vx,vy,0]` | 1 |
| `navigation` | 导航到目标点 | goal `[x, y, z]` | 4 |
| `nav_climb` | 导航并攀爬 | goal `[x, y, z]` | 5 |
| `climb_align` | 攀爬对正 | goal `[x, y, z, yaw]` | 4 |
| `climb` | 攀爬 | velocity `[vx, 0, 0]` | 2 |
| `push_box` | 推箱子 | goal `[x, y, z]` | 3 |
| `way_select` | 路线选择 | velocity `[0, vy, 0]` | 1 |

`walk` 支持 `route_side` = 前/后/左/右，对应 `[vx,0,0]` / `[-vx,0,0]` / `[0,-vy,0]` / `[0,vy,0]`。

所有技能的 MCP 注册参数均有默认值，规划器漏传参数不会导致 TypeError。

每个技能文件职责：
- 定义技能函数（参数接收 + 请求组装）
- 必要时做少量任务内目标计算
- 调用 `Excu_Module` 的 `wait_skill_feedback()` 执行和验收
- 通过 `register_tools(mcp)` 将自己注册为 MCP 工具

### `vision/vlm_observe.py` — 视觉观察技能

`vlm_observe` 的实现：

- 调用 `Data_Module/vlm.py`（VLMCore）获取视觉描述
- 调用 `Hardware_Module.get_state()` 获取实时状态
- 合并视觉事实和物体事实
- 返回 `visual_context` + `scene_facts` + `env_state`

## 对外暴露方式

最外层 agent 视角下，使用 2 个高层工具：

- `vlm_observe` — 观察环境
- `robot_act` — 执行动作链

`robot_act` 内部：
1. 同步 live data 到 object_facts（`Hardware_Module/registry.py`）
2. 组装 planner_context（`Data_Module/context.py`）
3. 高层规划（`Planner_Module/planner.py`）
4. 参数计算（`Data_Module/params.py`）
5. 低层执行（`Planner_Module/executor.py`）
6. 逐技能执行和验收（`Excu_Module`）

## 注册链路

```text
Robot_Module/tools.py
-> register_all()
   -> tasks/__init__.py: register_tools(mcp)
      -> tasks/bishe/*.py: 每个技能注册自己
   -> vision/vlm_observe.py: register_tools(mcp) -> vlm skill
   -> tools.py: vlm_observe (顶层 Vision tool)
   -> tools.py: robot_act (顶层 Action tool)
```

## 返回结果约定

动作技能返回两层结果：

1. `execution_feedback`
   - `signal` — SUCCESS / FAILURE
   - `message` — 执行描述
   - `validation` — 校验详情
2. `execution_result`
   - 执行模式、backend、参数、校验细节

关键校验字段：
- `execution_feedback.validation.verified`
- `execution_feedback.validation.meets_requirements`

## 与其他模块的关系

### 与 Excu_Module

技能文件调用 `Excu_Module/executor.py` 的 `wait_skill_feedback()` 执行和验收。

### 与 Data_Module

- `tools.py` 调用 `Data_Module` 加载 facts、参数计算、构建上下文
- `vision/vlm_observe.py` 调用 `Data_Module/vlm.py`（VLMCore）

### 与 Hardware_Module

- `vision/vlm_observe.py` 调用 `Hardware_Module.get_state()` 获取实时状态
- `tools.py` 调用 `Hardware_Module/registry.py` 同步数据

### 与 Planner_Module

- `tools.py` 创建 Planner 和 TaskExecutor 实例
- 调用 `Excu_Module/pipeline.py` 的 `run_pipeline()` 编排整个流程

## 本地运行

```bash
cd /home/xcj/work/FinalProject
python run.py
```
