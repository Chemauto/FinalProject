# FinalProject

面向四足机器人的 LLM agent 项目。把"对话、任务规划、参数计算、动作执行、状态验收"拆成清晰模块，便于从 `Bishe` 扩展到其他任务后端。

## 五层架构

```
Hardware_Module  →  Data_Module  →  Planner_Module  →  Excu_Module  →  Robot_Module
  硬件抽象层        数据感知层       LLM 规划层         执行流水线层      技能注册层
```

最外层只暴露两个高层工具：

- `vlm_observe` — 观察当前环境
- `robot_act` — 执行机器人动作链

### 各层职责

- **`Hardware_Module`**
  硬件抽象层。屏蔽 sim/Go2 后端差异，提供统一状态入口 `get_state()`。内置 `sim`（IsaacLab 仿真文件 I/O）和 `go2`（Unitree Go2 ROS2）两种后端。

- **`Data_Module`**
  数据感知层。VLM 视觉语义、object_facts 加载/标准化、参数计算、规划上下文构建。

- **`Planner_Module`**
  LLM 规划层。高层 CoT 推理生成任务序列 + 规则覆盖（box-assisted / climbable-obstacle）+ 低层 LLM 工具选择。不导入具体技能，通过依赖注入和回调工作。

- **`Excu_Module`**
  执行流水线层。统一执行、等待反馈、状态校验、命令下发（file/udp/ros 三种后端）、pipeline 编排。不包含任何项目特定代码。

- **`Robot_Module`**
  技能注册层。MCP FastMCP 工具注册、任务分发、具体技能实现。包含项目特定代码（如 Bishe）。

换项目只改 `Robot_Module` + `Hardware_Module/backends/`，**零修改** `Data_Module`、`Planner_Module`、`Excu_Module`。

## 主链路

```text
用户输入
-> run.py -> cli.py
   -> 顶层 LLM 判断：直接回复 / vlm_observe / robot_act

vlm_observe
-> Robot_Module/tools.py 顶层 Vision tool
-> Robot_Module/vision/vlm_observe.py 下层 vision skill: vlm
-> Data_Module/vlm.py (VLMCore)
-> Hardware_Module.get_state()
-> 返回 visual_context + scene_facts + env_state

robot_act
-> Robot_Module/tools.py
   -> Hardware_Module/registry.py 同步 live data 到 object_facts
   -> Data_Module/context.py 组装 planner_context
   -> Planner_Module/planner.py 高层规划（含规则覆盖）
   -> Data_Module/params.py 参数计算
   -> Planner_Module/executor.py 低层执行
   -> Robot_Module/tasks/bishe/*.py 具体技能实现
   -> Excu_Module 统一执行和验收
   -> Hardware_Module.get_state() 提供真实状态
```

## 目录结构

```text
Hardware_Module/
  __init__.py       对外暴露 get_state, list_registered_task_types
  registry.py       后端注册表、TaskBackend 加载、sync 接口
  schema.py         统一状态解析引擎
  backends/
    sim/            IsaacLab 仿真后端
      schema.py     STATE_SCHEMA + TASK_DATA
      data.py       仿真数据获取
      state.py      仿真状态辅助
    go2/            Unitree Go2 ROS2 后端
      schema.py     STATE_SCHEMA + TASK_DATA
      data.py       ROS2 订阅节点

Data_Module/
  __init__.py       对外暴露 VLMCore, load_object_facts, ParameterCalculator 等
  vlm.py            VLMCore 视觉感知
  image_source.py   图像来源（仿真截图/USB摄像头/ROS2 topic）
  vlm_utils.py      视觉工具函数
  facts.py          object_facts 加载与标准化
  params.py         ParameterCalculator 参数计算器
  context.py        build_context() + build_planner_context()
  schema.py         RobotStateSnapshot dataclass
  prompts/          VLM prompt YAML

Planner_Module/
  __init__.py       对外暴露 Planner, TaskExecutor, TaskIntent
  planner.py        高层任务规划（CoT + 规则覆盖）
  executor.py       低层任务执行（LLM 工具选择）
  schema.py         TaskIntent dataclass
  prompts/          高层/低层 prompt YAML

Excu_Module/
  __init__.py       对外暴露 run_pipeline, ExecutionFeedback 等
  pipeline.py       执行流水线编排（plan -> execute -> assess 循环）
  executor.py       串联执行前状态、命令下发、执行后状态、validation
  runtime.py        执行协议（file/udp/ros）、命令下发
  state.py          轮询判定、导航到达判定（读 Hardware_Module.get_state()）
  skill_base.py     SkillBase 技能基类
  skill_registry.py 全局技能注册 + 可插拔钩子
  schema.py         ExecutionFeedback dataclass

Robot_Module/
  __init__.py       对外暴露 register_all, mcp, get_tool_definitions
  tools.py          轻量 MCP 工具注册中心（vlm_observe + robot_act）
  tasks/
    __init__.py     任务分发注册表
    bishe/          Bishe 任务技能（walk, navigation, nav_climb, climb_align, climb, push_box, way_select）
  vision/
    __init__.py
    vlm_observe.py  视觉观察技能

cli.py              TUI 交互界面入口
run.py              项目入口（调用 cli.main）
config/
  object_facts.json 物体事实配置
```

## 动作技能

当前 Bishe 任务固定为 7 个动作技能：

- `walk` — 直行
- `navigation` — 导航到目标点
- `nav_climb` — 导航并攀爬
- `climb_align` — 攀爬对正
- `climb` — 攀爬
- `push_box` — 推箱子
- `way_select` — 路线选择

## 规则覆盖

`Planner_Module/planner.py` 内置两种自动规划规则，当 LLM 规划不合理时强制覆盖：

| 场景 | 触发条件 | 任务链 |
|------|---------|--------|
| box-assisted | 两侧高台阻挡 + 有可移动箱子 + 箱子可做台阶 | `push_box -> climb_align -> climb -> nav_climb` |
| climbable-obstacle | 两侧都有平台 + 一侧可攀爬 + 无箱子 | `way_select -> nav_climb` |
| 单侧障碍 | 只有一侧有平台 | 不触发规则，`navigation` 自带避障 |

## 状态原则

- `Data_Module`（VLM）负责理解场景的大致样子
- `Hardware_Module` 负责提供真实状态和具体数值
- `Planner_Module`（LLM）同时参考两者做规划
- `Excu_Module` 只用真实状态做执行验收
- 给规划大模型的状态输入必须是固定 JSON 结构，而不是零散自然语言

统一状态入口：

```python
from Hardware_Module import get_state
state = get_state()
```

标准状态至少包含 `connected`, `task_type`, `observation`, `runtime`。

## 执行原则

统一执行入口在 `Excu_Module`：

- `runtime.py` — 执行协议（file/udp/ros 三种后端）、命令下发
- `state.py` — 从 `Hardware_Module` 读取实时状态，提供轮询判定和校验
- `executor.py` — 串联"发命令 -> 轮询等待 -> 提前停止或超时 -> 验收"
- `pipeline.py` — 编排 plan -> execute -> assess 循环

关键原则：

- 规划可以在缺少 live 数据时继续
- 动作执行不能假成功
- 没有实时状态时，动作直接失败
- 导航技能统一 0.5 秒轮询，到达阈值 0.13m
- push_box 检测箱子与目标距离，到达阈值 0.08m
- 最终看的是 `validation.verified` 和 `validation.meets_requirements`

## 最短运行方式（仿真）

1. 启动 IsaacLab EnvTest player

```bash
cd /home/xcj/work/IsaacLab/IsaacLabBisShe
python NewTools/envtest_model_use_player.py --scene_id 3
```

2. 启动交互入口

```bash
cd /home/xcj/work/FinalProject
python run.py
```

3. 输入任务，例如：

```text
导航到6,0,0点
```

## 最短运行方式（Go2 真机）

1. 确保 ROS2 环境已加载，Go2 驱动节点已启动
2. 配置环境变量：

```bash
export FINALPROJECT_ROBOT_TYPE=go2
export FINALPROJECT_EXECUTION_BACKEND=ros
```

3. 启动交互入口

```bash
cd /home/xcj/work/FinalProject
python run.py
```

## 当前限制

- 仿真后端（`sim`）通过 `/tmp/*.txt` 文件读写状态和命令，不适用于真实环境
- 真机后端（`Go2`）通过 ROS2 topic 通信，需要 ROS2 环境
- 没有实时状态时，`robot_act` 仍可能完成规划，但执行阶段会失败
- `Excu_Module`、`Planner_Module`、`Data_Module` 不包含任何 Bishe 代码，换项目零修改
- 真机部署时物体位置需要额外感知管线（LiDAR / 深度相机），当前由仿真器直接提供

## 相关文档

- `Sim2Real.md` — Sim 到 Real 部署指南
- `Hardware_Module/README.md`
- `Data_Module/README.md`
- `Planner_Module/README.md`
- `Excu_Module/README.md`
- `Robot_Module/README.md`
