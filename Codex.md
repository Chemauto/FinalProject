# FinalProject 重构说明书

生成时间：2026-04-21  
项目原路径：`/home/xcj/work/FinalProject`  


本文档是重构前的项目全量分析记录。用户计划只保留本文件，然后基于本文件逐步重构项目；其余源码、配置、文档、缓存、Git 元数据和本地文件将迁移到备份目录。

## 1. 项目一句话概述

FinalProject 是一个面向四足机器人 Unitree Go2 / IsaacLab EnvTest 仿真的 LLM Agent 控制系统。用户通过自然语言输入任务，系统通过顶层对话模型判断是否需要观察或行动，然后调用 VLM 感知、LLM 高层规划、规则覆盖、几何参数计算、技能执行、实时状态轮询和执行验收，完成机器人动作链。

最外层只暴露两个高层工具：

- `vlm_observe`：观察当前环境，返回视觉语义、结构化场景事实和硬件状态摘要。
- `robot_act`：根据用户动作意图运行内部规划与执行流水线。

当前架构的核心目标是把项目拆成五层：

```text
Hardware_Module  ->  Data_Module  ->  Planner_Module  ->  Excu_Module  ->  Robot_Module
硬件抽象层           数据感知层        LLM 规划层          执行流水线层       技能注册层
```

实际入口在 `run.py` / `cli.py`，交互界面拆到了 `TUI_Module`。

## 2. 项目当前形态

### 2.1 原始顶层目录

```text
.
├── run.py
├── cli.py
├── README.md
├── 重构.md
├── Sim2Real.md
├── requirements.txt
├── ProxyFix.sh
├── .env
├── .gitignore
├── .vscode/
├── config/
│   └── object_facts.json
├── Data_Module/
├── Planner_Module/
├── Excu_Module/
├── Hardware_Module/
├── Robot_Module/
├── TUI_Module/
├── deploy/
└── docs/plans/
```

项目中还存在 `__pycache__` 和旧路径缓存，如 `Robot_Module/module/.../__pycache__`，说明此前有旧模块被删除或迁移过，但 Python 缓存文件还留在工作区。

### 2.2 入口文件

`run.py` 非常薄，只调用 `cli.main()`：

```python
from cli import main

if __name__ == "__main__":
    main()
```

`cli.py` 负责：

- 清理代理相关环境变量，避免 OpenAI 兼容 API 请求被系统代理影响。
- 加载 `.env`。
- 创建 OpenAI 兼容客户端，默认 base URL 是 DashScope OpenAI 兼容地址。
- 注册机器人工具。
- 启动 Rich TUI 主循环。
- 处理 slash commands。
- 把普通输入交给 `TUI_Module.agent_turn.run_agent_turn()`。

启动方式：

```bash
python run.py
```

### 2.3 依赖

`requirements.txt` 中声明的核心依赖：

```text
openai>=1.0.0
python-dotenv>=1.0.0
pyyaml>=6.0
rich>=13.0.0
numpy>=1.26.0
fastmcp>=0.1.0
pygame>=2.5.0
```

实际代码还可选使用：

- `cv2`：USB 摄像头或 ROS2 图像保存。
- `rclpy`、`nav_msgs`、`std_msgs`、`geometry_msgs`、`sensor_msgs`、`cv_bridge`：Go2 ROS2 真机或 Sim2Sim 桥接。

注意：`requirements.txt` 注释里写“已移除 ROS2 依赖”，但当前代码仍保留可选 ROS2 路径。重构时要统一说明：ROS2 是可选后端依赖，不应写成已移除。

### 2.4 敏感配置

原项目有 `.env`，其中包含 `Test_API_KEY`。本文档不记录具体密钥值。重构时建议：

- 不把 `.env` 纳入新仓库。
- 改用 `.env.example`。
- 如果原密钥是真实可用密钥，建议轮换。

## 3. 运行时配置

当前项目使用大量环境变量控制模型、后端、状态轮询和命令下发。

| 变量 | 默认值 | 作用 |
|---|---|---|
| `Test_API_KEY` | 无 | OpenAI 兼容 API Key，`cli.py`、`Data_Module.vlm`、`Robot_Module.pipeline_factory` 会读取 |
| `FINALPROJECT_BASE_URL` | `https://dashscope.aliyuncs.com/compatible-mode/v1` | OpenAI 兼容 API base URL |
| `FINALPROJECT_AGENT_MODEL` | `qwen3.6-plus` 或 `qwen3.5-plus` | 顶层 Agent 和 Planner 模型名，不同文件默认值不一致 |
| `FINALPROJECT_ROBOT_TYPE` | `sim` | 硬件后端，支持 `sim` / `go2` |
| `FINALPROJECT_ACTION_TASK` | `bishe` | 动作技能任务集 |
| `FINALPROJECT_TASK` | `bishe` | 兼容旧变量，被 `FINALPROJECT_ACTION_TASK` 兜底使用 |
| `FINALPROJECT_OBJECT_FACTS_PATH` | `config/object_facts.json` | 物体事实文件 |
| `FINALPROJECT_VLM_IMAGE_PATH` | `/tmp/envtest_front_camera.png` | VLM 优先读取的仿真图片 |
| `FINALPROJECT_VLM_ROS2_IMAGE_TOPIC` | `/go2/wrist_camera/image_raw` | Go2 摄像头 ROS2 topic |
| `FINALPROJECT_ENVTEST_ROOT` | `/home/xcj/work/IsaacLab/IsaacLabBisShe` | 仿真 EnvTest 根目录 |
| `FINALPROJECT_NAV_BACKEND` | `file` | 命令下发后端：`file` / `udp` / `ros` |
| `FINALPROJECT_NAV_UDP_HOST` | `127.0.0.1` | UDP 命令目标主机 |
| `FINALPROJECT_NAV_UDP_PORT` | `5566` | UDP 命令目标端口 |
| `FINALPROJECT_MODEL_USE_FILE` | `/tmp/model_use.txt` | file 后端 model_use 文件 |
| `FINALPROJECT_VELOCITY_FILE` | `/tmp/envtest_velocity_command.txt` | file 后端速度命令文件 |
| `FINALPROJECT_GOAL_FILE` | `/tmp/envtest_goal_command.txt` | file 后端目标命令文件 |
| `FINALPROJECT_START_FILE` | `/tmp/envtest_start.txt` | file 后端启动/停止文件 |
| `FINALPROJECT_RESET_FILE` | `/tmp/envtest_reset.txt` | file 后端 reset 文件 |
| `FINALPROJECT_NAV_COMMAND_SETTLE_SEC` | `0.15` | 命令下发后的等待时间 |
| `FINALPROJECT_NAV_STOP_SETTLE_SEC` | `0.1` | stop 后等待时间 |
| `FINALPROJECT_NAV_AUTO_IDLE` | `true` | stop 后是否写回 idle |
| `FINALPROJECT_WAY_SELECT_POLICY` | `walk` | `way_select` 用横移还是导航 |
| `FINALPROJECT_STATUS_POLL_SEC` | `0.5` | 导航状态轮询间隔 |
| `FINALPROJECT_NAV_ARRIVAL_TOL_M` | `0.13` | 导航到达阈值 |
| `FINALPROJECT_NAV_STABLE_POSITION_DELTA_M` | `0.08` | 导航稳定位置变化阈值 |
| `FINALPROJECT_NAV_REQUIRED_STABLE_POLLS` | `3` | 导航到达所需稳定轮询次数 |
| `FINALPROJECT_STATUS_STALE_SEC` | `2.0` | 状态过期阈值 |
| `FINALPROJECT_STATUS_READY_TIMEOUT_SEC` | `2.0` | 等待实时状态就绪超时 |
| `FINALPROJECT_DISPLACEMENT_POLL_SEC` | `0.1` | 位移状态轮询间隔 |
| `FINALPROJECT_DISPLACEMENT_TOL_M` | `0.05` | 位移到达容差 |
| `FINALPROJECT_DISPLACEMENT_STABLE_POLLS` | `1` | 位移达标稳定轮询次数 |
| `FINALPROJECT_DIRECTION_GRACE_SEC` | `1.0` | 位移方向检查宽限期 |
| `FINALPROJECT_WALK_SPEED_MPS` | `0.6` | walk 默认速度 |
| `FINALPROJECT_CLIMB_SPEED_MPS` | `0.4` | climb 默认速度 |
| `FINALPROJECT_CLIMB_DURATION_SEC` | `4.0` | climb 默认执行时长 |

重要不一致：

- `cli.py` 和 `TUI_Module.commands` 默认模型是 `qwen3.6-plus`。
- `Planner_Module.planner` 默认模型是 `qwen3.5-plus`。
- `README.md`、`重构.md`、代码中的默认值并非完全一致。

重构时建议集中到一个 `settings.py` 或 `config.py`，所有模块通过同一配置对象读取。

## 4. 端到端主链路

### 4.1 对话和工具选择

```text
用户输入
-> run.py
-> cli.py
-> TUI_Module.agent_turn.run_agent_turn()
-> 顶层 LLM 根据 AGENT_SYSTEM_PROMPT 判断：
   1. 直接回复
   2. 调用 vlm_observe
   3. 调用 robot_act
```

顶层系统提示词要求：

- 涉及运动时先调用 `vlm_observe`。
- 执行动作时调用 `robot_act`。
- `robot_act` 应带上 `agent_thought`。
- 如果已有 `vlm_observe` 结果，`robot_act` 优先复用 `visual_context` 和 `scene_facts`。

### 4.2 `vlm_observe` 链路

```text
vlm_observe
-> Robot_Module.tools.vlm_observe()
-> Robot_Module.vision.vlm_observe.vlm()
-> Data_Module.vlm.VLMCore.describe_structured()
-> Data_Module.image_source.ImageSource.get_image()
-> OpenAI 兼容 VLM API
-> VLMCore.build_scene_facts()
-> Hardware_Module.get_state()
-> 返回 visual_context + scene_facts + env_state
```

图像来源优先级：

1. 显式传入 `image_path`。
2. `/tmp/envtest_front_camera.png` 或 `FINALPROJECT_VLM_IMAGE_PATH`。
3. Go2 ROS2 图片 topic，仅 `FINALPROJECT_ROBOT_TYPE=go2` 时尝试。
4. USB 摄像头。
5. `Data_Module/assets/2.png` 默认图片。

### 4.3 `robot_act` 链路

```text
robot_act
-> Robot_Module.tools._run_robot_act_pipeline()
-> Robot_Module.pipeline_factory.run_robot_act_pipeline()
-> Hardware_Module.registry.sync_object_facts_from_live_data()
-> Data_Module.facts.load_object_facts()
-> Data_Module.vlm.VLMCore.merge_scene_facts()
-> Data_Module.context.build_planner_context()
-> Planner_Module.planner.Planner.plan_tasks()
-> Planner_Module.rule_overrides.apply_rule_overrides()
-> Data_Module.params.ParameterCalculator.annotate_tasks()
-> Excu_Module.pipeline.run_pipeline()
-> Robot_Module.tasks.bishe.<skill>()
-> Excu_Module.executor.wait_skill_feedback()
-> Excu_Module.runtime.apply_envtest_command()
-> Excu_Module.state.wait_for_*_completion()
-> Hardware_Module.get_state()
-> 返回执行结果和 validation
```

执行期间如果有 TUI `on_event` 回调，技能会在后台线程运行，主线程每秒读取状态并显示实时坐标。

## 5. 模块详解

## 5.1 `Hardware_Module`：硬件抽象层

职责：屏蔽 sim / Go2 后端差异，提供统一状态入口。

核心接口：

```python
from Hardware_Module import get_state

state = get_state()
state = get_state(task_type="go2")
```

对外状态至少包含：

```python
{
    "connected": bool,
    "task_type": "sim" | "go2",
    "robot_type": "sim" | "go2",
    "observation": {...},
    "runtime": {...},
}
```

主要文件：

| 文件 | 职责 |
|---|---|
| `Hardware_Module/__init__.py` | 延迟暴露 `get_state()`、`list_registered_task_types()` |
| `Hardware_Module/registry.py` | 后端注册、`TaskBackend`、加载后端、同步 object_facts |
| `Hardware_Module/schema.py` | 根据 `STATE_SCHEMA` 解析统一状态 |
| `Hardware_Module/backends/sim/schema.py` | sim 后端状态映射与 `TASK_DATA` |
| `Hardware_Module/backends/sim/data.py` | 读取 EnvTest `/tmp/envtest_*.json` 和控制文件，构建 live snapshot |
| `Hardware_Module/backends/go2/schema.py` | Go2 后端状态映射和 topic 名 |
| `Hardware_Module/backends/go2/data.py` | ROS2 订阅 `/go2/odom`、`/go2/skill_status`、`/go2/scene_objects` |

后端注册位于 `registry.py`：

```python
register_task("sim", "Hardware_Module.backends.sim.schema", ...)
register_task("go2", "Hardware_Module.backends.go2.schema", ...)
```

状态解析机制：

- 每个后端提供 `STATE_SCHEMA`。
- `schema.py` 的 `_parse_schema()` 递归解析。
- 字段可用 `source` 从原始数据取值。
- 可用 `default` 给缺省值。
- 可用 `literal` 给固定值。

sim 后端原始数据主要来自：

- `/tmp/model_use.txt`
- `/tmp/envtest_velocity_command.txt`
- `/tmp/envtest_goal_command.txt`
- `/tmp/envtest_live_status.json`
- `/tmp/envtest_start.txt`
- `/tmp/envtest_reset.txt`
- IsaacLab EnvTest 进程和场景布局模块。

Go2 后端 ROS2 输入：

- `/go2/odom`：`nav_msgs/Odometry`，读取 `pose.pose.position`。
- `/go2/skill_status`：`std_msgs/String` JSON。
- `/go2/scene_objects`：`std_msgs/String` JSON array。

当前设计优点：

- 上层基本只依赖 `get_state()`，不直接读具体文件或 ROS2 topic。
- 后端是注册式的。
- Go2 依赖是可选导入，缺失 ROS2 时不会直接导致项目不可启动。

当前风险：

- `TaskBackend` 名字更像任务后端，但实际代表硬件/机器人状态后端，命名容易混淆。
- `go2/data.py` 中 `_Go2StateNode` 不是继承 `Node`，而是持有 `self.node`，可以工作但抽象不一致。
- `sync_object_facts_from_live_data()` 写入 JSON，是硬件层和数据层之间的副作用，重构时建议改为返回结构化 snapshot，由上层决定是否落盘。

## 5.2 `Data_Module`：数据感知层

职责：组合 VLM 视觉语义、结构化物体事实、几何参数计算，为 planner 提供结构化上下文。

主要文件：

| 文件 | 职责 |
|---|---|
| `Data_Module/vlm.py` | `VLMCore`，读取图片、调用 VLM、解析结构化 JSON、生成 scene_facts |
| `Data_Module/image_source.py` | 图片来源管理 |
| `Data_Module/vlm_utils.py` | API key、图片转 data URL、响应文本提取 |
| `Data_Module/facts.py` | 加载和标准化 `object_facts.json` |
| `Data_Module/params.py` | `ParameterCalculator` 几何参数补全 |
| `Data_Module/context.py` | `build_context()`、`build_planner_context()` |
| `Data_Module/schema.py` | `RobotStateSnapshot` dataclass |
| `Data_Module/prompts/VlmPrompt.yaml` | VLM 提示词 |

`object_facts` 标准结构：

```json
{
  "navigation_goal": [5.0, 0.0, 0.0],
  "robot_pose": [1.42, 0.109, 0.36],
  "constraints": {
    "max_climb_height_m": 0.3,
    "push_only_on_ground": true,
    "climb_requires_adjacency": true
  },
  "objects": [
    {
      "id": "left_low_obstacle",
      "type": "platform",
      "center": [3.0, 0.75, 0.15],
      "size": [2.0, 1.5, 0.3],
      "movable": false
    }
  ]
}
```

`facts.normalize_object_facts()` 当前只保留：

- `navigation_goal`
- `robot_pose`
- `constraints`
- `objects`

它会丢弃 `scene_objects`、`runtime_state`、`runtime_objects`、`scene_layout_objects` 等扩展字段。这在当前代码中可接受，因为 planner 只用标准字段；但重构时如果需要保留调试信息，应拆分为 `raw_payload` 和 `normalized_payload`。

`VLMCore` 输出视觉 JSON 必须包含：

```json
{
  "ground": "...",
  "left_side": "...",
  "right_side": "...",
  "front_area": "...",
  "obstacles": [],
  "suspected_height_diff": true,
  "uncertainties": []
}
```

`VLMCore.build_scene_facts()` 会把视觉字段转换为：

- `summary`
- `terrain_features`
- `interactive_objects`
- `route_options`
- `constraints`
- `uncertainties`

`VLMCore.merge_scene_facts()` 优先使用 `object_facts` 派生的结构化事实，再把 VLM 的不确定项和视觉摘要合并进去。

`ParameterCalculator` 当前只补这些参数：

- `navigation` / `nav_climb`：从 `object_facts.navigation_goal` 补 `goal` 和 `target`。
- `push_box`：选择可移动箱子和目标平台，补 `target_position=auto`、`box_height`。
- `climb_align`：补 `stage`、`target`、`goal_command`。
- `climb`：补 `stage`、`target`。
- `walk` / `way_select`：直接透传 LLM 参数。

当前设计优点：

- VLM 和结构化几何分离。
- 几何参数计算没有放在 planner 里。
- `Planner` 通过依赖注入使用 `ParameterCalculator`。

当前风险：

- `VLMCore.__init__` 会直接读取 API key 并创建客户端，测试时需要 mock。
- `Data_Module/image_source.py` 里 `Video_Port="video3"`，摄像头端口和真实设备绑定偏硬。
- VLM prompt 明确“不要判断能否通行”，但 `build_scene_facts()` 会基于文本做 traversable 推断，这是可接受的后处理，但需要在新架构中命名清楚。

## 5.3 `Planner_Module`：LLM 规划层

职责：把用户动作意图、视觉上下文、结构化场景事实、物体事实、机器人状态转换为任务序列；在已知场景中用规则覆盖修正 LLM 规划。

主要文件：

| 文件 | 职责 |
|---|---|
| `Planner_Module/planner.py` | `Planner` 主类，调用 LLM，解析响应，应用规则覆盖 |
| `Planner_Module/parsing.py` | JSON 去 code fence、解析、标准化任务 |
| `Planner_Module/rule_overrides.py` | box-assisted 和 climbable-obstacle 规则覆盖 |
| `Planner_Module/schema.py` | `TaskIntent` dataclass |
| `Planner_Module/prompts/highlevel_prompt.yaml` | 高层规划提示词 |

`Planner.plan_tasks()` 输入：

```python
tasks, meta = planner.plan_tasks(
    user_input,
    agent_thought,
    tools,
    visual_context,
    scene_facts=planner_context,
    object_facts=object_facts,
    robot_state=robot_state,
    on_event=on_event,
)
```

规划模型输出 JSON 期望格式：

```json
{
  "scene_assessment": "对当前任务和环境的简要判断",
  "tasks": [
    {
      "step": 1,
      "task": "子任务描述",
      "function": "navigation",
      "params": {"goal": "5,0,0", "target": "目标点"},
      "reason": "规划依据"
    }
  ],
  "summary": "整体任务概述"
}
```

`parse_plan_response()` 会标准化任务为：

```python
{
    "step": ...,
    "task": ...,
    "type": "未分类",
    "function": "待LLM决定",
    "params": {},
    "reason": "未提供规划依据",
}
```

规则覆盖一：`box-assisted`

触发条件：

- 用户请求是导航/前往类任务。
- 有可移动箱子。
- 有平台。
- 箱子高度 `0 < box_height <= climb_limit`。
- 平台高度 `platform_height > climb_limit`。
- 平台剩余高差 `platform_height - box_height <= climb_limit`。
- 如果有 route_options，则左右都 blocked。

覆盖任务链：

```text
push_box -> climb_align -> climb -> nav_climb
```

规则覆盖二：`climbable-obstacle`

触发条件：

- 用户请求是导航/前往类任务。
- 存在左右两侧平台。
- 至少一个平台高度 `0 < height <= max_climb_height_m`。
- 无需箱子辅助。

覆盖任务链：

```text
way_select -> nav_climb
```

当前设计优点：

- Planner 不直接导入 Data、Excu、Robot、Hardware，上下文由上层注入。
- 已知几何场景不完全依赖 LLM。
- LLM 失败时有规则 fallback。

当前风险：

- Prompt 要求模型自己计算距离和方向，但部分精确参数又由 `ParameterCalculator` 覆盖，职责边界需要重新写清。
- `robot_state` 被传入 prompt，但 `build_planner_context()` 里也有 `robot_state` 字段，命名重复。
- 规则覆盖里对“左右 blocked”的判断依赖 scene_facts 的 route_options，object_facts 不足时规则可能不触发。

## 5.4 `Excu_Module`：执行流水线层

职责：通用执行基础设施，不应该包含 Bishe 项目专用逻辑。它负责计划执行编排、命令下发、状态轮询、技能停止和结果验收。

主要文件：

| 文件 | 职责 |
|---|---|
| `Excu_Module/pipeline.py` | plan -> annotate -> execute -> assess 编排，支持 on_event |
| `Excu_Module/executor.py` | 单技能执行编排，发命令、等待反馈、停技能、构建 validation |
| `Excu_Module/runtime.py` | file / udp / ros 三种命令下发协议 |
| `Excu_Module/state.py` | 统一读取硬件状态，导航/位移轮询完成判定 |
| `Excu_Module/skill_base.py` | `SkillBase` 抽象基类 |
| `Excu_Module/skill_registry.py` | 技能实例注册和导航 model_use 码注册 |
| `Excu_Module/schema.py` | `ExecutionFeedback` dataclass |

`run_pipeline()` 步骤：

1. 触发 `plan_start`。
2. 调 `planner.plan_tasks()`。
3. 调 `planner.annotate_tasks()`。
4. 逐个任务执行：
   - 触发 `step_start`。
   - 调 `execute_tool_fn(function_name, params)`。
   - 调 `assess_result()` 判定成功。
   - 触发 `step_done`。
5. 失败且允许重规划时触发 `replan`。
6. 触发 `pipeline_done`。

当前 `max_replans=0`，实际上没有自动重规划。

`assess_result()` 成功判定优先看：

- `validation.verified is True`
- `validation.meets_requirements is True`

如果没有 validation，则回退到：

- `result.success == True`
- `feedback.signal == "SUCCESS"`

`wait_skill_feedback()` 执行逻辑：

1. 读取 runtime config。
2. 等待实时状态就绪。
3. 下发 `model_use`、`velocity`、`goal`。
4. 设置 `start=True`。
5. 如果 model_use 属于导航类，调用 `wait_for_navigation_completion()`。
6. 如果有 velocity 和 distance，调用 `wait_for_displacement_completion()`。
7. 否则按固定时间等待。
8. 调 `stop_envtest_skill()`。
9. 返回 `execution_feedback` 和 `validation`。

命令后端：

- `file`：写 `/tmp/model_use.txt`、速度、目标、start、reset 文件。
- `udp`：发 `model_use=...; velocity=...; goal=...; start=...` 文本。
- `ros`：发布 `/go2/skill_command`、`/go2/cmd_vel`、`/go2/goal_pose`。

导航完成判定：

- 默认 0.5 秒轮询。
- 到达阈值 0.13m。
- 姿态变化低于 0.08m 视为稳定。
- 默认需要 3 次稳定轮询。
- 状态超过 2 秒未更新视为 stale。
- 如果到达前 `start=False` 或 model_use 切出导航类，则失败。

位移完成判定：

- 默认 0.1 秒轮询。
- 位移容差 0.05m。
- 默认 1.0 秒方向检查宽限期。
- 宽限期内不检查反向，避免技能切换时惯性误判。
- 宽限期后如果进度明显为负，判定方向错误。
- 如果技能已启动后 `start=False` 且未完成，判定提前停止。

当前设计优点：

- 不把动作技能实现和执行验收混在一起。
- 明确拒绝“没有实时状态也假成功”。
- 有统一 validation 数据结构。

当前风险：

- `Excu_Module.skill_registry` README 描述中提到许多 hook，但当前代码实际只有 skill registry 和 navigation model_use 码，没有 README 中描述的 rule planner/context hook 等扩展函数。
- `pipeline.py` 注释说“每 2s 轮询”，常量实际是 1.0 秒。
- `wait_skill_feedback()` 兜底固定时间分支使用 `actual_wait_sec = max(timeout_sec, execution_time_sec)`，可能比预期等待更久。

## 5.5 `Robot_Module`：技能注册层和项目技能实现层

职责：注册 MCP 工具、管理任务技能集合、实现 Bishe 任务技能、装配 `robot_act` pipeline。

主要文件：

| 文件 | 职责 |
|---|---|
| `Robot_Module/tools.py` | FastMCP 实例、工具注册、顶层 `vlm_observe` / `robot_act` |
| `Robot_Module/tool_runtime.py` | 执行注册工具、同步/异步兼容、标准化返回 |
| `Robot_Module/pipeline_factory.py` | `robot_act` 胶水装配 |
| `Robot_Module/tasks/__init__.py` | 任务类型到技能模块的注册表 |
| `Robot_Module/tasks/bishe/*.py` | 7 个 Bishe 技能 |
| `Robot_Module/vision/vlm_observe.py` | 底层 VLM skill |
| `Robot_Module/tests/test_agent_tools.py` | 旧测试，当前疑似失效 |

顶层工具：

- `vlm_observe(image_path="")`
- `robot_act(user_intent, agent_thought="", observation_context="", scene_facts_json="")`

动作技能表：

| 技能 | 功能 | model_use | 命令类型 | 关键参数 |
|---|---|---:|---|---|
| `walk` | 沿前/后/左/右行走固定距离 | 1 | velocity | `route_side`, `distance`, `target`, `speech` |
| `navigation` | 导航到目标点 | 4 | goal | `goal`, `target`, `speech` |
| `nav_climb` | 导航到目标并攀爬 | 5 | goal | `goal`, `target`, `speech` |
| `climb_align` | 攀爬前导航对正 | 4 | goal | `stage`, `target`, `goal_command`, `speech` |
| `climb` | 攀爬指定高度 | 2 | velocity | `height`, `stage`, `target`, `speech` |
| `push_box` | 推箱子到目标位置 | 3 | goal | `box_height`, `target_position`, `speech` |
| `way_select` | 选择左/右路线 | 1 或 4 | velocity 或 goal | `direction`, `lateral_distance`, `target`, `speech` |

`model_use` 约定：

```text
0 = idle
1 = walk / way_select(walk policy)
2 = climb
3 = push_box
4 = navigation / climb_align / way_select(navigation policy)
5 = nav_climb
```

技能注册链路：

```text
Robot_Module.tools.register_all()
-> Robot_Module.tasks.register_tools(mcp)
   -> Robot_Module.tasks.bishe.walk.register_tools(mcp)
   -> navigation
   -> nav_climb
   -> climb_align
   -> climb
   -> push_box
   -> way_select
-> Robot_Module.vision.vlm_observe.register_tools(mcp)
-> tools.py 注册顶层 vlm_observe
-> tools.py 注册顶层 robot_act
-> snapshot_tool_definitions(mcp)
```

`tool_runtime.normalize_tool_result()` 标准化结果：

```python
{
    "success": bool,
    "feedback": {...},
    "result": parsed_result,
}
```

如果存在 validation：

- `verified=False` 或 `meets_requirements=False` 会让 `success=False`。
- `verified=True` 且 `meets_requirements=True` 才确认成功。

当前设计优点：

- 顶层 Agent 只看两个工具，不直接暴露所有动作技能。
- 动作技能本身很薄，多数执行细节交给 `Excu_Module`。
- 所有技能 MCP 参数都有默认值，减少 planner 漏传参数导致 TypeError 的概率。

当前风险：

- `Robot_Module/tests/test_agent_tools.py` 仍导入 `Robot_Module.agent_tools`、`Robot_Module.skill`、`Robot_Module.module.Vision.vlm` 等旧路径，而当前源码没有这些模块，测试大概率已经失效。
- 技能函数返回 JSON 字符串，内部再反解析，重构时建议统一返回 dict，只在 MCP 边界序列化。
- `climb_align.execute()` 在无法解析 `goal_command` 时把 `target` 字符串传给 `execute_goal_navigation_skill()`，后者要求显式目标列表，可能触发 ValueError。

## 5.6 `TUI_Module`：Rich 交互层

职责：管理会话状态、slash commands、Rich 渲染、顶层 Agent 回合和工具分发。

主要文件：

| 文件 | 职责 |
|---|---|
| `TUI_Module/session.py` | 系统提示词、`AgentRuntime`、`InteractiveSessionState` |
| `TUI_Module/commands.py` | `/help`、`/model`、`/tools`、`/status`、`/reset`、`/vlm`、`/quit` |
| `TUI_Module/renderers.py` | Rich panel/table/status line/pipeline event renderers |
| `TUI_Module/agent_turn.py` | 顶层 LLM 回合、tool calls 循环、tool result 写回 messages |

slash commands：

| 命令 | 作用 |
|---|---|
| `/help` | 显示命令 |
| `/model` | 查看模型 |
| `/model <name>` | 切换模型并写入 `.env` |
| `/tools` | 显示顶层工具、视觉技能、动作技能 |
| `/status` | 显示机器人实时状态摘要 |
| `/reset` | 重置会话和 runtime |
| `/vlm` | 开关 VLM |
| `/quit` | 退出 |

Pipeline 事件：

| 事件 | 渲染作用 |
|---|---|
| `plan_start` | 用户动作意图面板 |
| `llm_planning` | Planning 提示 |
| `plan_done` | 任务计划表 |
| `plan_thinking` | 推理面板 |
| `execute_start` | Executing 提示 |
| `step_start` | 当前步骤、函数、参数、理由 |
| `step_progress` | 实时坐标 |
| `step_done` | 单步 OK / FAIL |
| `rule_override` | 规则修正提示 |
| `replan` | 重规划提示 |
| `pipeline_done` | 总结 |
| `pipeline_error` | 错误面板 |

当前设计优点：

- CLI 主文件较薄。
- `on_event` 让 pipeline 不直接依赖 Rich。
- 工具调用结果会写回 `session.messages`，支持多轮上下文。

当前风险：

- 顶层系统提示词强制运动前先 VLM；如果 VLM 关闭或 API 不可用，会影响本可直接执行的动作。
- `run_agent_turn()` 最多循环 4 次，遇到 `robot_act` 后直接 break，可能没有最终自然语言总结。
- `/model` 会写 `.env`，这是 UI 命令产生文件副作用；重构时可保留但应明确。

## 5.7 `deploy`：Go2 Sim2Sim 桥接

`deploy/go2_skill_bridge.py` 用于：

```text
FinalProject Go2 ROS2 后端 <-> deploy/go2_skill_bridge.py <-> IsaacLabBisShe UDP socket
```

订阅：

- `/go2/skill_command`
- `/go2/cmd_vel`
- `/go2/goal_pose`

发布：

- `/go2/odom`
- `/go2/skill_status`
- `/go2/scene_objects`

读取：

- `/tmp/envtest_live_status.json`

桥接转换：

- `skill_command_to_udp_text()` 把 JSON 转为 `model_use=4; goal=...; start=1`。
- `twist_to_velocity()` 把 `Twist` 转为 `[vx, vy, vz]`。
- `pose_to_goal()` 把 `PoseStamped` 转为 `[x, y, z, yaw]`。
- `build_skill_status()` 把 EnvTest snapshot 转成 `/go2/skill_status` JSON。
- `build_scene_objects()` 把 platform/box 转成 scene objects array。

启动顺序见原 `deploy/README.md` 和 `Sim2Real.md`。

## 6. 数据契约

### 6.1 统一硬件状态 `get_state()`

示例结构：

```json
{
  "connected": true,
  "task_type": "sim",
  "robot_type": "sim",
  "observation": {
    "agent_position": [1.42, 0.109, 0.36],
    "environment": {
      "scene_id": 1,
      "obstacles": [],
      "goal": [5.0, 0.0, 0.0],
      "envtest_alignment": {
        "platform_1": {},
        "platform_2": {},
        "box": null
      }
    },
    "action_result": {
      "skill": "idle",
      "model_use": 0,
      "vel_command": [0.0, 0.0, 0.0],
      "start": false
    }
  },
  "runtime": {
    "timestamp": 1776757558.7668746,
    "skill": "idle",
    "model_use": 0,
    "goal": null,
    "start": false,
    "scene_objects": [],
    "envtest_alignment": {}
  }
}
```

### 6.2 Planner task

```json
{
  "step": 1,
  "task": "导航到目标点",
  "type": "导航",
  "function": "navigation",
  "params": {
    "goal": "5,0,0",
    "target": "目标点",
    "speech": "开始导航"
  },
  "reason": "用户要求到达明确目标坐标"
}
```

### 6.3 技能返回结构

技能最终应返回：

```json
{
  "skill": "navigation",
  "action_id": "navigation-xxxxxxxx",
  "execution_feedback": {
    "signal": "SUCCESS",
    "skill": "navigation",
    "message": "导航已到达目标附近",
    "validation": {
      "verified": true,
      "meets_requirements": true,
      "source": "comm_state",
      "summary": "导航已到达目标附近"
    }
  },
  "execution_result": {
    "mode": "comm_state_navigation",
    "backend": "file",
    "task_type": "sim"
  },
  "control_command": {
    "model_use": 4,
    "goal": [5.0, 0.0, 0.0],
    "estimated_execution_time_sec": 10.0
  },
  "backend": "file",
  "status": "success"
}
```

### 6.4 `robot_act` 返回结构

```json
{
  "status": "success",
  "summary": {
    "total_tasks": 1,
    "success_count": 1,
    "failure_count": 0
  },
  "results": [],
  "session_snapshot": {
    "sync": {
      "scene_id": 1,
      "model_use": 0,
      "objects_count": 1
    }
  }
}
```

### 6.5 ROS2 topic 契约

真机到代码：

| Topic | 类型 | 内容 |
|---|---|---|
| `/go2/odom` | `nav_msgs/Odometry` | 机器人位置 x/y/z |
| `/go2/skill_status` | `std_msgs/String` JSON | skill、model_use、start、goal、vel_command、scene_id、envtest_alignment |
| `/go2/scene_objects` | `std_msgs/String` JSON array | objects: id/type/center/size/movable |
| `/go2/wrist_camera/image_raw` | `sensor_msgs/Image` | VLM 图像，bgr8 |
| `/go2/state` | 自定义/预留 | 当前未实际订阅 |

代码到真机：

| Topic | 类型 | 内容 |
|---|---|---|
| `/go2/skill_command` | `std_msgs/String` JSON | model_use/start/goal/velocity/reset |
| `/go2/cmd_vel` | `geometry_msgs/Twist` | 速度 |
| `/go2/goal_pose` | `geometry_msgs/PoseStamped` | 导航目标 |

## 7. 已知问题和重构风险

1. 测试已落后于源码。

   `Robot_Module/tests/test_agent_tools.py` 导入旧模块路径：`Robot_Module.agent_tools`、`Robot_Module.skill`、`Robot_Module.module.Vision.vlm`。当前项目没有这些源码文件，只有部分旧 `__pycache__`，说明测试需要重写。

2. 文档与代码存在不一致。

   README 中描述的 `skill_registry` hook 比当前代码多；requirements 中说 ROS2 已移除，但代码仍可选使用 ROS2；模型默认值在文件之间不一致。

3. 配置分散。

   环境变量散落在 CLI、Data、Planner、Excu、Hardware、Robot 中。重构时应先建立统一配置层。

4. `object_facts.json` 同时承担静态真值、运行时状态、同步缓存。

   当前 `facts.normalize_object_facts()` 又只保留部分字段，容易让“写入了但读取时丢弃”的行为变隐蔽。建议拆成：

   - `SceneFacts`
   - `ObjectFacts`
   - `RuntimeSnapshot`
   - `PlannerContext`

5. 硬件同步函数会直接写文件。

   `Hardware_Module.registry.sync_object_facts_from_live_data()` 调后端函数写 JSON。更干净的方式是硬件层只返回 live snapshot，由数据层决定如何 merge/cache。

6. 顶层 Agent 过度依赖 VLM。

   系统提示要求运动前必须观察。如果只是明确坐标导航，VLM 不一定必要。新架构可让策略变成“需要环境判断时观察”，而不是硬性所有运动都观察。

7. 技能返回在 dict / JSON string 间反复转换。

   MCP 边界可以序列化，但内部应统一 dict，避免重复 `json.loads` 和错误吞噬。

8. `climb_align` 参数兜底可能失败。

   它无法解析 `goal_command` 时把 `target` 字符串传给要求显式坐标的 `execute_goal_navigation_skill()`。重构时应强制 `goal_command` 必须是坐标，或由参数计算器保证补齐。

9. `.env` 中有真实 API key 风险。

   新项目应只保留 `.env.example`，真实密钥不入库。

10. 项目路径中有硬编码旧用户名。

   README、deploy 文档和默认 `FINALPROJECT_ENVTEST_ROOT` 使用 `/home/xcj/work/...`，当前工作区是 `/home/robot/work/FinalProject`。重构时应使用相对路径或环境变量。

## 8. 建议重构路线

建议不要一次性复刻全部模块。可以按“可运行骨架 -> 数据契约 -> 单技能闭环 -> TUI -> Go2”的顺序重构。

### 阶段 0：建立新项目骨架

目标：

- 只创建最少文件。
- 定义配置层、数据模型、日志、测试骨架。
- 不接入 LLM，不接入 ROS2。

建议结构：

```text
finalproject/
  __init__.py
  config.py
  models.py
  hardware/
  perception/
  planning/
  execution/
  skills/
  tui/
  app.py
tests/
README.md
.env.example
pyproject.toml
```

第一批数据模型：

- `RobotPose`
- `SceneObject`
- `RuntimeStatus`
- `HardwareState`
- `ObjectFacts`
- `PlannerContext`
- `TaskStep`
- `SkillCommand`
- `SkillResult`
- `ValidationResult`

### 阶段 1：先重建硬件状态层

先实现：

- `HardwareBackend` 协议。
- `SimBackend`，只读 `/tmp/envtest_live_status.json`。
- `get_state()`。
- 单元测试覆盖状态解析。

不要一开始就做：

- 进程扫描。
- IsaacLab 动态 import。
- Go2 ROS2。

### 阶段 2：重建执行层

先实现 file 后端：

- 写 model_use、velocity、goal、start 文件。
- stop/idle。
- `wait_for_navigation_completion()`。
- `wait_for_displacement_completion()`。

用 fake backend 测试：

- 无实时状态 -> 执行失败。
- 导航到达 -> 成功。
- 导航提前停止 -> 失败。
- 位移方向错误 -> 失败。
- direction grace -> 不误判。

### 阶段 3：重建技能层

先实现三个技能：

1. `navigation`
2. `walk`
3. `climb`

确认返回统一 dict：

```python
SkillResult(
    status="success" | "failure",
    feedback=ExecutionFeedback(...),
    command=SkillCommand(...),
    validation=ValidationResult(...),
)
```

再加入：

- `nav_climb`
- `climb_align`
- `push_box`
- `way_select`

### 阶段 4：重建 Data/Planner

先不要接真实 LLM。用 rule planner 跑通：

- 明确坐标 -> `navigation`
- 短距离 -> `walk`
- 可攀爬平台 -> `way_select -> nav_climb`
- 箱子辅助登台 -> `push_box -> climb_align -> climb -> nav_climb`

然后再接 LLM planner：

- Prompt 输入必须是固定 JSON。
- LLM 输出必须 JSON schema 校验。
- LLM 结果经过 rule override。
- 几何参数统一由参数计算器补齐。

### 阶段 5：重建 VLM

把 VLM 作为可选 perception provider：

- 没 API key 时不阻塞纯动作。
- 可用 fake image provider 测试。
- VLM 只输出视觉事实。
- 结构化物体事实优先级高于 VLM 猜测。

### 阶段 6：重建 TUI

TUI 只调用应用服务：

```python
AgentService.observe()
AgentService.act(user_input)
AgentService.chat(user_input)
```

Rich 渲染不进入执行层。Pipeline 继续用 event callback。

### 阶段 7：接 Go2 / Sim2Sim

最后接：

- ROS2 Go2 backend。
- ROS2 command publisher。
- `deploy/go2_skill_bridge.py`。
- Sim2Real 文档。

Go2 相关依赖做 extras：

```text
pip install finalproject[go2]
```

## 9. 推荐的新模块边界

### 9.1 配置层

集中读取环境变量：

```python
@dataclass(frozen=True)
class Settings:
    api_key: str | None
    base_url: str
    agent_model: str
    planner_model: str
    robot_type: str
    action_task: str
    object_facts_path: Path
    nav_backend: str
```

所有模块接收 `settings` 或具体字段，不再到处 `os.getenv()`。

### 9.2 硬件层接口

```python
class HardwareBackend(Protocol):
    name: str
    def get_state(self) -> HardwareState: ...
```

硬件层只返回状态，不写 planner 缓存。

### 9.3 执行层接口

```python
class CommandTransport(Protocol):
    def send(self, command: SkillCommand) -> None: ...
    def stop(self) -> None: ...
```

`file`、`udp`、`ros` 是 transport，不和 skill 逻辑混合。

### 9.4 技能层接口

```python
class Skill(Protocol):
    name: str
    model_use: int
    async def execute(self, request: SkillRequest) -> SkillResult: ...
```

技能只负责：

- 参数校验。
- 构造 `SkillCommand`。
- 选择执行等待方式。

### 9.5 Planner 层接口

```python
class Planner(Protocol):
    def plan(self, context: PlannerContext) -> list[TaskStep]: ...
```

可以组合：

```text
LLMPlanner -> RuleOverridePlanner -> ParameterAnnotator
```

### 9.6 应用服务层

新增一个明确的 orchestration 层，取代当前散在 `Robot_Module.pipeline_factory` 和 TUI 里的胶水：

```python
class AgentService:
    def observe(self) -> ObservationResult: ...
    async def act(self, user_intent: str, observation: ObservationResult | None) -> ActionResult: ...
```

## 10. 重构时优先保留的行为

这些行为是项目价值核心，建议优先保留：

1. 顶层只暴露 `vlm_observe` 和 `robot_act`。
2. 规划输入使用结构化 JSON，而不是零散自然语言。
3. 有实时状态才允许动作成功。
4. validation 优先级高于模型自称成功。
5. 导航到达使用距离阈值和稳定轮询。
6. 位移执行有方向检查和宽限期。
7. 已知几何场景使用规则覆盖，不能完全依赖 LLM。
8. sim / go2 后端通过统一 `get_state()` 屏蔽。
9. 执行事件通过 callback 输出，TUI 不进入核心逻辑。
10. 技能注册按任务类型可替换。

## 11. 可以删除或暂缓的内容

重构初期可以暂缓：

- Rich TUI 的复杂表格渲染。
- ROS2 图像抓取。
- USB 摄像头。
- IsaacLab 进程扫描和动态 import 场景布局。
- `pygame` 相关依赖。
- 旧 `Robot_Module/tests/test_agent_tools.py`。
- `__pycache__`。
- 旧设计文档中的长篇迁移历史。

先保留：

- 核心数据模型。
- `get_state()`。
- file transport。
- `navigation` 和 `walk` 闭环。
- 最小 planner。
- validation。

## 12. 从备份恢复旧代码

重构后当前目录只保留本文件。旧项目文件迁移到：

```text
/home/robot/work/backup
```

如果需要恢复某个文件，可从备份复制回来，例如：

```bash
cp /home/robot/work/backup/README.md /home/robot/work/FinalProject/
cp -r /home/robot/work/backup/Robot_Module /home/robot/work/FinalProject/
```

如果 `.git` 也被迁移，则当前目录不再是 Git 仓库。需要恢复 Git 元数据时：

```bash
cp -a /home/robot/work/backup/.git /home/robot/work/FinalProject/
```

## 13. 重构验收建议

每个阶段都应该有可运行验收，而不是等最终整合：

1. 配置层：能加载 `.env.example`，缺 key 不崩。
2. 硬件层：fake/sim state 能解析为 `HardwareState`。
3. 执行层：fake state 能验证导航成功/失败。
4. 技能层：`navigation`、`walk` 能构造正确 command 并返回 validation。
5. Planner：给定 object_facts 能生成正确任务链。
6. AgentService：`act("导航到5,0,0")` 能跑完整 pipeline。
7. TUI：能显示 status、tools、pipeline events。
8. Go2：ROS2 topic payload 与 `Sim2Real.md` 契约一致。

## 14. 最小重构目标

第一个可用版本只需要做到：

```text
python run.py
-> 输入：导航到 5,0,0
-> planner 输出 navigation(goal=5,0,0)
-> file transport 写 /tmp/envtest_* 控制文件
-> fake/sim backend 读取 live state
-> 到达后 validation success
-> TUI 显示 OK
```

这个闭环跑通后，再逐步加 VLM、规则覆盖、push_box、Go2。

