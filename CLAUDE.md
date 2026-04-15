# CLAUDE.md
本文件面向在本仓库中继续开发的协作者，描述当前真实代码行为、数据契约、执行约束和常见坑。

## 1. 项目定位
FinalProject 是一个面向 Unitree Go2 的 LLM 导航系统，当前主后端是 IsaacLab EnvTest。
系统不是纯语言链路，而是三类输入融合：
1. 用户文本
2. VLM 结构化视觉
3. `config/object_facts.json` 中的几何真值与运行态

最外层只对外暴露 2 个高层技能：
- `vlm_observe`：环境观测
- `robot_act`：动作执行（内部复用 7 个动作技能）

7 个内部动作技能：
- `walk`、`navigation`、`nav_climb`、`climb_align`、`climb`、`push_box`、`way_select`

## 2. 当前端到端流程
```text
user_input
-> Interactive_Module/interactive.py（顶层 agent）
   -> 顶层 LLM（qwen3.6-plus）判断：直接回复 / vlm_observe / robot_act
   -> 渐进式渲染：Thinking -> vlm_observe -> Thinking -> robot_act -> Assistant

robot_act 内部流程：
-> Comm_Module/Status/envtest_status_sync.update_object_facts_runtime()
-> load_object_facts()
-> VLMCore.merge_scene_facts()
-> HighLevelPlanner.plan_tasks()
-> ParameterCalculator.annotate_tasks()
-> LowLevelExecutor.execute_single_task()
-> Robot_Module/module/navigation.py
-> IsaacLab EnvTest
-> execution_feedback
```

关键原则：
- 顶层 agent 先输出思考，再调用工具
- 涉及运动时，先 `vlm_observe` 再 `robot_act`
- 同步先于规划，先算参数再执行
- 已有 `calculated_parameters` 时，低层不要重新猜
- 失败时最多重规划 1 次，仍失败则停止

## 3. 目录与职责
### `Interactive_Module`
- `interactive.py`：CLI 入口；顶层 agent 循环、渐进式渲染、工具调度
  - `run_agent_turn()`：agent 主循环，按执行顺序实时渲染 Thinking / tool 结果 / Assistant
  - `execute_tool()`：执行 `vlm_observe` 或 `robot_act`，robot_act 时用 `redirect_stderr` 抑制噪音
  - `_render_thinking_panel()` / `_render_tool_result_panel()`：渐进式渲染函数

### `Comm_Module`
- `execution_comm.py`：ROS/UDP/File 执行反馈通信
- `Status/envtest_status_sync.py`：同步 live EnvTest；更新 `config/object_facts.json`；支持从用户输入提取覆盖值
- `Status/sync_envtest_status.py`：手动同步脚本

### `LLM_Module`
- `llm_core.py`：总控，包含 `HighLevelPlanner`（高层规划）和 `LLMAgent`（动作规划与执行代理）
  - `HighLevelPlanner.plan_tasks()`：高层规划，只负责技能序列和原因
  - `LLMAgent.run_pipeline()`：串联规划、参数计算、执行、重规划
- `llm_lowlevel.py`：低层执行，优先使用 `calculated_parameters` 直接执行
- `parameter_calculator.py`：参数计算，是当前最关键的纠偏层
- `object_facts_loader.py`：读取并规范化 `object_facts.json`

### `VLM_Module`
- `vlm_core.py`：输出结构化视觉 JSON，构建并融合 `scene_facts`
- `prompts/VlmPrompt.yaml`：VLM 输出格式约束，当前要求带 `envtest_alignment`

### `Robot_Module`
- `skill.py`：统一注册工具（FastMCP）
- `agent_tools.py`：`robot_act` 封装，`_StreamingBuffer` + `redirect_stdout` 捕获内部 pipeline stdout
- `module/Action/navigation.py`：7 个技能的真实执行逻辑，日志输出到 stderr
- `module/Vision/vlm.py`：`vlm_observe` 封装

## 4. object_facts.json
默认路径：`config/object_facts.json`
这是当前规划的高可信输入，优先级高于 VLM。

关键字段：
```json
{
  "navigation_goal": [20.0, 0.0, 0.0],
  "robot_pose": [0.0, 0.0, 0.0],
  "constraints": {"max_climb_height_m": 0.3},
  "objects": [
    {"id": "platform_left_low", "type": "platform", "center": [3.0, 0.75, 0.0], "size": [2.0, 1.5, 0.3], "movable": false}
  ],
  "runtime_state": {
    "scene_id": 3,
    "model_use": 0,
    "start": 0,
    "pose_command": null,
    "vel_command": [0.0, 0.0, 0.0],
    "robot_pose": [-0.075, 0.496, 0.279],
    "goal": null
  }
}
```

理解方式：
- `objects`：规划器真正看的场景物体
- `runtime_state`：EnvTest 当前运行态
- `navigation_goal`：最终目标点
- `constraints`：参数计算和技能校验的硬约束

如果 `objects` 里残留旧场景，高层会按旧场景规划，所以同步必须发生在规划前。

## 5. live EnvTest 同步机制
`robot_act` 调用时会先同步 live EnvTest。
同步来源包括：
- 正在运行的 `envtest_model_use_player.py`
- `/tmp/model_use.txt`
- `/tmp/envtest_velocity_command.txt`
- `/tmp/envtest_goal_command.txt`
- `/tmp/envtest_start.txt`

同步结果会写回：
- `runtime_state.scene_id`
- `runtime_state.model_use`
- `runtime_state.start`
- `runtime_state.pose_command`
- `runtime_state.vel_command`
- `runtime_state.goal`
- top-level `objects`

top-level `objects` 会优先按 `scene_id` 重建，而不是继续沿用旧 JSON。

## 6. 用户输入覆盖规则
交互入口支持从自然语言里直接提取：
- `前往20,0,0`
- `前往 5，0，0 坐标处`
- `导航到 [7, 1.5, 0.2]`
- `pose_command=[2.8, 0.12, 0, 0]`
- `vel_command=[0, 0, 0]`

当前会在规划前覆盖：
- `navigation_goal`
- `runtime_state.pose_command`
- `runtime_state.vel_command`

如果用户没有提供，则保持当前文件内容不变。

## 7. VLM 输出约束
VLM 当前必须尽量输出结构化 JSON，核心字段：
- `ground`
- `left_side`
- `right_side`
- `front_area`
- `obstacles`
- `suspected_height_diff`
- `uncertainties`
- `envtest_alignment`

`envtest_alignment` 需要对齐 IsaacLab 状态面板：
- `platform_1`
- `platform_2`
- `box`

每个非空对象至少包含：
- `name`
- `type`
- `side`
- `height_m`
- `summary`

若改了 IsaacLab 场景命名，记得同步更新：
- `VLM_Module/prompts/VlmPrompt.yaml`
- `VLM_Module/vlm_core.py`
- `Comm_Module/Status/envtest_status_sync.py`

## 8. 高层规划的职责边界
高层规划只负责：
- 判断路线
- 决定技能序列
- 给出每步原因

高层不应该直接决定：
- 精确坐标
- 最终高度数值
- 速度和执行时长

这些应留给 `parameter_calculator.py` 或 `navigation.py`。

## 9. 参数计算的职责边界
`LLM_Module/parameter_calculator.py` 负责：
- `way_select` 的方向
- `push_box` 的目标位置
- `climb` 的目标平台和高度
- `walk` 的距离、路线侧别和目标点

当前必须守住的几个不变量：
- `way_select` 后要保存当前路线侧别
- `climb` 优先选任务文本明确写出的平台 ID
- 如果任务没写平台 ID，再按当前路线侧别找同侧平台
- `walk` 起点必须继承前一步动作后的 `current_pose`

当前已经修掉的典型错误：
- 规划说"走右侧"，参数却算成左侧
- 已经 `way_select` 到左侧，`climb` 还去爬右侧最高平台
- 用户输入新坐标后，`walk` 还沿用旧 `navigation_goal`

## 10. 技能真实行为
### `walk`
- 默认速度：`0.6 m/s`
- 预计时长：按距离估算，加 buffer
- 默认后端：写 EnvTest 控制文件

### `way_select`
- 默认不是独立策略，而是横向 `walk`
- 左侧：`velocity=[0.0, 0.5, 0.0]`
- 右侧：`velocity=[0.0, -0.5, 0.0]`
- 固定执行：`3.0 秒`
- 若设置 `FINALPROJECT_WAY_SELECT_POLICY=navigation`，才切到导航模式

### `climb`
- 最大允许高度：`0.3 m`
- 默认速度：`0.6 m/s`
- 默认执行时间：`12.0 秒`
- 当前通过统一 EnvTest 控制后端下发
- 使用 `model_use=2`

### `push_box`
- 通过目标点或自动模式下发
- 当前仍是规则式几何目标点，不是完整物理规划器

## 11. 日志与渲染机制
### stdout 链路
- `llm_core.py` / `llm_lowlevel.py` 的 print 输出到 stdout
- `agent_tools.py` 使用 `_StreamingBuffer` + `redirect_stdout` 捕获这些 stdout
- `_StreamingBuffer` 通过 `log_callback` 逐行转发
- `interactive.py` 的 `render_stream_line()` 将转发内容打印到 `sys.__stdout__`
- 结果：████ 规划块、⚙️🔧 执行信息在终端实时可见

### stderr 抑制
- `navigation.py` 的 `[go2.skill]`、`[go2.speech]` 输出到 stderr
- `interactive.py` 的 `execute_tool()` 在 robot_act 分支使用 `redirect_stderr` 抑制

### 渐进式渲染
- `run_agent_turn()` 在 agent 循环内按执行顺序实时渲染
- Thinking 面板在 tool 执行前渲染
- Tool 结果面板在 tool 执行后立即渲染
- Assistant 面板在最终回复时渲染

## 12. 重要环境变量
- `FINALPROJECT_OBJECT_FACTS_PATH`
- `FINALPROJECT_WAY_SELECT_POLICY`
- `FINALPROJECT_NAV_BACKEND`
- `FINALPROJECT_CLIMB_DURATION_SEC`
- `FINALPROJECT_CLIMB_SPEED_MPS`
- `FINALPROJECT_AGENT_MODEL`

改执行行为前，先确认是不是环境变量覆盖了默认值。

## 13. 常见失败模式
### 失败模式 1：规划用的是旧场景
现象：`scene_id` 已变，但高层还按旧 `objects` 规划。
优先检查：
- `robot_act` 是否先执行了 live sync
- `config/object_facts.json` 的 top-level `objects` 是否已更新

### 失败模式 2：已经到左边了，`climb` 还报右侧 0.5 米不可爬
现象：规划显示 `platform_left_low`，参数计算却给出 `platform_right_high`。
根因：参数计算没有继承 `way_select` 的路线上下文。
如果回归，重点查 `parameter_calculator.py`。

### 失败模式 3：用户输入新坐标，但 `walk` 还是旧目标点
优先检查：
- `extract_navigation_goal_override()`
- `update_object_facts_runtime()`
- `object_facts.json` 中 `navigation_goal` 是否已更新

### 失败模式 4：VLM 识别和 IsaacLab 面板不一致
优先检查：
- `envtest_alignment`
- `VlmPrompt.yaml`
- 当前图片是否对应正确 scene

### 失败模式 5：Thinking 面板出现在内部日志之后
优先检查：
- `interactive.py` 的 `execute_tool()` 中是否有外层 `redirect_stdout` 阻断了 `agent_tools.py` 的 `_StreamingBuffer` 链路
- 只应使用 `redirect_stderr`，不应使用 `redirect_stdout`

## 14. 推荐调试命令
手动同步当前场景：
```bash
cd /home/robot/work/FinalProject/Comm_Module/Status
python sync_envtest_status.py --live-envtest
```

查看 object facts：
```bash
sed -n '1,220p' /home/robot/work/FinalProject/config/object_facts.json
```

直接跑交互入口：
```bash
cd /home/robot/work/FinalProject/Interactive_Module
python interactive.py
```

语法检查：
```bash
python -m py_compile Interactive_Module/*.py LLM_Module/*.py VLM_Module/*.py Robot_Module/module/navigation.py
```

## 15. 修改代码时的约定
- 改场景同步逻辑时，优先保证"同步先于规划"
- 改技能行为时，同时更新 README 和本文件
- 改 `object_facts` 结构时，同时更新 loader、sync、README、Dataflow
- 改 VLM 输出结构时，同时更新 prompt、normalize、README
- 改 `way_select` 或 `climb` 规则时，至少手工验证一条左侧场景和一条箱子场景
- 改 `interactive.py` 渲染逻辑时，确保不使用 `redirect_stdout`（会阻断 `_StreamingBuffer` 链路）

## 16. 当前限制
- `robot_pose` 目前主要按位置使用，没有完整姿态推导
- `push_box` 没有完整接触/拓扑规划
- 实时物理状态仍主要通过 scene preset 和控制文件推断，不是完整仿真状态镜像
- 如果 IsaacLab 场景定义改名，FinalProject 这边的对齐映射也要同步修改
