# CLAUDE.md
本文件面向在本仓库中继续开发的协作者，描述当前真实代码行为、数据契约、执行约束和常见坑。

## 1. 项目定位
FinalProject 是一个面向 Unitree Go2 的 LLM 导航系统，当前主后端是 IsaacLab EnvTest。
系统不是纯语言链路，而是三类输入融合：
1. 用户文本
2. VLM 结构化视觉
3. `config/object_facts.json` 中的几何真值与运行态

当前只对外暴露 4 个工具：
- `walk`
- `climb`
- `push_box`
- `way_select`

## 2. 当前端到端流程
```text
user_input
-> Interactive_Module/interactive.py
-> envtest_status_sync.update_object_facts_runtime()
-> load_object_facts()
-> VLMCore.describe_structured()
-> VLMCore.build_scene_facts()
-> VLMCore.merge_scene_facts()
-> HighLevelPlanner.plan_tasks()
-> ParameterCalculator.annotate_tasks()
-> LowLevelExecutor.execute_single_task()
-> Robot_Module/module/navigation.py
-> IsaacLab EnvTest
-> execution_feedback
```

关键原则：
- 先同步运行态，再规划
- 先算参数，再执行
- 已有 `calculated_parameters` 时，低层不要重新猜
- 任一步失败后停止后续任务

## 3. 目录与职责
### `Interactive_Module`
- `interactive.py`：CLI 入口；每次用户输入后先做 EnvTest 同步，再读 object facts、调 VLM/LLM
- `envtest_status_sync.py`：同步 live EnvTest；更新 `config/object_facts.json`；支持从用户输入提取覆盖值
- `sync_envtest_status.py`：手动同步脚本

### `LLM_Module`
- `llm_highlevel.py`：高层规划，只负责技能序列和原因
- `parameter_calculator.py`：参数计算，是当前最关键的纠偏层
- `llm_lowlevel.py`：顺序执行单个任务，优先使用 `calculated_parameters`
- `object_facts_loader.py`：读取并规范化 `object_facts.json`

### `VLM_Module`
- `vlm_core.py`：输出结构化视觉 JSON，构建并融合 `scene_facts`
- `prompts/VlmPrompt.yaml`：VLM 输出格式约束，当前要求带 `envtest_alignment`

### `Robot_Module`
- `skill.py`：注册工具
- `module/navigation.py`：4 个技能的真实执行逻辑

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
交互模式会在每次用户输入后先同步 live EnvTest。
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
- `Interactive_Module/envtest_status_sync.py`

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
- 规划说“走右侧”，参数却算成左侧
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
- 默认执行时间：`15.0 秒`
- 当前默认通过 `Socket/envtest_socket_client.py` 下发
- 使用 `model_use=2`

### `push_box`
- 通过目标点或自动模式下发
- 当前仍是规则式几何目标点，不是完整物理规划器

## 11. 重要环境变量
- `FINALPROJECT_OBJECT_FACTS_PATH`
- `FINALPROJECT_WAY_SELECT_POLICY`
- `FINALPROJECT_NAV_BACKEND`
- `FINALPROJECT_CLIMB_DURATION_SEC`
- `FINALPROJECT_CLIMB_SPEED_MPS`
- `FINALPROJECT_CLIMB_USE_SOCKET_CLIENT`

改执行行为前，先确认是不是环境变量覆盖了默认值。

## 12. 常见失败模式
### 失败模式 1：规划用的是旧场景
现象：`scene_id` 已变，但高层还按旧 `objects` 规划。
优先检查：
- 交互模式是否先执行了 live sync
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

## 13. 推荐调试命令
手动同步当前场景：
```bash
cd /home/xcj/work/FinalProject/Interactive_Module
python sync_envtest_status.py --live-envtest
```

查看 object facts：
```bash
sed -n '1,220p' /home/xcj/work/FinalProject/config/object_facts.json
```

直接跑交互入口：
```bash
cd /home/xcj/work/FinalProject/Interactive_Module
python interactive.py
```

语法检查：
```bash
python -m py_compile Interactive_Module/*.py LLM_Module/*.py VLM_Module/*.py Robot_Module/module/navigation.py
```

## 14. 修改代码时的约定
- 改场景同步逻辑时，优先保证“同步先于规划”
- 改技能行为时，同时更新 README 和本文件
- 改 `object_facts` 结构时，同时更新 loader、sync、README、Dataflow
- 改 VLM 输出结构时，同时更新 prompt、normalize、README
- 改 `way_select` 或 `climb` 规则时，至少手工验证一条左侧场景和一条箱子场景

## 15. 当前限制
- `robot_pose` 目前主要按位置使用，没有完整姿态推导
- `push_box` 没有完整接触/拓扑规划
- 实时物理状态仍主要通过 scene preset 和控制文件推断，不是完整仿真状态镜像
- 如果 IsaacLab 场景定义改名，FinalProject 这边的对齐映射也要同步修改
