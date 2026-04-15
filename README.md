# FinalProject
基于 LLM 决策的四足机器人导航项目，面向 Unitree Go2，当前主执行后端是 IsaacLab EnvTest。

## 当前能力
- 最外层智能体只对外暴露 2 个高层技能：`vlm_observe`、`robot_act`
- `robot_act` 内部继续复用 7 个动作技能：`walk`、`navigation`、`nav_climb`、`climb_align`、`climb`、`push_box`、`way_select`
- 规划优先级：`navigation` 最简单，`climb` 次之，`push_box + climb_align + climb` 最复杂
- 最大单步攀爬高度：`0.3 m`
- 几何真值优先级高于 VLM
- 任一步失败后直接停止后续任务，不自动重规划

## 主链路
```text
用户输入
-> Interactive_Module/interactive.py
-> 顶层 LLM 判断
   -> 直接回复
   -> 或调用 vlm_observe
   -> 或调用 robot_act
-> robot_act 内部再复用原有规划/参数计算/执行链
```

## 核心模块
- `Interactive_Module/interactive.py`：基于 `rich` 的 TUI 入口
- `Comm_Module/Status/envtest_status_sync.py`：同步 EnvTest 状态并写回 `object_facts.json`
- `LLM_Module/llm_highlevel.py`：高层规划
- `LLM_Module/parameter_calculator.py`：参数计算
- `LLM_Module/llm_lowlevel.py`：低层执行
- `VLM_Module/vlm_core.py`：VLM 输出、`scene_facts` 构建与融合
- `Robot_Module/module/navigation.py`：7 个技能的真实执行逻辑

## 输入与优先级
- 用户文本：任务目标与可选覆盖参数
- VLM：视觉语义与不确定项
- `config/object_facts.json`：几何真值、运行态、目标点
- 冲突时以 `object_facts.json` 为准

## object_facts.json
默认路径：`config/object_facts.json`
示例文件：`config/object_facts.example.json`

最关键字段：
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

说明：
- `objects`：当前规划真正使用的物体
- `runtime_state`：EnvTest 运行态和用户覆盖值
- `navigation_goal`：`navigation` 参数计算的最终目标点；当规划仍需短距路线前进时也可供 `walk` 使用

## live EnvTest 同步
交互模式下，每次用户输入后、规划前，系统会优先同步 live EnvTest。
同步内容包括：
- `scene_id / model_use / start`
- `/tmp/model_use.txt`
- `/tmp/envtest_velocity_command.txt`
- `/tmp/envtest_goal_command.txt`
- top-level `objects`
- `runtime_state.pose_command / vel_command / goal`

手动同步：
```bash
cd /home/xcj/work/FinalProject/Comm_Module/Status
python sync_envtest_status.py --live-envtest
```

从文本文件同步：
```bash
python sync_envtest_status.py --status-file /path/to/status.txt
```

## 用户输入可覆盖的数据
当前支持从用户文本里直接提取：
- `前往20,0,0`
- `前往 5，0，0 坐标处`
- `导航到 [7, 1.5, 0.2]`
- `pose_command=[2.8, 0.12, 0, 0]`
- `vel_command=[0, 0, 0]`

规划前会覆盖：
- `navigation_goal`
- `runtime_state.pose_command`
- `runtime_state.vel_command`

## VLM 输出
VLM 当前输出结构化 JSON，核心字段：
- `ground`
- `left_side`
- `right_side`
- `front_area`
- `obstacles`
- `suspected_height_diff`
- `uncertainties`
- `envtest_alignment`

其中 `envtest_alignment.platform_1 / platform_2 / box` 会尽量与 IsaacLab `EnvTest Live Status` 对齐。

## 参数计算
`LLM_Module/parameter_calculator.py` 负责：
- `navigation`：目标点与 `goal_command`
- `climb_align`：攀爬前对正的目标平台/箱子上下文
- `way_select`：左右方向
- `push_box`：默认自动推箱模式
- `climb`：真实高度与目标平台
- `walk`：沿当前路线继续前进时的距离、路线侧别、目标点

当前约束：
- `way_select` 后，后续 `climb` 会继承当前已选路线
- 任务文本里若明确写了平台名，例如 `platform_left_low`，会优先选该平台

## 技能当前行为
- `walk`：默认速度 `0.6 m/s`，用于沿当前路线继续前进
- `navigation`：通过 `goal_command` 下发目标点，使用 EnvTest `model_use=4 / NavigationWalk` 自动导航到目标位置；具备绕障能力，但不能直接翻越高台
- `nav_climb`：通过 `goal_command` 下发目标点，使用 EnvTest `model_use=5 / NavigationClimb` 直接翻越高台前进；不负责常规绕障
- `navigation`：默认每 `0.5s` 轮询 `/tmp/envtest_live_status.json`，按“目标距离 + 连续位置稳定”判定 SUCCESS，而不是仅按预计时间返回成功
- `climb_align`：在正式 `climb` 前，使用 `model_use=4` 导航到箱子或平台前的攀爬起点
- `way_select`：默认是横向 `walk`；左侧 `velocity=[0.0,0.5,0.0]`，右侧 `velocity=[0.0,-0.5,0.0]`，固定 `3s`
- `climb`：最大 `0.3m`，默认速度 `0.6 m/s`，默认执行 `12s`，通过统一 EnvTest 控制后端下发 `model_use=2`
- `push_box`：箱子辅助场景下默认使用自动模式，不显式下发推箱目标点

## 最短运行方式
1. 启动 IsaacLab player
```bash
cd /home/xcj/work/IsaacLab/IsaacLabBisShe
python NewTools/envtest_model_use_player.py --scene_id 3
```
默认会持续写出 `/tmp/envtest_live_status.json`，供 FinalProject 轮询导航完成状态。
2. 启动交互入口
```bash
cd /home/xcj/work/FinalProject/Interactive_Module
python interactive.py
```
启动后可使用 `/help`、`/tools`、`/reset`、`/status`、`/vlm`、`/quit` 管理当前会话。

3. 输入任务
```text
前往20,0,0
```

说明：
- 纯寒暄、解释、问答时，顶层智能体应直接回复，不调用技能
- 需要环境信息时，优先调用 `vlm_observe`
- 需要真实动作时，调用 `robot_act`
- `robot_act` 内部仍会优先复用现有规划和动作执行逻辑

## 常用环境变量
- `FINALPROJECT_OBJECT_FACTS_PATH`
- `FINALPROJECT_WAY_SELECT_POLICY`
- `FINALPROJECT_NAV_BACKEND`
- `FINALPROJECT_STATUS_FILE`
- `FINALPROJECT_STATUS_POLL_SEC`
- `FINALPROJECT_NAV_ARRIVAL_TOL_M`
- `FINALPROJECT_NAV_STABLE_POSITION_DELTA_M`
- `FINALPROJECT_NAV_REQUIRED_STABLE_POLLS`
- `FINALPROJECT_STATUS_STALE_SEC`
- `FINALPROJECT_CLIMB_DURATION_SEC`
- `FINALPROJECT_CLIMB_SPEED_MPS`
- `FINALPROJECT_POST_PUSH_SETTLE_SEC`
- `FINALPROJECT_PRE_CLIMB_SETTLE_SEC`
- `FINALPROJECT_POST_ALIGN_SETTLE_SEC`
- `FINALPROJECT_CLIMB_PREALIGN_OFFSET_M`

## 当前限制
- `robot_pose` 目前主要只按位置使用
- `push_box` 当前默认依赖 EnvTest 自带的自动推箱目标推理
- 若 `objects` 残留旧场景，规划会出错，所以同步必须发生在规划前
- 若 live EnvTest 与本地 JSON 不一致，交互模式会以同步后的文件为准
- 当前只保留文字 TUI 交互入口，不再提供语音交互模式

## 相关文档
- `Dataflow.md`
- `CLAUDE.md`
- `VLM_Module/README.md`
- `Robot_Module/README.md`
