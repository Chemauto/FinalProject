# FinalProject
基于 LLM 决策的四足机器人导航项目，面向 Unitree Go2，当前主执行后端是 IsaacLab EnvTest。

## 当前能力
- 只对外暴露 5 个技能：`walk`、`navigation`、`climb`、`push_box`、`way_select`
- 最大单步攀爬高度：`0.3 m`
- 几何真值优先级高于 VLM
- 任一步失败后直接停止后续任务，不自动重规划

## 主链路
```text
用户输入
-> Interactive_Module/interactive.py
-> 同步 live EnvTest 到 config/object_facts.json
-> 读取 object_facts
-> 调用 VLM 输出结构化视觉
-> 融合 scene_facts + object_facts
-> 高层 LLM 规划 tasks
-> ParameterCalculator 生成参数 JSON
-> 低层执行器直接调用工具
-> Robot_Module/module/navigation.py
-> 写 /tmp 控制文件 或 调 Socket client
-> IsaacLab EnvTest 执行
-> 返回 execution_feedback
```

## 核心模块
- `Interactive_Module/interactive.py`：CLI 入口
- `Interactive_Module/envtest_status_sync.py`：同步 EnvTest 状态并写回 `object_facts.json`
- `LLM_Module/llm_highlevel.py`：高层规划
- `LLM_Module/parameter_calculator.py`：参数计算
- `LLM_Module/llm_lowlevel.py`：低层执行
- `VLM_Module/vlm_core.py`：VLM 输出、`scene_facts` 构建与融合
- `Robot_Module/module/navigation.py`：5 个技能的真实执行逻辑

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
cd /home/xcj/work/FinalProject/Interactive_Module
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
- `way_select`：左右方向
- `push_box`：默认自动推箱模式
- `climb`：真实高度与目标平台
- `walk`：沿当前路线继续前进时的距离、路线侧别、目标点

当前约束：
- `way_select` 后，后续 `climb` 会继承当前已选路线
- 任务文本里若明确写了平台名，例如 `platform_left_low`，会优先选该平台

## 技能当前行为
- `walk`：默认速度 `0.6 m/s`，用于沿当前路线继续前进
- `navigation`：通过 `goal_command` 下发目标点，使用 EnvTest `model_use=4` 自动导航到目标位置
- `way_select`：默认是横向 `walk`；左侧 `velocity=[0.0,0.5,0.0]`，右侧 `velocity=[0.0,-0.5,0.0]`，固定 `3s`
- `climb`：最大 `0.3m`，默认速度 `0.6 m/s`，默认执行 `15s`，默认通过 `Socket/envtest_socket_client.py` 下发 `model_use=2`
- `push_box`：箱子辅助场景下默认使用自动模式，不显式下发推箱目标点

## 最短运行方式
1. 启动 IsaacLab player
```bash
cd /home/xcj/work/IsaacLab/IsaacLabBisShe
python NewTools/envtest_model_use_player.py --scene_id 3
```
2. 启动交互入口
```bash
cd /home/xcj/work/FinalProject/Interactive_Module
python interactive.py
```
3. 输入任务
```text
前往20,0,0
```

说明：
- 当用户说“前往某个目标点/坐标/位置”时，高层规划会优先调用 `navigation`
- `walk` 主要保留给沿路线直行、横向切换后的继续前进等速度式动作
- 箱子辅助场景默认收敛为 `push_box -> climb -> navigation`

## 常用环境变量
- `FINALPROJECT_OBJECT_FACTS_PATH`
- `FINALPROJECT_WAY_SELECT_POLICY`
- `FINALPROJECT_NAV_BACKEND`
- `FINALPROJECT_CLIMB_DURATION_SEC`
- `FINALPROJECT_CLIMB_SPEED_MPS`
- `FINALPROJECT_CLIMB_USE_SOCKET_CLIENT`

## 当前限制
- `robot_pose` 目前主要只按位置使用
- `push_box` 当前默认依赖 EnvTest 自带的自动推箱目标推理
- 若 `objects` 残留旧场景，规划会出错，所以同步必须发生在规划前
- 若 live EnvTest 与本地 JSON 不一致，交互模式会以同步后的文件为准

## 相关文档
- `Dataflow.md`
- `CLAUDE.md`
- `VLM_Module/README.md`
- `Robot_Module/README.md`
