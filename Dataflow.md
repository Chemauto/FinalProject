# FinalProject Dataflow
```text
用户输入
-> Interactive_Module/interactive.py: process_user_input()
-> 先同步 live EnvTest 到 config/object_facts.json
   -> scene_id / model_use / start / pose_command / vel_command
   -> 按 IsaacLab scene_layout 重建 objects
   -> 用户输入若含 pose_command=[...] / vel_command=[...]，覆盖 runtime_state
-> load_object_facts(object_facts.json)
-> VLMCore.describe_structured()
-> VLMCore.build_scene_facts() + merge_scene_facts(scene_facts, object_facts)
-> LLMAgent.run_pipeline()
   -> HighLevelPlanner.plan_tasks()
   -> ParameterCalculator.annotate_tasks()
   -> LowLevelExecutor.execute_single_task()
   -> execute_tool()
   -> Robot_Module/module/navigation.py
   -> 写 /tmp 控制文件或发 UDP/ROS
   -> IsaacLab EnvTest 执行
   -> execution_feedback 返回
-> 打印 LLM思考 / 任务总结 / 完成数
```
## 1. 输入
- 文本输入来自 CLI，例如 `前往目标点`
- 入口函数是 `process_user_input()`
- 处理顺序是“先更新数据，再做 VLM/LLM 规划”
## 2. object_facts.json
- 路径: `config/object_facts.json`
- 这是高可信几何真值，优先级高于 VLM
- 当前关键字段:
  - `navigation_goal`: 目标点
  - `robot_pose`: 机器人位姿
  - `constraints`: 攀爬/推箱子约束
  - `objects`: 规划时真正使用的物体列表
  - `runtime_state`: EnvTest 运行态和用户覆盖参数
  - `scene_objects`: 按 `scene_id` 重建出的 IsaacLab 场景物体

```json
{
  "navigation_goal": [5.0, 0.0, 0.0],
  "robot_pose": [0.0, 0.0, 0.0],
  "constraints": {"max_climb_height_m": 0.3},
  "objects": [...],
  "runtime_state": {
    "scene_id": 0,
    "model_use": 1,
    "pose_command": [x, y, z],
    "vel_command": [vx, vy, wz]
  }
}
```
## 3. VLM
- 输入: 摄像头图片或默认图片
- 调用: `VLMCore.describe_structured()`
- 输出:
  - `visual_context`: 给终端显示，也传给 LLM
  - `scene_facts`: 结构化视觉结果，如 `ground`、`front_area`、`obstacles`、`uncertainties`

## 4. VLM 与 object_facts 融合
- 调用: `VLMCore.merge_scene_facts(scene_facts, object_facts)`
- 规则:
  - `object_facts` 负责几何真值
  - VLM 负责视觉补充
  - 冲突时以 `object_facts` 为准

## 5. 上层 LLM
- 输入: `user_input + visual_context + scene_facts + object_facts`
- 调用: `HighLevelPlanner.plan_tasks()`
- 输出: `tasks`
- 每个 task 至少含 `step / task / type / function / reason`
- 终端会打印 `任务概述`、`子任务序列`、`LLM思考`

## 6. 参数 JSON
- 调用: `ParameterCalculator.annotate_tasks(tasks, object_facts)`
- 为 task 增加:
  - `parameter_context`
  - `calculated_parameters`
- 典型结果:
  - `way_select -> {"direction":"left","lateral_distance":0.5}`
  - `push_box -> {"box_height":0.2,"target_position":"[2.8, 0.12, 0.0]"}`
  - `climb -> {"height":0.2,"stage":"platform_front"}`
  - `walk -> {"route_side":"前方","distance":1.505,"target":"目标点[5, 0, 0]"}`

## 7. 下层 LLM 与函数调用
- 调用: `LowLevelExecutor.execute_single_task()`
- 若 task 已带 `calculated_parameters`，低层优先直接执行，不重新猜参数
- 函数调用链:
  - `execute_tool()`
  - `Robot_Module.skill.get_skill_function()`
  - `navigation.py` 中的 `walk / climb / push_box / way_select`

## 8. 执行后端与输出
- `navigation.py` 会写:
  - `/tmp/model_use.txt`
  - `/tmp/envtest_velocity_command.txt`
  - `/tmp/envtest_goal_command.txt`
  - `/tmp/envtest_start.txt`
- IsaacLab EnvTest 读取这些控制文件执行
- 返回数据包括 `execution_feedback / execution_result / control_command / status`
- CLI 最终输出每一步成功失败、任务总结、`完成 x/y 个任务成功`
