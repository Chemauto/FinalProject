# FinalProject Dataflow

```text
用户输入
-> Interactive_Module/interactive.py: process_user_input()
-> 先同步 live EnvTest 到 config/object_facts.json
-> load_object_facts(object_facts.json)
   -> 这里只保留 normalized object_facts:
      navigation_goal / robot_pose / constraints / objects
-> VLMCore.describe_structured()
-> VLMCore.build_scene_facts()
-> VLMCore.merge_scene_facts(scene_facts, object_facts)
-> LLMAgent.run_pipeline()
   -> HighLevelPlanner.plan_tasks()
      -> 高层 LLM 输出抽象任务 tasks
      -> 规则层可能修正 tasks
   -> ParameterCalculator.annotate_tasks()
      -> 给每个 task 补 parameter_context / calculated_parameters
   -> LowLevelExecutor.execute_single_task()
      -> 若已有 calculated_parameters: 直接执行工具
      -> 否则: 调低层 LLM 决定工具调用
   -> execute_tool()
   -> Robot_Module/module/navigation.py
   -> 写 /tmp 控制文件或发 UDP/ROS
   -> IsaacLab EnvTest 执行
   -> execution_feedback 返回
-> 打印 LLM思考 / 任务总结 / 完成数
```

## 1. 入口
- 文本输入来自 CLI，例如 `前往目标点`
- 入口函数是 [interactive.py](/home/xcj/work/FinalProject/Interactive_Module/interactive.py#L257)
- 当前顺序是：
  - 先同步 EnvTest 运行态
  - 再加载 `object_facts`
  - 再做 VLM
  - 最后进入 LLM 双层流水线

## 2. object_facts 的真实输入
- 文件路径通常是 `config/object_facts.json`
- 但注意：当前真正传给 LLM 和参数计算器的，不是原始 JSON 全量内容，而是 `load_object_facts()` 归一化后的结果
- 归一化代码在 [object_facts_loader.py](/home/xcj/work/FinalProject/LLM_Module/object_facts_loader.py#L59)
- 当前只保留 4 类字段：
  - `navigation_goal`
  - `robot_pose`
  - `constraints`
  - `objects`

```json
{
  "navigation_goal": [5.0, 0.0, 0.0],
  "robot_pose": [0.0, 0.0, 0.0],
  "constraints": {
    "max_climb_height_m": 0.3,
    "push_only_on_ground": true,
    "climb_requires_adjacency": true
  },
  "objects": [
    {
      "id": "platform_left_low",
      "type": "platform",
      "center": [3.0, 0.75, 0.15],
      "size": [2.0, 1.5, 0.3],
      "movable": false
    }
  ]
}
```

- `runtime_state`、`scene_objects` 虽然可能存在于原始 `object_facts.json`，但当前不会直接进入高层 LLM 的 `object_facts` 输入

## 3. VLM 输入输出
- 图片来源是摄像头或默认图片
- 调用入口在 [interactive.py](/home/xcj/work/FinalProject/Interactive_Module/interactive.py#L298)
- `describe_structured()` 输出结构化视觉结果
- 然后由 [vlm_core.py](/home/xcj/work/FinalProject/VLM_Module/vlm_core.py#L183) 转成 `scene_facts`

VLM 侧产物分两类：
- `visual_context`
  - 主要给终端打印，也会传给高层和低层 LLM
- `scene_facts`
  - 主要给规划用，包含例如：
  - `terrain_features`
  - `route_options`
  - `interactive_objects`
  - `obstacles`
  - `uncertainties`

## 4. VLM 与 object_facts 如何融合
- 在 [interactive.py](/home/xcj/work/FinalProject/Interactive_Module/interactive.py#L311) 调用 `VLMCore.merge_scene_facts(scene_facts, object_facts)`
- 实际意图是：
  - `object_facts` 提供高可信几何信息
  - VLM 提供视觉补充
  - 发生冲突时优先相信 `object_facts`

## 5. 上层 LLM 实际看到了什么
- 高层规划入口在 [llm_core.py](/home/xcj/work/FinalProject/LLM_Module/llm_core.py#L47)
- 真正向高层 LLM 发送内容的地方在 [llm_highlevel.py](/home/xcj/work/FinalProject/LLM_Module/llm_highlevel.py#L48)

高层 LLM 输入由这 4 部分组成：
- `user_input`
- `visual_context`
- `scene_facts`
- `object_facts`

高层 LLM 输出是一个 JSON 计划，之后被解析成 `tasks`

每个 task 至少包含：
- `step`
- `task`
- `type`
- `function`
- `reason`

例如：

```json
{
  "step": 1,
  "task": "登上高台后，调用 navigation 前往目标点 [5.0, 0.0, 0.0]",
  "type": "导航",
  "function": "navigation",
  "reason": "目标点明确，登台后使用 navigation 自动规划剩余路径"
}
```

## 6. 高层 LLM 输出后，还会被规则层修正
- 高层 LLM 的原始输出不是最终结果
- 在 [llm_highlevel.py](/home/xcj/work/FinalProject/LLM_Module/llm_highlevel.py#L73) 之后，还会经过规则层修正

当前规则层会做几类事情：
- 如果检测到目标点可直接到达，强制改成单步 `navigation`
- 如果检测到箱子辅助攀爬场景，强制改成 `push_box -> climb_align -> climb -> navigation`
- 如果检测到单侧可直接攀爬，修正错误的 `walk` 方案

所以最终进入执行阶段的 `tasks`，是：
- 高层 LLM 输出
- 再经过规则修正

而不是原始 LLM 文本直接执行

## 7. 参数解析到底发生在什么时候
- 参数补全发生在高层规划之后
- 调用位置在 [llm_core.py](/home/xcj/work/FinalProject/LLM_Module/llm_core.py#L144)
- 实现代码在 [parameter_calculator.py](/home/xcj/work/FinalProject/LLM_Module/parameter_calculator.py#L12)

`ParameterCalculator.annotate_tasks()` 会给每个 task 补两个字段：
- `parameter_context`
- `calculated_parameters`

例如：
- `way_select`

```json
{
  "parameter_context": {
    "route_side": "left",
    "route_side_label": "左侧"
  },
  "calculated_parameters": {
    "direction": "left",
    "lateral_distance": 0.5,
    "target": "前方目标点"
  }
}
```

- `navigation`

```json
{
  "parameter_context": {
    "start_pose_xyz": [0.0, 0.0, 0.0],
    "goal_pose_xyz": [5.0, 0.0, 0.0]
  },
  "calculated_parameters": {
    "goal_command": "[5, 0, 0]",
    "target": "目标点[5, 0, 0]"
  }
}
```

- `climb`

```json
{
  "parameter_context": {
    "target_object": "platform_left_low",
    "relative_height_m": 0.3
  },
  "calculated_parameters": {
    "height": 0.3,
    "stage": "platform_left_low",
    "target": "platform_left_low"
  }
}
```

## 8. 参数解析以后，怎么又让 LLM 知道的
- 这里最容易误解
- 结论是：
  - 参数解析以后，不会回头重跑高层 LLM
  - 参数会被写回 task 结构
  - 然后交给低层执行器

低层执行入口在 [llm_lowlevel.py](/home/xcj/work/FinalProject/LLM_Module/llm_lowlevel.py#L142)

分两种情况：

1. task 已经有 `function + calculated_parameters`
- 直接执行工具
- 不再调用低层 LLM
- 代码在 [llm_lowlevel.py](/home/xcj/work/FinalProject/LLM_Module/llm_lowlevel.py#L167)

2. task 还没有足够明确的参数
- 才调用低层 LLM
- 这时低层 prompt 会显式带上：
  - `task_description`
  - `task_type`
  - `suggested_function`
  - `planning_reason`
  - `parameter_context`
  - `calculated_parameters`
  - `visual_context`
  - `previous_result`
- 代码在 [llm_lowlevel.py](/home/xcj/work/FinalProject/LLM_Module/llm_lowlevel.py#L188)

所以准确说法不是“参数解析后再告诉高层 LLM”，而是：
- 参数解析后写进 task
- 低层拿这个 task 直接执行
- 只有必要时，低层 LLM 才会看到这些参数

## 9. 最终如何一步步执行
- 主执行循环在 [llm_core.py](/home/xcj/work/FinalProject/LLM_Module/llm_core.py#L73)
- 执行顺序是串行的，不是并行的

流程是：
1. 取第一个 task
2. 执行
3. 如果成功，把这一步结果记入 `previous_result`
4. 继续下一步
5. 任何一步失败，就中止后续任务

`previous_result` 的传递位置在 [llm_core.py](/home/xcj/work/FinalProject/LLM_Module/llm_core.py#L101)

## 10. 工具是怎么真正被调用的
- 最终工具调用经过 [interactive.py](/home/xcj/work/FinalProject/Interactive_Module/interactive.py#L96) 的 `execute_tool()`
- 它会：
  - 根据函数名找到技能函数
  - 调用 `Robot_Module` 的异步技能
  - 把原始返回归一化

返回归一化逻辑在 [interactive.py](/home/xcj/work/FinalProject/Interactive_Module/interactive.py#L68)

成功判定依赖两层：
- `status == "success"`
- `execution_feedback.signal == "SUCCESS"`

只有两者同时满足，CLI 才会认为当前步骤成功并继续后续步骤

## 11. 一个完整例子
用户输入：

```text
前往5,0,0点
```

处理链路大致如下：

```text
用户输入
-> interactive.py 同步 EnvTest / 读取 object_facts / 调 VLM
-> 高层 LLM 看到:
   user_input + visual_context + scene_facts + normalized object_facts
-> 输出抽象任务:
   例如 "调用 navigation 前往目标点"
-> 规则层检查:
   如果前方可直接到达，则保留或改写成单步 navigation
-> 参数计算:
   goal_command="[5, 0, 0]"
-> 低层执行器:
   发现参数已齐全，直接执行 navigation
-> execute_tool()
-> Robot_Module/module/navigation.py
-> EnvTest 执行
-> 返回 execution_feedback
-> 成功则继续下一步；若这是最后一步则结束
```

## 12. 一句话总结
- 高层 LLM 负责“分解任务”
- 规则层负责“修正明显不合理的规划”
- 参数计算器负责“把抽象任务变成具体参数”
- 低层执行器负责“直接执行，必要时才再问低层 LLM”
- `Robot_Module` 负责“真正控制 IsaacLab/EnvTest”
