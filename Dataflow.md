# FinalProject Dataflow

## 总览

```text
用户输入
-> Interactive_Module/interactive.py
   -> 顶层 agent（qwen3.6-plus）循环
      -> LLM 判断：直接回复 / vlm_observe / robot_act
      -> 渐进式渲染：Thinking -> vlm_observe -> Thinking -> robot_act -> Assistant

vlm_observe 路径：
-> Robot_Module/module/Vision/vlm.py
-> VLMCore.describe_structured() + VLMCore.build_scene_facts()
-> 返回 visual_context + scene_facts

robot_act 路径：
-> Robot_Module/agent_tools.py: run_robot_act_pipeline()
   -> Comm_Module/Status/envtest_status_sync 同步 live EnvTest
   -> load_object_facts(object_facts.json)
   -> VLMCore.merge_scene_facts(scene_facts, object_facts)
   -> LLMAgent.run_pipeline()
      -> HighLevelPlanner.plan_tasks()
         -> 高层 LLM 输出抽象任务 tasks
      -> ParameterCalculator.annotate_tasks()
         -> 给每个 task 补 parameter_context / calculated_parameters
      -> LowLevelExecutor.execute_single_task()
         -> 若已有 calculated_parameters: 直接执行工具
         -> 否则: 调低层 LLM 决定工具调用
      -> _execute_registered_tool()
      -> Robot_Module/module/navigation.py
      -> 写 /tmp 控制文件或发 UDP/ROS
      -> IsaacLab EnvTest 执行
      -> execution_feedback 返回
```

## 1. 入口
- 文本输入来自 CLI，例如 `前往目标点`
- 入口函数是 `interactive.py` 的 `main()`
- 顶层 agent 循环在 `run_agent_turn()` 中

## 2. 顶层 agent 循环

`run_agent_turn()` 是一个最多 4 轮的循环：

```text
for _ in range(4):
    response = LLM(messages + tools)
    if no tool_calls:
        -> 渲染 Assistant 面板，结束
    if content (thinking):
        -> 渲染 Thinking 面板
    for tool_call in tool_calls:
        -> execute_tool(tool_name, args)
        -> 渲染 tool 结果面板
        -> 将结果追加到 messages，继续下一轮
```

### 渐进式渲染

渲染不是在循环结束后一次性完成，而是按执行顺序实时进行：

1. **Thinking 面板**：LLM 返回 thinking content 时立即渲染（在 tool 执行之前）
2. **vlm_observe 面板**：只显示 `visual_context` 内容
3. **robot_act 摘要面板**：只显示 `total_tasks` / `success_count` / `failure_count`
4. **Assistant 面板**：LLM 返回纯文本回复时渲染

### 日志流式转发

`robot_act` 内部日志（`llm_core.py` / `llm_lowlevel.py` 的 stdout print）通过以下链路实时显示：

```text
llm_core.py print()
-> agent_tools.py: redirect_stdout(_StreamingBuffer)
-> _StreamingBuffer.write() -> log_callback(line)
-> interactive.py: render_stream_line(console, line)
-> sys.__stdout__
```

## 3. object_facts 的真实输入
- 文件路径通常是 `config/object_facts.json`
- 真正传给 LLM 和参数计算器的，是 `load_object_facts()` 归一化后的结果
- 归一化代码在 `LLM_Module/object_facts_loader.py`
- 当前只保留 4 类字段：
  - `navigation_goal`
  - `robot_pose`
  - `constraints`
  - `objects`

## 4. VLM 输入输出
- 图片来源是摄像头或默认图片
- `describe_structured()` 输出结构化视觉结果
- 然后由 `vlm_core.py` 转成 `scene_facts`

VLM 侧产物分两类：
- `visual_context`
  - 主要给终端显示和传给顶层 agent
  - 也传入 robot_act 的 `observation_context`
- `scene_facts`
  - 主要给规划用
  - 包含 `terrain_features` / `route_options` / `interactive_objects` / `obstacles` / `uncertainties`

## 5. VLM 与 object_facts 如何融合
- 在 `agent_tools.py` 的 `run_robot_act_pipeline()` 中调用 `VLMCore.merge_scene_facts(scene_facts, object_facts)`
- `object_facts` 提供高可信几何信息
- VLM 提供视觉补充
- 发生冲突时优先相信 `object_facts`

## 6. 上层 LLM（robot_act 内部的高层规划）
- 高层规划入口在 `LLM_Module/llm_core.py` 的 `HighLevelPlanner.plan_tasks()`
- 输入由 4 部分组成：`user_input` / `visual_context` / `scene_facts` / `object_facts`
- 输出是一个 JSON 计划，解析成 `tasks`

每个 task 至少包含：`step` / `task` / `type` / `function` / `reason`

## 7. 参数计算
- 调用位置在 `LLM_Module/llm_core.py` 的 `LLMAgent._apply_parameter_calculation()`
- 实现代码在 `LLM_Module/parameter_calculator.py`

`ParameterCalculator.annotate_tasks()` 会给每个 task 补两个字段：
- `parameter_context`
- `calculated_parameters`

## 8. 低层执行

低层执行入口在 `LLM_Module/llm_lowlevel.py` 的 `LowLevelExecutor.execute_single_task()`。

分两种情况：

1. task 已经有 `function + calculated_parameters`
   - 直接执行工具，不再调用低层 LLM

2. task 还没有足够明确的参数
   - 调用低层 LLM 决定工具调用
   - 低层 prompt 带上：`task_description` / `task_type` / `suggested_function` / `planning_reason` / `parameter_context` / `calculated_parameters` / `visual_context` / `previous_result`

## 9. 执行流程

主执行循环在 `LLM_Module/llm_core.py` 的 `LLMAgent.run_pipeline()`。
执行顺序是串行的，失败时最多重规划 1 次。

```text
1. plan_tasks() -> 高层规划
2. _apply_parameter_calculation() -> 参数补全
3. for task in tasks:
     execute_single_task()
     if success: previous_result = result
     if failure and attempt < max_replans:
        -> 重规划
     if failure and attempt >= max_replans:
        -> 中止
```

## 10. 工具调用

- `vlm_observe`：由 `interactive.py` 的 `execute_tool()` 直接调用 `Robot_Module/module/Vision/vlm.py`
- `robot_act`：由 `interactive.py` 的 `execute_tool()` 调用 `Robot_Module/agent_tools.py` 的 `run_robot_act_pipeline()`
- 内部动作技能（walk、climb 等）：由 `agent_tools.py` 的 `_execute_registered_tool()` 调用

成功判定依赖两层：
- `status == "success"`
- `execution_feedback.signal == "SUCCESS"`

## 11. 一个完整例子
用户输入：`往前走1米`

```text
用户输入
-> interactive.py 顶层 agent
-> Thinking: "需要先观测环境，再执行前移动作"
-> vlm_observe: 获取视觉上下文
-> Thinking: "前方平整无障碍，可以直行1米"
-> robot_act:
   -> 同步 EnvTest / 读取 object_facts
   -> 高层 LLM 规划: [walk, 距离1米]
   -> 参数计算: distance=1.0, route_side="前方"
   -> 低层执行器: 直接执行 walk
   -> navigation.py: 下发速度命令
   -> EnvTest 执行
   -> 返回 SUCCESS
-> robot_act 摘要: total_tasks=1, success_count=1
-> Assistant: "已完成向前移动1米"
```

## 12. 一句话总结
- 顶层 agent（interactive.py）负责"思考 + 决策调用哪个高层工具"
- `vlm_observe` 负责"看环境"
- `robot_act` 内部的高层 LLM 负责"分解任务"
- 参数计算器负责"把抽象任务变成具体参数"
- 低层执行器负责"直接执行，必要时才再问低层 LLM"
- `Robot_Module` 负责"真正控制 IsaacLab/EnvTest"
- 渐进式渲染确保用户按执行顺序实时看到每个阶段
