# Remove Replanning And Increase Feedback Timeout Design

## Goal

移除当前执行链路中的失败后重规划逻辑，让任务在单步失败时直接停止，并把等待执行反馈的默认超时时间从 10 秒统一提升到 20 秒。

## Scope

本次只处理两类行为变化：

1. 删除总控、高层接口、prompt、文档中的重规划能力和相关字段。
2. 将执行反馈等待默认超时由 10 秒改为 20 秒，并同步更新文档。

不修改 `object_facts`、`scene_facts`、参数计算、VLM 结构化输出格式。

## Design

### 1. 执行流

当前 [llm_core.py](/home/robot/work/FinalProject/LLM_Module/llm_core.py) 在技能失败后会构造 `replan_context`，再次调用高层规划器，并在新任务链上继续执行。

这次改为：

- 高层只产出一次任务链。
- 执行层顺序执行。
- 任一步骤失败后立即停止后续任务。
- 保留已有的执行反馈打印和任务总结。

这样做的原因很直接：

- 当前 demo 目标是稳定验证 `VLM/object_facts -> 规划 -> 参数计算 -> 技能执行` 主链路。
- 失败后重规划增加了额外状态和 prompt 复杂度，但对当前验证收益不高。
- 用户明确要求移除这一层行为。

### 2. 高层接口与 Prompt

当前高层提示词和接口仍暴露 `replan_context` 占位和说明。

这次改为：

- 删除 [interactive.py](/home/robot/work/FinalProject/Interactive_Module/interactive.py) 动态 prompt 注入中的 `replan_context`
- 删除 [llm_highlevel.py](/home/robot/work/FinalProject/LLM_Module/llm_highlevel.py) 中的 `replan_context` 参数和 prompt 格式化
- 删除 [highlevel_prompt.yaml](/home/robot/work/FinalProject/LLM_Module/prompts/highlevel_prompt.yaml) 中所有重规划描述
- 删除 [lowlevel_prompt.yaml](/home/robot/work/FinalProject/LLM_Module/prompts/lowlevel_prompt.yaml) 中“重规划后的替代步骤”规则

这样可以把高层职责收敛回：

- 只根据当前输入做一次规划
- 不再接受失败上下文

### 3. 超时统一到 20 秒

当前执行反馈等待默认值在两处：

- [execution_comm.py](/home/robot/work/FinalProject/Comm_Module/execution_comm.py) 中的 `wait_for_execution_feedback(... timeout_sec=10.0)`
- [navigation_demo.py](/home/robot/work/FinalProject/Robot_Module/module/navigation_demo.py) 中技能等待反馈默认 `timeout_sec=10.0`

这次改为统一 20 秒。

原因：

- demo 环境下人工确认、ROS 通信和执行反馈可能超过 10 秒
- 20 秒比 10 秒更稳，但仍然足够短，不至于把失败等待拉得过长

### 4. README

[README.md](/home/robot/work/FinalProject/README.md) 目前已经更新为较完整的链路说明，但其中仍然写着“一次重规划”。

这次会同步改成：

- 主链路不再包含重规划
- 失败处理为“停止后续任务”
- 等待反馈默认超时为 20 秒

### 5. Testing

本次行为改动需要覆盖这两类测试：

1. 移除重规划后，执行失败应立即停止。
2. 默认超时值应为 20 秒。

已有重规划测试要改写，不再断言第二次 `plan_tasks()` 调用，而是断言：

- 失败后没有再次规划
- 结果数量符合“执行到失败步骤即停止”

## Files

- Modify: [LLM_Module/llm_core.py](/home/robot/work/FinalProject/LLM_Module/llm_core.py)
- Modify: [LLM_Module/llm_highlevel.py](/home/robot/work/FinalProject/LLM_Module/llm_highlevel.py)
- Modify: [Interactive_Module/interactive.py](/home/robot/work/FinalProject/Interactive_Module/interactive.py)
- Modify: [LLM_Module/prompts/highlevel_prompt.yaml](/home/robot/work/FinalProject/LLM_Module/prompts/highlevel_prompt.yaml)
- Modify: [LLM_Module/prompts/lowlevel_prompt.yaml](/home/robot/work/FinalProject/LLM_Module/prompts/lowlevel_prompt.yaml)
- Modify: [Comm_Module/execution_comm.py](/home/robot/work/FinalProject/Comm_Module/execution_comm.py)
- Modify: [Robot_Module/module/navigation_demo.py](/home/robot/work/FinalProject/Robot_Module/module/navigation_demo.py)
- Modify: [README.md](/home/robot/work/FinalProject/README.md)
- Modify: [tests/test_llm_replan.py](/home/robot/work/FinalProject/tests/test_llm_replan.py)
- Add or modify timeout coverage in tests as needed
