# LLM_Module

## 作用

`LLM_Module` 负责把用户指令变成可执行技能链。
当前主链路是：

`user_input -> high-level plan -> parameter calculation -> low-level execution -> result assessment`

失败时最多重规划 1 次（`max_replans=1`），仍失败则停止后续任务。

## 主要文件

- `llm_core.py`
  总控，包含 `HighLevelPlanner` 和 `LLMAgent`
- `llm_lowlevel.py`
  低层执行，调用已注册工具
- `object_facts_loader.py`
  读取并归一化 `object_facts.json`
- `parameter_calculator.py`
  把抽象任务补成具体技能参数

## 当前输入

- `user_input`
- `visual_context`
- `scene_facts`
- `object_facts`
- `tools`

## 当前输出

- 规划输出：任务序列 `tasks`
- 参数层输出：`parameter_context` 和 `calculated_parameters`
- 执行输出：技能结果、`execution_feedback`、`validation`

## 当前行为

- 规划层失败时会重规划最多 1 次（`max_replans=1`）
- 低层若看到 `calculated_parameters`，优先直接执行，不再重新猜参数
- 当前以通用动作为主，不在高层写死特定任务的执行协议
- 技能真正执行已经由 `Excu_Module` 负责
- `llm_core.py` 不再只依赖 `signal == SUCCESS`
- 当前会读取动作返回的 `validation`，判断动作是否真的满足任务要求
- 如果拿不到真实执行数据，动作会被判失败并触发重规划或中止

## 成功判定

当前成功判定分两层：

1. `Robot_Module + Excu_Module` 返回结构化执行结果
2. `LLM_Module/llm_core.py` 调用 `_assess_execution_result()` 做最终判断

重点看这两个字段：

- `execution_feedback.validation.verified`
- `execution_feedback.validation.meets_requirements`

只有真实状态校验通过，`LLM_Module` 才会继续执行下一步。

## 与 parameter_calculator 的关系

`parameter_calculator.py` 是当前最关键的桥接层。

它的职责是：

- 把高层任务翻译成技能参数
- 尽量把低层执行变成"直接调用"
- 让技能文件只负责接收参数和调用执行层

当前推荐分工：

- 高层规划决定技能序列
- 参数计算决定具体参数
- 技能文件把参数填入函数
- `Excu_Module` 执行并验收

当前 `push_box` 参数计算：

- `_build_adjacent_ground_position()` 计算箱子目标位置
- 只改变 x 方向（箱子推到 platform 前沿），y 保持箱子当前值
- `calculated_parameters["target_position"]` 是具体 `[x, y, z]` 坐标，不再是 `"auto"`
- 这样 `Excu_Module` 可以用到达容许误差（0.1m）判定箱子是否到达目标

## 提示词

- `prompts/highlevel_prompt.yaml`
- `prompts/lowlevel_prompt.yaml`

## 依赖

- `openai`
- `pyyaml`
- 项目根目录 `.env` 中的 `Test_API_KEY`

## 入口

通常由 `Interactive_Module/interactive.py` 通过 `robot_act` 间接调用：

```bash
python3 Interactive_Module/interactive.py
```

## stdout 输出

`llm_core.py` 和 `llm_lowlevel.py` 的 print 输出（████ 规划块、⚙️🔧 执行信息等）通过 `agent_tools.py` 的 `_StreamingBuffer` 机制捕获，经 `log_callback` 转发到交互终端实时显示。

当前典型输出变化：

- 低层仍会打印 `📨 [执行反馈] ...`
- 上层不再打印“已返回成功信号”
- 现在会打印 `✅ [结果校验] ...`，内容来自真实状态校验摘要
