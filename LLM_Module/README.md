# LLM_Module

## 作用

`LLM_Module` 负责把用户指令变成可执行技能链。
当前主链路是：

`user_input -> highlevel plan -> parameter calculation -> lowlevel execution -> tool result`

失败时不再重规划，当前策略是直接停止后续任务。

## 主要文件

- `llm_core.py`：总控，串联规划、参数计算、执行
- `llm_highlevel.py`：高层规划，输出任务序列
- `llm_lowlevel.py`：低层执行，调用工具
- `object_facts_loader.py`：读取并归一化 `object_facts.json`
- `parameter_calculator.py`：把抽象任务补成具体参数

## 当前输入

- `user_input`：用户自然语言指令
- `visual_context`：VLM 输出的结构化视觉描述
- `scene_facts`：由 VLM 和规则层整理出的场景事实
- `object_facts`：外部几何真值，优先级高于 VLM
- `tools`：当前可调用技能列表

## 当前输出

- 高层输出：任务序列 `tasks`
- 参数层输出：`calculated_parameters`
- 低层输出：工具调用结果与执行反馈

## 当前行为

- 高层只规划一次，不接受 `replan_context`
- 低层若看到 `calculated_parameters`，优先直接执行，不再重新猜参数
- 技能执行反馈默认等待 `20 秒`
- 支持的核心场景：
  - 单侧可攀爬：`way_select -> climb`
  - 箱子辅助攀爬：`push_box -> climb_align -> climb -> navigation`

## 提示词

- `prompts/highlevel_prompt.yaml`：高层规划提示词
- `prompts/lowlevel_prompt.yaml`：低层执行提示词

## 依赖

- `openai`
- `pyyaml`
- 项目根目录 `.env` 中的 `Test_API_KEY`

## 入口

通常由 `Interactive_Module/interactive.py` 调用：

```bash
python3 Interactive_Module/interactive.py
```
