# Planner

## 作用

`Planner` 负责 LLM 调用、提示词配置和任务规划。

## 文件

```text
llm_core.py
prompts/planner_prompt.yaml
```

## llm_core.py

负责：

- 读取 `.env`
- 读取 `planner_prompt.yaml`
- `chat(messages, tools=None)`：调用 LLM，支持 tools 参数，返回 tool_calls 或文本
- `stream_chat(messages)`：流式调用 LLM
- `make_plan(messages)`：带工具定义调 LLM，返回计划或文本

`make_plan` 接收 session 的 messages（含完整对话历史），从 `Executor/tools` 获取 tool definitions 传给 LLM。LLM 返回 tool_calls 时返回计划，否则返回文本。

如果 LLM 返回 tool_calls，`make_plan` 会保留模型正文中的规划说明，TUI 将其显示为 `Plan`。

## prompts/planner_prompt.yaml

保存：

- `model`
- `system_prompt`
- `user_prompt`

当前默认模型是：

```text
glm-5.1
```

提示词只写通用规则：

- 新动作任务先 `observe`
- 坐标和物体位置优先参考 `observe` 返回的 `robot_state` 和 ROS2 结构化物体
- 工具执行结果以 `feedback` 为准

具体技能用途写在 `Executor/tools.py` 的 tool description 中，不在 prompt 里单独硬编码。

## 当前边界

`Planner` 只负责 LLM 调用和规划。

它不负责：

- 技能执行（在 `Executor`）
- TUI 显示（在 `Tui`）
- 状态反馈（在 `Executor`）
- ROS2 对接
