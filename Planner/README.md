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

## prompts/planner_prompt.yaml

保存：

- `model`
- `system_prompt`
- `user_prompt`

当前默认模型是：

```text
glm-5.1
```

## 当前边界

`Planner` 只负责 LLM 调用和规划。

它不负责：

- 技能执行（在 `Executor`）
- TUI 显示（在 `Tui`）
- 状态反馈（在 `Executor`）
- ROS2 对接
