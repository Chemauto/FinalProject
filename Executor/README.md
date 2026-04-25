# Executor

## 作用

`Executor` 负责技能注册、技能执行和状态管理。

## 文件

```text
__init__.py
robot_ws.py
state.py
skills.py
tools.py
executor.py
```

## state.py

保存服务器传来的最新状态和反馈：

```python
state = {
    "robot": {"x": 0, "y": 0, "z": 0},
    "box": {"x": 0, "y": 1, "z": 0},
    "current_skill": None,
    "last_action": None,
    "latest_feedback": None,
}
```

提供：

- `reset_state()`
- `update_latest_state(payload)`
- `update_latest_feedback(payload)`
- `fmt_robot()`
- `fmt_box()`
- `format_latest_state()`
- `format_feedback()`

## robot_ws.py

负责和机器人服务器通信：

- `send_skill_command(skill, args, emit)` 发送技能启动信号
- 接收 `state` 消息并更新 `Executor.state`
- 接收同一 `action_id` 的 `feedback` 后返回
- `ROBOT_WS_URL` 默认 `ws://127.0.0.1:8765`

## skills.py

技能函数只负责发送启动信号：

```text
Nav(x, y, z)       发送 nav 命令
walk(direction, v) 发送 walk_skill 命令
Push(x, y, z)      发送 push 命令
climb(height)      发送 climb 命令，客户端保留最高 0.3m 约束
```

动作是否成功由服务器 `feedback.signal` 决定。

## tools.py

用 FastMCP 注册观察和动作技能：

- `@mcp.tool()` 装饰器注册技能到 MCP
- `get_tool_definitions()` 返回 OpenAI function calling 格式的工具定义
- `call_tool(name, args, emit)` 根据工具名调用对应技能

## executor.py

`run_plan(tool_calls, emit)` 接收 LLM 返回的 tool_calls 列表，逐个执行：

- 每步 emit `tool` 事件显示当前步骤
- 调用 `call_tool` 执行技能
- 失败时 emit `error` 并停止
- 返回每一步的结构化结果，供 TUI 喂回 LLM


不调用 LLM，用于本地测试。

## 当前边界

`Executor` 不直接渲染 TUI，也不调用 LLM。

真实机器人闭环在服务器端完成，Executor 只发命令、收状态、收反馈。
