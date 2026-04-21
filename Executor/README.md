# Executor

## 作用

`Executor` 负责技能注册、技能执行和状态管理。

## 文件

```text
__init__.py
state.py
skills.py
tools.py
executor.py
demo_executor.py
```

## state.py

保存假状态：

```python
state = {
    "robot": {"x": 0, "y": 0, "z": 0},
    "box": {"x": 0, "y": 1, "z": 0},
    "last_action": None,
}
```

提供：

- `reset_state()`
- `fmt_robot()`
- `fmt_box()`

## skills.py

手动实现技能：

```text
Nav(x, y, z)       0.5m/s 直线导航
walk(direction, v) 按速度 v 移动
Push(x, y, z)      0.2m/s 推箱子
climb(height)      0.1m/s 攀爬，最高 0.3m
```

技能每 `0.2s` 更新一次状态，约等于 `5Hz`。

所有技能支持 `emit` 参数，用于实时输出状态。

## tools.py

用 FastMCP 注册 4 个技能：

- `@mcp.tool()` 装饰器注册技能到 MCP
- `get_tool_definitions()` 返回 OpenAI function calling 格式的工具定义
- `call_tool(name, args, emit)` 根据工具名调用对应技能

## executor.py

`run_plan(tool_calls, emit)` 接收 LLM 返回的 tool_calls 列表，逐个执行：

- 每步 emit `tool` 事件显示当前步骤
- 调用 `call_tool` 执行技能
- 失败时 emit `error` 并停止

## demo_executor.py

`/demo` 命令使用，固定步骤执行：

```text
Nav(0, 1, 0) -> Push(0, 2, 0) -> climb(0.3) -> Nav(0, 8, 0)
```

不调用 LLM，用于本地测试。

## 当前边界

`Executor` 不直接渲染 TUI，也不调用 LLM。

后续真实机器人接入时，可以保留技能函数名，替换内部实现。
