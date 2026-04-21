# Executor

## 作用

`Executor` 目前负责手动技能模拟和 demo 执行。

它现在不是真实机器人执行层，而是一个用于验证 TUI 事件链路的假执行层。

## 文件

```text
__init__.py
state.py
skills.py
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

`sleep=False` 可用于测试时跳过真实等待。

## demo_executor.py

`/demo` 会调用：

```python
run_demo_task(emit)
```

当前 demo 步骤：

```text
Nav(0, 1, 0)
Push(box, 0, 2, 0)
climb(0.3)
Nav(0, 8, 0)
```

执行期间通过 `emit(type, content)` 向 TUI 发送：

```text
plan
tool
status
error
```

## 当前边界

`Executor` 不直接渲染 TUI，也不调用 LLM。

后续真实机器人接入时，可以保留技能函数名，替换内部实现。
