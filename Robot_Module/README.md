# Robot_Module

## 作用

`Robot_Module/skill.py` 是机器人工具注册入口。
它统一注册底盘控制、追击和视觉相关工具，并把动作发送给后续模块执行。

## 依赖

- `fastmcp`
- 项目内的 `ros_topic_comm.py`
- `LLM_Module` 或 `Interactive_Module` 会调用这里的工具

## 使用

通常不单独运行，主要由上层模块导入：

```python
from Robot_Module.skill import register_all_modules, get_tool_definitions
```

## 输入输出

- 输入：工具调用请求和参数
- 输出：对应工具结果，以及机器人动作指令
