"""
底盘控制模块 (Base Control Module)

负责机器人的底盘移动和旋转功能。

Functions:
    - move_forward: 向前移动指定距离
    - move_backward: 向后移动指定距离
    - turn: 旋转指定角度
    - stop: 紧急停止
"""

import sys
import json
import inspect
from multiprocessing import Queue

# 全局动作队列（用于与仿真器通信）
_action_queue = None


def set_action_queue(queue: Queue = None):
    """设置全局动作队列（如果不提供，则使用共享队列）"""
    global _action_queue

    # 如果没有提供队列，使用共享队列
    if queue is None:
        _action_queue = _get_shared_queue()
    else:
        _action_queue = queue

    print("[base.py] 动作队列已设置", file=sys.stderr)


def get_action_queue() -> Queue:
    """获取全局动作队列"""
    return _action_queue


def _get_shared_queue():
    """获取共享队列"""
    from pathlib import Path

    # 添加项目根目录到路径
    project_root = Path(__file__).parent.parent.parent
    sys.path.insert(0, str(project_root))

    from shared_queue import get_shared_queue
    return get_shared_queue()


# ==============================================================================
# 工具函数实现
# ==============================================================================

async def move_forward(distance: float = 1.0, speed: float = 0.3) -> str:
    """向前移动指定距离

    机器人沿当前朝向向前移动。

    Args:
        distance: 移动距离（米），默认1.0米
        speed: 移动速度（米/秒），默认0.3米/秒

    Returns:
        动作指令JSON字符串
    """
    print(f"[base.move_forward] 前进 {distance}m, 速度 {speed}m/s", file=sys.stderr)

    action = {
        'action': 'move_forward',
        'parameters': {'distance': distance, 'speed': speed}
    }

    # 发送到仿真器（如果队列可用）
    if _action_queue:
        _action_queue.put(action)

    return json.dumps(action, ensure_ascii=False)


async def move_backward(distance: float = 1.0, speed: float = 0.3) -> str:
    """向后移动指定距离

    机器人沿当前朝向向后移动。

    Args:
        distance: 移动距离（米），默认1.0米
        speed: 移动速度（米/秒），默认0.3米/秒

    Returns:
        动作指令JSON字符串
    """
    print(f"[base.move_backward] 后退 {distance}m, 速度 {speed}m/s", file=sys.stderr)

    action = {
        'action': 'move_backward',
        'parameters': {'distance': distance, 'speed': speed}
    }

    # 发送到仿真器（如果队列可用）
    if _action_queue:
        _action_queue.put(action)

    return json.dumps(action, ensure_ascii=False)


async def turn(angle: float = 90.0, angular_speed: float = 0.5) -> str:
    """旋转指定角度

    机器人原地旋转指定角度。正值为左转（逆时针），负值为右转（顺时针）。

    Args:
        angle: 旋转角度（度），正值为左转（逆时针），负值为右转（顺时针），默认90.0度
        angular_speed: 角速度（弧度/秒），默认0.5弧度/秒

    Returns:
        动作指令JSON字符串
    """
    direction = "左转" if angle > 0 else "右转"
    print(f"[base.turn] {direction} {abs(angle)}°, 角速度 {angular_speed}rad/s", file=sys.stderr)

    action = {
        'action': 'turn',
        'parameters': {'angle': angle, 'angular_speed': angular_speed}
    }

    # 发送到仿真器（如果队列可用）
    if _action_queue:
        _action_queue.put(action)

    return json.dumps(action, ensure_ascii=False)


async def stop() -> str:
    """紧急停止机器人

    立即停止机器人所有运动。

    Returns:
        动作指令JSON字符串
    """
    print("[base.stop] 停止机器人", file=sys.stderr)

    action = {
        'action': 'stop',
        'parameters': {}
    }

    # 发送到仿真器（如果队列可用）
    if _action_queue:
        _action_queue.put(action)

    return json.dumps(action, ensure_ascii=False)


# ==============================================================================
# MCP 注册函数
# ==============================================================================

def _extract_tool_metadata(func):
    """从函数提取工具元数据（OpenAI function calling 格式）"""
    name = func.__name__
    doc = inspect.getdoc(func) or ""

    # 解析函数签名
    sig = inspect.signature(func)
    parameters = {}
    required = []

    for param_name, param in sig.parameters.items():
        param_type = "string"
        if param.annotation != inspect.Parameter.empty:
            if param.annotation == int:
                param_type = "number"
            elif param.annotation == float:
                param_type = "number"
            elif param.annotation == bool:
                param_type = "boolean"

        param_info = {"type": param_type}

        # 从 docstring 中提取参数描述
        if "Args:" in doc:
            for line in doc.split("Args:")[1].split("\n"):
                if f"{param_name}:" in line:
                    parts = line.split(":", 1)
                    if len(parts) == 2:
                        param_desc = parts[1].strip()
                        if param_desc:
                            param_info["description"] = param_desc
                    break

        parameters[param_name] = param_info

        # 检查是否是必需参数
        if param.default == inspect.Parameter.empty:
            required.append(param_name)

    # 构建工具元数据
    metadata = {
        "description": doc.split("\n\n")[0].split("\n")[0] if doc else "",
        "parameters": {
            "type": "object",
            "properties": parameters,
            "required": required
        }
    }

    return name, metadata


def register_tools(mcp, tool_registry=None, tool_metadata=None):
    """
    注册底盘相关的所有工具函数到 MCP 服务器

    使用方法：
        1. 编写异步函数（带完整 docstring）
        2. 在此函数中添加：mcp.tool()(your_function)

    Args:
        mcp: FastMCP 服务器实例
        tool_registry: 工具函数注册表（可选）
        tool_metadata: 工具元数据注册表（可选）
    """
    # 要注册的工具函数列表
    tools = [move_forward, move_backward, turn, stop]

    for func in tools:
        # 注册到 FastMCP
        mcp.tool()(func)

        # 提取并存储元数据（用于 LLM function calling）
        if tool_registry is not None and tool_metadata is not None:
            name, metadata = _extract_tool_metadata(func)
            tool_registry[name] = func
            tool_metadata[name] = metadata

    print(f"[base.py:register_tools] 底盘控制模块已注册 ({len(tools)} 个工具)", file=sys.stderr)
