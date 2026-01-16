"""
底盘控制模块 (Base Control Module)

负责机器人的底盘移动和旋转功能。
"""

import sys
import json

# 全局动作队列（用于与仿真器通信）
_action_queue = None


def _get_action_queue():
    """获取动作队列（懒加载：首次使用时自动初始化 ROS 队列）"""
    global _action_queue
    if _action_queue is None:
        from pathlib import Path
        project_root = Path(__file__).parent.parent.parent
        sys.path.insert(0, str(project_root))
        from ros_topic_comm import get_shared_queue
        _action_queue = get_shared_queue()
        print("[base.py] ROS队列已自动初始化", file=sys.stderr)
    return _action_queue


# ==============================================================================
# MCP 注册函数
# ==============================================================================

def register_tools(mcp):
    """
    注册底盘相关的所有工具函数到 MCP 服务器

    Args:
        mcp: FastMCP 服务器实例

    Returns:
        工具函数字典 {name: function}
    """

    @mcp.tool()
    async def move_forward(distance: float = 1.0, speed: float = 0.3) -> str:
        """向前移动指定距离

        机器人沿当前朝向向前移动。

        Args:
            distance: 移动距离（米），默认1.0米
            speed: 移动速度（米/秒），默认0.3米/秒

        Returns:
            动作指令JSON字符串

        Examples:
            move_forward(distance=2.0, speed=0.5)  # 前进2米，速度0.5m/s
            move_forward(distance=1.0)  # 前进1米，默认速度
        """
        print(f"[base.move_forward] 前进 {distance}m, 速度 {speed}m/s", file=sys.stderr)

        action = {
            'action': 'move_forward',
            'parameters': {'distance': distance, 'speed': speed}
        }

        # 发送到仿真器
        _get_action_queue().put(action)

        return json.dumps(action, ensure_ascii=False)

    @mcp.tool()
    async def move_backward(distance: float = 1.0, speed: float = 0.3) -> str:
        """向后移动指定距离

        机器人沿当前朝向向后移动。

        Args:
            distance: 移动距离（米），默认1.0米
            speed: 移动速度（米/秒），默认0.3米/秒

        Returns:
            动作指令JSON字符串

        Examples:
            move_backward(distance=1.5, speed=0.4)  # 后退1.5米，速度0.4m/s
        """
        print(f"[base.move_backward] 后退 {distance}m, 速度 {speed}m/s", file=sys.stderr)

        action = {
            'action': 'move_backward',
            'parameters': {'distance': distance, 'speed': speed}
        }

        # 发送到仿真器
        _get_action_queue().put(action)

        return json.dumps(action, ensure_ascii=False)

    @mcp.tool()
    async def turn(angle: float = 90.0, angular_speed: float = 0.5) -> str:
        """旋转指定角度

        机器人原地旋转指定角度。正值为左转（逆时针），负值为右转（顺时针）。

        Args:
            angle: 旋转角度（度），正值为左转（逆时针），负值为右转（顺时针），默认90.0度
            angular_speed: 角速度（弧度/秒），默认0.5弧度/秒

        Returns:
            动作指令JSON字符串

        Examples:
            turn(angle=90.0)  # 左转90度
            turn(angle=-45.0, angular_speed=0.8)  # 右转45度，角速度0.8rad/s
        """
        direction = "左转" if angle > 0 else "右转"
        print(f"[base.turn] {direction} {abs(angle)}°, 角速度 {angular_speed}rad/s", file=sys.stderr)

        action = {
            'action': 'turn',
            'parameters': {'angle': angle, 'angular_speed': angular_speed}
        }

        # 发送到仿真器
        _get_action_queue().put(action)

        return json.dumps(action, ensure_ascii=False)

    @mcp.tool()
    async def stop() -> str:
        """紧急停止机器人

        立即停止机器人所有运动。

        Returns:
            动作指令JSON字符串

        Examples:
            stop()  # 立即停止
        """
        print("[base.stop] 停止机器人", file=sys.stderr)

        action = {
            'action': 'stop',
            'parameters': {}
        }

        # 发送到仿真器
        _get_action_queue().put(action)

        return json.dumps(action, ensure_ascii=False)

    print("[base.py:register_tools] 底盘控制模块已注册 (4 个工具)", file=sys.stderr)

    # 返回工具函数字典
    return {
        'move_forward': move_forward,
        'move_backward': move_backward,
        'turn': turn,
        'stop': stop
    }
