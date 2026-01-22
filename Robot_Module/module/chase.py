"""
追击模块 (Chase Module)

实现机器人自动追击目标的功能
"""

import sys
import json
import asyncio
from typing import Dict, Optional

# 全局变量（懒加载）
_action_queue = None
_robot_getter = None


def _get_action_queue():
    """获取动作队列（懒加载）"""
    global _action_queue
    if _action_queue is None:
        from pathlib import Path
        project_root = Path(__file__).parent.parent.parent
        sys.path.insert(0, str(project_root))
        from ros_topic_comm import get_shared_queue
        _action_queue = get_shared_queue()
        print("[chase.py] ROS队列已初始化", file=sys.stderr)
    return _action_queue


async def _move_forward(distance: float = 1.0, speed: float = 0.3):
    """向前移动（内部函数）"""
    action = {
        'action': 'move_forward',
        'parameters': {'distance': distance, 'speed': speed}
    }
    _get_action_queue().put(action)
    # 等待移动完成
    await asyncio.sleep(distance / speed if speed > 0 else 1)


async def _turn(angle: float = 90.0, angular_speed: float = 0.5):
    """旋转（内部函数）"""
    action = {
        'action': 'turn',
        'parameters': {'angle': angle, 'angular_speed': angular_speed}
    }
    _get_action_queue().put(action)
    # 等待旋转完成（估算时间）
    await asyncio.sleep(abs(angle) / 180.0 * 3.14159 / angular_speed if angular_speed > 0 else 1)


def _get_robot_position() -> Optional[Dict]:
    """
    获取机器人当前位置

    注意：这需要从仿真器获取，目前返回模拟值
    实际使用时需要实现ROS话题订阅或共享内存
    """
    # TODO: 实现从仿真器获取真实位置
    # 目前返回中心位置作为示例
    return {'x': 400.0, 'y': 300.0, 'angle': 0.0}


# ==============================================================================
# MCP 注册函数
# ==============================================================================

def register_tools(mcp):
    """
    注册追击相关的工具函数到 MCP 服务器

    Args:
        mcp: FastMCP 服务器实例

    Returns:
        工具函数字典 {name: function}
    """

    @mcp.tool()
    async def chase_target(
        target_x: float,
        target_y: float,
        threshold: float = 20.0,
        step_distance: float = 0.5
    ) -> str:
        """自动追击指定坐标的目标

        机器人会自动计算目标角度和距离，执行旋转和前进动作，直到接近目标。

        Args:
            target_x: 目标X坐标（像素）
            target_y: 目标Y坐标（像素）
            threshold: 到达阈值（像素），默认20像素，距离小于此值视为到达
            step_distance: 每步移动距离（米），默认0.5米

        Returns:
            执行结果JSON字符串，包含状态、步数、最终距离等信息

        Examples:
            chase_target(target_x=700, target_y=300)  # 追击坐标(700, 300)的目标
            chase_target(target_x=100, target_y=100, threshold=30)  # 自定义阈值
        """
        print(f"[chase.chase_target] 开始追击: ({target_x}, {target_y})",
              file=sys.stderr)

        # 导入追击控制器
        project_root = Path(__file__).parent.parent.parent
        sys.path.insert(0, str(project_root))
        from Test_Module.chase_core import ChaseController

        # 创建控制器
        controller = ChaseController(
            robot_getter=_get_robot_position,
            move_forward_fn=_move_forward,
            turn_fn=_turn
        )

        # 设置参数
        controller.arrival_threshold = threshold
        controller.step_distance = step_distance

        # 执行追击
        result = await controller.chase_target(
            {'x': target_x, 'y': target_y},
            progress_callback=None
        )

        print(f"[chase.chase_target] 追击完成: {result.get('message', '')}",
              file=sys.stderr)

        return json.dumps(result, ensure_ascii=False)

    @mcp.tool()
    async def chase_nearest_enemy(enemy_positions: str) -> str:
        """追击最近的敌人

        从给定的敌人列表中选择最近的一个进行追击。

        Args:
            enemy_positions: 敌人位置列表JSON字符串，格式为：
                           [{"id": "1", "x": 100, "y": 200}, ...]

        Returns:
            执行结果JSON字符串

        Examples:
            positions = '[{"id": "1", "x": 100, "y": 200}, {"id": "2", "x": 500, "y": 400}]'
            chase_nearest_enemy(positions)
        """
        print(f"[chase.chase_nearest_enemy] 分析敌人位置", file=sys.stderr)

        # 解析敌人位置
        try:
            enemies = json.loads(enemy_positions)
        except json.JSONDecodeError as e:
            return json.dumps({
                "success": False,
                "error": f"JSON解析错误: {e}"
            }, ensure_ascii=False)

        # 获取机器人位置
        robot_pos = _get_robot_position()
        if not robot_pos:
            return json.dumps({
                "success": False,
                "error": "无法获取机器人位置"
            }, ensure_ascii=False)

        # 找到最近的敌人
        import math
        nearest = None
        min_dist = float('inf')

        for enemy in enemies:
            dist = math.sqrt(
                (enemy['x'] - robot_pos['x'])**2 +
                (enemy['y'] - robot_pos['y'])**2
            )
            if dist < min_dist:
                min_dist = dist
                nearest = enemy

        if not nearest:
            return json.dumps({
                "success": False,
                "error": "没有找到敌人"
            }, ensure_ascii=False)

        print(f"[chase.chase_nearest_enemy] 最近的敌人: {nearest['id']} "
              f"at ({nearest['x']}, {nearest['y']})", file=sys.stderr)

        # 追击最近的敌人
        result = await chase_target(nearest['x'], nearest['y'])

        # 添加敌人ID信息
        result_dict = json.loads(result)
        result_dict['enemy_id'] = nearest['id']

        return json.dumps(result_dict, ensure_ascii=False)

    @mcp.tool()
    async def calculate_chase_angle(
        robot_x: float,
        robot_y: float,
        robot_angle: float,
        target_x: float,
        target_y: float
    ) -> str:
        """计算追击所需的角度（辅助工具）

        计算从机器人当前位置朝向目标所需旋转的角度。

        Args:
            robot_x: 机器人X坐标
            robot_y: 机器人Y坐标
            robot_angle: 机器人当前角度（度）
            target_x: 目标X坐标
            target_y: 目标Y坐标

        Returns:
            计算结果JSON字符串，包含目标角度、旋转角度差、距离等信息

        Examples:
            calculate_chase_angle(100, 300, 0, 700, 300)
            # 返回: {"target_angle": 0, "angle_diff": 0, "distance": 600}
        """
        # 导入计算函数
        project_root = Path(__file__).parent.parent.parent
        sys.path.insert(0, str(project_root))
        from Test_Module.chase_core import ChaseController

        controller = ChaseController()

        # 计算目标角度
        target_angle = controller.calculate_target_angle(
            robot_x, robot_y, target_x, target_y
        )

        # 计算角度差
        angle_diff = controller.calculate_angle_difference(
            robot_angle, target_angle
        )

        # 计算距离
        distance = controller.calculate_distance(
            robot_x, robot_y, target_x, target_y
        )

        result = {
            "target_angle": round(target_angle, 2),
            "angle_diff": round(angle_diff, 2),
            "distance_pixels": round(distance, 2),
            "distance_meters": round(distance / 100.0, 2),
            "direction": "左转" if angle_diff > 0 else "右转" if angle_diff < 0 else "保持"
        }

        print(f"[chase.calculate_chase_angle] 目标角度: {target_angle:.1f}°, "
              f"旋转: {angle_diff:.1f}°, 距离: {distance:.1f}px",
              file=sys.stderr)

        return json.dumps(result, ensure_ascii=False)

    print("[chase.py:register_tools] 追击模块已注册 (3 个工具)", file=sys.stderr)

    return {
        'chase_target': chase_target,
        'chase_nearest_enemy': chase_nearest_enemy,
        'calculate_chase_angle': calculate_chase_angle
    }
