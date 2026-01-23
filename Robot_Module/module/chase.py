"""
追击模块 (Chase Module)

实现机器人自动追击目标的功能
提供2个 MCP 工具函数：
1. get_enemy_positions - 获取敌人位置
2. chase_enemy - 追击最近的敌人（自动完成整个流程）
"""

import sys
import json
import asyncio
from pathlib import Path
from typing import Dict, Optional

# 全局变量（懒加载）
_action_queue = None

# 追击参数配置
MAX_STEP_DISTANCE = 1.0  # 最大步长（米）
ARRIVAL_THRESHOLD = 5.0  # 到达阈值（像素）


def _get_action_queue():
    """获取动作队列（懒加载）"""
    global _action_queue
    if _action_queue is None:
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
    await asyncio.sleep(distance / speed if speed > 0 else 1)


async def _turn(angle: float = 90.0, angular_speed: float = 0.5):
    """旋转（内部函数）"""
    action = {
        'action': 'turn',
        'parameters': {'angle': angle, 'angular_speed': angular_speed}
    }
    _get_action_queue().put(action)
    await asyncio.sleep(abs(angle) / 180.0 * 3.14159 / angular_speed if angular_speed > 0 else 1)


def _get_robot_position() -> Optional[Dict]:
    """从 ROS 订阅器获取机器人当前位置"""
    project_root = Path(__file__).parent.parent.parent
    sys.path.insert(0, str(project_root))
    from ros_topic_comm import get_robot_state

    state = get_robot_state()
    if state:
        return {
            'x': state.get('x', 400.0),
            'y': state.get('y', 300.0),
            'angle': state.get('angle', 0.0)
        }
    return {'x': 400.0, 'y': 300.0, 'angle': 0.0}


def _calculate_step_distance(current_distance_pixels: float) -> float:
    """使用PID控制计算步长"""
    distance_meters = current_distance_pixels / 100.0
    step_distance = min(distance_meters * 0.8, MAX_STEP_DISTANCE)
    return max(step_distance, 0.1)


# ==============================================================================
# MCP 工具注册
# ==============================================================================

def register_tools(mcp):
    """注册追击相关的工具函数到 MCP 服务器"""

    @mcp.tool()
    async def get_enemy_positions() -> str:
        """获取当前仿真器中的所有敌人位置

        从仿真器获取当前所有敌人的位置信息，用于追击或任务规划。

        Returns:
            敌人位置列表JSON字符串，格式为：
            [{"id": "1", "x": 100, "y": 200}, {"id": "2", "x": 500, "y": 400}]

        Examples:
            get_enemy_positions()
        """
        project_root = Path(__file__).parent.parent.parent
        sys.path.insert(0, str(project_root))
        from ros_topic_comm import get_enemy_positions as get_positions

        # 等待并重试
        positions = []
        for attempt in range(5):
            positions = get_positions()
            if len(positions) > 0:
                break
            print(f"[chase.get_enemy_positions] 第{attempt+1}次尝试: 未收到敌人位置，等待...", file=sys.stderr)
            await asyncio.sleep(0.2)

        print(f"[chase.get_enemy_positions] 最终获取到 {len(positions)} 个敌人", file=sys.stderr)
        return json.dumps(positions, ensure_ascii=False)

    @mcp.tool()
    async def chase_enemy(enemy_positions: str) -> str:
        """追击最近的敌人

        从给定的敌人列表中选择最近的一个进行追击。
        这个工具会：
        1. 找到距离最近的敌人
        2. 旋转到目标方向
        3. 前进到目标位置
        4. 清除已追击的敌人

        Args:
            enemy_positions: 敌人位置列表JSON字符串，格式为：
                           [{"id": "1", "x": 100, "y": 200}, ...]

        Returns:
            执行结果JSON字符串

        Examples:
            positions = '[{"id": "1", "x": 100, "y": 200}, {"id": "2", "x": 500, "y": 400}]'
            chase_enemy(positions)
        """
        print(f"[chase.chase_enemy] 开始追击敌人", file=sys.stderr)

        # 解析敌人位置
        try:
            positions = json.loads(enemy_positions)
        except json.JSONDecodeError as e:
            return json.dumps({
                "success": False,
                "error": f"JSON解析错误: {e}"
            }, ensure_ascii=False)

        print(f"[chase.chase_enemy] 接收到 {len(positions)} 个敌人", file=sys.stderr)

        if not positions:
            return json.dumps({
                "success": False,
                "error": "没有找到敌人，请先在仿真器中生成敌人"
            }, ensure_ascii=False)

        project_root = Path(__file__).parent.parent.parent
        sys.path.insert(0, str(project_root))
        from ros_topic_comm import get_robot_state_subscriber
        from Test_Module.chase_core import ChaseController

        # 获取机器人位置
        subscriber = get_robot_state_subscriber()
        subscriber.spin_once()
        robot_pos = _get_robot_position()
        print(f"[chase.chase_enemy] 机器人位置: ({robot_pos['x']:.1f}, {robot_pos['y']:.1f}), 角度: {robot_pos['angle']:.1f}°",
              file=sys.stderr)

        # 找到最近的敌人
        import math
        nearest = None
        min_dist = float('inf')

        for enemy in positions:
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
                "error": "无法找到最近的敌人"
            }, ensure_ascii=False)

        target_x = nearest['x']
        target_y = nearest['y']

        print(f"[chase.chase_enemy] 追击目标: {nearest['id']} at ({target_x}, {target_y})",
              file=sys.stderr)
        print(f"[chase.chase_enemy] 距离: {min_dist:.1f} 像素 ({min_dist/100:.2f} 米)",
              file=sys.stderr)

        # 计算角度和距离
        controller = ChaseController()
        target_angle = controller.calculate_target_angle(
            robot_pos['x'], robot_pos['y'], target_x, target_y
        )
        angle_diff = controller.calculate_angle_difference(
            robot_pos['angle'], target_angle
        )

        # 详细调试信息
        import math
        dx = target_x - robot_pos['x']
        dy = target_y - robot_pos['y']
        print(f"[chase.chase_enemy] 调试: dx={dx}, dy={dy}", file=sys.stderr)
        print(f"[chase.chase_enemy] 调试: atan2(-dy, dx)={math.degrees(math.atan2(-dy, dx)):.1f}°", file=sys.stderr)
        print(f"[chase.chase_enemy] 目标角度: {target_angle:.1f}°, 需要旋转: {angle_diff:.1f}°",
              file=sys.stderr)

        # 步骤1: 旋转
        if abs(angle_diff) > 5:
            print(f"[chase.chase_enemy] 步骤1: 旋转前 - 机器人角度: {robot_pos['angle']:.1f}°", file=sys.stderr)
            print(f"[chase.chase_enemy] 步骤1: 旋转 {angle_diff:.1f}°", file=sys.stderr)
            await _turn(angle=angle_diff, angular_speed=0.5)
            await asyncio.sleep(3)

            subscriber.spin_once()
            robot_pos = _get_robot_position()
            print(f"[chase.chase_enemy] 步骤1: 旋转后 - 机器人角度: {robot_pos['angle']:.1f}°", file=sys.stderr)
        else:
            print(f"[chase.chase_enemy] 步骤1: 角度已对准，无需旋转", file=sys.stderr)

        # 步骤2: 前进（使用PID控制，直到到达）
        step_count = 0
        max_steps = 20

        while step_count < max_steps:
            subscriber.spin_once()
            state = _get_robot_position()
            current_dist = controller.calculate_distance(
                state['x'], state['y'],
                target_x, target_y
            )

            print(f"[chase.chase_enemy] 步骤2.{step_count + 1}: 当前距离 {current_dist:.1f} 像素",
                  file=sys.stderr)

            if current_dist < ARRIVAL_THRESHOLD:
                print(f"[chase.chase_enemy] ✓ 已到达目标！误差 {current_dist:.1f} 像素", file=sys.stderr)
                break

            # 使用PID控制计算前进距离
            step_distance = _calculate_step_distance(current_dist)
            print(f"[chase.chase_enemy]  前进 {step_distance:.2f} 米 (PID控制)", file=sys.stderr)

            await _move_forward(distance=step_distance, speed=0.3)
            await asyncio.sleep(2)

            subscriber.spin_once()
            step_count += 1

        print(f"[chase.chase_enemy] 追击完成！", file=sys.stderr)

        # 清除已追击的敌人
        from ros_topic_comm import remove_enemy
        print(f"[chase.chase_enemy] 清除敌人: {nearest['id']}", file=sys.stderr)
        remove_enemy(nearest['id'])

        return json.dumps({
            "success": True,
            "message": f"追击完成，到达目标 ({target_x}, {target_y})，已清除敌人 {nearest['id']}"
        }, ensure_ascii=False)

    print("[chase.py:register_tools] 追击模块已注册 (2 个工具)", file=sys.stderr)

    return {
        'get_enemy_positions': get_enemy_positions,
        'chase_enemy': chase_enemy
    }
