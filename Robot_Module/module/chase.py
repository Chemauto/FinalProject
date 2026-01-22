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

# 追击参数配置
MAX_STEP_DISTANCE = 0.5  # 最大步长（米）
ARRIVAL_THRESHOLD = 5.0  # 到达阈值（像素）


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


def _calculate_step_distance(current_distance_pixels: float) -> float:
    """使用PID控制计算步长（距离越近步长越小）

    Args:
        current_distance_pixels: 当前距离（像素）

    Returns:
        步长（米）
    """
    # 将像素转换为米
    distance_meters = current_distance_pixels / 100.0

    # PID控制：步长与距离成正比，但不超过最大值
    # 距离越远，步长越大（最大0.5米）
    # 距离越近，步长越小（最小0.1米，保证不会太小）
    step_distance = min(distance_meters * 0.8, MAX_STEP_DISTANCE)
    step_distance = max(step_distance, 0.1)  # 最小0.1米

    return step_distance


def _get_robot_position() -> Optional[Dict]:
    """从 ROS 订阅器获取机器人当前位置"""
    from pathlib import Path
    project_root = Path(__file__).parent.parent.parent
    sys.path.insert(0, str(project_root))
    from ros_topic_comm import get_robot_state

    # get_robot_state() 已经通过订阅器获取实时位置
    state = get_robot_state()
    if state:
        return {
            'x': state.get('x', 400.0),
            'y': state.get('y', 300.0),
            'angle': state.get('angle', 0.0)
        }
    # 默认位置
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
    async def chase_enemy() -> str:
        """追击敌人（简化版，直接调用 move_forward 和 turn）

        自动获取敌人位置并追击最近的敌人。这个工具会：
        1. 获取当前所有敌人的位置
        2. 找到距离最近的敌人
        3. 计算需要旋转的角度
        4. 调用 turn() 旋转到目标方向
        5. 调用 move_forward() 前进到目标位置

        Returns:
            执行结果JSON字符串

        Examples:
            chase_enemy()  # 自动追击最近的敌人
        """
        print(f"[chase.chase_enemy] 开始追击敌人", file=sys.stderr)

        # 获取敌人位置（带重试）
        from pathlib import Path
        project_root = Path(__file__).parent.parent.parent
        sys.path.insert(0, str(project_root))
        from ros_topic_comm import get_enemy_positions as get_positions
        from ros_topic_comm import get_robot_state_subscriber
        from Test_Module.chase_core import ChaseController

        # 处理 ROS 回调，确保收到最新消息
        subscriber = get_robot_state_subscriber()
        for i in range(10):  # 尝试10次
            subscriber.spin_once()
            positions = get_positions()
            if len(positions) > 0:
                break
            print(f"[chase.chase_enemy] 第{i+1}次尝试获取敌人位置，等待...", file=sys.stderr)
            await asyncio.sleep(0.3)  # 等待300ms

        print(f"[chase.chase_enemy] 最终获取到 {len(positions)} 个敌人", file=sys.stderr)

        if not positions:
            return json.dumps({
                "success": False,
                "error": "没有找到敌人，请先在仿真器中生成敌人"
            }, ensure_ascii=False)

        # 获取机器人位置
        subscriber.spin_once()  # 确保获取最新位置
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

        print(f"[chase.chase_enemy] 目标角度: {target_angle:.1f}°, 需要旋转: {angle_diff:.1f}°",
              file=sys.stderr)

        # 步骤1: 旋转
        if abs(angle_diff) > 5:
            print(f"[chase.chase_enemy] 步骤1: 旋转 {angle_diff:.1f}°", file=sys.stderr)
            await _turn(angle=angle_diff, angular_speed=0.5)
            await asyncio.sleep(3)  # 等待旋转完成

            # 更新机器人位置
            subscriber.spin_once()
            robot_pos = _get_robot_position()
        else:
            print(f"[chase.chase_enemy] 步骤1: 角度已对准，无需旋转", file=sys.stderr)

        # 步骤2: 前进（使用PID控制，直到到达）
        step_count = 0

        while True:
            subscriber.spin_once()
            state = _get_robot_position()
            current_dist = controller.calculate_distance(
                state['x'], state['y'],
                target_x, target_y
            )

            print(f"[chase.chase_enemy] 步骤2.{step_count + 1}: 当前距离 {current_dist:.1f} 像素",
                  file=sys.stderr)

            if current_dist < ARRIVAL_THRESHOLD:  # 到达阈值（5像素）
                print(f"[chase.chase_enemy] ✓ 已到达目标！误差 {current_dist:.1f} 像素", file=sys.stderr)
                break

            # 使用PID控制计算前进距离
            step_distance = _calculate_step_distance(current_dist)
            print(f"[chase.chase_enemy]  前进 {step_distance:.2f} 米 (PID控制)", file=sys.stderr)

            await _move_forward(distance=step_distance, speed=0.3)
            await asyncio.sleep(2)  # 等待移动完成

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
        from pathlib import Path
        project_root = Path(__file__).parent.parent.parent
        sys.path.insert(0, str(project_root))
        from ros_topic_comm import get_enemy_positions as get_positions

        # 等待并重试，确保收到敌人位置
        positions = []
        for attempt in range(5):  # 最多尝试5次
            positions = get_positions()
            if len(positions) > 0:
                break
            print(f"[chase.get_enemy_positions] 第{attempt+1}次尝试: 未收到敌人位置，等待...", file=sys.stderr)
            await asyncio.sleep(0.2)  # 等待200ms

        print(f"[chase.get_enemy_positions] 最终获取到 {len(positions)} 个敌人", file=sys.stderr)
        return json.dumps(positions, ensure_ascii=False)

    @mcp.tool()
    async def chase_target(
        target_x: float,
        target_y: float,
        threshold: float = 20.0,
        step_distance: float = 0.5
    ) -> str:
        """自动追击指定坐标的目标

        计算目标位置的角度和距离，然后调用 turn() 和 move_forward() 机器人移动到目标。
        这个工具会自动处理旋转和前进的完整过程。

        Args:
            target_x: 目标X坐标（像素）
            target_y: 目标Y坐标（像素）
            threshold: 到达阈值（像素），默认20像素
            step_distance: 每步移动距离（米），默认0.5米

        Returns:
            执行结果JSON字符串

        Examples:
            chase_target(target_x=700, target_y=300)
            chase_target(target_x=100, target_y=100, threshold=30)
        """
        print(f"[chase.chase_target] 开始追击: ({target_x}, {target_y})",
              file=sys.stderr)

        # 导入追击控制器
        project_root = Path(__file__).parent.parent.parent
        sys.path.insert(0, str(project_root))
        from Test_Module.chase_core import ChaseController
        from ros_topic_comm import get_robot_state_subscriber

        # 处理 ROS 回调
        subscriber = get_robot_state_subscriber()
        subscriber.spin_once()

        # 获取当前机器人位置
        robot_pos = _get_robot_position()
        print(f"[chase.chase_target] 机器人当前位置: ({robot_pos['x']}, {robot_pos['y']})",
              file=sys.stderr)

        # 创建控制器
        controller = ChaseController(
            robot_getter=_get_robot_position,
            move_forward_fn=_move_forward,
            turn_fn=_turn
        )

        # 设置参数
        controller.arrival_threshold = threshold
        controller.step_distance = step_distance

        print(f"[chase.chase_target] 开始调用 ChaseController，最大步数: {controller.max_steps}",
              file=sys.stderr)

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

    print("[chase.py:register_tools] 追击模块已注册 (5 个工具)", file=sys.stderr)

    return {
        'chase_enemy': chase_enemy,
        'get_enemy_positions': get_enemy_positions,
        'chase_target': chase_target,
        'chase_nearest_enemy': chase_nearest_enemy,
        'calculate_chase_angle': calculate_chase_angle
    }
