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
import math
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
    # 等待旋转完成（基于角度和角速度计算，再额外缓冲50%）
    estimated_time = abs(angle) / 180.0 * 3.14159 / angular_speed if angular_speed > 0 else 1
    await asyncio.sleep(estimated_time * 1.5 + 0.5)


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
# 核心计算函数（原 chase_core.py）
# ==============================================================================

def _calculate_target_angle(robot_x: float, robot_y: float,
                           target_x: float, target_y: float) -> float:
    """
    计算目标方向角度

    Args:
        robot_x, robot_y: 机器人坐标
        target_x, target_y: 目标坐标

    Returns:
        目标角度（度），范围 [0, 360)
        0° = 东，90° = 北，180° = 西，270° = 南
    """
    # 计算向量
    dx = target_x - robot_x
    dy = target_y - robot_y  # 屏幕坐标系y向下

    # 计算角度（注意：dy取负值，因为屏幕y向下）
    angle_rad = math.atan2(-dy, dx)
    angle_deg = math.degrees(angle_rad)

    # 归一化到 [0, 360)
    angle_deg = angle_deg % 360

    return angle_deg


def _calculate_angle_difference(current_angle: float,
                                target_angle: float) -> float:
    """
    计算角度差

    Args:
        current_angle: 当前角度（度）
        target_angle: 目标角度（度）

    Returns:
        角度差（度），范围 [-180, 180]
        正值 = 左转，负值 = 右转
    """
    diff = target_angle - current_angle

    # 标准化到 [-180, 180]
    diff = (diff + 180) % 360 - 180

    return diff


def _calculate_distance(x1: float, y1: float,
                        x2: float, y2: float) -> float:
    """
    计算两点间距离

    Args:
        x1, y1: 点1坐标
        x2, y2: 点2坐标

    Returns:
        距离（像素）
    """
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


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
        from ros_topic_comm import get_enemy_positions as get_positions, get_enemy_positions_subscriber

        # 获取订阅器（首次调用时会初始化ROS连接）
        subscriber = get_enemy_positions_subscriber()
        print(f"[chase.get_enemy_positions] 订阅器已创建，等待ROS连接建立...", file=sys.stderr)

        # 首次初始化等待：给ROS订阅者时间建立连接
        # 注意：订阅器初始化需要额外0.5秒（在 ros_topic_comm.py 中）
        await asyncio.sleep(1.0)

        # 强制刷新：多次调用 spin_once 确保接收到最新消息
        print(f"[chase.get_enemy_positions] 开始刷新ROS回调...", file=sys.stderr)

        # 积极刷新：多次快速调用 spin_once，持续等待消息
        for i in range(50):  # 增加到50次刷新
            subscriber.spin_once()
            if i % 10 == 9:  # 每10次稍微等待
                await asyncio.sleep(0.2)

        # 等待消息处理完成
        await asyncio.sleep(0.5)

        # 获取敌人位置
        positions = get_positions()

        print(f"[chase.get_enemy_positions] 获取到 {len(positions)} 个敌人: {positions}", file=sys.stderr)
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

        # 获取机器人位置订阅器
        subscriber = get_robot_state_subscriber()

        # 等待一小段时间，获取最新的机器人位置
        await asyncio.sleep(0.5)
        subscriber.spin_once()
        robot_pos = _get_robot_position()
        print(f"[chase.chase_enemy] 机器人位置: ({robot_pos['x']:.1f}, {robot_pos['y']:.1f}), 角度: {robot_pos['angle']:.1f}°",
              file=sys.stderr)

        # 找到最近的敌人
        nearest = None
        min_dist = float('inf')

        for enemy in positions:
            dist = _calculate_distance(
                robot_pos['x'], robot_pos['y'],
                enemy['x'], enemy['y']
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
        target_angle = _calculate_target_angle(
            robot_pos['x'], robot_pos['y'], target_x, target_y
        )
        angle_diff = _calculate_angle_difference(
            robot_pos['angle'], target_angle
        )

        # 详细调试信息
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

            # 等待并检查旋转是否完成
            for check in range(10):
                await asyncio.sleep(0.5)
                subscriber.spin_once()
                robot_pos = _get_robot_position()
                new_angle_diff = _calculate_angle_difference(
                    robot_pos['angle'], target_angle
                )
                if abs(new_angle_diff) < 10:  # 角度差小于10度认为完成
                    break
                print(f"[chase.chase_enemy]  旋转检查 {check+1}: 角度差 {new_angle_diff:.1f}°", file=sys.stderr)

            print(f"[chase.chase_enemy] 步骤1: 旋转后 - 机器人角度: {robot_pos['angle']:.1f}°", file=sys.stderr)
        else:
            print(f"[chase.chase_enemy] 步骤1: 角度已对准，无需旋转", file=sys.stderr)

        # 步骤2: 前进（使用PID控制，直到到达）
        step_count = 0
        max_steps = 30
        prev_dist = float('inf')
        dist_increase_count = 0  # 距离连续增大的次数

        while step_count < max_steps:
            subscriber.spin_once()
            state = _get_robot_position()
            current_dist = _calculate_distance(
                state['x'], state['y'],
                target_x, target_y
            )

            print(f"[chase.chase_enemy] 步骤2.{step_count + 1}: 当前距离 {current_dist:.1f} 像素",
                  file=sys.stderr)

            if current_dist < ARRIVAL_THRESHOLD:
                print(f"[chase.chase_enemy] ✓ 已到达目标！误差 {current_dist:.1f} 像素", file=sys.stderr)
                break

            # 检测距离是否在增大（说明方向偏离）
            if current_dist > prev_dist:
                dist_increase_count += 1
                print(f"[chase.chase_enemy]  ⚠️  距离增大 ({prev_dist:.1f} → {current_dist:.1f})，第{dist_increase_count}次",
                      file=sys.stderr)

                # 连续3次距离增大，需要重新校正角度
                if dist_increase_count >= 3:
                    print(f"[chase.chase_enemy]  🔧 重新校正角度...", file=sys.stderr)

                    # 重新计算目标角度
                    new_target_angle = _calculate_target_angle(
                        state['x'], state['y'], target_x, target_y
                    )
                    new_angle_diff = _calculate_angle_difference(
                        state['angle'], new_target_angle
                    )

                    print(f"[chase.chase_enemy]  当前角度: {state['angle']:.1f}°, 目标: {new_target_angle:.1f}°, 差值: {new_angle_diff:.1f}°",
                          file=sys.stderr)

                    # 如果角度差超过5度，重新旋转
                    if abs(new_angle_diff) > 5:
                        await _turn(angle=new_angle_diff, angular_speed=0.5)
                        await asyncio.sleep(0.5)
                        dist_increase_count = 0  # 重置计数
                    else:
                        # 角度没问题但还是增大，可能是步长太大，减小步长
                        print(f"[chase.chase_enemy]  角度已对准但仍偏离，减小步长", file=sys.stderr)
                        step_distance = min(current_dist / 100.0 * 0.3, 0.2)  # 使用更小的步长
                        await _move_forward(distance=step_distance, speed=0.2)
                        await asyncio.sleep(step_distance / 0.2 * 1.5 + 0.3)
                        step_count += 1
                        prev_dist = current_dist
                        continue
            else:
                dist_increase_count = 0  # 距离减小，重置计数

            # 使用PID控制计算前进距离
            step_distance = _calculate_step_distance(current_dist)
            print(f"[chase.chase_enemy]  前进 {step_distance:.2f} 米 (PID控制)", file=sys.stderr)

            await _move_forward(distance=step_distance, speed=0.3)

            # 等待前进完成（距离/速度 + 缓冲）
            await asyncio.sleep(step_distance / 0.3 * 1.2 + 0.3)

            subscriber.spin_once()
            step_count += 1
            prev_dist = current_dist

        print(f"[chase.chase_enemy] 追击完成！", file=sys.stderr)

        # 清除已追击的敌人（发送清除命令给仿真器）
        from ros_topic_comm import remove_enemy
        print(f"[chase.chase_enemy] 清除敌人: {nearest['id']}", file=sys.stderr)
        remove_enemy(nearest['id'])

        # 等待仿真器处理清除命令并发布新的敌人位置
        # 仿真器会立即发布更新后的位置（见 simulator.py spawn_enemy_at）
        print(f"[chase.chase_enemy] 等待仿真器更新敌人位置...", file=sys.stderr)
        await asyncio.sleep(1.0)  # 给仿真器足够的时间处理

        # 多次刷新ROS回调，确保接收到仿真器发布的最新位置
        for i in range(10):
            subscriber.spin_once()
            if i % 3 == 0:
                await asyncio.sleep(0.1)  # 每3次稍微等待一下

        print(f"[chase.chase_enemy] 清除流程完成", file=sys.stderr)

        return json.dumps({
            "success": True,
            "message": f"追击完成，到达目标 ({target_x}, {target_y})，已清除敌人 {nearest['id']}"
        }, ensure_ascii=False)

    print("[chase.py:register_tools] 追击模块已注册 (2 个工具)", file=sys.stderr)

    return {
        'get_enemy_positions': get_enemy_positions,
        'chase_enemy': chase_enemy
    }
