#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
追击核心算法模块

实现机器人追击目标的核心算法：
- 计算目标角度
- 计算距离
- 自动追击控制
"""

import math
import asyncio
import sys
from typing import Dict, Tuple, Optional


class ChaseController:
    """追击控制器"""

    def __init__(self, robot_getter=None, move_forward_fn=None, turn_fn=None):
        """
        初始化追击控制器

        Args:
            robot_getter: 获取机器人位置的函数
            move_forward_fn: 向前移动函数
            turn_fn: 旋转函数
        """
        self.get_robot_position = robot_getter
        self.move_forward = move_forward_fn
        self.turn = turn_fn

        # 追击参数
        self.arrival_threshold = 20  # 到达阈值（像素）
        self.step_distance = 0.5     # 每步移动距离（米）
        self.max_steps = 100         # 最大步数限制

    def calculate_target_angle(self, robot_x: float, robot_y: float,
                               target_x: float, target_y: float) -> float:
        """
        计算目标方向角度

        Args:
            robot_x, robot_y: 机器人坐标
            target_x, target_y: 目标坐标

        Returns:
            目标角度（度），范围 [0, 360)
            0° = 东，90° = 北，180° = 西，270° = 南

        示例:
            >>> calculate_target_angle(100, 300, 700, 300)
            0.0  # 目标在正东方向

            >>> calculate_target_angle(400, 500, 400, 100)
            90.0  # 目标在正北方向
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

    def calculate_angle_difference(self, current_angle: float,
                                   target_angle: float) -> float:
        """
        计算角度差

        Args:
            current_angle: 当前角度（度）
            target_angle: 目标角度（度）

        Returns:
            角度差（度），范围 [-180, 180]
            正值 = 左转，负值 = 右转

        示例:
            >>> calculate_angle_difference(0, 90)
            90  # 需要左转90度

            >>> calculate_angle_difference(90, 0)
            -90  # 需要右转90度

            >>> calculate_angle_difference(350, 10)
            20  # 需要左转20度（跨越0°）
        """
        diff = target_angle - current_angle

        # 标准化到 [-180, 180]
        diff = (diff + 180) % 360 - 180

        return diff

    def calculate_distance(self, x1: float, y1: float,
                          x2: float, y2: float) -> float:
        """
        计算两点间距离

        Args:
            x1, y1: 点1坐标
            x2, y2: 点2坐标

        Returns:
            距离（像素）

        示例:
            >>> calculate_distance(0, 0, 300, 400)
            500.0
        """
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def distance_to_meters(self, pixels: float) -> float:
        """像素转米（1米 = 100像素）"""
        return pixels / 100.0

    async def chase_step(self, target_pos: Dict[str, float],
                        progress_callback=None) -> Dict:
        """
        执行单步追击

        Args:
            target_pos: 目标位置 {'x': x, 'y': y}
            progress_callback: 进度回调函数

        Returns:
            执行结果字典
        """
        # 获取机器人当前位置
        robot_pos = self.get_robot_position()
        if not robot_pos:
            return {"success": False, "error": "无法获取机器人位置"}

        robot_x = robot_pos['x']
        robot_y = robot_pos['y']
        robot_angle = robot_pos['angle']

        target_x = target_pos['x']
        target_y = target_pos['y']

        # 计算距离
        distance = self.calculate_distance(robot_x, robot_y, target_x, target_y)

        # 检查是否到达
        if distance < self.arrival_threshold:
            return {
                "success": True,
                "status": "arrived",
                "message": f"已到达目标！距离: {distance:.1f}像素",
                "distance": distance
            }

        # 计算目标角度
        target_angle = self.calculate_target_angle(
            robot_x, robot_y, target_x, target_y
        )

        # 计算角度差
        angle_diff = self.calculate_angle_difference(robot_angle, target_angle)

        # 如果角度差较大，先旋转
        if abs(angle_diff) > 5:  # 5度容差
            if progress_callback:
                progress_callback({
                    "action": "turn",
                    "angle": angle_diff,
                    "current_angle": robot_angle,
                    "target_angle": target_angle
                })

            await self.turn(angle=angle_diff, angular_speed=0.5)

            return {
                "success": True,
                "status": "turned",
                "message": f"旋转 {angle_diff:.1f}°",
                "angle_diff": angle_diff
            }

        # 角度对准后，向前移动
        if progress_callback:
            progress_callback({
                "action": "move",
                "distance": self.step_distance,
                "remaining_distance": distance
            })

        await self.move_forward(distance=self.step_distance, speed=0.3)

        return {
            "success": True,
            "status": "moved",
            "message": f"前进 {self.step_distance}m",
            "distance": distance
        }

    async def chase_target(self, target_pos: Dict[str, float],
                          progress_callback=None) -> Dict:
        """
        完整追击流程

        Args:
            target_pos: 目标位置 {'x': x, 'y': y}
            progress_callback: 进度回调函数

        Returns:
            最终结果
        """
        print(f"[ChaseController] 开始追击目标: ({target_pos['x']}, {target_pos['y']})",
              file=sys.stderr)

        steps = 0

        while steps < self.max_steps:
            result = await self.chase_step(target_pos, progress_callback)

            if result["status"] == "arrived":
                print(f"[ChaseController] ✓ 追击成功！总步数: {steps + 1}",
                      file=sys.stderr)
                return result

            # 等待动作完成
            await asyncio.sleep(0.5)

            steps += 1

        return {
            "success": False,
            "error": f"超过最大步数限制 ({self.max_steps})"
        }


# 辅助函数

def format_angle(angle: float) -> str:
    """格式化角度显示"""
    return f"{angle:.1f}°"


def format_distance(meters: float) -> str:
    """格式化距离显示"""
    return f"{meters:.2f}m"


def get_direction_text(angle_diff: float) -> str:
    """获取旋转方向文本"""
    if abs(angle_diff) < 5:
        return "保持方向"
    elif angle_diff > 0:
        return f"左转 {abs(angle_diff):.1f}°"
    else:
        return f"右转 {abs(angle_diff):.1f}°"
