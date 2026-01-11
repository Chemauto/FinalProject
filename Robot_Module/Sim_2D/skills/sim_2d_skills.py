#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sim 2D Robot Skills

2D仿真机器人技能实现 (TurtleBot等)
"""

from typing import Dict, Any


def skill_move_forward(distance: float = 1.0, speed: float = 0.2) -> Dict[str, Any]:
    """
    向前移动

    Args:
        distance: 移动距离(米)
        speed: 移动速度(m/s)

    Returns:
        执行结果
    """
    # 确保distance是数值类型
    if isinstance(distance, str):
        distance = float(distance)

    return {
        'action': 'navigate',
        'parameters': {
            'direction': 'front',
            'distance': f'{distance}m'
        }
    }


def skill_move_backward(distance: float = 1.0, speed: float = 0.2) -> Dict[str, Any]:
    """
    向后移动

    Args:
        distance: 移动距离(米)
        speed: 移动速度(m/s)

    Returns:
        执行结果
    """
    # 确保distance是数值类型
    if isinstance(distance, str):
        distance = float(distance)

    return {
        'action': 'navigate',
        'parameters': {
            'direction': 'back',
            'distance': f'{distance}m'
        }
    }


def skill_turn(angle: float, angular_speed: float = 0.5) -> Dict[str, Any]:
    """
    原地旋转
    - 正角度(>0): 向左转(逆时针)
    - 负角度(<0): 向右转(顺时针)

    Args:
        angle: 旋转角度(度), 正值为左转，负值为右转
        angular_speed: 角速度(rad/s)

    Returns:
        执行结果
    """
    if isinstance(angle, str):
        angle = float(angle)

    # 直接传递角度（保留符号），让仿真器处理转向方向
    return {
        'action': 'turn',
        'parameters': {
            'angle': f'{angle}deg'
        }
    }


def skill_stop() -> Dict[str, Any]:
    """
    立即停止

    Returns:
        执行结果
    """
    return {
        'action': 'stop',
        'parameters': {}
    }
