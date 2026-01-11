#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Go2 Robot Skills

Unitree Go2四足机器人技能实现
"""

from typing import Dict, Any


def skill_move_forward(distance: float = 1.0, speed: float = 0.3) -> Dict[str, Any]:
    """
    向前移动

    Args:
        distance: 移动距离(米)
        speed: 移动速度(m/s)

    Returns:
        执行结果
    """
    # 调用ROS2或Dora控制器
    return {
        'action': 'navigate',
        'parameters': {
            'direction': 'front',
            'distance': f'{distance}m'
        }
    }


def skill_move_backward(distance: float = 1.0, speed: float = 0.3) -> Dict[str, Any]:
    """
    向后移动

    Args:
        distance: 移动距离(米)
        speed: 移动速度(m/s)

    Returns:
        执行结果
    """
    return {
        'action': 'navigate',
        'parameters': {
            'direction': 'back',
            'distance': f'{distance}m'
        }
    }


def skill_rotate_left(angle: float = 90.0, angular_speed: float = 0.5) -> Dict[str, Any]:
    """
    原地左转

    Args:
        angle: 旋转角度(度)
        angular_speed: 角速度(rad/s)

    Returns:
        执行结果
    """
    return {
        'action': 'turn_left',
        'parameters': {
            'angle': f'{angle}deg'
        }
    }


def skill_rotate_right(angle: float = 90.0, angular_speed: float = 0.5) -> Dict[str, Any]:
    """
    原地右转

    Args:
        angle: 旋转角度(度)
        angular_speed: 角速度(rad/s)

    Returns:
        执行结果
    """
    return {
        'action': 'turn_right',
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


def skill_move_omni(direction: str, distance: float = 1.0, speed: float = 0.3) -> Dict[str, Any]:
    """
    全向移动(支持斜向移动)

    Args:
        direction: 移动方向 (forward, backward, left, right, diagonal_forward_left等)
        distance: 移动距离(米)
        speed: 移动速度(m/s)

    Returns:
        执行结果
    """
    return {
        'action': 'move_omni',
        'parameters': {
            'direction': direction,
            'distance': f'{distance}m'
        }
    }
