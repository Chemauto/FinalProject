#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Go2 Quadruped Robot Skills

Unitree Go2 四足机器人技能实现

支持动作:
- move_forward/backward: 前后移动
- turn_left/right: 原地转向
- strafe_left/right: 侧向移动
- stand: 站立
- sit: 蹲下
- wave: 摇摆打招呼
"""

from typing import Dict, Any


def skill_move_forward(distance: float = 1.0, speed: float = 0.3) -> Dict[str, Any]:
    """
    向前移动

    Args:
        distance: 移动距离(米)
        speed: 移动速度(m/s), 建议 0.1-0.5

    Returns:
        执行结果
    """
    if isinstance(distance, str):
        distance = float(distance)

    return {
        'action': 'navigate',
        'parameters': {
            'direction': 'front',
            'distance': f'{distance}m',
            'speed': speed
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
    if isinstance(distance, str):
        distance = float(distance)

    return {
        'action': 'navigate',
        'parameters': {
            'direction': 'back',
            'distance': f'{distance}m',
            'speed': speed
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

    return {
        'action': 'turn',
        'parameters': {
            'angle': f'{angle}deg',
            'angular_speed': angular_speed
        }
    }


def skill_turn_left(angle: float = 90.0, angular_speed: float = 0.5) -> Dict[str, Any]:
    """
    向左转

    Args:
        angle: 旋转角度(度)
        angular_speed: 角速度(rad/s)

    Returns:
        执行结果
    """
    if isinstance(angle, str):
        angle = float(angle)

    return {
        'action': 'turn_left',
        'parameters': {
            'angle': f'{angle}deg',
            'angular_speed': angular_speed
        }
    }


def skill_turn_right(angle: float = 90.0, angular_speed: float = 0.5) -> Dict[str, Any]:
    """
    向右转

    Args:
        angle: 旋转角度(度)
        angular_speed: 角速度(rad/s)

    Returns:
        执行结果
    """
    if isinstance(angle, str):
        angle = float(angle)

    return {
        'action': 'turn_right',
        'parameters': {
            'angle': f'{angle}deg',
            'angular_speed': angular_speed
        }
    }


def skill_strafe_left(distance: float = 0.5, speed: float = 0.2) -> Dict[str, Any]:
    """
    向左侧移动 (螃蟹步)

    Args:
        distance: 移动距离(米)
        speed: 移动速度(m/s)

    Returns:
        执行结果
    """
    if isinstance(distance, str):
        distance = float(distance)

    return {
        'action': 'strafe',
        'parameters': {
            'direction': 'left',
            'distance': f'{distance}m',
            'speed': speed
        }
    }


def skill_strafe_right(distance: float = 0.5, speed: float = 0.2) -> Dict[str, Any]:
    """
    向右侧移动 (螃蟹步)

    Args:
        distance: 移动距离(米)
        speed: 移动速度(m/s)

    Returns:
        执行结果
    """
    if isinstance(distance, str):
        distance = float(distance)

    return {
        'action': 'strafe',
        'parameters': {
            'direction': 'right',
            'distance': f'{distance}m',
            'speed': speed
        }
    }


def skill_stop() -> Dict[str, Any]:
    """
    立即停止运动

    Returns:
        执行结果
    """
    return {
        'action': 'stop',
        'parameters': {}
    }


def skill_stand() -> Dict[str, Any]:
    """
    站立姿态

    Returns:
        执行结果
    """
    return {
        'action': 'stand',
        'parameters': {}
    }


def skill_sit() -> Dict[str, Any]:
    """
    蹲下姿态

    Returns:
        执行结果
    """
    return {
        'action': 'sit',
        'parameters': {}
    }


def skill_wave(duration: float = 2.0) -> Dict[str, Any]:
    """
    摇摆打招呼 (前右腿)

    Args:
        duration: 持续时间(秒)

    Returns:
        执行结果
    """
    if isinstance(duration, str):
        duration = float(duration)

    return {
        'action': 'wave',
        'parameters': {
            'duration': f'{duration}s'
        }
    }


def skill_lie_down() -> Dict[str, Any]:
    """
    躺下休息

    Returns:
        执行结果
    """
    return {
        'action': 'lie_down',
        'parameters': {}
    }


def skill_walk_forward(duration: float = 3.0, speed: float = 0.3) -> Dict[str, Any]:
    """
    持续前进一段时间 (基于时间的控制)

    Args:
        duration: 持续时间(秒)
        speed: 前进速度(m/s)

    Returns:
        执行结果
    """
    if isinstance(duration, str):
        duration = float(duration)

    return {
        'action': 'walk',
        'parameters': {
            'direction': 'front',
            'duration': f'{duration}s',
            'speed': speed
        }
    }
