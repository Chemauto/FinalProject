#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
4Lun Robot Skills

4-wheeled omnidirectional robot skills.
"""

from typing import Dict, Any

def move(direction: str, distance: float = 1.0, speed: float = 0.3) -> Dict[str, Any]:
    """
    向指定方向移动指定的距离。

    Args:
        direction: 移动方向,可以是 '前' (front), '后' (back), '左' (left), '右' (right)。
        distance: 移动距离(米)。
        speed: 移动速度(m/s)。

    Returns:
        A dictionary containing the action and its parameters.
    """
    direction_map = {
        "前": "front", "front": "front",
        "后": "back", "back": "back",
        "左": "left", "left": "left",
        "右": "right", "right": "right"
    }
    
    normalized_direction = direction_map.get(direction.lower())

    if normalized_direction in ["front", "back"]:
        return {
            'action': 'navigate',
            'parameters': {
                'direction': normalized_direction,
                'distance': f'{distance}m',
                'speed': speed
            }
        }
    elif normalized_direction in ["left", "right"]:
        return {
            'action': 'strafe',
            'parameters': {
                'direction': normalized_direction,
                'distance': f'{distance}m',
                'speed': speed
            }
        }
    else:
        return {
            'action': 'error',
            'parameters': {'message': f"未知的移动方向: {direction}"}
        }

def turn(direction: str, angle: float = 90.0, angular_speed: float = 0.5) -> Dict[str, Any]:
    """
    原地向左或向右旋转指定的角度。

    Args:
        direction: 旋转方向, '左' (left) 或 '右' (right)。
        angle: 旋转角度(度)。
        angular_speed: 角速度(rad/s)。

    Returns:
        A dictionary containing the action and its parameters.
    """
    direction_map = {
        "左": "left", "left": "left",
        "右": "right", "right": "right"
    }
    
    normalized_direction = direction_map.get(direction.lower())

    if normalized_direction == "left":
        return {
            'action': 'turn_left',
            'parameters': {
                'angle': f'{angle}deg',
                'angular_speed': angular_speed
            }
        }
    elif normalized_direction == "right":
        return {
            'action': 'turn_right',
            'parameters': {
                'angle': f'{angle}deg',
                'angular_speed': angular_speed
            }
        }
    else:
        return {
            'action': 'error',
            'parameters': {'message': f"未知的旋转方向: {direction}"}
        }

def stop() -> Dict[str, Any]:
    """
    立即停止所有动作。

    Returns:
        A dictionary containing the stop action.
    """
    return {
        'action': 'stop',
        'parameters': {}
    }
