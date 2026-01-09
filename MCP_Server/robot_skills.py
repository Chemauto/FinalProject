# -*- coding: utf-8 -*-
"""
Robot Skills Definition
定义机器人可执行的所有skill函数
"""
from typing import Optional, Dict, Any, List
from enum import Enum


class DistanceUnit(Enum):
    """距离单位"""
    CM = "cm"
    M = "m"
    MM = "mm"


class Direction(Enum):
    """方向"""
    FRONT = "front"
    BACK = "back"
    LEFT = "left"
    RIGHT = "right"


class RobotSkills:
    """机器人Skills类"""

    def __init__(self, adapter):
        """
        初始化机器人skills

        Args:
            adapter: 通信适配器（Dora/Roda等）
        """
        self.adapter = adapter
        self.current_state = {
            "position": {"x": 0, "y": 0},
            "orientation": 0,  # 角度
        }

    # ==================== 导航类Skills ====================

    def turn_left(self, angle: float = 90.0) -> Dict[str, Any]:
        """
        左转指定角度

        Args:
            angle: 转向角度（度），默认90度

        Returns:
            执行结果
        """
        print(f"[Robot Skill] turn_left: {angle}°")
        command = {
            "action": "navigate",
            "parameters": {
                "angle": f"-{angle}deg"
            }
        }
        result = self.adapter.send_command(command)
        self.current_state["orientation"] -= angle
        return {
            "success": True,
            "action": "turn_left",
            "angle": angle,
            "result": result
        }

    def turn_right(self, angle: float = 90.0) -> Dict[str, Any]:
        """
        右转指定角度

        Args:
            angle: 转向角度（度），默认90度

        Returns:
            执行结果
        """
        print(f"[Robot Skill] turn_right: {angle}°")
        command = {
            "action": "navigate",
            "parameters": {
                "angle": f"{angle}deg"
            }
        }
        result = self.adapter.send_command(command)
        self.current_state["orientation"] += angle
        return {
            "success": True,
            "action": "turn_right",
            "angle": angle,
            "result": result
        }

    def move_forward(self, distance: float, unit: str = "m") -> Dict[str, Any]:
        """
        向前移动

        Args:
            distance: 距离数值
            unit: 单位（m/cm/mm），默认m

        Returns:
            执行结果
        """
        print(f"[Robot Skill] move_forward: {distance}{unit}")
        command = {
            "action": "navigate",
            "parameters": {
                "direction": "front",
                "distance": f"{distance}{unit}"
            }
        }
        result = self.adapter.send_command(command)
        return {
            "success": True,
            "action": "move_forward",
            "distance": distance,
            "unit": unit,
            "result": result
        }

    def move_backward(self, distance: float, unit: str = "m") -> Dict[str, Any]:
        """
        向后移动

        Args:
            distance: 距离数值
            unit: 单位（m/cm/mm），默认m

        Returns:
            执行结果
        """
        print(f"[Robot Skill] move_backward: {distance}{unit}")
        command = {
            "action": "navigate",
            "parameters": {
                "direction": "back",
                "distance": f"{distance}{unit}"
            }
        }
        result = self.adapter.send_command(command)
        return {
            "success": True,
            "action": "move_backward",
            "distance": distance,
            "unit": unit,
            "result": result
        }

    def move_left(self, distance: float, unit: str = "m") -> Dict[str, Any]:
        """
        向左移动

        Args:
            distance: 距离数值
            unit: 单位（m/cm/mm），默认m

        Returns:
            执行结果
        """
        print(f"[Robot Skill] move_left: {distance}{unit}")
        command = {
            "action": "navigate",
            "parameters": {
                "direction": "left",
                "distance": f"{distance}{unit}"
            }
        }
        result = self.adapter.send_command(command)
        return {
            "success": True,
            "action": "move_left",
            "distance": distance,
            "unit": unit,
            "result": result
        }

    def move_right(self, distance: float, unit: str = "m") -> Dict[str, Any]:
        """
        向右移动

        Args:
            distance: 距离数值
            unit: 单位（m/cm/mm），默认m

        Returns:
            执行结果
        """
        print(f"[Robot Skill] move_right: {distance}{unit}")
        command = {
            "action": "navigate",
            "parameters": {
                "direction": "right",
                "distance": f"{distance}{unit}"
            }
        }
        result = self.adapter.send_command(command)
        return {
            "success": True,
            "action": "move_right",
            "distance": distance,
            "unit": unit,
            "result": result
        }

    def navigate_to(self, location: str,
                   direction: Optional[str] = None,
                   distance: Optional[str] = None) -> Dict[str, Any]:
        """
        导航到指定位置

        Args:
            location: 目标位置名称
            direction: 相对方向（front/back/left/right）
            distance: 距离（如"30cm", "1.5m"）

        Returns:
            执行结果
        """
        print(f"[Robot Skill] navigate_to: {location}")
        parameters = {"location": location}
        if direction:
            parameters["direction"] = direction
        if distance:
            parameters["distance"] = distance

        command = {
            "action": "navigate",
            "parameters": parameters
        }
        result = self.adapter.send_command(command)
        return {
            "success": True,
            "action": "navigate_to",
            "location": location,
            "result": result
        }

    # ==================== 操作类Skills ====================

    def pick_up(self, object_name: str) -> Dict[str, Any]:
        """
        抓取物体

        Args:
            object_name: 物体名称

        Returns:
            执行结果
        """
        print(f"[Robot Skill] pick_up: {object_name}")
        command = {
            "action": "pick",
            "parameters": {
                "object": object_name
            }
        }
        result = self.adapter.send_command(command)
        return {
            "success": True,
            "action": "pick_up",
            "object": object_name,
            "result": result
        }

    def place(self, object_name: str, location: str) -> Dict[str, Any]:
        """
        放置物体

        Args:
            object_name: 物体名称
            location: 目标位置

        Returns:
            执行结果
        """
        print(f"[Robot Skill] place: {object_name} at {location}")
        command = {
            "action": "place",
            "parameters": {
                "object": object_name,
                "location": location
            }
        }
        result = self.adapter.send_command(command)
        return {
            "success": True,
            "action": "place",
            "object": object_name,
            "location": location,
            "result": result
        }

    # ==================== 复合动作Skills ====================

    def turn_then_move(self, turn_angle: float, move_distance: float,
                      move_unit: str = "m", turn_direction: str = "left") -> Dict[str, Any]:
        """
        转向然后移动（复合动作）

        Args:
            turn_angle: 转向角度
            move_distance: 移动距离
            move_unit: 移动距离单位
            turn_direction: 转向方向（left/right）

        Returns:
            执行结果
        """
        print(f"[Robot Skill] turn_then_move: {turn_direction} {turn_angle}°, then move {move_distance}{move_unit}")

        results = []
        # 先转向
        if turn_direction == "left":
            results.append(self.turn_left(turn_angle))
        else:
            results.append(self.turn_right(turn_angle))

        # 再移动
        results.append(self.move_forward(move_distance, move_unit))

        return {
            "success": True,
            "action": "turn_then_move",
            "steps": results
        }

    def move_square(self, side_length: float, unit: str = "m") -> Dict[str, Any]:
        """
        沿正方形路径移动（复合动作）

        Args:
            side_length: 边长
            unit: 单位

        Returns:
            执行结果
        """
        print(f"[Robot Skill] move_square: {side_length}{unit}")
        results = []
        for i in range(4):
            results.append(self.move_forward(side_length, unit))
            results.append(self.turn_left(90.0))

        return {
            "success": True,
            "action": "move_square",
            "side_length": side_length,
            "unit": unit,
            "steps": results
        }

    # ==================== 工具类Skills ====================

    def stop(self) -> Dict[str, Any]:
        """
        停止机器人

        Returns:
            执行结果
        """
        print(f"[Robot Skill] stop")
        command = {
            "action": "stop",
            "parameters": {}
        }
        result = self.adapter.send_command(command)
        return {
            "success": True,
            "action": "stop",
            "result": result
        }

    def get_status(self) -> Dict[str, Any]:
        """
        获取机器人当前状态

        Returns:
            当前状态
        """
        print(f"[Robot Skill] get_status")
        return {
            "success": True,
            "action": "get_status",
            "state": self.current_state
        }

    def wait(self, seconds: float = 1.0) -> Dict[str, Any]:
        """
        等待指定时间

        Args:
            seconds: 等待秒数

        Returns:
            执行结果
        """
        import time
        print(f"[Robot Skill] wait: {seconds}s")
        time.sleep(seconds)
        return {
            "success": True,
            "action": "wait",
            "seconds": seconds
        }
