#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
执行监控器
实时监控任务执行状态，检测异常，触发重新规划
"""
import time
import asyncio
from typing import Dict, Any, Optional, Callable
from enum import Enum
from dataclasses import dataclass


class AnomalyType(Enum):
    """异常类型枚举"""
    TIMEOUT = "timeout"
    STUCK = "stuck"                    # 卡住（位置长时间不变）
    OSCILLATION = "oscillation"        # 振荡（来回移动）
    ENVIRONMENT_CHANGE = "environment_change"  # 环境变化
    SENSOR_FAILURE = "sensor_failure"  # 传感器失效
    UNKNOWN = "unknown"


@dataclass
class Anomaly:
    """异常数据类"""
    type: AnomalyType
    description: str
    severity: str  # low, medium, high
    data: Optional[Dict[str, Any]] = None


class ExecutionMonitor:
    """
    执行监控器

    职责：
    1. 实时状态检测
    2. 异常检测
    3. 执行反馈
    4. 环境变化检测
    """

    def __init__(self,
                 monitoring_interval: float = 0.1,
                 timeout_threshold: float = 30.0,
                 stuck_threshold: float = 5.0):
        """
        初始化执行监控器

        Args:
            monitoring_interval: 监控检查间隔（秒）
            timeout_threshold: 超时阈值（秒）
            stuck_threshold: 卡住检测阈值（位置不变的时间，秒）
        """
        self.monitoring_interval = monitoring_interval
        self.timeout_threshold = timeout_threshold
        self.stuck_threshold = stuck_threshold

        # 状态跟踪
        self.execution_start_time = None
        self.last_position = None
        self.last_position_update_time = None
        self.position_history = []  # 用于振荡检测

    async def monitor_execution(self,
                                task: Dict[str, Any],
                                execute_fn: Callable,
                                get_state_fn: Optional[Callable] = None) -> Dict[str, Any]:
        """
        监控任务执行

        Args:
            task: 要执行的任务
            execute_fn: 执行函数（异步或同步）
            get_state_fn: 获取当前状态的函数（可选）

        Returns:
            执行结果
        """
        self.execution_start_time = time.time()
        last_check_time = time.time()

        try:
            # 如果是异步函数
            if asyncio.iscoroutinefunction(execute_fn):
                result = await execute_fn()
            else:
                # 如果是同步函数，在线程池中执行
                loop = asyncio.get_event_loop()
                result = await loop.run_in_executor(None, execute_fn)

            return result

        except Exception as e:
            return {
                "status": "failed",
                "error": str(e),
                "task": task
            }

    def detect_anomaly(self,
                       current_state: Dict[str, Any],
                       task: Dict[str, Any]) -> Optional[Anomaly]:
        """
        检测异常

        Args:
            current_state: 当前状态
            task: 当前任务

        Returns:
            检测到的异常，如果没有异常则返回None
        """
        current_time = time.time()

        # ==================== 1. 超时检测 ====================
        if self.execution_start_time:
            elapsed = current_time - self.execution_start_time
            if elapsed > self.timeout_threshold:
                return Anomaly(
                    type=AnomalyType.TIMEOUT,
                    description=f"任务执行超时（{elapsed:.1f}秒）",
                    severity="high",
                    data={"elapsed_time": elapsed, "threshold": self.timeout_threshold}
                )
        # ==========================================================

        # ==================== 2. 卡住检测 ====================
        if current_state and "position" in current_state:
            current_position = current_state["position"]

            if self.last_position is not None:
                # 检查位置是否变化
                if self._position_unchanged(current_position, self.last_position):
                    if self.last_position_update_time:
                        stuck_duration = current_time - self.last_position_update_time
                        if stuck_duration > self.stuck_threshold:
                            return Anomaly(
                                type=AnomalyType.STUCK,
                                description=f"机器人卡住（{stuck_duration:.1f}秒未移动）",
                                severity="medium",
                                data={
                                    "stuck_duration": stuck_duration,
                                    "position": current_position,
                                    "threshold": self.stuck_threshold
                                }
                            )
                else:
                    # 位置已更新，记录时间
                    self.last_position_update_time = current_time

                    # ==================== 3. 振荡检测 ====================
                    # 记录位置历史
                    self.position_history.append({
                        "time": current_time,
                        "position": current_position
                    })

                    # 保持历史记录不超过20个
                    if len(self.position_history) > 20:
                        self.position_history.pop(0)

                    # 检测振荡（至少需要6个位置点）
                    if len(self.position_history) >= 6:
                        if self._detect_oscillation():
                            return Anomaly(
                                type=AnomalyType.OSCILLATION,
                                description="检测到振荡行为（来回移动）",
                                severity="medium",
                                data={
                                    "oscillation_count": len(self.position_history),
                                    "positions": self.position_history[-6:]
                                }
                            )
                    # ==========================================================

            # 保存当前位置
            self.last_position = current_position

            # 初始化位置更新时间
            if self.last_position_update_time is None:
                self.last_position_update_time = current_time
        # ==========================================================

        # ==================== 4. 传感器失效检测 ====================
        if current_state and "sensor_status" in current_state:
            sensor_status = current_state["sensor_status"]

            # 检查各个传感器
            failed_sensors = []
            for sensor_name, status in sensor_status.items():
                if status == "failed" or status == "error" or status is False:
                    failed_sensors.append(sensor_name)

            if failed_sensors:
                return Anomaly(
                    type=AnomalyType.SENSOR_FAILURE,
                    description=f"传感器失效: {', '.join(failed_sensors)}",
                    severity="high",
                    data={"failed_sensors": failed_sensors, "sensor_status": sensor_status}
                )
        # ==========================================================

        # ==================== 5. 环境变化检测 ====================
        if current_state and "environment_version" in current_state:
            # 使用版本号检测环境变化
            current_version = current_state["environment_version"]

            if hasattr(self, 'last_environment_version'):
                if current_version != self.last_environment_version:
                    return Anomaly(
                        type=AnomalyType.ENVIRONMENT_CHANGE,
                        description="检测到环境变化",
                        severity="high",
                        data={
                            "old_version": self.last_environment_version,
                            "new_version": current_version
                        }
                    )

            self.last_environment_version = current_version

        # 或者通过标志位检测
        if current_state and current_state.get("environment_changed", False):
            return Anomaly(
                type=AnomalyType.ENVIRONMENT_CHANGE,
                description="检测到环境变化",
                severity="high",
                data=current_state.get("environment_change_details", {})
            )
        # ==========================================================

        # 没有检测到异常
        return None

    def reset(self):
        """重置监控状态"""
        self.execution_start_time = None
        self.last_position = None
        self.last_position_update_time = None
        self.position_history = []
        if hasattr(self, 'last_environment_version'):
            delattr(self, 'last_environment_version')

    def _position_unchanged(self, pos1: Dict, pos2: Dict, threshold: float = 0.01) -> bool:
        """
        检查两个位置是否相同

        Args:
            pos1: 位置1 {"x": ..., "y": ..., "z": ...}
            pos2: 位置2 {"x": ..., "y": ..., "z": ...}
            threshold: 距离阈值（米），默认0.01米

        Returns:
            是否相同
        """
        distance = self._calculate_distance(pos1, pos2)
        return distance < threshold

    def _calculate_distance(self, pos1: Dict, pos2: Dict) -> float:
        """
        计算两点间欧几里得距离

        Args:
            pos1: 位置1
            pos2: 位置2

        Returns:
            距离（米）
        """
        dx = pos1.get("x", 0) - pos2.get("x", 0)
        dy = pos1.get("y", 0) - pos2.get("y", 0)
        dz = pos1.get("z", 0) - pos2.get("z", 0)
        return (dx**2 + dy**2 + dz**2)**0.5

    def _detect_oscillation(self, window_size: int = 6) -> bool:
        """
        检测振荡行为

        振荡定义：机器人来回移动但最终回到原点附近
        检测方法：检查最近window_size个位置点是否形成振荡模式

        Args:
            window_size: 检测窗口大小，默认6个位置点

        Returns:
            是否检测到振荡
        """
        if len(self.position_history) < window_size:
            return False

        # 获取最近的窗口位置
        recent_positions = self.position_history[-window_size:]

        # 计算起始位置和结束位置
        start_pos = recent_positions[0]["position"]
        end_pos = recent_positions[-1]["position"]

        # 如果起始和结束位置很近，但中间位置远离，则是振荡
        distance_start_end = self._calculate_distance(start_pos, end_pos)

        # 计算中间位置的最大偏离
        max_deviation = 0.0
        for record in recent_positions[1:-1]:
            deviation = self._calculate_distance(start_pos, record["position"])
            max_deviation = max(max_deviation, deviation)

        # 振荡判断条件：
        # 1. 起始和结束位置很近（< 0.5米）
        # 2. 中间有明显偏离（> 0.5米）
        if distance_start_end < 0.5 and max_deviation > 0.5:
            return True

        # 另一种检测方法：检查方向变化
        # 如果方向变化超过2次，也可能是振荡
        direction_changes = 0
        prev_direction = None

        for i in range(1, len(recent_positions)):
            curr_pos = recent_positions[i]["position"]
            prev_pos = recent_positions[i-1]["position"]

            # 计算移动方向
            dx = curr_pos.get("x", 0) - prev_pos.get("x", 0)
            dy = curr_pos.get("y", 0) - prev_pos.get("y", 0)

            # 判断大致方向（0-7，8个方向）
            if abs(dx) > abs(dy):
                direction = 0 if dx > 0 else 4  # 东/西
            else:
                direction = 2 if dy > 0 else 6  # 南/北

            if prev_direction is not None and direction != prev_direction:
                direction_changes += 1

            prev_direction = direction

        # 如果方向变化超过2次，认为是振荡
        if direction_changes > 2:
            return True

        return False

    def _calculate_average_position(self, position_records: list) -> Dict:
        """
        计算平均位置

        Args:
            position_records: 位置记录列表

        Returns:
            平均位置 {"x": ..., "y": ..., "z": ...}
        """
        if not position_records:
            return {"x": 0, "y": 0, "z": 0}

        sum_x = sum(r["position"].get("x", 0) for r in position_records)
        sum_y = sum(r["position"].get("y", 0) for r in position_records)
        sum_z = sum(r["position"].get("z", 0) for r in position_records)

        count = len(position_records)

        return {
            "x": sum_x / count,
            "y": sum_y / count,
            "z": sum_z / count
        }
