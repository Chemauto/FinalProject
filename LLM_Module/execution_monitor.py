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

        # 1. 超时检测
        if self.execution_start_time:
            elapsed = current_time - self.execution_start_time
            if elapsed > self.timeout_threshold:
                return Anomaly(
                    type=AnomalyType.TIMEOUT,
                    description=f"任务执行超时（{elapsed:.1f}秒）",
                    severity="high",
                    data={"elapsed_time": elapsed}
                )

        # 2. 卡住检测（需要位置信息）
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
                                data={"stuck_duration": stuck_duration}
                            )
                else:
                    # 位置已更新
                    self.last_position_update_time = current_time

                    # 记录位置历史用于振荡检测
                    self.position_history.append({
                        "time": current_time,
                        "position": current_position
                    })

                    # 3. 振荡检测
                    if self._detect_oscillation():
                        return Anomaly(
                            type=AnomalyType.OSCILLATION,
                            description="检测到振荡行为（来回移动）",
                            severity="medium"
                        )

            self.last_position = current_position

            # 初始化位置更新时间
            if self.last_position_update_time is None:
                self.last_position_update_time = current_time

        # 4. 传感器失效检测
        if current_state and "sensor_status" in current_state:
            sensor_status = current_state["sensor_status"]
            if sensor_status.get("lidar") == "failed" or sensor_status.get("camera") == "failed":
                return Anomaly(
                    type=AnomalyType.SENSOR_FAILURE,
                    description="传感器失效",
                    severity="high",
                    data=sensor_status
                )

        return None

    def _position_unchanged(self, pos1: Dict[str, float], pos2: Dict[str, float], threshold: float = 0.01) -> bool:
        """
        检查两个位置是否相同

        Args:
            pos1: 位置1
            pos2: 位置2
            threshold: 变化阈值

        Returns:
            是否相同
        """
        if not pos1 or not pos2:
            return False

        dx = abs(pos1.get("x", 0) - pos2.get("x", 0))
        dy = abs(pos1.get("y", 0) - pos2.get("y", 0))
        dz = abs(pos1.get("z", 0) - pos2.get("z", 0))

        return (dx + dy + dz) < threshold

    def _detect_oscillation(self, window_size: int = 10) -> bool:
        """
        检测振荡行为

        Args:
            window_size: 检测窗口大小

        Returns:
            是否检测到振荡
        """
        if len(self.position_history) < window_size * 2:
            return False

        # 获取最近的窗口数据
        recent_positions = self.position_history[-window_size:]
        older_positions = self.position_history[-window_size*2:-window_size]

        # 计算两个窗口的平均位置
        recent_avg = self._calculate_average_position(recent_positions)
        older_avg = self._calculate_average_position(older_positions)

        # 如果平均位置接近，但中间有较大变化，说明在振荡
        distance = self._calculate_distance(recent_avg, older_avg)

        # 检查位置变化
        max_variation = 0.0
        for i in range(1, len(self.position_history)):
            dist = self._calculate_distance(
                self.position_history[i]["position"],
                self.position_history[i-1]["position"]
            )
            max_variation = max(max_variation, dist)

        # 如果有较大变化但最终回到原点，认为是振荡
        return distance < 0.1 and max_variation > 0.5

    def _calculate_average_position(self, position_records: list) -> Dict[str, float]:
        """计算平均位置"""
        if not position_records:
            return {"x": 0, "y": 0, "z": 0}

        sum_x = sum(r["position"].get("x", 0) for r in position_records)
        sum_y = sum(r["position"].get("y", 0) for r in position_records)
        sum_z = sum(r["position"].get("z", 0) for r in position_records)

        n = len(position_records)
        return {
            "x": sum_x / n,
            "y": sum_y / n,
            "z": sum_z / n
        }

    def _calculate_distance(self, pos1: Dict[str, float], pos2: Dict[str, float]) -> float:
        """计算两点间距离"""
        dx = pos1.get("x", 0) - pos2.get("x", 0)
        dy = pos1.get("y", 0) - pos2.get("y", 0)
        dz = pos1.get("z", 0) - pos2.get("z", 0)
        return (dx**2 + dy**2 + dz**2) ** 0.5

    def reset(self):
        """重置监控状态"""
        self.execution_start_time = None
        self.last_position = None
        self.last_position_update_time = None
        self.position_history = []
