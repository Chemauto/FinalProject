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
        # ==================== 后续添加异常检测逻辑 ====================
        # TODO: 根据实际需求添加以下检测：
        #
        # 1. 超时检测
        #    if elapsed > self.timeout_threshold:
        #        return Anomaly(type=AnomalyType.TIMEOUT, ...)
        #
        # 2. 卡住检测（位置长时间不变）
        #    if position_unchanged_duration > self.stuck_threshold:
        #        return Anomaly(type=AnomalyType.STUCK, ...)
        #
        # 3. 振荡检测（来回移动）
        #    if oscillation_detected:
        #        return Anomaly(type=AnomalyType.OSCILLATION, ...)
        #
        # 4. 传感器失效检测
        #    if sensor_status == "failed":
        #        return Anomaly(type=AnomalyType.SENSOR_FAILURE, ...)
        #
        # 5. 环境变化检测
        #    if environment_changed:
        #        return Anomaly(type=AnomalyType.ENVIRONMENT_CHANGE, ...)
        #
        # ===============================================================

        # 暂时不检测异常，返回None
        return None

    def reset(self):
        """重置监控状态"""
        self.execution_start_time = None
        self.last_position = None
        self.last_position_update_time = None
        self.position_history = []
