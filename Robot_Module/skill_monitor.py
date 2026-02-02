#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
技能执行监控器 (Skill Execution Monitor)

监控每个 MCP 工具（skill）的执行状态：
- 执行超时
- 执行失败
- 返回值验证
- 重试机制
- 执行统计
"""

import time
import sys
from typing import Dict, Any, Optional, Callable
from dataclasses import dataclass, field
from enum import Enum
from datetime import datetime
import json


class SkillStatus(Enum):
    """技能执行状态"""
    PENDING = "pending"           # 待执行
    RUNNING = "running"           # 执行中
    SUCCESS = "success"           # 成功
    FAILED = "failed"             # 失败
    TIMEOUT = "timeout"           # 超时
    RETRYING = "retrying"         # 重试中
    SKIPPED = "skipped"           # 跳过


@dataclass
class SkillExecutionRecord:
    """技能执行记录"""
    skill_name: str
    parameters: Dict[str, Any]
    status: SkillStatus = SkillStatus.PENDING
    start_time: Optional[float] = None
    end_time: Optional[float] = None
    duration: Optional[float] = None
    result: Optional[Any] = None
    error: Optional[str] = None
    retry_count: int = 0
    max_retries: int = 3
    execution_log: list = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        return {
            "skill_name": self.skill_name,
            "parameters": self.parameters,
            "status": self.status.value,
            "start_time": self.start_time,
            "end_time": self.end_time,
            "duration": self.duration,
            "result": str(self.result) if self.result else None,
            "error": self.error,
            "retry_count": self.retry_count,
            "max_retries": self.max_retries,
            "execution_log": self.execution_log
        }


class SkillExecutionMonitor:
    """
    技能执行监控器

    功能：
    1. 监控每个技能的执行状态
    2. 检测执行超时
    3. 验证返回值格式
    4. 自动重试失败技能
    5. 统计执行数据
    """

    # 默认超时配置（秒）
    DEFAULT_TIMEOUTS = {
        "move_forward": 30.0,
        "move_backward": 30.0,
        "turn": 20.0,
        "stop": 5.0,
        "get_enemy_positions": 10.0,
        "chase_enemy": 60.0,
        "detect_color_and_act": 30.0
    }

    def __init__(self,
                 default_timeout: float = 30.0,
                 max_retries: int = 3,
                 enable_logging: bool = True):
        """
        初始化技能执行监控器

        Args:
            default_timeout: 默认超时时间（秒）
            max_retries: 最大重试次数
            enable_logging: 是否启用日志记录
        """
        self.default_timeout = default_timeout
        self.max_retries = max_retries
        self.enable_logging = enable_logging

        # 执行历史记录
        self.execution_history: list[SkillExecutionRecord] = []

        # 统计信息
        self.stats = {
            "total_executions": 0,
            "success_count": 0,
            "failed_count": 0,
            "timeout_count": 0,
            "retry_count": 0
        }

    def get_timeout(self, skill_name: str) -> float:
        """获取指定技能的超时时间"""
        return self.DEFAULT_TIMEOUTS.get(skill_name, self.default_timeout)

    async def execute_skill(self,
                           skill_func: Callable,
                           skill_name: str,
                           parameters: Dict[str, Any],
                           validate_result: Optional[Callable] = None) -> SkillExecutionRecord:
        """
        执行技能并进行监控

        Args:
            skill_func: 技能函数（异步函数）
            skill_name: 技能名称
            parameters: 技能参数
            validate_result: 结果验证函数（可选）

        Returns:
            技能执行记录
        """
        # 创建执行记录
        record = SkillExecutionRecord(
            skill_name=skill_name,
            parameters=parameters,
            status=SkillStatus.PENDING,
            max_retries=self.max_retries
        )

        self._log(record, f"开始执行技能: {skill_name}")

        # 执行技能（带重试）
        retry_count = 0
        while retry_count <= self.max_retries:
            try:
                record.status = SkillStatus.RUNNING
                record.retry_count = retry_count
                record.start_time = time.time()

                self._log(record, f"第 {retry_count + 1} 次尝试执行")

                # 计算超时时间
                timeout = self.get_timeout(skill_name)

                # 执行技能（带超时检测）
                import asyncio
                result = await asyncio.wait_for(
                    skill_func(**parameters),
                    timeout=timeout
                )

                record.end_time = time.time()
                record.duration = record.end_time - record.start_time
                record.result = result

                # 验证结果
                if validate_result:
                    is_valid, error_msg = validate_result(result)
                    if not is_valid:
                        raise ValueError(f"结果验证失败: {error_msg}")

                # 执行成功
                record.status = SkillStatus.SUCCESS
                self._log(record, f"✅ 执行成功，耗时 {record.duration:.2f} 秒")
                self.stats["success_count"] += 1
                break

            except asyncio.TimeoutError:
                record.end_time = time.time()
                record.duration = record.end_time - record.start_time
                record.status = SkillStatus.TIMEOUT
                record.error = f"执行超时（超过 {timeout} 秒）"

                self._log(record, f"⏱️  执行超时（{timeout:.1f}秒）")
                self.stats["timeout_count"] += 1

                if retry_count < self.max_retries:
                    record.status = SkillStatus.RETRYING
                    self._log(record, f"准备重试...")
                    retry_count += 1
                    self.stats["retry_count"] += 1
                else:
                    self._log(record, f"❌ 达到最大重试次数，执行失败")
                    self.stats["failed_count"] += 1
                    break

            except Exception as e:
                record.end_time = time.time()
                record.duration = record.end_time - record.start_time
                record.status = SkillStatus.FAILED
                record.error = str(e)

                self._log(record, f"❌ 执行失败: {e}")
                self.stats["failed_count"] += 1

                if retry_count < self.max_retries:
                    record.status = SkillStatus.RETRYING
                    self._log(record, f"准备重试...")
                    retry_count += 1
                    self.stats["retry_count"] += 1
                else:
                    self._log(record, f"❌ 达到最大重试次数，执行失败")
                    break

        self.stats["total_executions"] += 1
        self.execution_history.append(record)

        return record

    def _log(self, record: SkillExecutionRecord, message: str):
        """记录日志"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        log_entry = f"[{timestamp}] {message}"

        if self.enable_logging:
            print(f"[SkillMonitor] {log_entry}", file=sys.stderr)

        record.execution_log.append(log_entry)

    def get_execution_stats(self) -> Dict[str, Any]:
        """获取执行统计信息"""
        total = self.stats["total_executions"]
        if total == 0:
            return {
                "total_executions": 0,
                "success_rate": 0.0,
                "failure_rate": 0.0,
                "timeout_rate": 0.0,
                "avg_retry_rate": 0.0
            }

        return {
            "total_executions": total,
            "success_count": self.stats["success_count"],
            "failed_count": self.stats["failed_count"],
            "timeout_count": self.stats["timeout_count"],
            "retry_count": self.stats["retry_count"],
            "success_rate": self.stats["success_count"] / total * 100,
            "failure_rate": self.stats["failed_count"] / total * 100,
            "timeout_rate": self.stats["timeout_count"] / total * 100,
            "avg_retry_rate": self.stats["retry_count"] / total
        }

    def get_recent_executions(self, n: int = 10) -> list[Dict[str, Any]]:
        """获取最近的执行记录"""
        recent = self.execution_history[-n:]
        return [record.to_dict() for record in recent]

    def print_summary(self):
        """打印执行摘要"""
        stats = self.get_execution_stats()

        print("\n" + "="*60, file=sys.stderr)
        print("📊 [技能执行监控器] 统计摘要", file=sys.stderr)
        print("="*60, file=sys.stderr)
        print(f"  总执行次数: {stats['total_executions']}", file=sys.stderr)
        print(f"  成功: {stats['success_count']} ({stats['success_rate']:.1f}%)", file=sys.stderr)
        print(f"  失败: {stats['failed_count']} ({stats['failure_rate']:.1f}%)", file=sys.stderr)
        print(f"  超时: {stats['timeout_count']} ({stats['timeout_rate']:.1f}%)", file=sys.stderr)
        print(f"  重试次数: {stats['retry_count']}", file=sys.stderr)
        print("="*60 + "\n", file=sys.stderr)

        # 打印最近执行记录
        recent = self.get_recent_executions(5)
        if recent:
            print("最近执行记录:", file=sys.stderr)
            for record in recent:
                status_icon = {
                    SkillStatus.SUCCESS: "✅",
                    SkillStatus.FAILED: "❌",
                    SkillStatus.TIMEOUT: "⏱️ ",
                    SkillStatus.RETRYING: "🔄"
                }.get(SkillStatus(record["status"]), "❓")

                print(f"  {status_icon} {record['skill_name']} - "
                      f"{record['status']} "
                      f"({record['duration']:.2f}s)" if record['duration'] else f"  {status_icon} {record['skill_name']} - {record['status']}",
                      file=sys.stderr)

    def reset(self):
        """重置监控器"""
        self.execution_history.clear()
        self.stats = {
            "total_executions": 0,
            "success_count": 0,
            "failed_count": 0,
            "timeout_count": 0,
            "retry_count": 0
        }


# ============================================================================
# 结果验证函数
# ============================================================================

def validate_json_result(result: str) -> tuple[bool, Optional[str]]:
    """
    验证 JSON 结果

    Args:
        result: 技能返回的 JSON 字符串

    Returns:
        (is_valid, error_message)
    """
    try:
        parsed = json.loads(result)
        if not isinstance(parsed, dict):
            return False, "结果不是字典类型"
        return True, None
    except json.JSONDecodeError as e:
        return False, f"JSON 解析失败: {e}"


def validate_action_result(result: str) -> tuple[bool, Optional[str]]:
    """
    验证动作执行结果

    Args:
        result: 技能返回的结果字符串

    Returns:
        (is_valid, error_message)
    """
    # 先验证是有效的 JSON
    is_valid, error = validate_json_result(result)
    if not is_valid:
        return False, error

    parsed = json.loads(result)

    # 检查是否有 action 字段（底盘控制技能）
    if "action" in parsed:
        if not isinstance(parsed["action"], str):
            return False, "action 字段必须是字符串"
        return True, None

    # 检查是否有 success 字段（复杂技能）
    if "success" in parsed:
        if not isinstance(parsed["success"], bool):
            return False, "success 字段必须是布尔值"
        return True, None

    # 其他格式也接受
    return True, None
