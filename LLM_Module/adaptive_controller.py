#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自适应控制器
协调高层规划、低层执行和重新规划
"""
import asyncio
from typing import Dict, List, Any, Optional, Callable
from enum import Enum

from .high_level_llm import HighLevelLLM
from .low_level_llm import LowLevelLLM, ExecutionStatus
from .task_queue import TaskQueue, Task, TaskStatus
from .execution_monitor import ExecutionMonitor, Anomaly, AnomalyType


class ReplanLevel(Enum):
    """重新规划级别"""
    PARAMETER_ADJUSTMENT = 1  # 参数调整（不改变任务）
    SKILL_REPLACEMENT = 2     # 技能替换（相同目标，不同方法）
    TASK_REORDER = 3          # 任务重排（调整顺序）
    FULL_REPLAN = 4           # 完全重新规划


class AdaptiveController:
    """
    自适应控制器

    协调高层规划、低层执行和重新规划：
    1. 初始规划
    2. 执行循环
    3. 异常检测
    4. 自适应重新规划
    """

    def __init__(self,
                 high_level_llm: HighLevelLLM,
                 low_level_llm: LowLevelLLM,
                 execution_monitor: Optional[ExecutionMonitor] = None):
        """
        初始化自适应控制器

        Args:
            high_level_llm: 高层LLM实例
            low_level_llm: 低层LLM实例
            execution_monitor: 执行监控器实例（可选）
        """
        self.high_level_llm = high_level_llm
        self.low_level_llm = low_level_llm
        self.execution_monitor = execution_monitor or ExecutionMonitor()
        self.task_queue = TaskQueue()

        # 统计信息
        self.original_user_input = ""
        self.replan_count = 0
        self.max_replans = 3

    async def run(self,
                  user_input: str,
                  tools: List[Dict],
                  execute_tool_fn: Callable,
                  available_skills: List[str],
                  env_state: Optional[Dict[str, Any]] = None) -> List[Dict[str, Any]]:
        """
        运行自适应控制循环

        Args:
            user_input: 用户自然语言指令
            tools: 可用工具列表（OpenAI function calling格式）
            execute_tool_fn: 工具执行函数
            available_skills: 可用技能名称列表
            env_state: 初始环境状态

        Returns:
            执行结果列表
        """
        print("\n" + "█"*60)
        print(f"📥 [自适应控制器] 用户输入: {user_input}")
        print("█"*60)

        self.original_user_input = user_input

        # 1. 初始规划
        print("\n🔵 [阶段1] 初始任务规划")
        tasks = self.high_level_llm.plan_tasks(
            user_input=user_input,
            available_skills=available_skills,
            env_state=env_state
        )

        if not tasks:
            print("❌ [错误] 未能生成任务规划")
            return []

        self.task_queue.set_tasks(tasks)

        # 2. 执行循环
        print("\n🔵 [阶段2] 执行任务序列")
        results = []

        while not self.task_queue.is_empty() and self.replan_count < self.max_replans:
            task = self.task_queue.get_next_task()

            if task is None:
                break

            print(f"\n【步骤 {task.step}/{len(self.task_queue.tasks)}】")

            # 执行任务
            result = await self.execute_with_monitoring(
                task=task,
                tools=tools,
                execute_tool_fn=execute_tool_fn,
                env_state=env_state
            )

            results.append(result)

            # 3. 处理结果
            await self.handle_execution_result(task, result, env_state, available_skills)

            # 更新进度
            progress = self.task_queue.get_progress()
            print(f"\n📊 [进度] {progress['completed']}/{progress['total']} "
                  f"({progress['progress_percent']:.1f}%)")

        # 4. 完成
        print("\n" + "█"*60)
        print("✅ [执行完成] 任务总结")
        print("█"*60)

        self.task_queue.print_summary()

        return results

    async def execute_with_monitoring(self,
                                      task: Task,
                                      tools: List[Dict],
                                      execute_tool_fn: Callable,
                                      env_state: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        带监控的任务执行（简化版）

        Args:
            task: 任务对象
            tools: 工具列表
            execute_tool_fn: 工具执行函数
            env_state: 环境状态（暂未使用）

        Returns:
            执行结果
        """
        # ==================== 后续添加监控逻辑 ====================
        # TODO: 后续可以在这里添加：
        # 1. 后台监控任务 - 在执行时定期检测异常
        # 2. 实时状态检查 - 检查机器人状态、环境变化等
        # 3. 异常处理 - 根据检测到的异常类型触发相应处理
        # ==========================================================

        # 重置监控器
        self.execution_monitor.reset()

        # 获取上一步结果
        previous_result = self._get_previous_result()

        try:
            # 直接执行任务（暂不启动后台监控）
            result = self.low_level_llm.execute_task(
                task_description=task.task,
                tools=tools,
                execute_tool_fn=execute_tool_fn,
                previous_result=previous_result
            )

            return result

        except Exception as e:
            return {
                "status": "failed",
                "error": str(e),
                "task": task.task
            }

    async def _monitor_task_execution(self,
                                      task: Task,
                                      env_state: Dict[str, Any]) -> Optional[Anomaly]:
        """
        监控任务执行（后台运行）- 暂未使用

        后续可以启用此方法来实现实时后台监控
        """
        # TODO: 后续添加后台监控逻辑
        # 目前返回None，不检测异常
        return None

    async def handle_execution_result(self,
                                      task: Task,
                                      result: Dict[str, Any],
                                      env_state: Dict[str, Any],
                                      available_skills: List[str]):
        """
        处理执行结果（简化版）

        Args:
            task: 任务对象
            result: 执行结果
            env_state: 环境状态
            available_skills: 可用技能列表
        """
        # ==================== 后续添加异常处理逻辑 ====================
        # TODO: 后续可以在这里添加：
        # 1. 异常检测 - 根据执行结果判断是否需要重新规划
        # 2. 重新规划决策 - 根据异常类型选择重新规划级别
        # 3. 自动恢复 - 尝试不同的方法完成相同目标
        # ==========================================================

        status = result.get("status", ExecutionStatus.FAILED.value)

        if status == ExecutionStatus.SUCCESS.value:
            # 任务成功
            print(f"✅ [成功] 任务完成: {task.task}")
            self.task_queue.mark_completed(task, result)

        elif status == ExecutionStatus.REQUIRES_REPLANNING.value:
            # 需要重新规划（低层LLM返回）
            print(f"🔄 [重新规划] 检测到环境变化")
            await self.trigger_replanning(
                task=task,
                result=result,
                env_state=env_state,
                available_skills=available_skills,
                level=ReplanLevel.FULL_REPLAN
            )

        else:
            # 任务失败
            reason = result.get("error", "Unknown error")
            print(f"❌ [失败] 任务失败: {task.task}")
            print(f"   原因: {reason}")

            self.task_queue.mark_failed(task, reason)

            # ==================== 后续添加重试逻辑 ====================
            # TODO: 后续可以添加失败后的自动重试和重新规划
            # 目前失败后不自动重新规划，继续执行下一个任务
            # ==========================================================

    async def trigger_replanning(self,
                                 task: Task,
                                 result: Dict[str, Any],
                                 env_state: Dict[str, Any],
                                 available_skills: List[str],
                                 level: ReplanLevel):
        """
        触发重新规划

        Args:
            task: 失败的任务
            result: 执行结果
            env_state: 环境状态
            available_skills: 可用技能列表
            level: 重新规划级别
        """
        self.replan_count += 1

        if self.replan_count > self.max_replans:
            print(f"⚠️  [警告] 已达到最大重新规划次数 ({self.max_replans})，停止重新规划")
            return

        print(f"\n🔄 [重新规划] 第 {self.replan_count} 次 (级别: {level.name})")

        # 调用高层LLM重新规划
        failure_reason = result.get("error", "Unknown error")

        new_tasks = self.high_level_llm.replan_tasks(
            failed_task={
                "step": task.step,
                "task": task.task,
                "type": task.type
            },
            env_state=env_state,
            failure_reason=failure_reason,
            original_user_input=self.original_user_input,
            available_skills=available_skills
        )

        if new_tasks:
            # 插入新任务到队列
            self.task_queue.insert_tasks(new_tasks, at_front=True)
            print(f"✅ [重新规划] 已添加 {len(new_tasks)} 个新任务")
        else:
            print(f"⚠️  [重新规划] 未能生成新任务，将尝试重试原任务")
            if task.can_retry():
                self.task_queue.retry_task(task)

    def _should_replan(self, task: Task, result: Dict[str, Any]) -> bool:
        """
        判断是否应该重新规划（暂未使用）

        后续可以根据任务失败类型、重试次数等判断是否需要重新规划

        Args:
            task: 任务对象
            result: 执行结果

        Returns:
            是否应该重新规划
        """
        # ==================== 后续添加重新规划判断逻辑 ====================
        # TODO: 根据实际需求添加判断逻辑：
        # 1. 环境变化导致失败 -> return True
        # 2. 达到最大重试次数 -> return True
        # 3. 特定错误类型（obstacle, blocked）-> return True
        # ==========================================================

        # 目前不自动重新规划
        return False

    def _determine_replan_level(self, result: Dict[str, Any]) -> ReplanLevel:
        """
        确定重新规划级别（暂未使用）

        后续可以根据错误类型自动选择合适的重新规划级别

        Args:
            result: 执行结果

        Returns:
            重新规划级别
        """
        # ==================== 后续添加级别判断逻辑 ====================
        # TODO: 根据错误类型选择重新规划级别：
        # 1. 环境变化 -> FULL_REPLAN
        # 2. 障碍物 -> SKILL_REPLACEMENT
        # 3. 超时/卡住 -> PARAMETER_ADJUSTMENT
        # 4. 振荡 -> TASK_REORDER
        # ==========================================================

        # 目前默认参数调整
        return ReplanLevel.PARAMETER_ADJUSTMENT

    def _determine_replan_level_from_anomaly(self, anomaly: Dict[str, Any]) -> ReplanLevel:
        """
        根据监控器检测到的异常确定重新规划级别（暂未使用）

        后续可以根据异常类型和严重程度自动选择重新规划级别

        Args:
            anomaly: 异常信息字典

        Returns:
            重新规划级别
        """
        # ==================== 后续添加异常级别映射逻辑 ====================
        # TODO: 根据异常类型选择重新规划级别：
        # 1. environment_change/sensor_failure -> FULL_REPLAN
        # 2. timeout -> PARAMETER_ADJUSTMENT
        # 3. stuck (high severity) -> SKILL_REPLACEMENT
        # 4. stuck (low severity) -> PARAMETER_ADJUSTMENT
        # 5. oscillation -> TASK_REORDER
        # ==========================================================

        # 目前默认参数调整
        return ReplanLevel.PARAMETER_ADJUSTMENT

    def _get_previous_result(self) -> Optional[Any]:
        """
        获取上一步的执行结果

        Returns:
            上一步的结果，如果没有则返回None
        """
        # 获取最后一个已完成的任务结果
        for task in reversed(self.task_queue.tasks):
            if task.status == TaskStatus.COMPLETED and task.result:
                return task.result.get("result")
        return None

    def reset(self):
        """重置控制器状态"""
        self.task_queue = TaskQueue()
        self.replan_count = 0
        self.execution_monitor.reset()
