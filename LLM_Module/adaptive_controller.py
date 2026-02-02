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
        带监控的任务执行

        Args:
            task: 任务对象
            tools: 工具列表
            execute_tool_fn: 工具执行函数
            env_state: 环境状态

        Returns:
            执行结果
        """
        # 重置监控器
        self.execution_monitor.reset()

        # 记录执行开始时间
        import time
        self.execution_monitor.execution_start_time = time.time()

        # 获取上一步结果
        previous_result = self._get_previous_result()

        # ==================== 启动监控 ====================
        print("\n" + "─"*60)
        print("🔍 [监控启动] 两层监控已激活")
        print("─"*60)

        # 第一层：环境状态检查
        if env_state:
            print("\n📍 [第一层：环境监控]")
            env_anomaly = self.execution_monitor.detect_anomaly(
                current_state=env_state,
                task={"task": task.task, "type": task.type}
            )

            if env_anomaly:
                print(f"  ⚠️  检测到异常: {env_anomaly.description}")
                print(f"  📊 严重程度: {env_anomaly.severity}")
                print(f"  📋 异常数据: {env_anomaly.data}")
            else:
                print("  ✅ 环境状态正常")
                if "position" in env_state:
                    pos = env_state["position"]
                    print(f"  📍 位置: x={pos.get('x', 0):.2f}, y={pos.get('y', 0):.2f}, z={pos.get('z', 0):.2f}")
                if "sensor_status" in env_state:
                    sensors = env_state["sensor_status"]
                    print(f"  🔌 传感器: {', '.join([f'{k}={v}' for k, v in sensors.items()])}")
        else:
            print("\n📍 [第一层：环境监控] ⚠️  未提供环境状态")
            env_anomaly = None

        # 第二层：技能执行监控（在 execute_tool_fn 中进行）
        print("\n🔧 [第二层：技能监控]")
        print(f"  📋 执行任务: {task.task}")
        print(f"  🏷️  任务类型: {task.type}")
        # ============================================================

        try:
            # 执行任务
            result = self.low_level_llm.execute_task(
                task_description=task.task,
                tools=tools,
                execute_tool_fn=execute_tool_fn,
                previous_result=previous_result
            )

            # ==================== 显示执行结果和监控状态 ====================
            print("\n" + "─"*60)
            print("📊 [执行结果]")
            print("─"*60)

            status = result.get("status", "unknown")
            status_icon = {
                "success": "✅",
                "failed": "❌",
                "requires_replanning": "🔄",
                "timeout": "⏱️ "
            }.get(status, "❓")

            print(f"{status_icon} 执行状态: {status}")

            # 显示工具调用信息
            if "tool_used" in result:
                print(f"🔧 使用工具: {result['tool_used']}")
            if "parameters" in result:
                print(f"📋 参数: {result['parameters']}")

            # 显示执行时间
            elapsed_time = time.time() - self.execution_monitor.execution_start_time
            print(f"⏱️  执行时间: {elapsed_time:.2f}秒")

            # 检查是否有环境异常
            if env_state and env_anomaly is None:
                # 再次检查环境（可能在执行过程中发生变化）
                final_env_anomaly = self.execution_monitor.detect_anomaly(
                    current_state=env_state,
                    task={"task": task.task, "type": task.type}
                )
                if final_env_anomaly:
                    print(f"\n⚠️  [环境监控] 检测到异常: {final_env_anomaly.description}")
                    result["anomaly_detected"] = True
                    result["anomaly"] = {
                        "type": final_env_anomaly.type.value,
                        "description": final_env_anomaly.description,
                        "severity": final_env_anomaly.severity,
                        "data": final_env_anomaly.data
                    }
                else:
                    print("\n✅ [环境监控] 执行过程中环境正常")
            elif env_anomaly:
                # 执行前就检测到异常
                print(f"\n⚠️  [环境监控] 执行前已检测到异常: {env_anomaly.description}")
                result["anomaly_detected"] = True
                result["anomaly"] = {
                    "type": env_anomaly.type.value,
                    "description": env_anomaly.description,
                    "severity": env_anomaly.severity,
                    "data": env_anomaly.data
                }

            print("─"*60 + "\n")
            # ============================================================

            return result

        except Exception as e:
            print("\n" + "─"*60)
            print("❌ [执行异常]")
            print("─"*60)
            print(f"💥 异常类型: {type(e).__name__}")
            print(f"📝 异常信息: {str(e)}")
            print("─"*60 + "\n")

            return {
                "status": "failed",
                "error": str(e),
                "task": task.task
            }

    async def _monitor_task_execution(self,
                                      task: Task,
                                      env_state: Dict[str, Any]) -> Optional[Anomaly]:
        """
        监控任务执行（后台运行）

        Args:
            task: 任务对象
            env_state: 环境状态

        Returns:
            检测到的异常，如果没有异常则返回None
        """
        try:
            while True:
                # 定期检测异常
                anomaly = self.execution_monitor.detect_anomaly(
                    current_state=env_state,
                    task={"task": task.task, "type": task.type}
                )

                if anomaly:
                    return anomaly

                # 等待下一次检查
                await asyncio.sleep(self.execution_monitor.monitoring_interval)

        except asyncio.CancelledError:
            # 任务被取消（正常结束）
            return None

    async def handle_execution_result(self,
                                      task: Task,
                                      result: Dict[str, Any],
                                      env_state: Dict[str, Any],
                                      available_skills: List[str]):
        """
        处理执行结果

        Args:
            task: 任务对象
            result: 执行结果
            env_state: 环境状态
            available_skills: 可用技能列表
        """
        status = result.get("status", ExecutionStatus.FAILED.value)

        if status == ExecutionStatus.SUCCESS.value:
            # ==================== 检查是否有异常 ====================
            if result.get("anomaly_detected"):
                # 监控器检测到异常，需要重新规划
                anomaly = result.get("anomaly", {})
                print(f"⚠️  [异常] 任务执行成功但检测到异常: {anomaly.get('description', 'Unknown')}")

                await self.trigger_replanning(
                    task=task,
                    result=result,
                    env_state=env_state,
                    available_skills=available_skills,
                    level=self._determine_replan_level_from_anomaly(anomaly)
                )
            else:
                # ==================== 完全成功 ====================
                print(f"✅ [成功] 任务完成: {task.task}")
                self.task_queue.mark_completed(task, result)
            # ==========================================================

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

            # ==================== 判断是否需要重新规划 ====================
            if self._should_replan(task, result):
                print(f"🔄 [决策] 失败需要重新规划")
                await self.trigger_replanning(
                    task=task,
                    result=result,
                    env_state=env_state,
                    available_skills=available_skills,
                    level=self._determine_replan_level(result)
                )
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
        判断是否应该重新规划

        Args:
            task: 任务对象
            result: 执行结果

        Returns:
            是否应该重新规划
        """
        # 1. 如果达到最大重试次数，必须重新规划
        if not task.can_retry():
            return True

        # 2. 某些类型的错误需要重新规划
        error = result.get("error", "").lower()

        # 环境相关错误
        if "environment" in error:
            return True

        # 障碍物错误
        if "obstacle" in error or "blocked" in error:
            return True

        # 目标丢失
        if "target" in error and ("lost" in error or "not found" in error):
            return True

        # 其他情况不重新规划（使用重试机制）
        return False

    def _determine_replan_level(self, result: Dict[str, Any]) -> ReplanLevel:
        """
        确定重新规划级别

        Args:
            result: 执行结果

        Returns:
            重新规划级别
        """
        error = result.get("error", "").lower()

        # 环境变化 -> 完全重新规划
        if "environment" in error:
            return ReplanLevel.FULL_REPLAN

        # 障碍物 -> 技能替换
        if "obstacle" in error or "blocked" in error:
            return ReplanLevel.SKILL_REPLACEMENT

        # 超时 -> 参数调整
        if "timeout" in error:
            return ReplanLevel.PARAMETER_ADJUSTMENT

        # 卡住 -> 技能替换
        if "stuck" in error:
            return ReplanLevel.SKILL_REPLACEMENT

        # 振荡 -> 任务重排
        if "oscillation" in error:
            return ReplanLevel.TASK_REORDER

        # 默认参数调整
        return ReplanLevel.PARAMETER_ADJUSTMENT

    def _determine_replan_level_from_anomaly(self, anomaly: Dict[str, Any]) -> ReplanLevel:
        """
        根据监控器检测到的异常确定重新规划级别

        Args:
            anomaly: 异常信息字典

        Returns:
            重新规划级别
        """
        anomaly_type = anomaly.get("type", "")
        severity = anomaly.get("severity", "medium")

        # 环境变化 -> 完全重新规划
        if anomaly_type == "environment_change":
            return ReplanLevel.FULL_REPLAN

        # 传感器失效 -> 完全重新规划
        if anomaly_type == "sensor_failure":
            return ReplanLevel.FULL_REPLAN

        # 超时 -> 参数调整
        if anomaly_type == "timeout":
            return ReplanLevel.PARAMETER_ADJUSTMENT

        # 卡住 -> 根据严重程度决定
        if anomaly_type == "stuck":
            if severity == "high":
                return ReplanLevel.SKILL_REPLACEMENT
            else:
                return ReplanLevel.PARAMETER_ADJUSTMENT

        # 振荡 -> 任务重排
        if anomaly_type == "oscillation":
            return ReplanLevel.TASK_REORDER

        # 默认参数调整
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
