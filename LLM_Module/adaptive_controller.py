#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化版自适应控制器

仅依赖 high_level_llm / low_level_llm，内部实现任务队列与评价器，
用于保留核心功能：执行、反馈评估、重规划。
"""
import asyncio
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple
from collections import deque

from .high_level_llm import HighLevelLLM
from .low_level_llm import ExecutionStatus, LowLevelLLM


class ReplanLevel(Enum):
    PARAMETER_ADJUSTMENT = 1
    SKILL_REPLACEMENT = 2
    TASK_REORDER = 3
    FULL_REPLAN = 4


@dataclass
class ReplanDecision:
    need_replan: bool = False
    scope: str = "none"
    reason: str = ""
    replace_node_ids: List[str] = field(default_factory=list)
    confidence: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "need_replan": self.need_replan,
            "scope": self.scope,
            "reason": self.reason,
            "replace_node_ids": self.replace_node_ids,
            "confidence": self.confidence,
        }


@dataclass
class Task:
    step: int
    task: str
    type: str
    retry_count: int = 0
    max_retries: int = 2
    status: str = "pending"
    result: Optional[Dict[str, Any]] = None
    error: Optional[str] = None

    def can_retry(self) -> bool:
        return self.retry_count < self.max_retries


class TaskQueue:
    def __init__(self):
        self.tasks: List[Task] = []
        self.current_index = 0
        self.completed_count = 0
        self.failed_count = 0

    def set_tasks(self, tasks_data: List[Dict[str, Any]]):
        self.tasks = [
            Task(
                step=t.get("step", i + 1),
                task=t.get("task", ""),
                type=t.get("type", "综合"),
            )
            for i, t in enumerate(tasks_data)
        ]
        self.current_index = 0
        self.completed_count = 0
        self.failed_count = 0
        print(f"📋 [任务队列] 已加载 {len(self.tasks)} 个任务")

    def get_next_task(self) -> Optional[Task]:
        while self.current_index < len(self.tasks):
            task = self.tasks[self.current_index]

            if task.status == "completed":
                self.current_index += 1
                continue

            if task.status in {"pending", "retry"}:
                task.status = "in_progress"
                return task

            if task.status == "failed":
                if task.can_retry():
                    task.status = "in_progress"
                    return task
                self.current_index += 1
                continue

            self.current_index += 1

        return None

    def mark_completed(self, task: Task, result: Dict[str, Any]):
        task.status = "completed"
        task.result = result
        self.completed_count += 1
        self.current_index += 1

    def mark_failed(self, task: Task, error: str):
        task.status = "failed"
        task.error = error
        self.failed_count += 1

    def retry_task(self, task: Task):
        if task.can_retry():
            task.retry_count += 1
            task.status = "retry"

    def insert_tasks(self, tasks_data: List[Dict[str, Any]], at_front: bool = True):
        new_tasks = [
            Task(
                step=t.get("step", 1),
                task=t.get("task", ""),
                type=t.get("type", "综合"),
            )
            for t in tasks_data
        ]

        if not new_tasks:
            return

        if at_front:
            idx = self.current_index
            self.tasks[idx:idx] = new_tasks
            self._renumber()
        else:
            self.tasks.extend(new_tasks)
            self._renumber()

    def _renumber(self):
        for i, task in enumerate(self.tasks, 1):
            task.step = i

    def is_empty(self) -> bool:
        return self.current_index >= len(self.tasks)

    def get_progress(self) -> Dict[str, Any]:
        total = len(self.tasks)
        return {
            "total": total,
            "completed": self.completed_count,
            "failed": self.failed_count,
            "progress_percent": (self.completed_count / total * 100) if total else 100.0,
        }

    def print_summary(self):
        p = self.get_progress()
        print("\n" + "=" * 60)
        print("📊 [任务队列摘要]")
        print(f"  总任务数: {p['total']}")
        print(f"  已完成: {p['completed']}")
        print(f"  失败: {p['failed']}")
        print(f"  进度: {p['progress_percent']:.1f}%")
        print("=" * 60)


class Evaluator:
    """简化评价器：根据失败率、环境版本变化、卡住状态触发重规划。"""

    def __init__(self, failure_window: int = 4):
        self._status_window: Deque[str] = deque(maxlen=failure_window)
        self._last_env_version: Optional[int] = None
        self._pos_window: Deque[Tuple[float, float, float]] = deque(maxlen=6)

    def evaluate(self, task_id: str, result: Dict[str, Any], env_state: Dict[str, Any]) -> ReplanDecision:
        status = result.get("status", "failed")
        self._status_window.append(status)
        self._update_position(env_state)

        if status == ExecutionStatus.REQUIRES_REPLANNING.value:
            return ReplanDecision(True, "full", "low_level_requires_replanning", [task_id], 0.95)

        if self._environment_changed(env_state):
            return ReplanDecision(True, "branch", "environment_version_changed", [task_id], 0.9)

        if self._stuck():
            return ReplanDecision(True, "branch", "robot_stuck", [task_id], 0.85)

        if status == ExecutionStatus.FAILED.value:
            failed = sum(1 for x in self._status_window if x == ExecutionStatus.FAILED.value)
            if failed / len(self._status_window) >= 0.5:
                return ReplanDecision(True, "branch", "high_failure_ratio", [task_id], 0.8)

        return ReplanDecision(False, "none", "stable", [], 0.6)

    def _environment_changed(self, env_state: Dict[str, Any]) -> bool:
        version = env_state.get("environment_version")
        if version is None:
            return False
        if self._last_env_version is None:
            self._last_env_version = int(version)
            return False
        changed = int(version) != self._last_env_version
        self._last_env_version = int(version)
        return changed

    def _update_position(self, env_state: Dict[str, Any]):
        pos = env_state.get("position", {})
        if not isinstance(pos, dict):
            pos = {}
        self._pos_window.append((
            float(pos.get("x", 0.0)),
            float(pos.get("y", 0.0)),
            float(pos.get("z", 0.0)),
        ))

    def _stuck(self) -> bool:
        if len(self._pos_window) < 6:
            return False
        start = self._pos_window[0]
        end = self._pos_window[-1]
        d = ((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2 + (end[2] - start[2]) ** 2) ** 0.5
        return d < 0.02


class AdaptiveController:
    def __init__(
        self,
        high_level_llm: HighLevelLLM,
        low_level_llm: LowLevelLLM,
        execution_hz: float = 10.0,
        max_replans: int = 3,
    ):
        self.high_level_llm = high_level_llm
        self.low_level_llm = low_level_llm
        self.task_queue = TaskQueue()
        self.evaluator = Evaluator()

        self.execution_interval = 1.0 / max(execution_hz, 1e-6)
        self.max_replans = max_replans
        self.replan_count = 0
        self.original_user_input = ""

    async def run(
        self,
        user_input: str,
        tools: List[Dict[str, Any]],
        execute_tool_fn: Callable,
        available_skills: List[str],
        env_state: Optional[Dict[str, Any]] = None,
        env_state_provider: Optional[Callable[[], Dict[str, Any]]] = None,
        input1: Optional[Dict[str, Any]] = None,
        input2: Optional[Dict[str, Any]] = None,
        input3: Optional[Dict[str, Any]] = None,
        image_path: Optional[str] = None,
    ) -> List[Dict[str, Any]]:
        self.replan_count = 0
        self.original_user_input = (input1 or {}).get("text", user_input) if isinstance(input1, dict) else user_input

        current_env = self._resolve_env_state(env_state, env_state_provider, input2)

        print("\n" + "█" * 60)
        print(f"📥 [自适应控制器] 用户输入: {self.original_user_input}")
        print("█" * 60)

        tasks = self.high_level_llm.plan_tasks(
            user_input=self.original_user_input,
            available_skills=available_skills,
            env_state=self._build_planning_context(current_env, input2, input3),
            image_path=image_path,
        )
        if not tasks:
            return []

        self.task_queue.set_tasks(tasks)
        results: List[Dict[str, Any]] = []

        while not self.task_queue.is_empty() and self.replan_count < self.max_replans:
            loop_start = time.monotonic()
            task = self.task_queue.get_next_task()
            if task is None:
                break

            print(f"\n【步骤 {task.step}/{len(self.task_queue.tasks)}】")
            previous_result = self._get_previous_result()

            result = await asyncio.to_thread(
                self.low_level_llm.execute_task,
                task.task,
                tools,
                execute_tool_fn,
                previous_result,
                current_env,
            )
            results.append(result)

            current_env = self._resolve_env_state(env_state, env_state_provider, input2)
            decision = self.evaluator.evaluate(str(task.step), result, current_env)

            status = result.get("status", ExecutionStatus.FAILED.value)
            if status == ExecutionStatus.SUCCESS.value:
                self.task_queue.mark_completed(task, result)
                print(f"✅ [成功] {task.task}")
            else:
                reason = result.get("error", "Unknown error")
                self.task_queue.mark_failed(task, reason)
                print(f"❌ [失败] {task.task} | {reason}")

            if decision.need_replan:
                await self._trigger_replanning(task, result, current_env, available_skills, decision)
            elif status != ExecutionStatus.SUCCESS.value and task.can_retry():
                self.task_queue.retry_task(task)

            p = self.task_queue.get_progress()
            print(f"📊 [进度] {p['completed']}/{p['total']} ({p['progress_percent']:.1f}%)")

            elapsed = time.monotonic() - loop_start
            if elapsed < self.execution_interval:
                await asyncio.sleep(self.execution_interval - elapsed)

        print("\n" + "█" * 60)
        print("✅ [执行完成] 任务总结")
        print("█" * 60)
        self.task_queue.print_summary()
        return results

    async def _trigger_replanning(
        self,
        task: Task,
        result: Dict[str, Any],
        env_state: Dict[str, Any],
        available_skills: List[str],
        decision: ReplanDecision,
    ):
        self.replan_count += 1
        if self.replan_count > self.max_replans:
            return

        level = self._map_level(decision)
        print(f"🔄 [重新规划] 第 {self.replan_count} 次 ({level.name}) | {decision.reason}")

        new_tasks = self.high_level_llm.replan_tasks(
            failed_task={"step": task.step, "task": task.task, "type": task.type},
            env_state={"env": env_state, "decision": decision.to_dict()},
            failure_reason=result.get("error", decision.reason),
            original_user_input=self.original_user_input,
            available_skills=available_skills,
        )

        if new_tasks:
            self.task_queue.insert_tasks(new_tasks, at_front=True)
            print(f"✅ [重新规划] 新增 {len(new_tasks)} 个任务")

    def _map_level(self, decision: ReplanDecision) -> ReplanLevel:
        if decision.scope == "full":
            return ReplanLevel.FULL_REPLAN
        if decision.reason in {"high_failure_ratio", "robot_stuck"}:
            return ReplanLevel.SKILL_REPLACEMENT
        return ReplanLevel.PARAMETER_ADJUSTMENT

    def _resolve_env_state(
        self,
        base_env: Optional[Dict[str, Any]],
        env_state_provider: Optional[Callable[[], Dict[str, Any]]],
        input2: Optional[Dict[str, Any]],
    ) -> Dict[str, Any]:
        env = dict(base_env or {})
        if env_state_provider:
            try:
                env.update(env_state_provider() or {})
            except Exception:
                pass
        if isinstance(input2, dict):
            env.update(input2)
        return env

    def _build_planning_context(
        self,
        env_state: Dict[str, Any],
        input2: Optional[Dict[str, Any]],
        input3: Optional[Dict[str, Any]],
    ) -> Dict[str, Any]:
        context = dict(env_state)
        if isinstance(input2, dict):
            context["input2"] = input2
        if isinstance(input3, dict):
            context["input3"] = input3
        return context

    def _get_previous_result(self) -> Optional[Any]:
        for task in reversed(self.task_queue.tasks):
            if task.status == "completed" and task.result:
                return task.result.get("result")
        return None
