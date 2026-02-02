#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务队列管理模块
管理任务序列，支持插入、重试、重新规划
"""
from typing import List, Dict, Any, Optional
from dataclasses import dataclass, field
from enum import Enum


class TaskStatus(Enum):
    """任务状态枚举"""
    PENDING = "pending"          # 待执行
    IN_PROGRESS = "in_progress"  # 执行中
    COMPLETED = "completed"      # 已完成
    FAILED = "failed"            # 失败
    SKIPPED = "skipped"          # 跳过


@dataclass
class Task:
    """任务数据类"""
    step: int
    task: str
    type: str
    status: TaskStatus = TaskStatus.PENDING
    retry_count: int = 0
    max_retries: int = 3
    result: Optional[Dict[str, Any]] = None
    error: Optional[str] = None
    dependencies: List[int] = field(default_factory=list)  # 依赖的任务步骤

    def can_retry(self) -> bool:
        """检查是否可以重试"""
        return self.retry_count < self.max_retries

    def increment_retry(self):
        """增加重试计数"""
        self.retry_count += 1


class TaskQueue:
    """
    任务队列管理器

    支持功能：
    - 添加任务到队列
    - 获取下一个待执行任务
    - 任务完成/失败标记
    - 任务重试
    - 插入新任务到队列前端
    - 获取队列状态
    """

    def __init__(self):
        self.tasks: List[Task] = []
        self.current_index = 0
        self.completed_count = 0
        self.failed_count = 0

    def set_tasks(self, tasks_data: List[Dict[str, Any]]):
        """
        设置任务列表（从高层LLM的规划结果）

        Args:
            tasks_data: 任务数据列表，格式：[{"step": 1, "task": "...", "type": "..."}, ...]
        """
        self.tasks = []
        self.current_index = 0
        self.completed_count = 0
        self.failed_count = 0

        for task_data in tasks_data:
            task = Task(
                step=task_data["step"],
                task=task_data["task"],
                type=task_data["type"],
                status=TaskStatus.PENDING
            )
            self.tasks.append(task)

        print(f"📋 [任务队列] 已加载 {len(self.tasks)} 个任务")

    def get_next_task(self) -> Optional[Task]:
        """
        获取下一个待执行的任务

        Returns:
            Task对象，如果没有待执行任务则返回None
        """
        while self.current_index < len(self.tasks):
            task = self.tasks[self.current_index]

            # 跳过已完成或跳过的任务
            if task.status in [TaskStatus.COMPLETED, TaskStatus.SKIPPED]:
                self.current_index += 1
                continue

            # 返回待执行的任务
            if task.status == TaskStatus.PENDING:
                task.status = TaskStatus.IN_PROGRESS
                return task

            # 如果任务失败且不能重试，跳过
            if task.status == TaskStatus.FAILED and not task.can_retry():
                print(f"⚠️  [任务队列] 任务步骤{task.step}已达到最大重试次数，跳过")
                task.status = TaskStatus.SKIPPED
                self.failed_count += 1
                self.current_index += 1
                continue

            # 如果任务失败但可以重试，返回该任务
            if task.status == TaskStatus.FAILED and task.can_retry():
                task.status = TaskStatus.IN_PROGRESS
                task.increment_retry()
                print(f"🔄 [任务队列] 重试任务步骤{task.step} (第{task.retry_count}次)")
                return task

            self.current_index += 1

        return None

    def mark_completed(self, task: Task, result: Dict[str, Any]):
        """
        标记任务为已完成

        Args:
            task: 任务对象
            result: 执行结果
        """
        task.status = TaskStatus.COMPLETED
        task.result = result
        self.completed_count += 1
        self.current_index += 1

    def mark_failed(self, task: Task, error: str):
        """
        标记任务为失败

        Args:
            task: 任务对象
            error: 错误信息
        """
        task.status = TaskStatus.FAILED
        task.error = error
        # 失败计数在get_next_task中更新

    def retry_task(self, task: Task):
        """
        重试失败的任务

        Args:
            task: 任务对象
        """
        if task.can_retry():
            task.status = TaskStatus.PENDING
            print(f"🔄 [任务队列] 任务步骤{task.step}将被重试")

    def insert_tasks(self, tasks_data: List[Dict[str, Any]], at_front: bool = True):
        """
        插入新任务到队列

        Args:
            tasks_data: 要插入的任务数据列表
            at_front: 是否插入到队列前端（用于重新规划的任务）
        """
        new_tasks = []
        for task_data in tasks_data:
            task = Task(
                step=task_data["step"],
                task=task_data["task"],
                type=task_data["type"],
                status=TaskStatus.PENDING
            )
            new_tasks.append(task)

        if at_front:
            # 插入到当前任务之后
            insert_index = self.current_index
            self.tasks[insert_index:insert_index] = new_tasks

            # 重新编号后续任务的步骤
            self._renumber_tasks(insert_index)
            print(f"📋 [任务队列] 在当前位置插入了 {len(new_tasks)} 个新任务")
        else:
            # 添加到队列末尾
            self.tasks.extend(new_tasks)
            print(f"📋 [任务队列] 在队列末尾添加了 {len(new_tasks)} 个新任务")

    def _renumber_tasks(self, start_index: int):
        """重新编号任务的步骤号"""
        current_step = self.tasks[start_index].step
        for i in range(start_index, len(self.tasks)):
            self.tasks[i].step = current_step
            current_step += 1

    def is_empty(self) -> bool:
        """检查队列是否为空"""
        return self.current_index >= len(self.tasks)

    def get_progress(self) -> Dict[str, Any]:
        """
        获取队列进度信息

        Returns:
            包含进度的字典
        """
        total = len(self.tasks)
        pending = sum(1 for t in self.tasks if t.status == TaskStatus.PENDING)
        in_progress = sum(1 for t in self.tasks if t.status == TaskStatus.IN_PROGRESS)

        return {
            "total": total,
            "completed": self.completed_count,
            "failed": self.failed_count,
            "pending": pending,
            "in_progress": in_progress,
            "current_step": self.current_index + 1 if not self.is_empty() else total,
            "progress_percent": (self.completed_count / total * 100) if total > 0 else 100
        }

    def get_task_by_step(self, step: int) -> Optional[Task]:
        """根据步骤号获取任务"""
        for task in self.tasks:
            if task.step == step:
                return task
        return None

    def print_summary(self):
        """打印任务队列摘要"""
        progress = self.get_progress()
        print("\n" + "="*60)
        print(f"📊 [任务队列摘要]")
        print(f"  总任务数: {progress['total']}")
        print(f"  已完成: {progress['completed']} ({progress['completed']}/{progress['total']})")
        print(f"  失败: {progress['failed']}")
        print(f"  待执行: {progress['pending']}")
        print(f"  进度: {progress['progress_percent']:.1f}%")
        print("="*60)

        # 打印每个任务的状态
        print("\n任务详情:")
        for task in self.tasks:
            status_icon = {
                TaskStatus.PENDING: "⏳",
                TaskStatus.IN_PROGRESS: "▶️",
                TaskStatus.COMPLETED: "✅",
                TaskStatus.FAILED: "❌",
                TaskStatus.SKIPPED: "⏭️"
            }.get(task.status, "❓")

            retry_info = f" (重试{task.retry_count}/{task.max_retries})" if task.retry_count > 0 else ""
            print(f"  {status_icon} 步骤{task.step}: {task.task} [{task.status.value}]{retry_info}")
