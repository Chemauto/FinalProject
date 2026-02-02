#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
实时任务监控器

独立运行的监控脚本，实时显示任务执行进度。
通过读取 TaskQueue 的状态来监控执行情况。
"""

import asyncio
import time
import sys
from pathlib import Path
from typing import Dict, Any, Optional
from datetime import datetime

# 添加项目路径
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from LLM_Module.task_queue import TaskQueue, TaskStatus


class TaskProgressMonitor:
    """
    任务进度监控器

    实时监控任务队列的执行进度，不断刷新显示。
    """

    def __init__(self, task_queue: TaskQueue, refresh_interval: float = 0.5):
        """
        初始化监控器

        Args:
            task_queue: 任务队列对象
            refresh_interval: 刷新间隔（秒）
        """
        self.task_queue = task_queue
        self.refresh_interval = refresh_interval
        self.running = False
        self.start_time = None
        self.last_update_time = None

    async def start(self):
        """启动监控"""
        self.running = True
        self.start_time = time.time()

        print("\n" + "="*70)
        print("🔍 [实时监控] 任务执行监控器已启动")
        print("="*70)

        # 不换行，准备实时更新
        sys.stdout.write("\n")

        while self.running:
            await self.update_display()
            await asyncio.sleep(self.refresh_interval)

            # 检查是否所有任务都完成
            if self.task_queue.is_empty():
                await self.update_display()  # 最后更新一次
                break

        self.stop()

    def stop(self):
        """停止监控"""
        self.running = False
        print("\n" + "="*70)
        print("✅ [监控结束] 所有任务已完成")
        print("="*70)

        # 打印最终统计
        self.print_final_summary()

    async def update_display(self):
        """更新显示（使用 ANSI 控制码实现原地刷新）"""
        # 移动光标到上方（覆盖之前的输出）
        sys.stdout.write("\033[F" * 15)  # 上移15行
        sys.stdout.flush()

        # 清空行并打印新内容
        self._print_header()
        self._print_progress()
        self._print_task_list()
        self._print_footer()

        sys.stdout.flush()

    def _print_header(self):
        """打印头部信息"""
        elapsed = time.time() - self.start_time if self.start_time else 0
        progress = self.task_queue.get_progress()

        print("╔" + "═"*68 + "╗")
        print("║" + "🔍 任务执行监控".center(68) + "║")
        print("║" + "═"*68 + "║")
        print(f"║ ⏱️  运行时间: {elapsed:>6.1f}秒   "
              f"📊 总任务: {progress['total']:>2}   "
              f"✅ 完成: {progress['completed']:>2}   "
              f"⏳ 进行中: {progress['in_progress']:>1} ║")
        print(f"║ 📈 进度: {progress['progress_percent']:>5.1f}%   "
              f"❌ 失败: {progress['failed']:>2}   "
              f"⏭️  跳过: {0:>2}   "
              f"⏸️  待执行: {progress['pending']:>2} ║")
        print("╠" + "═"*68 + "╣")

    def _print_progress(self):
        """打印进度条"""
        progress = self.task_queue.get_progress()
        total = progress['total']
        completed = progress['completed']

        if total == 0:
            bar_length = 0
        else:
            bar_length = int(completed / total * 50)

        bar = "█" * bar_length + "░" * (50 - bar_length)
        print(f"║ 进度条: [{bar}] {progress['progress_percent']:>5.1f}% ║")

    def _print_task_list(self):
        """打印任务列表"""
        tasks = self.task_queue.tasks

        if not tasks:
            print("║" + " "*68 + "║")
            print("║" + "📋 暂无任务".center(68) + "║")
            print("║" + " "*68 + "║")
            return

        # 只显示最近的任务（最多8个）
        display_tasks = tasks[:8]

        print("║" + "─"*68 + "║")
        print("║" + "📋 任务列表".center(68) + "║")
        print("║" + "─"*68 + "║")

        for task in display_tasks:
            # 状态图标
            status_icons = {
                TaskStatus.PENDING: "⏳",
                TaskStatus.IN_PROGRESS: "▶️",
                TaskStatus.COMPLETED: "✅",
                TaskStatus.FAILED: "❌",
                TaskStatus.SKIPPED: "⏭️"
            }

            icon = status_icons.get(task.status, "❓")
            status_text = task.status.value.upper().ljust(10)

            # 任务描述（限制长度）
            task_desc = task.task[:40] + "..." if len(task.task) > 40 else task.task
            task_desc = task_desc.ljust(43)

            # 重试信息
            retry_info = ""
            if task.retry_count > 0:
                retry_info = f" (重试{task.retry_count}/{task.max_retries})"

            print(f"║ {icon} [{status_text}] 步骤{task.step:>2}: {task_desc}{retry_info:<10} ║")

        # 如果有更多任务
        if len(tasks) > 8:
            print(f"║ ... 还有 {len(tasks) - 8} 个任务".rjust(70) + " ║")

    def _print_footer(self):
        """打印底部"""
        progress = self.task_queue.get_progress()

        # 当前任务
        current_task = None
        for task in self.task_queue.tasks:
            if task.status == TaskStatus.IN_PROGRESS:
                current_task = task
                break

        if current_task:
            print("║" + "─"*68 + "║")
            print(f"║ ▶️  当前执行: 步骤{current_task.step} - {current_task.task[:55]}... ║")
        else:
            print("║" + "─"*68 + "║")
            print("║" + "⏸️  等待任务开始...".center(68) + "║")

        print("╚" + "═"*68 + "╝")

    def print_final_summary(self):
        """打印最终统计摘要"""
        progress = self.task_queue.get_progress()
        elapsed = time.time() - self.start_time if self.start_time else 0

        print("\n📊 最终统计:")
        print(f"  ⏱️  总耗时: {elapsed:.2f}秒")
        print(f"  📋 总任务数: {progress['total']}")
        print(f"  ✅ 成功: {progress['completed']} ({progress['completed']/progress['total']*100 if progress['total'] > 0 else 0:.1f}%)")
        print(f"  ❌ 失败: {progress['failed']} ({progress['failed']/progress['total']*100 if progress['total'] > 0 else 0:.1f}%)")
        print(f"  ⏭️  跳过: {0}")
        print(f"  📈 平均每个任务: {elapsed/progress['total'] if progress['total'] > 0 else 0:.2f}秒")

        # 打印失败的任务
        failed_tasks = [t for t in self.task_queue.tasks if t.status == TaskStatus.FAILED]
        if failed_tasks:
            print("\n❌ 失败的任务:")
            for task in failed_tasks:
                print(f"  步骤{task.step}: {task.task}")
                if task.error:
                    print(f"    原因: {task.error}")


async def monitor_task_queue_async(task_queue: TaskQueue, refresh_interval: float = 0.5):
    """
    异步监控任务队列

    Args:
        task_queue: 任务队列对象
        refresh_interval: 刷新间隔（秒）
    """
    monitor = TaskProgressMonitor(task_queue, refresh_interval)
    await monitor.start()


def monitor_task_queue(task_queue: TaskQueue, refresh_interval: float = 0.5):
    """
    同步监控任务队列（包装函数）

    Args:
        task_queue: 任务队列对象
        refresh_interval: 刷新间隔（秒）
    """
    asyncio.run(monitor_task_queue_async(task_queue, refresh_interval))


# ============================================================================
# 使用示例
# ============================================================================

async def example_usage():
    """使用示例"""
    from LLM_Module.task_queue import TaskQueue, Task, TaskStatus

    # 创建任务队列
    queue = TaskQueue()
    queue.set_tasks([
        {"step": 1, "task": "向前移动1米", "type": "移动"},
        {"step": 2, "task": "向左转90度", "type": "旋转"},
        {"step": 3, "task": "向前移动2米", "type": "移动"},
        {"step": 4, "task": "追击敌人", "type": "追击"},
    ])

    # 启动监控（后台运行）
    monitor = TaskProgressMonitor(queue, refresh_interval=0.5)
    monitor_task = asyncio.create_task(monitor.start())

    # 模拟执行任务
    for task in queue.tasks:
        # 标记为执行中
        task.status = TaskStatus.IN_PROGRESS
        await asyncio.sleep(1.5)  # 模拟执行

        # 随机失败（演示）
        if task.step == 3:
            task.status = TaskStatus.FAILED
            task.error = "遇到障碍物"
        else:
            task.status = TaskStatus.COMPLETED
            task.result = {"status": "success"}

        # 更新队列计数
        if task.status == TaskStatus.COMPLETED:
            queue.completed_count += 1

    # 等待监控结束
    await monitor_task


if __name__ == "__main__":
    print("\n" + "="*70)
    print("🔍 任务执行监控器 - 示例运行")
    print("="*70)
    print("\n说明: 此监控器会实时刷新显示任务执行进度")
    print("使用 ANSI 控制码实现原地刷新，不会产生大量输出\n")

    # 直接运行演示（移除交互式输入）
    asyncio.run(example_usage())

    print("\n演示完成！")
    print("\n💡 使用方法:")
    print("  from LLM_Module.task_monitor import monitor_task_queue")
    print("  monitor_task_queue(task_queue, refresh_interval=0.5)")
