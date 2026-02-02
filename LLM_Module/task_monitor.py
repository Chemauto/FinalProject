#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
实时任务监控器

监控 interactive.py 中执行任务时的实时进度。
通过文件共享方式读取 TaskQueue 状态。
"""

import asyncio
import time
import sys
import json
from pathlib import Path
from datetime import datetime
from typing import Dict, Any, Optional


class TaskProgressMonitor:
    """
    任务进度监控器

    通过读取状态文件来监控 TaskQueue 的执行进度。
    适用于监控 interactive.py 的任务执行。
    """

    def __init__(self, state_file: str = "/tmp/task_queue_state.json", refresh_interval: float = 0.3):
        """
        初始化监控器

        Args:
            state_file: 状态文件路径
            refresh_interval: 刷新间隔（秒）
        """
        self.state_file = Path(state_file)
        self.refresh_interval = refresh_interval
        self.running = False
        self.start_time = None
        self.last_state = None

    async def start(self):
        """启动监控"""
        self.running = True
        self.start_time = time.time()

        # 清除旧的状态文件
        if self.state_file.exists():
            self.state_file.unlink()

        print("\n" + "="*70)
        print("🔍 [实时监控] 任务执行监控器已启动")
        print("="*70)
        print(f"📁 状态文件: {self.state_file}")
        print(f"⏱️  刷新间隔: {self.refresh_interval}秒")
        print("\n等待任务开始...")
        print("="*70 + "\n")

        # 等待状态文件出现
        while self.running:
            if self.state_file.exists():
                break
            await asyncio.sleep(0.5)

        # 开始监控
        while self.running:
            await self.update_display()

            # 检查是否所有任务都完成
            state = self.read_state()
            if state and state.get("all_completed", False):
                await self.update_display()  # 最后更新一次
                break

            await asyncio.sleep(self.refresh_interval)

        self.stop()

    def stop(self):
        """停止监控"""
        self.running = False

        # 删除状态文件
        if self.state_file.exists():
            try:
                self.state_file.unlink()
            except Exception:
                pass  # 忽略删除失败

        print("\n" + "="*70)
        print("✅ [监控结束] 所有任务已完成")
        print("="*70)

        # 打印最终统计
        # 注意：此时状态文件已删除，需要从缓存中读取
        if self.last_state:
            self._print_final_summary_from_cache()
        else:
            self.print_final_summary()

    def read_state(self) -> Optional[Dict[str, Any]]:
        """读取状态文件"""
        try:
            if not self.state_file.exists():
                return None

            with open(self.state_file, 'r') as f:
                return json.load(f)
        except Exception:
            return None

    async def update_display(self):
        """更新显示"""
        state = self.read_state()

        if not state:
            self._print_waiting()
            return

        # 缓存最后一次状态
        self.last_state = state

        # 清空屏幕并重新绘制
        sys.stdout.write("\033[H\033[J")  # 清屏
        sys.stdout.flush()

        self._print_header(state)
        self._print_progress(state)
        self._print_task_list(state)
        self._print_footer(state)

        sys.stdout.flush()

    def _print_waiting(self):
        """打印等待状态"""
        sys.stdout.write("\033[H\033[J")  # 清屏
        print("╔" + "═"*68 + "╗")
        print("║" + "🔍 任务执行监控".center(68) + "║")
        print("╠" + "═"*68 + "╣")
        print("║" + "⏸️  等待任务开始...".center(68) + "║")
        print("╚" + "═"*68 + "╝")
        sys.stdout.flush()

    def _print_header(self, state: Dict[str, Any]):
        """打印头部"""
        elapsed = time.time() - self.start_time if self.start_time else 0
        total = state.get("total", 0)
        completed = state.get("completed", 0)
        failed = state.get("failed", 0)
        in_progress = state.get("in_progress", 0)
        pending = state.get("pending", 0)

        print("╔" + "═"*68 + "╗")
        print("║" + "🔍 任务执行监控".center(68) + "║")
        print("║" + "═"*68 + "║")
        print(f"║ ⏱️  运行时间: {elapsed:>6.1f}秒   "
              f"📊 总任务: {total:>2}   "
              f"✅ 完成: {completed:>2}   "
              f"⏳ 进行中: {in_progress:>1} ║")
        print(f"║ 📈 进度: {(completed/total*100 if total > 0 else 0):>5.1f}%   "
              f"❌ 失败: {failed:>2}   "
              f"⏭️  跳过: {0:>2}   "
              f"⏸️  待执行: {pending:>2} ║")
        print("╠" + "═"*68 + "╣")

    def _print_progress(self, state: Dict[str, Any]):
        """打印进度条"""
        total = state.get("total", 0)
        completed = state.get("completed", 0)

        if total == 0:
            bar_length = 0
        else:
            bar_length = int(completed / total * 50)

        bar = "█" * bar_length + "░" * (50 - bar_length)
        progress_percent = completed / total * 100 if total > 0 else 0
        print(f"║ 进度条: [{bar}] {progress_percent:>5.1f}% ║")

    def _print_task_list(self, state: Dict[str, Any]):
        """打印任务列表"""
        tasks = state.get("tasks", [])

        if not tasks:
            print("║" + " "*68 + "║")
            print("║" + "📋 暂无任务".center(68) + "║")
            print("║" + " "*68 + "║")
            return

        print("║" + "─"*68 + "║")
        print("║" + "📋 任务列表".center(68) + "║")
        print("║" + "─"*68 + "║")

        # 只显示最近的任务（最多8个）
        display_tasks = tasks[:8]

        for task in display_tasks:
            # 状态图标
            status_icons = {
                "pending": "⏳",
                "in_progress": "▶️",
                "completed": "✅",
                "failed": "❌",
                "skipped": "⏭️"
            }

            icon = status_icons.get(task.get("status", "pending"), "❓")
            status = task.get("status", "pending").upper().ljust(10)

            # 任务描述
            task_desc = task.get("task", "")[:40]
            if len(task.get("task", "")) > 40:
                task_desc += "..."
            task_desc = task_desc.ljust(43)

            step = task.get("step", 0)
            retry_info = ""
            if task.get("retry_count", 0) > 0:
                retry_info = f" (重试{task.get('retry_count', 0)}/{task.get('max_retries', 3)})"

            print(f"║ {icon} [{status}] 步骤{step:>2}: {task_desc}{retry_info:<10} ║")

        # 如果有更多任务
        if len(tasks) > 8:
            print(f"║ ... 还有 {len(tasks) - 8} 个任务".rjust(70) + " ║")

    def _print_footer(self, state: Dict[str, Any]):
        """打印底部"""
        tasks = state.get("tasks", [])

        # 找到当前执行的任务
        current_task = None
        for task in tasks:
            if task.get("status") == "in_progress":
                current_task = task
                break

        print("║" + "─"*68 + "║")

        if current_task:
            task_desc = current_task.get("task", "")[:55]
            if len(current_task.get("task", "")) > 55:
                task_desc += "..."
            print(f"║ ▶️  当前执行: 步骤{current_task.get('step', 0)} - {task_desc}... ║")
        else:
            # 检查是否全部完成
            completed = state.get("completed", 0)
            total = state.get("total", 0)
            if completed == total and total > 0:
                print("║" + "✅ 所有任务已完成！".center(68) + "║")
            else:
                print("║" + "⏸️  等待任务开始...".center(68) + "║")

        print("╚" + "═"*68 + "╝")

    def print_final_summary(self):
        """打印最终统计"""
        state = self.read_state()
        if not state:
            return

        elapsed = time.time() - self.start_time if self.start_time else 0
        total = state.get("total", 0)
        completed = state.get("completed", 0)
        failed = state.get("failed", 0)

        print("\n📊 最终统计:")
        print(f"  ⏱️  总耗时: {elapsed:.2f}秒")
        print(f"  📋 总任务数: {total}")
        print(f"  ✅ 成功: {completed} ({completed/total*100 if total > 0 else 0:.1f}%)")
        print(f"  ❌ 失败: {failed} ({failed/total*100 if total > 0 else 0:.1f}%)")
        print(f"  📈 平均每个任务: {elapsed/total if total > 0 else 0:.2f}秒")

        # 打印失败的任务
        tasks = state.get("tasks", [])
        failed_tasks = [t for t in tasks if t.get("status") == "failed"]

        if failed_tasks:
            print("\n❌ 失败的任务:")
            for task in failed_tasks:
                print(f"  步骤{task.get('step', 0)}: {task.get('task', '')}")
                if task.get("error"):
                    print(f"    原因: {task.get('error', '')}")

    def _print_final_summary_from_cache(self):
        """从缓存的最后一次状态打印最终统计"""
        if not self.last_state:
            print("\n⚠️  无法获取最终统计信息")
            return

        state = self.last_state
        elapsed = time.time() - self.start_time if self.start_time else 0
        total = state.get("total", 0)
        completed = state.get("completed", 0)
        failed = state.get("failed", 0)

        print("\n📊 最终统计:")
        print(f"  ⏱️  总耗时: {elapsed:.2f}秒")
        print(f"  📋 总任务数: {total}")
        print(f"  ✅ 成功: {completed} ({completed/total*100 if total > 0 else 0:.1f}%)")
        print(f"  ❌ 失败: {failed} ({failed/total*100 if total > 0 else 0:.1f}%)")
        print(f"  📈 平均每个任务: {elapsed/total if total > 0 else 0:.2f}秒")

        # 打印失败的任务
        tasks = state.get("tasks", [])
        failed_tasks = [t for t in tasks if t.get("status") == "failed"]

        if failed_tasks:
            print("\n❌ 失败的任务:")
            for task in failed_tasks:
                print(f"  步骤{task.get('step', 0)}: {task.get('task', '')}")
                if task.get("error"):
                    print(f"    原因: {task.get('error', '')}")


# ============================================================================
# 辅助函数：用于在 AdaptiveController 中写入状态文件
# ============================================================================

def save_queue_state(task_queue, state_file: str = "/tmp/task_queue_state.json"):
    """
    保存 TaskQueue 状态到文件（供监控器读取）

    Args:
        task_queue: TaskQueue 对象
        state_file: 状态文件路径
    """
    try:
        state = {
            "total": len(task_queue.tasks),
            "completed": task_queue.completed_count,
            "failed": task_queue.failed_count,
            "in_progress": sum(1 for t in task_queue.tasks if t.status.value == "in_progress"),
            "pending": sum(1 for t in task_queue.tasks if t.status.value == "pending"),
            "all_completed": task_queue.is_empty(),
            "tasks": [
                {
                    "step": t.step,
                    "task": t.task,
                    "type": t.type,
                    "status": t.status.value,
                    "retry_count": t.retry_count,
                    "max_retries": t.max_retries,
                    "error": t.error
                }
                for t in task_queue.tasks
            ]
        }

        with open(state_file, 'w') as f:
            json.dump(state, f, indent=2)
    except Exception as e:
        pass  # 忽略错误，避免影响主流程


def cleanup_state_file(state_file: str = "/tmp/task_queue_state.json"):
    """
    清理状态文件（在程序退出时调用）

    Args:
        state_file: 状态文件路径
    """
    try:
        from pathlib import Path
        path = Path(state_file)
        if path.exists():
            path.unlink()
    except Exception:
        pass  # 忽略错误


# ============================================================================
# 主函数
# ============================================================================

async def main():
    """主函数 - 启动监控器"""
    monitor = TaskProgressMonitor(refresh_interval=0.3)
    await monitor.start()


if __name__ == "__main__":
    print("\n" + "="*70)
    print("🔍 任务执行监控器")
    print("="*70)
    print("\n说明: 此监控器会实时显示 interactive.py 的任务执行进度")
    print("通过读取状态文件 /tmp/task_queue_state.json 来监控任务队列\n")

    print("💡 使用方法:")
    print("  1. 在一个终端运行: python3 LLM_Module/task_monitor.py")
    print("  2. 在另一个终端运行: python3 Interactive_Module/interactive.py")
    print("  3. 在 interactive.py 中输入指令，监控器会实时显示进度\n")

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\n👋 监控器已停止")
