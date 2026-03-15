#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手动反馈工具。

用途：
- 监听 `/robot/skill_command`
- 把收到的技能命令放入待反馈队列
- 在终端按 `Y` 时，为当前队首命令发送 SUCCESS 反馈
- 在终端按 `N` 时，为当前队首命令发送 FAILURE 反馈

适合在没有真实执行器时，手动模拟“执行完成后再继续下一步”的闭环。
"""

from __future__ import annotations

import os
import queue
import select
import sys
import termios
import time
import tty
from pathlib import Path

project_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(project_root))

from Comm_Module.execution_comm import (
    get_skill_command_subscriber,
    publish_execution_feedback,
)


class ManualFeedbackCLI:
    """手动反馈命令行工具。"""

    def __init__(self):
        self.pending_commands: queue.Queue[dict] = queue.Queue()
        self.current_command: dict | None = None
        self.subscriber = get_skill_command_subscriber(self._enqueue_command)

    def _enqueue_command(self, command: dict) -> None:
        self.pending_commands.put(command)

    def _drain_pending_commands(self) -> None:
        updated = False
        try:
            while True:
                command = self.pending_commands.get_nowait()
                if self.current_command is None:
                    self.current_command = command
                else:
                    # 只在当前命令为空时切换，避免跳过还未反馈的命令
                    self.pending_commands.put(command)
                    break
                updated = True
        except queue.Empty:
            pass

        if updated and self.current_command is not None:
            self._print_current_command()

    def _print_current_command(self) -> None:
        command = self.current_command
        if command is None:
            print("\n[manual_feedback] 当前没有待反馈命令", file=sys.stderr)
            return

        print("\n" + "-" * 60, file=sys.stderr)
        print("[manual_feedback] 收到待反馈命令", file=sys.stderr)
        print(f"  action_id: {command.get('action_id', '')}", file=sys.stderr)
        print(f"  skill: {command.get('skill', '')}", file=sys.stderr)
        print(f"  parameters: {command.get('parameters', {})}", file=sys.stderr)
        print("  按 Y 发送成功反馈，按 N 发送失败反馈，按 Q 退出", file=sys.stderr)
        print("-" * 60, file=sys.stderr)

    def _send_feedback(self, signal: str) -> None:
        command = self.current_command
        if command is None:
            print("[manual_feedback] 没有可反馈的命令", file=sys.stderr)
            return

        skill = command.get("skill", "unknown")
        action_id = command.get("action_id", "")
        message = (
            f"手动确认 {skill} 执行成功"
            if signal == "SUCCESS"
            else f"手动确认 {skill} 执行失败"
        )

        publish_execution_feedback(
            action_id=action_id,
            skill_name=skill,
            signal=signal,
            message=message,
            result={"source": "manual_feedback_cli"},
        )
        print(f"[manual_feedback] 已发送 {signal} 反馈: {action_id}", file=sys.stderr)
        self.current_command = None
        self._drain_pending_commands()

    @staticmethod
    def _has_keyboard_input() -> bool:
        readable, _, _ = select.select([sys.stdin], [], [], 0)
        return bool(readable)

    def run(self) -> None:
        print("=" * 60, file=sys.stderr)
        print("[manual_feedback] 手动反馈工具已启动", file=sys.stderr)
        print("监听 /robot/skill_command，按 Y/N 发送反馈，按 Q 退出", file=sys.stderr)
        print("=" * 60, file=sys.stderr)

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)

        try:
            while True:
                self.subscriber.spin_once(timeout_sec=0.001)
                self._drain_pending_commands()

                if self._has_keyboard_input():
                    char = sys.stdin.read(1).strip().lower()
                    if char == "y":
                        self._send_feedback("SUCCESS")
                    elif char == "n":
                        self._send_feedback("FAILURE")
                    elif char == "q":
                        print("[manual_feedback] 退出", file=sys.stderr)
                        break

                time.sleep(0.05)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main() -> None:
    cli = ManualFeedbackCLI()
    cli.run()


if __name__ == "__main__":
    main()
