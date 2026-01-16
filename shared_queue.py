#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
共享队列 - 基于文件的跨进程通信
使用文件系统作为中介，在仿真器和交互界面之间传递命令
"""
import os
import json
import time
import tempfile
from pathlib import Path


class SharedCommandQueue:
    """基于文件的共享命令队列"""

    def __init__(self):
        # 在临时目录创建命令文件
        self.temp_dir = Path(tempfile.gettempdir()) / "robot_finalproject"
        self.temp_dir.mkdir(exist_ok=True)
        self.command_file = self.temp_dir / "commands.jsonl"
        self.lock_file = self.temp_dir / "queue.lock"

        # 清空旧文件
        if self.command_file.exists():
            self.command_file.unlink()

        print(f"[SharedQueue] 命令文件: {self.command_file}", file=__import__('sys').stderr)

    def put(self, action):
        """发送动作到队列"""
        with open(self.command_file, 'a') as f:
            f.write(json.dumps(action) + '\n')

    def get_nowait(self):
        """非阻塞获取队列中的动作"""
        if not self.command_file.exists():
            return None

        try:
            with open(self.command_file, 'r') as f:
                lines = f.readlines()

            if not lines:
                return None

            # 获取第一行
            command = json.loads(lines[0])

            # 重写文件，移除已读的行
            with open(self.command_file, 'w') as f:
                f.writelines(lines[1:])

            return command
        except (json.JSONDecodeError, FileNotFoundError, IndexError):
            return None

    def empty(self):
        """检查队列是否为空"""
        if not self.command_file.exists():
            return True
        try:
            with open(self.command_file, 'r') as f:
                return len(f.readlines()) == 0
        except:
            return True


# 全局实例
_shared_queue = None


def get_shared_queue():
    """获取共享命令队列"""
    global _shared_queue
    if _shared_queue is None:
        _shared_queue = SharedCommandQueue()
    return _shared_queue


if __name__ == "__main__":
    import sys
    queue = get_shared_queue()
    print(f"共享队列已创建: {queue.command_file}", file=sys.stderr)

    # 测试
    test_action = {'action': 'test', 'parameters': {'value': 123}}
    queue.put(test_action)
    print(f"发送测试命令: {test_action}", file=sys.stderr)

    result = queue.get_nowait()
    print(f"接收命令: {result}", file=sys.stderr)
