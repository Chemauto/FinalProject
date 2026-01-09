# -*- coding: utf-8 -*-
"""
Dora Adapter
将命令发送到Dora框架的simulator
"""
import sys
import os
from typing import Dict, Any
import pyarrow as pa

# Reconfigure stdout to use UTF-8 encoding on Windows
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

try:
    from dora import Node
    DORA_AVAILABLE = True
except ImportError:
    DORA_AVAILABLE = False
    print("WARNING: dora-rs not installed. Dora adapter will be disabled.")

from .base_adapter import BaseAdapter


class DoraAdapter(BaseAdapter):
    """Dora框架适配器"""

    def __init__(self, config: Dict[str, Any] = None):
        """
        初始化Dora适配器

        Args:
            config: 配置参数，可包含：
                   - node_id: Dora节点ID（可选）
                   - output_id: 输出ID，默认"command"
        """
        super().__init__(config)
        self.node = None
        self.output_id = self.config.get("output_id", "command")

        if not DORA_AVAILABLE:
            print("ERROR: dora-rs is not installed. Please run: pip install dora-rs")
            return

        try:
            self.node = Node()
            self.is_connected = True
            print("[DoraAdapter] Initialized successfully")
        except Exception as e:
            print(f"[DoraAdapter] Failed to initialize: {e}")
            self.is_connected = False

    def connect(self) -> bool:
        """
        建立连接（Dora节点在初始化时已连接）

        Returns:
            是否连接成功
        """
        if not DORA_AVAILABLE:
            print("[DoraAdapter] dora-rs not available")
            return False

        if not self.node:
            try:
                self.node = Node()
                self.is_connected = True
                print("[DoraAdapter] Connected")
                return True
            except Exception as e:
                print(f"[DoraAdapter] Connection failed: {e}")
                return False

        return self.is_connected

    def disconnect(self) -> bool:
        """
        断开连接

        Returns:
            是否断开成功
        """
        self.is_connected = False
        self.node = None
        print("[DoraAdapter] Disconnected")
        return True

    def send_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """
        发送命令到Dora simulator

        Args:
            command: 命令字典
                    {
                        "action": "navigate/pick/place/...",
                        "parameters": {...}
                    }

        Returns:
            执行结果
        """
        if not self.is_available():
            return {
                "success": False,
                "error": "Dora adapter not available"
            }

        try:
            # 将命令转换为PyArrow数组
            arrow_data = pa.array([command])

            # 发送到Dora输出流
            self.node.send_output(self.output_id, arrow_data)

            print(f"[DoraAdapter] Sent command: {command['action']} - {command.get('parameters', {})}")

            return {
                "success": True,
                "adapter": "dora",
                "command": command
            }

        except Exception as e:
            print(f"[DoraAdapter] Failed to send command: {e}")
            return {
                "success": False,
                "error": str(e),
                "command": command
            }

    def is_available(self) -> bool:
        """
        检查Dora适配器是否可用

        Returns:
            是否可用
        """
        return DORA_AVAILABLE and self.is_connected and self.node is not None
