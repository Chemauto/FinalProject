# -*- coding: utf-8 -*-
"""
Base Adapter Interface
所有适配器的基类，定义统一接口
"""
from abc import ABC, abstractmethod
from typing import Dict, Any


class BaseAdapter(ABC):
    """适配器基类"""

    def __init__(self, config: Dict[str, Any] = None):
        """
        初始化适配器

        Args:
            config: 配置参数
        """
        self.config = config or {}
        self.is_connected = False

    @abstractmethod
    def connect(self) -> bool:
        """
        建立连接

        Returns:
            是否连接成功
        """
        pass

    @abstractmethod
    def disconnect(self) -> bool:
        """
        断开连接

        Returns:
            是否断开成功
        """
        pass

    @abstractmethod
    def send_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """
        发送命令到底层系统

        Args:
            command: 命令字典，格式：
                    {
                        "action": "navigate/pick/place/...",
                        "parameters": {...}
                    }

        Returns:
            执行结果
        """
        pass

    @abstractmethod
    def is_available(self) -> bool:
        """
        检查适配器是否可用

        Returns:
            是否可用
        """
        pass

    def get_info(self) -> Dict[str, Any]:
        """
        获取适配器信息

        Returns:
            适配器信息
        """
        return {
            "type": self.__class__.__name__,
            "config": self.config,
            "connected": self.is_connected
        }
