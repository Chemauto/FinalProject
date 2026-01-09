# -*- coding: utf-8 -*-
"""
ROS2 Adapter
ROS2 (Robot Operating System 2) 适配器
通过ROS2的Publisher/Subscriber模式发送机器人控制命令
"""
import sys
import json
from typing import Dict, Any, Optional

# Reconfigure stdout to use UTF-8 encoding on Windows
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("INFO: rclpy not installed. ROS2 adapter will be disabled.")
    print("      To enable ROS2 support, install ROS2: https://docs.ros.org/en/humble/Installation.html")

from .base_adapter import BaseAdapter


class ROS2Node(Node):
    """ROS2节点类，用于接收MCP命令并发布到话题"""

    def __init__(self, node_name: str):
        """
        初始化ROS2节点

        Args:
            node_name: 节点名称
        """
        super().__init__(node_name)
        self.publisher = None

    def create_publisher_topic(self, topic_name: str, queue_size: int):
        """
        创建发布者

        Args:
            topic_name: 话题名称
            queue_size: 队列大小
        """
        self.publisher = self.create_publisher(String, topic_name, queue_size)


class ROS2Adapter(BaseAdapter):
    """ROS2框架适配器"""

    def __init__(self, config: Dict[str, Any] = None):
        """
        初始化ROS2适配器

        Args:
            config: 配置参数，可包含：
                   - node_name: ROS节点名称，默认"mcp_robot_control"
                   - topic_name: 发布话题名称，默认"/robot_command"
                   - queue_size: 发布队列大小，默认10
        """
        super().__init__(config)
        self.node_name = self.config.get("node_name", "mcp_robot_control")
        self.topic_name = self.config.get("topic_name", "/robot_command")
        self.queue_size = self.config.get("queue_size", 10)
        self.ros2_node = None
        self.executor = None
        self.is_initialized = False

        if not ROS2_AVAILABLE:
            print("[ROS2Adapter] rclpy not available")
            return

        try:
            # 初始化ROS2（如果还没初始化）
            if not rclpy.ok():
                rclpy.init(args=None)
                print(f"[ROS2Adapter] ROS2 initialized")

            # 创建ROS2节点
            self.ros2_node = ROS2Node(self.node_name)

            # 创建Publisher
            self.ros2_node.create_publisher_topic(self.topic_name, self.queue_size)

            # 创建executor并spin节点（非阻塞）
            self.executor = rclpy.executors.SingleThreadedExecutor()
            self.executor.add_node(self.ros2_node)

            # 稍微等待以确保连接建立
            self.executor.spin_once(timeout_sec=0.5)

            self.is_connected = True
            self.is_initialized = True
            print(f"[ROS2Adapter] Node '{self.node_name}' initialized")
            print(f"[ROS2Adapter] Publishing to topic '{self.topic_name}'")

        except Exception as e:
            print(f"[ROS2Adapter] Failed to initialize: {e}")
            self.is_connected = False

    def connect(self) -> bool:
        """
        建立连接（ROS2在初始化时已连接）

        Returns:
            是否连接成功
        """
        if not ROS2_AVAILABLE:
            print("[ROS2Adapter] rclpy not available")
            return False

        if self.is_initialized:
            return True

        # 尝试重新初始化
        try:
            if not rclpy.ok():
                rclpy.init(args=None)

            if not self.ros2_node:
                self.ros2_node = ROS2Node(self.node_name)
                self.ros2_node.create_publisher_topic(self.topic_name, self.queue_size)

                self.executor = rclpy.executors.SingleThreadedExecutor()
                self.executor.add_node(self.ros2_node)
                self.executor.spin_once(timeout_sec=0.5)

            self.is_connected = True
            self.is_initialized = True
            print("[ROS2Adapter] Connected")
            return True

        except Exception as e:
            print(f"[ROS2Adapter] Connection failed: {e}")
            return False

    def disconnect(self) -> bool:
        """
        断开连接

        Returns:
            是否断开成功
        """
        if self.ros2_node:
            if self.executor:
                self.executor.shutdown()
                self.executor = None

            self.ros2_node.destroy_node()
            self.ros2_node = None

        self.is_connected = False
        print("[ROS2Adapter] Disconnected")
        return True

    def send_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """
        发送命令到ROS2话题

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
                "error": "ROS2 adapter not available"
            }

        try:
            # 将命令字典转换为JSON字符串
            command_json = json.dumps(command, ensure_ascii=False)

            # 创建ROS消息
            msg = String()
            msg.data = command_json

            # 发布消息
            self.ros2_node.publisher.publish(msg)

            # 短暂spin以确保消息发送
            if self.executor:
                self.executor.spin_once(timeout_sec=0.1)

            print(f"[ROS2Adapter] Published to '{self.topic_name}': {command['action']}")

            return {
                "success": True,
                "adapter": "ros2",
                "topic": self.topic_name,
                "command": command
            }

        except Exception as e:
            print(f"[ROS2Adapter] Failed to send command: {e}")
            return {
                "success": False,
                "error": str(e),
                "command": command
            }

    def is_available(self) -> bool:
        """
        检查ROS2适配器是否可用

        Returns:
            是否可用
        """
        return ROS2_AVAILABLE and self.is_connected and self.ros2_node is not None

    def get_info(self) -> Dict[str, Any]:
        """
        获取ROS2适配器信息

        Returns:
            适配器信息
        """
        info = super().get_info()
        info.update({
            "node_name": self.node_name,
            "topic_name": self.topic_name,
            "queue_size": self.queue_size,
            "ros2_available": ROS2_AVAILABLE,
            "ros2_initialized": rclpy.ok() if ROS2_AVAILABLE else False
        })
        return info
