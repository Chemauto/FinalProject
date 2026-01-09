# -*- coding: utf-8 -*-
"""
ROS1 Adapter
ROS1 (Robot Operating System 1) 适配器
通过ROS1的Publisher/Subscriber模式发送机器人控制命令
"""
import sys
import json
from typing import Dict, Any, Optional

# Reconfigure stdout to use UTF-8 encoding on Windows
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

try:
    import rospy
    from std_msgs.msg import String
    ROS1_AVAILABLE = True
except ImportError:
    ROS1_AVAILABLE = False
    print("INFO: rospy not installed. ROS1 adapter will be disabled.")
    print("      To enable ROS1 support, install ROS1: http://wiki.ros.org/ROS/Installation")

from .base_adapter import BaseAdapter


class ROS1Adapter(BaseAdapter):
    """ROS1框架适配器"""

    def __init__(self, config: Dict[str, Any] = None):
        """
        初始化ROS1适配器

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
        self.publisher = None
        self.is_initialized = False

        if not ROS1_AVAILABLE:
            print("[ROS1Adapter] rospy not available")
            return

        try:
            # 初始化ROS节点（如果还没初始化）
            if not rospy.is_node_initialized():
                rospy.init_node(self.node_name, anonymous=True, disable_signals=True)
                print(f"[ROS1Adapter] ROS node '{self.node_name}' initialized")

            # 创建Publisher
            self.publisher = rospy.Publisher(
                self.topic_name,
                String,
                queue_size=self.queue_size
            )

            # 等待publisher连接
            rospy.sleep(0.5)

            self.is_connected = True
            self.is_initialized = True
            print(f"[ROS1Adapter] Publisher created on topic '{self.topic_name}'")

        except Exception as e:
            print(f"[ROS1Adapter] Failed to initialize: {e}")
            self.is_connected = False

    def connect(self) -> bool:
        """
        建立连接（ROS在初始化时已连接）

        Returns:
            是否连接成功
        """
        if not ROS1_AVAILABLE:
            print("[ROS1Adapter] rospy not available")
            return False

        if self.is_initialized:
            return True

        # 尝试重新初始化
        try:
            if not rospy.is_node_initialized():
                rospy.init_node(self.node_name, anonymous=True, disable_signals=True)

            if not self.publisher:
                self.publisher = rospy.Publisher(
                    self.topic_name,
                    String,
                    queue_size=self.queue_size
                )

            self.is_connected = True
            self.is_initialized = True
            print("[ROS1Adapter] Connected")
            return True

        except Exception as e:
            print(f"[ROS1Adapter] Connection failed: {e}")
            return False

    def disconnect(self) -> bool:
        """
        断开连接

        Returns:
            是否断开成功
        """
        if self.publisher:
            self.publisher.unregister()
            self.publisher = None

        self.is_connected = False
        print("[ROS1Adapter] Disconnected")
        return True

    def send_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """
        发送命令到ROS1话题

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
                "error": "ROS1 adapter not available"
            }

        try:
            # 将命令字典转换为JSON字符串
            command_json = json.dumps(command, ensure_ascii=False)

            # 创建ROS消息
            msg = String()
            msg.data = command_json

            # 发布消息
            self.publisher.publish(msg)

            print(f"[ROS1Adapter] Published to '{self.topic_name}': {command['action']}")

            return {
                "success": True,
                "adapter": "ros1",
                "topic": self.topic_name,
                "command": command
            }

        except Exception as e:
            print(f"[ROS1Adapter] Failed to send command: {e}")
            return {
                "success": False,
                "error": str(e),
                "command": command
            }

    def is_available(self) -> bool:
        """
        检查ROS1适配器是否可用

        Returns:
            是否可用
        """
        return ROS1_AVAILABLE and self.is_connected and self.publisher is not None

    def get_info(self) -> Dict[str, Any]:
        """
        获取ROS1适配器信息

        Returns:
            适配器信息
        """
        info = super().get_info()
        info.update({
            "node_name": self.node_name,
            "topic_name": self.topic_name,
            "queue_size": self.queue_size,
            "ros_available": ROS1_AVAILABLE,
            "ros_master_connected": rospy.is_initialized() if ROS1_AVAILABLE else False
        })
        return info
