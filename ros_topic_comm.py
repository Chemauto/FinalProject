#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS话题通讯模块
使用ROS2话题在仿真器和机器人控制模块之间传递命令
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sys
import queue

# ROS 初始化标记
_ros_initialized = False


def _ros_init():
    """初始化 ROS（每个进程只初始化一次）"""
    global _ros_initialized
    if not _ros_initialized:
        try:
            rclpy.init()
            _ros_initialized = True
            print("[ros_topic_comm] ROS 已初始化", file=sys.stderr)
        except Exception as e:
            # 可能已经被其他地方初始化过了
            print(f"[ros_topic_comm] ROS 初始化跳过: {e}", file=sys.stderr)


def _ros_shutdown():
    """标记引用释放（不主动调用 rclpy.shutdown，让进程退出时自动清理）"""
    pass


class ActionPublisher:
    """动作命令发布器 - 用于机器人控制模块发送命令"""

    def __init__(self):
        """初始化ROS节点和发布者"""
        _ros_init()
        self.node = Node('robot_action_publisher')
        self.publisher = self.node.create_publisher(
            String,
            '/robot/command',
            10  # QoS depth
        )
        print("[ActionPublisher] ROS话题发布器已创建: /robot/command", file=sys.stderr)

    def put(self, action):
        """发布动作到话题"""
        try:
            msg = String()
            msg.data = json.dumps(action, ensure_ascii=False)
            self.publisher.publish(msg)
            print(f"[ActionPublisher] 发布命令: {action}", file=sys.stderr)
        except Exception as e:
            print(f"[ActionPublisher] 发布失败: {e}", file=sys.stderr)

    def shutdown(self):
        """关闭ROS节点（不关闭 ROS context，由 Subscriber 负责）"""
        self.node.destroy_node()
        # 注意：Publisher 不调用 _ros_shutdown()，由 Subscriber 统一管理


class ActionSubscriber:
    """动作命令订阅器 - 用于仿真器接收命令"""

    def __init__(self, callback):
        """初始化ROS节点和订阅者

        Args:
            callback: 接收到消息时的回调函数，接收action字典作为参数
        """
        _ros_init()
        self.node = Node('simulator_action_subscriber')
        self.subscription = self.node.create_subscription(
            String,
            '/robot/command',
            self._message_callback,
            10  # QoS depth
        )
        self.callback = callback
        print("[ActionSubscriber] ROS话题订阅器已创建: /robot/command", file=sys.stderr)

    def _message_callback(self, msg):
        """内部回调：解析JSON并调用用户回调"""
        try:
            action = json.loads(msg.data)
            print(f"[ActionSubscriber] 接收命令: {action}", file=sys.stderr)
            self.callback(action)
        except json.JSONDecodeError as e:
            print(f"[ActionSubscriber] JSON解析失败: {e}", file=sys.stderr)
        except Exception as e:
            print(f"[ActionSubscriber] 处理失败: {e}", file=sys.stderr)

    def spin(self):
        """开始处理话题消息（阻塞）"""
        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            print("\n[ActionSubscriber] 被用户中断", file=sys.stderr)
        finally:
            self.shutdown()

    def spin_once(self, timeout_sec=0.001):
        """非阻塞处理一次话题消息

        用于在pygame等循环中定期处理ROS回调
        Args:
            timeout_sec: 超时时间（秒），默认0.001秒（几乎不阻塞）
        """
        try:
            rclpy.spin_once(self.node, timeout_sec=timeout_sec)
        except Exception:
            pass

    def shutdown(self):
        """关闭ROS节点"""
        self.node.destroy_node()
        _ros_shutdown()


# ==============================================================================
# 全局实例（单例模式）
# ==============================================================================

_publisher_instance = None
_subscriber_instance = None


def get_action_publisher():
    """获取动作发布器单例"""
    global _publisher_instance
    if _publisher_instance is None:
        _publisher_instance = ActionPublisher()
    return _publisher_instance


def get_action_subscriber(callback):
    """获取动作订阅器单例"""
    global _subscriber_instance
    if _subscriber_instance is None:
        _subscriber_instance = ActionSubscriber(callback)
    return _subscriber_instance


# ==============================================================================
# 命令队列：ROS话题通讯层
# ==============================================================================

class SharedCommandQueue:
    """ROS话题命令队列 - 提供发布/订阅接口"""

    def __init__(self):
        """初始化（延迟到首次使用时创建ROS节点）"""
        self._publisher = None
        self._subscriber = None
        self._pending_actions = queue.Queue()  # 线程安全的队列

    def put(self, action):
        """发送动作到队列"""
        if self._publisher is None:
            self._publisher = get_action_publisher()
        self._publisher.put(action)

    def get_nowait(self):
        """非阻塞获取队列中的动作"""
        try:
            return self._pending_actions.get_nowait()
        except queue.Empty:
            return None

    def empty(self):
        """检查队列是否为空"""
        return self._pending_actions.empty()

    def setup_subscriber(self):
        """设置订阅器（仿真器侧调用）"""
        if self._subscriber is None:
            self._subscriber = get_action_subscriber(
                lambda action: self._pending_actions.put(action)
            )

    def spin_once(self, timeout_sec=0.001):
        """处理一次ROS回调（仿真器在主循环中调用）"""
        if self._subscriber is not None:
            self._subscriber.spin_once(timeout_sec)

    def shutdown(self):
        """关闭ROS节点"""
        global _publisher_instance, _subscriber_instance
        if _publisher_instance is not None:
            _publisher_instance.shutdown()
            _publisher_instance = None
        if _subscriber_instance is not None:
            _subscriber_instance.shutdown()
            _subscriber_instance = None
        self._publisher = None
        self._subscriber = None


def get_shared_queue():
    """获取ROS话题命令队列"""
    return SharedCommandQueue()


if __name__ == "__main__":
    import sys

    # 测试发布器
    print("测试ROS话题通讯...", file=sys.stderr)

    queue = get_shared_queue()

    # 测试发布
    test_action = {'action': 'test', 'parameters': {'value': 123}}
    queue.put(test_action)
    print(f"发送测试命令: {test_action}", file=sys.stderr)

    # 等待用户输入以查看订阅器效果
    print("\n在另一个终端运行订阅器测试", file=sys.stderr)
    print("按Ctrl+C退出", file=sys.stderr)

    try:
        import time
        while True:
            queue.spin_once()
            time.sleep(0.1)
    except KeyboardInterrupt:
        queue.shutdown()
        print("\n测试结束", file=sys.stderr)
