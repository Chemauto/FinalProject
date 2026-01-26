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
        # 短暂延迟确保ROS连接建立
        import time
        time.sleep(0.2)

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

# 机器人状态（共享内存）
_robot_state = {
    'x': 400.0,
    'y': 300.0,
    'angle': 0.0
}


def get_robot_state():
    """获取机器人当前状态"""
    global _robot_state
    return _robot_state.copy()


def set_robot_state(x=None, y=None, angle=None):
    """更新机器人状态（仅用于本地）"""
    global _robot_state
    if x is not None:
        _robot_state['x'] = x
    if y is not None:
        _robot_state['y'] = y
    if angle is not None:
        _robot_state['angle'] = angle


# 机器人位置发布器和订阅器（用于跨进程通信）
_robot_state_publisher = None
_robot_state_subscriber = None
_cached_robot_state = None


class RobotStatePublisher:
    """机器人状态发布器"""
    def __init__(self):
        _ros_init()
        self.node = Node('robot_state_publisher')
        self.publisher = self.node.create_publisher(
            String,
            '/robot/state',
            10
        )
        print("[RobotStatePublisher] ROS话题发布器已创建: /robot/state", file=sys.stderr)
        import time
        time.sleep(0.2)

    def publish(self, x, y, angle):
        """发布机器人状态"""
        try:
            state = {'x': x, 'y': y, 'angle': angle}
            msg = String()
            msg.data = json.dumps(state, ensure_ascii=False)
            self.publisher.publish(msg)
        except Exception as e:
            print(f"[RobotStatePublisher] 发布失败: {e}", file=sys.stderr)

    def shutdown(self):
        """关闭"""
        self.node.destroy_node()


class RobotStateSubscriber:
    """机器人状态订阅器"""
    def __init__(self):
        _ros_init()
        self.node = Node('robot_state_subscriber')
        self.subscription = self.node.create_subscription(
            String,
            '/robot/state',
            self._message_callback,
            10
        )
        print("[RobotStateSubscriber] ROS话题订阅器已创建: /robot/state", file=sys.stderr)

    def _message_callback(self, msg):
        """内部回调"""
        global _cached_robot_state
        try:
            _cached_robot_state = json.loads(msg.data)
        except Exception as e:
            print(f"[RobotStateSubscriber] 解析失败: {e}", file=sys.stderr)

    def spin_once(self, timeout_sec=0.001):
        """处理一次ROS回调"""
        try:
            rclpy.spin_once(self.node, timeout_sec=timeout_sec)
        except Exception:
            pass

    def shutdown(self):
        """关闭"""
        self.node.destroy_node()


def publish_robot_state(x, y, angle):
    """发布机器人状态（仿真器调用）"""
    global _robot_state_publisher
    if _robot_state_publisher is None:
        _robot_state_publisher = RobotStatePublisher()
    _robot_state_publisher.publish(x, y, angle)


def get_robot_state_from_subscriber():
    """从订阅器获取机器人状态（交互程序调用）"""
    global _cached_robot_state, _robot_state_subscriber
    if _robot_state_subscriber is None:
        _robot_state_subscriber = RobotStateSubscriber()
    return _cached_robot_state


def get_robot_state_subscriber():
    """获取机器人状态订阅器实例"""
    global _robot_state_subscriber
    if _robot_state_subscriber is None:
        _robot_state_subscriber = RobotStateSubscriber()
    return _robot_state_subscriber


# 兼容接口：修改 get_robot_state 使用订阅器
_original_get_robot_state = get_robot_state


def get_robot_state():
    """获取机器人状态（跨进程，使用订阅器）"""
    global _cached_robot_state, _robot_state_subscriber

    # 如果订阅器还没创建，创建它
    if _robot_state_subscriber is None:
        _robot_state_subscriber = RobotStateSubscriber()

    # 处理一次 ROS 回调
    _robot_state_subscriber.spin_once()

    # 如果有缓存的状态，返回它
    if _cached_robot_state is not None:
        return _cached_robot_state.copy()

    # 否则返回默认值
    return _original_get_robot_state()


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


# ==============================================================================
# 敌人位置共享（使用ROS2话题）
# ==============================================================================

_enemy_positions_publisher = None
_enemy_positions_subscriber = None
_cached_enemy_positions = []

# 敌人清除命令（ROS2话题）
_enemy_remove_publisher = None
_enemy_remove_subscriber = None
_pending_enemy_removals = queue.Queue()  # 待清除的敌人ID队列


class EnemyRemovePublisher:
    """敌人清除命令发布器"""
    def __init__(self):
        _ros_init()
        self.node = Node('enemy_remove_publisher')
        self.publisher = self.node.create_publisher(
            String,
            '/robot/enemy_remove',
            10
        )
        print("[EnemyRemovePublisher] ROS话题发布器已创建: /robot/enemy_remove", file=sys.stderr)
        import time
        time.sleep(0.2)

    def publish(self, enemy_id):
        """发布敌人清除命令"""
        try:
            msg = String()
            msg.data = json.dumps({'enemy_id': enemy_id}, ensure_ascii=False)
            self.publisher.publish(msg)
            print(f"[EnemyRemovePublisher] 发送清除命令: {enemy_id}", file=sys.stderr)
        except Exception as e:
            print(f"[EnemyRemovePublisher] 发布失败: {e}", file=sys.stderr)

    def shutdown(self):
        """关闭"""
        self.node.destroy_node()


class EnemyRemoveSubscriber:
    """敌人清除命令订阅器"""
    def __init__(self):
        _ros_init()
        self.node = Node('enemy_remove_subscriber')
        self.subscription = self.node.create_subscription(
            String,
            '/robot/enemy_remove',
            self._message_callback,
            10
        )
        print("[EnemyRemoveSubscriber] ROS话题订阅器已创建: /robot/enemy_remove", file=sys.stderr)

    def _message_callback(self, msg):
        """内部回调"""
        try:
            command = json.loads(msg.data)
            enemy_id = command.get('enemy_id')
            if enemy_id:
                _pending_enemy_removals.put(enemy_id)
                print(f"[EnemyRemoveSubscriber] 收到清除命令: {enemy_id}", file=sys.stderr)
        except Exception as e:
            print(f"[EnemyRemoveSubscriber] 解析失败: {e}", file=sys.stderr)

    def spin_once(self, timeout_sec=0.001):
        """处理一次ROS回调"""
        try:
            rclpy.spin_once(self.node, timeout_sec=timeout_sec)
        except Exception:
            pass

    def shutdown(self):
        """关闭"""
        self.node.destroy_node()

    def get_pending_removals(self):
        """获取待清除的敌人ID列表"""
        removals = []
        try:
            while True:
                enemy_id = _pending_enemy_removals.get_nowait()
                removals.append(enemy_id)
        except queue.Empty:
            pass
        return removals


class EnemyPositionsPublisher:
    """敌人位置发布器"""
    def __init__(self):
        _ros_init()
        self.node = Node('enemy_positions_publisher')
        self.publisher = self.node.create_publisher(
            String,
            '/robot/enemies',
            10
        )
        print("[EnemyPositionsPublisher] ROS话题发布器已创建: /robot/enemies", file=sys.stderr)
        import time
        time.sleep(0.2)

    def publish(self, positions):
        """发布敌人位置"""
        try:
            msg = String()
            msg.data = json.dumps(positions, ensure_ascii=False)
            self.publisher.publish(msg)
            if len(positions) > 0:
                print(f"[EnemyPositionsPublisher] 已发布 {len(positions)} 个敌人位置", file=sys.stderr)
        except Exception as e:
            print(f"[EnemyPositionsPublisher] 发布失败: {e}", file=sys.stderr)

    def shutdown(self):
        """关闭"""
        self.node.destroy_node()


class EnemyPositionsSubscriber:
    """敌人位置订阅器"""
    def __init__(self):
        _ros_init()
        self.node = Node('enemy_positions_subscriber')
        self.subscription = self.node.create_subscription(
            String,
            '/robot/enemies',
            self._message_callback,
            10
        )
        print("[EnemyPositionsSubscriber] ROS话题订阅器已创建: /robot/enemies", file=sys.stderr)
        # 等待发布-订阅连接建立
        import time
        time.sleep(0.5)
        print("[EnemyPositionsSubscriber] 订阅器初始化完成", file=sys.stderr)

    def _message_callback(self, msg):
        """内部回调"""
        global _cached_enemy_positions
        try:
            new_positions = json.loads(msg.data)
            _cached_enemy_positions = new_positions
            print(f"[EnemyPositionsSubscriber] 已更新缓存: {len(_cached_enemy_positions)} 个敌人 {_cached_enemy_positions}", file=sys.stderr)
        except Exception as e:
            print(f"[EnemyPositionsSubscriber] 解析失败: {e}", file=sys.stderr)

    def spin_once(self, timeout_sec=0.001):
        """处理一次ROS回调"""
        try:
            rclpy.spin_once(self.node, timeout_sec=timeout_sec)
        except Exception:
            pass

    def shutdown(self):
        """关闭"""
        self.node.destroy_node()


def get_enemy_positions_publisher():
    """获取敌人位置发布器单例"""
    global _enemy_positions_publisher
    if _enemy_positions_publisher is None:
        _enemy_positions_publisher = EnemyPositionsPublisher()
    return _enemy_positions_publisher


def get_enemy_positions_subscriber():
    """获取敌人位置订阅器单例"""
    global _enemy_positions_subscriber
    if _enemy_positions_subscriber is None:
        _enemy_positions_subscriber = EnemyPositionsSubscriber()
    return _enemy_positions_subscriber


def publish_enemy_positions(positions):
    """发布敌人位置（仿真器调用）"""
    publisher = get_enemy_positions_publisher()
    publisher.publish(positions)


def get_enemy_positions_from_subscriber():
    """获取敌人位置（交互程序调用）"""
    global _cached_enemy_positions
    return _cached_enemy_positions


def set_enemy_positions(positions):
    """设置并发布敌人位置（兼容接口）"""
    publish_enemy_positions(positions)


def get_enemy_positions():
    """获取敌人位置（兼容接口）"""
    subscriber = get_enemy_positions_subscriber()
    subscriber.spin_once()
    return get_enemy_positions_from_subscriber()


# ==============================================================================
# YOLO 敌人位置共享（使用ROS2话题）
# ==============================================================================

_yolo_enemy_positions_subscriber = None
_cached_yolo_enemy_positions = []


class YoloEnemyPositionsSubscriber:
    """YOLO敌人位置订阅器 - 从 /robot/yolo_enemies 话题获取"""
    def __init__(self):
        _ros_init()
        self.node = Node('yolo_enemy_positions_subscriber')
        self.subscription = self.node.create_subscription(
            String,
            '/robot/yolo_enemies',
            self._message_callback,
            10
        )
        print("[YoloEnemyPositionsSubscriber] ROS话题订阅器已创建: /robot/yolo_enemies", file=sys.stderr)
        # 等待发布-订阅连接建立
        import time
        time.sleep(0.5)
        print("[YoloEnemyPositionsSubscriber] 订阅器初始化完成", file=sys.stderr)

    def _message_callback(self, msg):
        """内部回调"""
        global _cached_yolo_enemy_positions
        try:
            data = json.loads(msg.data)
            # YOLO 发布的数据格式: {"detections": [...], "timestamp": ..., ...}
            detections = data.get('detections', [])
            _cached_yolo_enemy_positions = detections
            print(f"[YoloEnemyPositionsSubscriber] 已更新缓存: {len(_cached_yolo_enemy_positions)} 个YOLO敌人 {_cached_yolo_enemy_positions}", file=sys.stderr)
        except Exception as e:
            print(f"[YoloEnemyPositionsSubscriber] 解析失败: {e}", file=sys.stderr)

    def spin_once(self, timeout_sec=0.001):
        """处理一次ROS回调"""
        try:
            rclpy.spin_once(self.node, timeout_sec=timeout_sec)
        except Exception:
            pass

    def shutdown(self):
        """关闭"""
        self.node.destroy_node()


def get_yolo_enemy_positions_subscriber():
    """获取YOLO敌人位置订阅器单例"""
    global _yolo_enemy_positions_subscriber
    if _yolo_enemy_positions_subscriber is None:
        _yolo_enemy_positions_subscriber = YoloEnemyPositionsSubscriber()
    return _yolo_enemy_positions_subscriber


def get_yolo_enemy_positions_from_subscriber():
    """获取YOLO敌人位置（从缓存）"""
    global _cached_yolo_enemy_positions
    return _cached_yolo_enemy_positions


def get_yolo_enemy_positions():
    """获取YOLO敌人位置（兼容接口）"""
    subscriber = get_yolo_enemy_positions_subscriber()
    subscriber.spin_once()
    return get_yolo_enemy_positions_from_subscriber()


# ==============================================================================
# 敌人清除命令接口
# ==============================================================================

def get_enemy_remove_publisher():
    """获取敌人清除命令发布器单例"""
    global _enemy_remove_publisher
    if _enemy_remove_publisher is None:
        _enemy_remove_publisher = EnemyRemovePublisher()
    return _enemy_remove_publisher


def get_enemy_remove_subscriber():
    """获取敌人清除命令订阅器单例"""
    global _enemy_remove_subscriber
    if _enemy_remove_subscriber is None:
        _enemy_remove_subscriber = EnemyRemoveSubscriber()
    return _enemy_remove_subscriber


def remove_enemy(enemy_id):
    """发送清除敌人命令（追击模块调用）"""
    publisher = get_enemy_remove_publisher()
    publisher.publish(enemy_id)


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
