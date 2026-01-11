#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
通用 ROS2 Robot Controller
从 Robot_Module 读取机器人配置，动态创建 ROS2 发布者
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import json
import sys
import os
from pathlib import Path
import yaml

# 添加项目根目录到路径
project_root = Path(__file__).parent.parent.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))


class GenericROS2Controller(Node):
    """
    通用 ROS2 机器人控制器

    从 Robot_Module 读取配置，动态创建 ROS2 话题发布者
    """

    def __init__(self, robot_name: str = "Sim_2D"):
        super().__init__('ros2_robot_controller')

        self.robot_name = robot_name
        self.config = self._load_robot_config(robot_name)
        self.topic_publishers = {}

        # 创建订阅者（接收 MCP 命令）
        self._setup_subscriber()

        # 创建发布者（根据配置）
        self._setup_publishers()

        self.get_logger().info(f'[ROS2] Generic Controller for {robot_name} Ready')
        print(f"[ROS2] 控制器已初始化: {robot_name}")
        print(f"[ROS2] 订阅话题: {self.config['ros2']['subscribe']['command_topic']}")

    def _load_robot_config(self, robot_name: str) -> dict:
        """从 Robot_Module 加载机器人配置"""
        config_path = project_root / 'Robot_Module' / robot_name / 'robot_config.yaml'

        if not config_path.exists():
            self.get_logger().warning(f'未找到配置文件: {config_path}')
            return self._get_default_config()

        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            print(f"[ROS2] 已加载配置: {config_path}")
            return config
        except Exception as e:
            self.get_logger().error(f'加载配置失败: {e}')
            return self._get_default_config()

    def _get_default_config(self) -> dict:
        """返回默认配置"""
        return {
            'robot': {'name': 'Default', 'type': 'differential_drive'},
            'ros2': {
                'subscribe': {'command_topic': '/robot_command'},
                'publish': {
                    'cmd_vel': '/cmd_vel'
                }
            }
        }

    def _setup_subscriber(self):
        """设置命令订阅者"""
        command_topic = self.config['ros2']['subscribe']['command_topic']
        self.subscription = self.create_subscription(
            String,
            command_topic,
            self.command_callback,
            10
        )

    def _setup_publishers(self):
        """根据配置动态创建发布者"""
        publish_config = self.config['ros2'].get('publish', {})

        # 创建 cmd_vel 发布者（如果配置了）
        if 'cmd_vel' in publish_config:
            topic = publish_config['cmd_vel']
            self.topic_publishers['cmd_vel'] = self.create_publisher(Twist, topic, 10)
            print(f"[ROS2] 发布话题: {topic} (Twist)")

        # 创建 gripper 发布者（如果配置了）
        if 'gripper' in publish_config:
            topic = publish_config['gripper']
            self.topic_publishers['gripper'] = self.create_publisher(Float64, topic, 10)
            print(f"[ROS2] 发布话题: {topic} (Float64)")

        # 创建 joint_states 发布者（如果配置了）
        if 'joint_states' in publish_config:
            topic = publish_config['joint_states']
            self.topic_publishers['joint_states'] = self.create_publisher(JointState, topic, 10)
            print(f"[ROS2] 发布话题: {topic} (JointState)")

    def command_callback(self, msg):
        """
        处理来自 MCP 的命令

        Args:
            msg: ROS2 String 消息
        """
        try:
            command = json.loads(msg.data)
            action = command['action']
            params = command.get('parameters', {})

            self.get_logger().info(f'收到命令: {action}, 参数: {params}')

            # 路由命令到对应的处理函数
            self._route_command(action, params)

        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON 解析失败: {e}')
        except Exception as e:
            self.get_logger().error(f'命令处理失败: {e}')

    def _route_command(self, action: str, params: dict):
        """
        根据命令类型路由到对应的 ROS2 话题

        Args:
            action: 动作类型
            params: 动作参数
        """
        # 导航类命令 -> cmd_vel
        if action in ['navigate', 'turn_left', 'turn_right', 'strafe']:
            self._publish_cmd_vel(action, params)
        elif action == 'stop':
            self._publish_stop()
        # 抓取类命令 -> gripper
        elif action == 'pick':
            self._publish_gripper(params, close=True)
        elif action == 'place':
            self._publish_gripper(params, close=False)
        # 关节控制 -> joint_states
        elif action == 'move_joint':
            self._publish_joint_states(params)
        else:
            self.get_logger().warning(f'未知动作: {action}')

    def _publish_cmd_vel(self, action: str, params: dict):
        """发布速度命令"""
        if 'cmd_vel' not in self.topic_publishers:
            self.get_logger().warning('cmd_vel 发布者未配置')
            return

        twist = Twist()
        speed = float(params.get('speed', 0.5))

        # 处理转向
        if action in ['turn_left', 'turn_right']:
            angle_str = params.get('angle', '90deg')
            angular_speed = float(params.get('angular_speed', 0.5))
            try:
                angle = float(angle_str.replace('deg', '').replace('-', ''))
                twist.angular.z = -angular_speed if action == 'turn_left' else angular_speed
                self.get_logger().info(f'转向: {angle}度, 角速度: {twist.angular.z}')
            except Exception as e:
                self.get_logger().error(f'无效的角度: {angle_str}, 错误: {e}')

        # 处理前后导航
        elif action == 'navigate':
            direction = params.get('direction', 'front')
            if direction == 'front':
                twist.linear.x = speed
            elif direction == 'back':
                twist.linear.x = -speed
            self.get_logger().info(f'导航: {direction}, 速度: {twist.linear.x}')
        
        # 处理左右平移
        elif action == 'strafe':
            direction = params.get('direction', 'left')
            if direction == 'left':
                twist.linear.y = speed
            elif direction == 'right':
                twist.linear.y = -speed
            self.get_logger().info(f'平移: {direction}, 速度: {twist.linear.y}')


        self.topic_publishers['cmd_vel'].publish(twist)

    def _publish_stop(self):
        """发布停止命令"""
        if 'cmd_vel' not in self.topic_publishers:
            return

        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        self.topic_publishers['cmd_vel'].publish(twist)
        self.get_logger().info('停止机器人')

    def _publish_gripper(self, params: dict, close: bool):
        """发布抓取器命令"""
        if 'gripper' not in self.topic_publishers:
            self.get_logger().warning('gripper 发布者未配置')
            return

        msg = Float64()
        # 抓取器位置：0.0(闭合) 到 1.0(张开)
        msg.data = 0.0 if close else 1.0

        self.topic_publishers['gripper'].publish(msg)
        object_name = params.get('object', 'unknown')
        action = '抓取' if close else '放置'
        self.get_logger().info(f'{action}: {object_name}')

    def _publish_joint_states(self, params: dict):
        """发布关节状态命令"""
        if 'joint_states' not in self.topic_publishers:
            self.get_logger().warning('joint_states 发布者未配置')
            return

        msg = JointState()
        # 这里需要根据实际机器人配置关节名称
        # 例如：msg.name = ['joint1', 'joint2', 'joint3']
        #       msg.position = [params.get('joint1', 0.0), ...]

        self.get_logger().info(f'关节控制: {params}')

    def _parse_distance(self, distance_str: str) -> float:
        """解析距离字符串"""
        if distance_str.endswith('m'):
            return float(distance_str.replace('m', ''))
        elif distance_str.endswith('cm'):
            return float(distance_str.replace('cm', '')) / 100
        elif distance_str.endswith('mm'):
            return float(distance_str.replace('mm', '')) / 1000
        else:
            return float(distance_str)


def main(args=None):
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description='通用 ROS2 机器人控制器')
    parser.add_argument(
        '--robot',
        type=str,
        default='Sim_2D',
        help='机器人名称 (对应 Robot_Module 中的文件夹名)'
    )
    parsed_args = parser.parse_args(args)

    rclpy.init(args=None)

    try:
        controller = GenericROS2Controller(robot_name=parsed_args.robot)
        print("[ROS2] 控制器运行中，按 Ctrl+C 退出...")
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("\n[ROS2] 关闭中...")
    finally:
        if 'controller' in locals() and rclpy.ok():
            controller.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
