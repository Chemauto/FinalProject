#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 Robot Controller Example
接收来自MCP Robot Control Server的命令并控制真实机器人
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json


class RobotController(Node):
    """ROS2机器人控制器"""

    def __init__(self):
        super().__init__('ros2_robot_controller')

        # 订阅MCP命令
        self.subscription = self.create_subscription(
            String,
            '/robot_command',
            self.command_callback,
            10
        )

        # 发布到机器人速度控制话题
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('[ROS2] Robot Controller Ready')
        print("[ROS2] Robot Controller Initialized")
        print("[ROS2] Listening to topic: /robot_command")
        print("[ROS2] Publishing to topic: /cmd_vel\n")

    def command_callback(self, msg):
        """
        处理来自MCP的命令

        Args:
            msg: ROS2 String消息
        """
        try:
            command = json.loads(msg.data)
            action = command['action']
            params = command.get('parameters', {})

            print(f"\n[ROS2] Received command:")
            print(f"  Action: {action}")
            print(f"  Parameters: {params}")

            # 执行对应的动作
            if action == "navigate":
                self.handle_navigate(params)
            elif action == "pick":
                self.handle_pick(params)
            elif action == "place":
                self.handle_place(params)
            elif action == "stop":
                self.handle_stop()
            else:
                print(f"[ROS2] Unknown action: {action}")

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse command: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing command: {e}')

    def handle_navigate(self, params):
        """
        处理导航命令

        Args:
            params: 导航参数字典
        """
        twist = Twist()

        # 处理转向
        if 'angle' in params:
            angle_str = params['angle']
            angle = float(angle_str.replace('deg', '').replace('-', ''))

            # 简单的转向实现（需要根据实际机器人调整）
            angular_speed = 0.5  # rad/s
            duration = (angle * 3.14159 / 180) / angular_speed

            if '-' in angle_str:
                twist.angular.z = -angular_speed
            else:
                twist.angular.z = angular_speed

            print(f"[ROS2] Turning {angle} degrees (duration: {duration:.2f}s)")
            self.execute_twist_for_duration(twist, duration)

        # 处理移动
        direction = params.get('direction', 'front')
        if 'distance' in params:
            distance_str = params['distance']
            if distance_str.endswith('m'):
                distance = float(distance_str.replace('m', ''))
            elif distance_str.endswith('cm'):
                distance = float(distance_str.replace('cm', '')) / 100
            elif distance_str.endswith('mm'):
                distance = float(distance_str.replace('mm', '')) / 1000
            else:
                distance = float(distance_str)

            speed = 0.5  # m/s
            duration = distance / speed

            twist = Twist()

            if direction == 'front':
                twist.linear.x = speed
            elif direction == 'back':
                twist.linear.x = -speed
            elif direction == 'left':
                twist.linear.y = speed
            elif direction == 'right':
                twist.linear.y = -speed

            print(f"[ROS2] Moving {direction} for {distance}m (duration: {duration:.2f}s)")
            self.execute_twist_for_duration(twist, duration)

    def execute_twist_for_duration(self, twist, duration):
        """
        执行Twist命令指定的时间

        Args:
            twist: Twist消息
            duration: 执行时长（秒）
        """
        # 发送速度命令
        self.cmd_vel_pub.publish(twist)

        # 等待指定时间
        import time
        time.sleep(duration)

        # 停止机器人
        self.stop_robot()

    def handle_pick(self, params):
        """
        处理抓取命令

        Args:
            params: 抓取参数
        """
        object_name = params.get('object', 'unknown')
        print(f"[ROS2] Picking up {object_name}")
        # TODO: 调用机械臂控制逻辑
        # 例如：self.arm_controller.pick(object_name)

    def handle_place(self, params):
        """
        处理放置命令

        Args:
            params: 放置参数
        """
        object_name = params.get('object', 'unknown')
        location = params.get('location', 'unknown')
        print(f"[ROS2] Placing {object_name} at {location}")
        # TODO: 调用机械臂控制逻辑
        # 例如：self.arm_controller.place(object_name, location)

    def handle_stop(self):
        """停止机器人"""
        print("[ROS2] Stopping robot")
        self.stop_robot()

    def stop_robot(self):
        """停止机器人运动"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    """主函数"""
    rclpy.init(args=args)

    robot_controller = RobotController()

    try:
        print("[ROS2] Controller is running. Press Ctrl+C to exit...")
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        print("\n[ROS2] Shutting down...")
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
