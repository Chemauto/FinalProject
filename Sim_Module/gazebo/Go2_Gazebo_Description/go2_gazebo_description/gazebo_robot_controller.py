#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Gazebo 麦克纳姆轮全向移动机器人控制器

接收来自 MCP 的命令并控制 Gazebo 中的麦克纳姆轮机器人
支持全向移动: 前后、左右、斜向、原地旋转
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
import time


class GazeboRobotController(Node):
    """Gazebo 麦克纳姆轮机器人控制器"""

    def __init__(self):
        super().__init__('gazebo_robot_controller')

        # 订阅 MCP 命令
        self.subscription = self.create_subscription(
            String,
            '/robot_command',
            self.command_callback,
            10
        )

        # 发布到机器人速度控制话题（麦克纳姆轮全向移动）
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_omni', 10)

        # 同时发布标准 /cmd_vel 以兼容导航栈
        self.cmd_vel_std_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('[Gazebo] Robot Controller Ready')
        print("[Gazebo] Robot Controller Initialized")
        print("[Gazebo] Robot Type: Mecanum Omnidirectional")
        print("[Gazebo] Listening to topic: /robot_command")
        print("[Gazebo] Publishing to topics: /cmd_vel_omni, /cmd_vel\n")

    def command_callback(self, msg):
        """
        处理来自 MCP 的命令

        Args:
            msg: ROS2 String 消息
        """
        try:
            command = json.loads(msg.data)
            action = command.get('action')
            params = command.get('parameters', {})

            print(f"\n[Gazebo] 收到命令:")
            print(f"  动作: {action}")
            print(f"  参数: {params}")

            if action == "move_omni":
                self.handle_move_omni(params)
            elif action == "rotate":
                self.handle_rotate(params)
            elif action == "stop":
                self.handle_stop()
            else:
                # Fallback for old commands for compatibility
                if action == "navigate":
                    self.handle_navigate(params)
                elif action == "turn_left":
                    self.handle_turn_left(params)
                elif action == "turn_right":
                    self.handle_turn_right(params)
                else:
                    print(f"[Gazebo] 未知或旧版动作: {action}")

        except json.JSONDecodeError as e:
            self.get_logger().error(f'命令解析失败: {e}')
        except Exception as e:
            self.get_logger().error(f'处理命令错误: {e}')

    def handle_move_omni(self, params):
        """
        处理全向移动命令
        """
        direction = params.get('direction', 'forward')
        distance_str = params.get('distance', '0m')
        
        if 'm' in distance_str:
            distance = float(distance_str.replace('m', ''))
        elif 'cm' in distance_str:
            distance = float(distance_str.replace('cm', '')) / 100.0
        else:
            distance = float(distance_str)

        speed = 0.3  # m/s
        duration = distance / speed if speed > 0 else 0

        twist = Twist()
        if direction == 'forward':
            twist.linear.x = speed
        elif direction == 'backward':
            twist.linear.x = -speed
        elif direction == 'left':
            twist.linear.y = speed
        elif direction == 'right':
            twist.linear.y = -speed
            
        print(f"[Gazebo] 全向移动 {direction} {distance}m (时长: {duration:.2f}秒)")
        self.execute_twist_for_duration(twist, duration)

    def handle_rotate(self, params):
        """
        处理旋转命令
        """
        angle_str = params.get('angle', '0deg')
        angle = float(angle_str.replace('deg', ''))

        angular_speed = 0.5  # rad/s
        # 正角度为逆时针（左转），负角度为顺时针（右转）
        duration = abs(angle * 3.14159 / 180) / angular_speed

        twist = Twist()
        if angle > 0:
            twist.angular.z = angular_speed  # 左转
        else:
            twist.angular.z = -angular_speed # 右转

        print(f"[Gazebo] 旋转 {angle} 度 (时长: {duration:.2f}秒)")
        self.execute_twist_for_duration(twist, duration)

    def handle_navigate(self, params):
        """
        处理导航命令

        Args:
            params: 导航参数字典
        """
        # 处理转向
        twist = Twist()

        if 'angle' in params:
            angle_str = params['angle']
            angle = float(angle_str.replace('deg', '').replace('-', ''))

            # 四足机器人转向（角速度）
            angular_speed = 0.5  # rad/s
            duration = (angle * 3.14159 / 180) / angular_speed

            if '-' in angle_str:
                twist.angular.z = -angular_speed
            else:
                twist.angular.z = angular_speed

            print(f"[Gazebo] 转向 {angle} 度 (时长: {duration:.2f}秒)")
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

            speed = 0.3  # m/s (四足机器人速度较慢)
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

            print(f"[Gazebo] 移动 {direction} {distance}m (时长: {duration:.2f}秒)")
            self.execute_twist_for_duration(twist, duration)

    def handle_turn_left(self, params):
        """
        处理左转命令

        Args:
            params: 转向参数
        """
        angle = params.get('angle', 90)
        if isinstance(angle, str):
            angle = float(angle.replace('deg', '').replace('-', ''))

        print(f"[Gazebo] 左转 {angle} 度")

        angular_speed = 0.5  # rad/s
        duration = (angle * 3.14159 / 180) / angular_speed

        twist = Twist()
        twist.angular.z = angular_speed

        self.execute_twist_for_duration(twist, duration)

    def handle_turn_right(self, params):
        """
        处理右转命令

        Args:
            params: 转向参数
        """
        angle = params.get('angle', 90)
        if isinstance(angle, str):
            angle = float(angle.replace('deg', '').replace('-', ''))

        print(f"[Gazebo] 右转 {angle} 度")

        angular_speed = 0.5  # rad/s
        duration = (angle * 3.14159 / 180) / angular_speed

        twist = Twist()
        twist.angular.z = -angular_speed

        self.execute_twist_for_duration(twist, duration)

    def execute_twist_for_duration(self, twist, duration):
        """
        执行 Twist 命令指定的时间

        Args:
            twist: Twist 消息
            duration: 执行时长（秒）
        """
        # 发送速度命令到两个话题
        self.cmd_vel_pub.publish(twist)
        self.cmd_vel_std_pub.publish(twist)

        # 等待指定时间
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
        print(f"[Gazebo] 抓取 {object_name}")
        # TODO: 调用机械臂控制逻辑
        print("[Gazebo] 抓取功能待实现")

    def handle_place(self, params):
        """
        处理放置命令

        Args:
            params: 放置参数
        """
        object_name = params.get('object', 'unknown')
        location = params.get('location', 'unknown')
        print(f"[Gazebo] 放置 {object_name} 到 {location}")
        # TODO: 调用机械臂控制逻辑
        print("[Gazebo] 放置功能待实现")

    def handle_stop(self):
        """停止机器人"""
        print("[Gazebo] 停止机器人")
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
        self.cmd_vel_std_pub.publish(twist)


def main(args=None):
    """主函数"""
    rclpy.init(args=args)

    gazebo_controller = GazeboRobotController()

    try:
        print("[Gazebo] 控制器运行中，按 Ctrl+C 退出...")
        rclpy.spin(gazebo_controller)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("\n[Gazebo] 关闭中...")
    finally:
        try:
            gazebo_controller.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
