#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 Robot Simulator - 可视化仿真器
类似于 Dora simulator.py，但使用 ROS2 话题通信
"""
import sys
import pygame
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
import threading

# Reconfigure stdout to use UTF-8 encoding on Windows
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

# --- Constants ---
WIDTH, HEIGHT = 800, 600
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRID_COLOR = (220, 220, 220)
GRID_SPACING = 50
ROBOT_BODY_COLOR = (60, 120, 180)
ROBOT_CABIN_COLOR = (255, 200, 0)
ROBOT_SIZE = 25
TEXT_COLOR = (50, 50, 50)


class Robot:
    """仿真器中的机器人"""

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.angle = 0  # 角度，0是东，90是北
        self.target_x = x
        self.target_y = y
        self.target_angle = 0
        self.speed = 0
        self.angular_speed = 0
        self.is_moving = False
        self.last_command = ""

    def update(self):
        """更新机器人位置（平滑移动到目标位置）"""
        # 平滑旋转
        if abs(self.angle - self.target_angle) > 1:
            diff = (self.target_angle - self.angle + 180) % 360 - 180
            self.angle += diff * 0.1
            self.angle %= 360
        else:
            self.angle = self.target_angle

        # 平滑移动
        if abs(self.x - self.target_x) > 1 or abs(self.y - self.target_y) > 1:
            self.is_moving = True
            self.x += (self.target_x - self.x) * 0.05
            self.y += (self.target_y - self.y) * 0.05
        else:
            self.is_moving = False

    def draw(self, screen):
        """绘制机器人"""
        # 计算机器人形状
        angle_rad = math.radians(self.angle)

        # 机器人主体（圆形）
        pygame.draw.circle(screen, ROBOT_BODY_COLOR, (int(self.x), int(self.y)), ROBOT_SIZE)

        # 方向指示器（三角形）
        front_x = self.x + ROBOT_SIZE * math.cos(angle_rad)
        front_y = self.y - ROBOT_SIZE * math.sin(angle_rad)
        pygame.draw.circle(screen, ROBOT_CABIN_COLOR, (int(front_x), int(front_y)), ROBOT_SIZE // 3)

        # 显示坐标和角度
        font = pygame.font.Font(None, 24)
        info_text = f"Pos: ({int(self.x)}, {int(self.y)})  Angle: {int(self.angle)}°"
        text_surface = font.render(info_text, True, TEXT_COLOR)
        screen.blit(text_surface, (10, 10))

        # 显示最后执行的命令
        if self.last_command:
            cmd_font = pygame.font.Font(None, 20)
            cmd_text = f"Command: {self.last_command}"
            cmd_surface = cmd_font.render(cmd_text, True, (100, 100, 100))
            screen.blit(cmd_surface, (10, 40))

    def set_navigation_goal(self, params):
        """设置导航目标"""
        self.last_command = str(params)
        print(f"[Simulator] Received command: {params}")

        # 处理转向
        angle_str = params.get('angle', None)
        if angle_str:
            try:
                # 提取角度值和符号
                angle_value = float(angle_str.replace('deg', ''))
                is_negative = angle_value < 0
                angle_value = abs(angle_value)

                # 根据角度符号确定转向方向
                # 正角度 = 左转（逆时针），负角度 = 右转（顺时针）
                if is_negative:
                    # 右转（顺时针）
                    self.target_angle = (self.angle - angle_value) % 360
                else:
                    # 左转（逆时针）
                    self.target_angle = (self.angle + angle_value) % 360

                print(f"[Simulator] Turning to {self.target_angle:.1f}° (from {self.angle:.1f}°)")
                return
            except ValueError:
                pass

        # 处理移动
        direction = params.get('direction', 'front')
        distance_str = params.get('distance', '0cm')

        try:
            if 'cm' in distance_str:
                distance = float(distance_str.replace('cm', ''))
            elif 'm' in distance_str:
                distance = float(distance_str.replace('m', '')) * 100
            else:
                distance = 0
        except ValueError:
            distance = 0

        if distance == 0:
            return

        # 根据当前角度计算移动向量
        move_angle_rad = math.radians(self.angle)

        if direction == 'back':
            move_angle_rad += math.pi
        elif direction == 'left':
            move_angle_rad -= math.pi / 2
        elif direction == 'right':
            move_angle_rad += math.pi / 2

        # 更新目标位置
        self.target_x = self.x + distance * math.cos(move_angle_rad)
        self.target_y = self.y - distance * math.sin(move_angle_rad)

        # 限制在屏幕范围内
        self.target_x = max(ROBOT_SIZE, min(WIDTH - ROBOT_SIZE, self.target_x))
        self.target_y = max(ROBOT_SIZE, min(HEIGHT - ROBOT_SIZE, self.target_y))


class ROS2SimulatorNode(Node):
    """ROS2 仿真器节点"""

    def __init__(self, robot):
        super().__init__('ros2_simulator')
        self.robot = robot

        # 订阅命令话题
        self.subscription = self.create_subscription(
            String,
            '/robot_command',
            self.command_callback,
            10
        )

        self.get_logger().info('[ROS2 Simulator] Ready')

    def command_callback(self, msg):
        """处理命令回调"""
        try:
            command = json.loads(msg.data)
            action = command['action']
            params = command.get('parameters', {})

            if action in ['navigate', 'turn_left', 'turn_right']:
                self.robot.set_navigation_goal(params)
            elif action == 'stop':
                self.robot.target_x = self.robot.x
                self.robot.target_y = self.robot.y
        except Exception as e:
            self.get_logger().error(f'Error processing command: {e}')


def draw_grid(screen):
    """绘制网格背景"""
    for x in range(0, WIDTH, GRID_SPACING):
        pygame.draw.line(screen, GRID_COLOR, (x, 0), (x, HEIGHT))
    for y in range(0, HEIGHT, GRID_SPACING):
        pygame.draw.line(screen, GRID_COLOR, (0, y), (WIDTH, y))


def main():
    """主函数"""
    # 初始化 Pygame
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("ROS2 Robot Simulator")
    clock = pygame.time.Clock()

    # 创建机器人
    robot = Robot(WIDTH // 2, HEIGHT // 2)

    # 初始化 ROS2
    rclpy.init()
    simulator_node = ROS2SimulatorNode(robot)

    # 在单独的线程中运行 ROS2 spin
    def ros_spin():
        rclpy.spin(simulator_node)

    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    print("="*60)
    print("ROS2 Robot Simulator")
    print("="*60)
    print(f"窗口大小: {WIDTH}x{HEIGHT}")
    print(f"机器人初始位置: ({robot.x}, {robot.y})")
    print(f"订阅话题: /robot_command")
    print("")
    print("提示：在另一个终端运行:")
    print("  cd ROS_Module/ros2")
    print("  python3 ros2_interactive_mcp.py")
    print("")
    print("或手动发送命令:")
    print('  ros2 topic pub /robot_command std_msgs/String "{data: \\"{\\"action\\": \\"navigate\\", \\"parameters\\": {\\"direction\\": \\"front\\", \\"distance\\": \\"1m\\"}}\\"}"')
    print("")
    print("按 Ctrl+C 或关闭窗口退出")
    print("="*60)

    running = True
    try:
        while running:
            # 处理 Pygame 事件
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False

            # 更新机器人
            robot.update()

            # 绘制
            screen.fill(WHITE)
            draw_grid(screen)
            robot.draw(screen)

            # 更新显示
            pygame.display.flip()
            clock.tick(60)

    except KeyboardInterrupt:
        print("\n[Simulator] 被用户中断")
    finally:
        # 清理
        simulator_node.destroy_node()
        rclpy.shutdown()
        pygame.quit()
        print("[Simulator] 已关闭")


if __name__ == "__main__":
    main()
