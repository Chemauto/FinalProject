#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2D Robot Simulator - 简化版仿真器
不使用 ROS2，直接通过 multiprocessing.Queue 通信
"""
import sys
import os
import pygame
import math

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

    def execute_action(self, action):
        """执行动作指令"""
        self.last_command = str(action)
        print(f"[Simulator] 执行动作: {action}", file=sys.stderr)

        action_type = action.get('action', '')
        params = action.get('parameters', {})

        if action_type == 'move_forward':
            self._move('front', params)
        elif action_type == 'move_backward':
            self._move('back', params)
        elif action_type == 'turn':
            self._turn(params)
        elif action_type == 'stop':
            self.target_x = self.x
            self.target_y = self.y

    def _move(self, direction, params):
        """移动处理"""
        distance = params.get('distance', 0.0)
        speed = params.get('speed', 0.3)

        # 转换距离为像素（1米 = 100像素）
        distance_pixels = distance * 100

        if distance_pixels == 0:
            return

        # 根据当前角度计算移动向量
        move_angle_rad = math.radians(self.angle)

        if direction == 'back':
            move_angle_rad += math.pi

        # 更新目标位置
        self.target_x = self.x + distance_pixels * math.cos(move_angle_rad)
        self.target_y = self.y - distance_pixels * math.sin(move_angle_rad)

        # 限制在屏幕范围内
        self.target_x = max(ROBOT_SIZE, min(WIDTH - ROBOT_SIZE, self.target_x))
        self.target_y = max(ROBOT_SIZE, min(HEIGHT - ROBOT_SIZE, self.target_y))

        print(f"[Simulator] 移动: {direction}, 距离={distance}m, 速度={speed}m/s", file=sys.stderr)

    def _turn(self, params):
        """旋转处理"""
        angle = params.get('angle', 0.0)
        angular_speed = params.get('angular_speed', 0.5)

        # 更新目标角度
        self.target_angle = (self.angle + angle) % 360

        print(f"[Simulator] 旋转: {angle}°, 角速度={angular_speed}rad/s", file=sys.stderr)


def draw_grid(screen):
    """绘制网格背景"""
    for x in range(0, WIDTH, GRID_SPACING):
        pygame.draw.line(screen, GRID_COLOR, (x, 0), (x, HEIGHT))
    for y in range(0, HEIGHT, GRID_SPACING):
        pygame.draw.line(screen, GRID_COLOR, (0, y), (WIDTH, y))


def main():
    """主函数 - 2D仿真器"""

    # 获取ROS话题队列
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    from ros_topic_comm import get_shared_queue, publish_robot_state

    # 初始化 Pygame
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("2D Robot Simulator")
    clock = pygame.time.Clock()

    # 创建机器人和ROS话题队列
    robot = Robot(WIDTH // 2, HEIGHT // 2)
    action_queue = get_shared_queue()

    # 设置订阅器
    action_queue.setup_subscriber()

    print("="*60, file=sys.stderr)
    print("2D Robot Simulator", file=sys.stderr)
    print("="*60, file=sys.stderr)
    print(f"窗口大小: {WIDTH}x{HEIGHT}", file=sys.stderr)
    print(f"机器人初始位置: ({robot.x}, {robot.y})", file=sys.stderr)
    print("通信方式: ROS2 Topic (/robot/command)", file=sys.stderr)
    print("", file=sys.stderr)
    print("按 ESC 或关闭窗口退出", file=sys.stderr)
    print("="*60, file=sys.stderr)

    running = True
    try:
        while running:
            # 处理ROS回调（非阻塞）
            action_queue.spin_once(timeout_sec=0.001)

            # 处理 Pygame 事件
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False

            # 检查队列中的动作指令（非阻塞）
            try:
                if not action_queue.empty():
                    action = action_queue.get_nowait()
                    robot.execute_action(action)
            except:
                pass

            # 更新机器人
            robot.update()

            # 发布机器人状态（位置和角度）
            publish_robot_state(robot.x, robot.y, robot.angle)

            # 绘制
            screen.fill(WHITE)
            draw_grid(screen)
            robot.draw(screen)

            # 更新显示
            pygame.display.flip()
            clock.tick(60)

    except KeyboardInterrupt:
        print("\n[Simulator] 被用户中断", file=sys.stderr)
    finally:
        action_queue.shutdown()
        pygame.quit()
        print("[Simulator] 已关闭", file=sys.stderr)


if __name__ == "__main__":
    main()
