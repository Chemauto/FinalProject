#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2D Robot Simulator - 仿真器（包含追击功能）

支持功能：
- 基本机器人控制（移动、旋转）
- 追击功能测试（鼠标点击生成敌人）
- 通过 ROS2 Topic 通信
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

# 添加项目路径
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, project_root)

# 导入敌人管理器（同目录）
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from enemy_manager import EnemyManager

# 导入 ROS 通信
from ros_topic_comm import get_shared_queue, set_robot_state, set_enemy_positions, publish_robot_state, get_enemy_remove_subscriber


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

        # 方向指示器（圆形）
        front_x = self.x + ROBOT_SIZE * math.cos(angle_rad)
        front_y = self.y - ROBOT_SIZE * math.sin(angle_rad)
        pygame.draw.circle(screen, ROBOT_CABIN_COLOR, (int(front_x), int(front_y)), ROBOT_SIZE // 3)

        # 显示坐标和角度（小字）
        font = pygame.font.Font(None, 24)
        info_text = f"Pos: ({int(self.x)}, {int(self.y)})  Angle: {int(self.angle)}°"
        text_surface = font.render(info_text, True, TEXT_COLOR)
        screen.blit(text_surface, (10, 10))

        # 显示最后执行的命令（小字）
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


class ChaseSimulator:
    """追击仿真器"""

    def __init__(self):
        pygame.init()

        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("追击功能测试 - 2D Robot Simulator")
        self.clock = pygame.time.Clock()
        self.running = True

        # 创建机器人
        self.robot = Robot(WIDTH // 2, HEIGHT // 2)

        # 创建敌人管理器
        self.enemy_manager = EnemyManager(bounds=(WIDTH, HEIGHT))

        # ROS 队列
        self.action_queue = get_shared_queue()
        self.action_queue.setup_subscriber()

        # 敌人清除命令订阅器
        self.enemy_remove_subscriber = get_enemy_remove_subscriber()

        # 选中敌人
        self.selected_enemy_id = None

        # 显示追击线
        self.show_chase_line = True

        # 打印帮助
        self._print_help()

    def _print_help(self):
        print("=" * 60, file=sys.stderr)
        print("追击功能测试仿真器", file=sys.stderr)
        print("=" * 60, file=sys.stderr)
        print("操作说明:", file=sys.stderr)
        print("  • 鼠标左键: 在点击位置生成敌人", file=sys.stderr)
        print("  • 按 C: 清除所有敌人", file=sys.stderr)
        print("  • 按 L: 切换追击线显示", file=sys.stderr)
        print("  • 按 ESC: 退出", file=sys.stderr)
        print("=" * 60, file=sys.stderr)

    def handle_events(self):
        """处理事件"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_c:
                    self.clear_enemies()
                elif event.key == pygame.K_l:
                    self.toggle_chase_line()

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # 左键
                    mouse_x, mouse_y = pygame.mouse.get_pos()
                    self.spawn_enemy_at(mouse_x, mouse_y)

    def spawn_enemy_at(self, x, y):
        """在指定位置生成敌人"""
        enemy = self.enemy_manager.spawn_enemy(x=x, y=y, move_mode="static")
        self.selected_enemy_id = enemy.id
        print(f"[Simulator] 生成敌人: {enemy.name} at ({x:.1f}, {y:.1f})", file=sys.stderr)

        # 立即发布敌人位置到 ROS 话题
        enemies = self.enemy_manager.get_all_enemies()
        enemy_positions = [
            {'id': e.id, 'x': e.x, 'y': e.y}
            for e in enemies
        ]
        set_enemy_positions(enemy_positions)
        print(f"[Simulator] 发布敌人位置: {enemy_positions}", file=sys.stderr)

    def clear_enemies(self):
        """清除所有敌人"""
        self.enemy_manager.clear_all()
        self.selected_enemy_id = None
        print("[Simulator] 清除所有敌人", file=sys.stderr)

    def toggle_chase_line(self):
        """切换追击线显示"""
        self.show_chase_line = not self.show_chase_line
        print(f"[Simulator] 追击线显示: {'开启' if self.show_chase_line else '关闭'}", file=sys.stderr)

    def update(self):
        """更新状态"""
        self.robot.update()

        # 更新敌人
        robot_pos = {'x': self.robot.x, 'y': self.robot.y, 'angle': self.robot.angle}
        self.enemy_manager.update(robot_pos=robot_pos)

        # 更新共享状态（本地）
        set_robot_state(x=robot_pos['x'], y=robot_pos['y'], angle=robot_pos['angle'])

        # 发布机器人状态到 ROS 话题（跨进程）- 每帧发布
        publish_robot_state(robot_pos['x'], robot_pos['y'], robot_pos['angle'])

        # 帧计数
        if hasattr(self, '_frame_count'):
            self._frame_count += 1
        else:
            self._frame_count = 0

        # 每3秒（180帧）更新敌人位置到 ROS 话题
        if self._frame_count % 180 == 0:
            enemies = self.enemy_manager.get_all_enemies()
            enemy_positions = [
                {'id': e.id, 'x': e.x, 'y': e.y}
                for e in enemies
            ]
            set_enemy_positions(enemy_positions)
            if len(enemy_positions) > 0:
                print(f"[Simulator] 发布敌人位置: {enemy_positions}", file=sys.stderr)

        # 处理动作队列
        try:
            if not self.action_queue.empty():
                action = self.action_queue.get_nowait()
                self.robot.execute_action(action)
        except:
            pass

        # 处理敌人清除命令
        self.enemy_remove_subscriber.spin_once(timeout_sec=0.001)
        removals = self.enemy_remove_subscriber.get_pending_removals()
        for enemy_id in removals:
            success = self.enemy_manager.remove_enemy(enemy_id)
            if success:
                print(f"[Simulator] 已清除敌人: {enemy_id}", file=sys.stderr)
                if self.selected_enemy_id == enemy_id:
                    self.selected_enemy_id = None

                # 立即发布更新后的敌人位置
                enemies = self.enemy_manager.get_all_enemies()
                enemy_positions = [
                    {'id': e.id, 'x': e.x, 'y': e.y}
                    for e in enemies
                ]
                set_enemy_positions(enemy_positions)
                print(f"[Simulator] 清除后发布敌人位置: {enemy_positions}", file=sys.stderr)

        # ROS 回调
        self.action_queue.spin_once(timeout_sec=0.001)

    def draw(self):
        """绘制场景"""
        self.screen.fill((250, 250, 250))

        # 绘制网格
        draw_grid(self.screen)

        # 绘制敌人
        self.enemy_manager.draw_all(self.screen)

        # 绘制追击线
        if self.show_chase_line and self.selected_enemy_id:
            enemy = self.enemy_manager.get_enemy(self.selected_enemy_id)
            if enemy:
                robot_pos = {'x': self.robot.x, 'y': self.robot.y, 'angle': self.robot.angle}
                self.enemy_manager.draw_line_to_robot(
                    self.screen,
                    robot_pos,
                    enemy.id,
                    color=(100, 150, 200)
                )

        # 绘制机器人
        self.robot.draw(self.screen)

        pygame.display.flip()

    def run(self):
        """运行仿真器"""
        try:
            while self.running:
                self.handle_events()
                self.update()
                self.draw()
                self.clock.tick(60)
        except KeyboardInterrupt:
            print("\n[Simulator] 被用户中断", file=sys.stderr)
        finally:
            print("[Simulator] 已关闭", file=sys.stderr)
            pygame.quit()


def main():
    """主函数"""
    simulator = ChaseSimulator()
    simulator.run()


if __name__ == "__main__":
    main()
