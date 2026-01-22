#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
增强版2D仿真器

在原有仿真器基础上添加：
- 敌人显示
- 追击连线
- 状态信息面板
- 机器人位置实时共享
"""

import sys
import os
import pygame
import math

# Reconfigure stdout to use UTF-8 encoding on Windows
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

# 添加项目路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 导入原有仿真器的 Robot 类
from Sim_Module.sim2d.simulator import Robot, draw_grid, WIDTH, HEIGHT

# 导入敌人管理器
from Test_Module.enemy_manager import EnemyManager

# --- 新增常量 ---
INFO_PANEL_HEIGHT = 120
INFO_BG_COLOR = (240, 240, 240)
INFO_BORDER_COLOR = (180, 180, 180)
CHASE_LINE_COLOR = (100, 150, 200)


class EnhancedRobot(Robot):
    """增强的机器人类，支持位置查询"""

    def __init__(self, x, y):
        super().__init__(x, y)
        self.chasing_target_id = None  # 当前追击的目标ID

    def get_position(self):
        """获取当前位置和角度"""
        return {
            'x': self.x,
            'y': self.y,
            'angle': self.angle
        }

    def draw(self, screen, show_direction=True):
        """绘制机器人（增强版）"""
        super().draw(screen)

        # 如果正在追击，显示追击状态
        if self.chasing_target_id:
            font = pygame.font.Font(None, 24)
            chase_text = f"追击目标: {self.chasing_target_id}"
            text_surface = font.render(chase_text, True, (200, 100, 50))
            screen.blit(text_surface, (10, 70))


class EnhancedSimulator:
    """增强仿真器主类"""

    def __init__(self):
        """初始化仿真器"""
        # 初始化 Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT + INFO_PANEL_HEIGHT))
        pygame.display.set_caption("增强版 2D Robot Simulator - 追击模式")
        self.clock = pygame.time.Clock()

        # 创建机器人
        self.robot = EnhancedRobot(WIDTH // 2, HEIGHT // 2)

        # 创建敌人管理器
        self.enemy_manager = EnemyManager(bounds=(WIDTH, HEIGHT))

        # 获取ROS话题队列
        from ros_topic_comm import get_shared_queue
        self.action_queue = get_shared_queue()
        self.action_queue.setup_subscriber()

        # 状态
        self.running = True
        self.show_chase_line = True
        self.selected_enemy_id = None
        self.mouse_mark_mode = False  # 鼠标标记模式

        # 导入坐标映射器
        try:
            from Yolo_Module.coordinate_mapper import CoordinateMapper
            self.coord_mapper = CoordinateMapper()
            # 自动检测窗口偏移
            self.coord_mapper.auto_detect_offset()
        except ImportError:
            print("[警告] 未找到 Yolo_Module，坐标映射可能不准确", file=sys.stderr)
            self.coord_mapper = None

        print("="*60, file=sys.stderr)
        print("增强版 2D Robot Simulator", file=sys.stderr)
        print("="*60, file=sys.stderr)
        print("功能:", file=sys.stderr)
        print("  • 按 R: 随机生成敌人", file=sys.stderr)
        print("  • 按 C: 清除所有敌人", file=sys.stderr)
        print("  • 按 L: 切换追击线显示", file=sys.stderr)
        print("  • 按 M: 进入/退出鼠标标记模式", file=sys.stderr)
        print("  • 按 1-9: 选择敌人", file=sys.stderr)
        print("  • 按 ESC: 退出", file=sys.stderr)
        print("="*60, file=sys.stderr)

    def spawn_random_enemy(self):
        """生成随机敌人"""
        modes = ["static", "random", "flee"]
        enemy = self.enemy_manager.spawn_enemy(move_mode=_modes[-1])
        self.selected_enemy_id = enemy.id
        print(f"[Simulator] 生成敌人: {enemy.name} at ({enemy.x:.1f}, {enemy.y:.1f})",
              file=sys.stderr)

    def clear_enemies(self):
        """清除所有敌人"""
        self.enemy_manager.clear_all()
        self.selected_enemy_id = None
        self.robot.chasing_target_id = None

    def toggle_chase_line(self):
        """切换追击线显示"""
        self.show_chase_line = not self.show_chase_line

    def toggle_mouse_mark_mode(self):
        """切换鼠标标记模式"""
        self.mouse_mark_mode = not self.mouse_mark_mode

        if self.mouse_mark_mode:
            print("\n[Simulator] >>> 进入鼠标标记模式 <<<", file=sys.stderr)
            print("  点击地图任意位置生成敌人", file=sys.stderr)
            print("  按 'M' 或 'ESC' 退出标记模式", file=sys.stderr)
        else:
            print("\n[Simulator] <<< 退出鼠标标记模式 >>>", file=sys.stderr)

    def spawn_enemy_at_position(self, x: float, y: float):
        """在指定位置生成敌人"""
        enemy = self.enemy_manager.spawn_enemy(
            x=x,
            y=y,
            color=(255, 50, 50),
            move_mode="static",
            name=f"点击敌人{self.enemy_manager.next_id}"
        )

        self.selected_enemy_id = enemy.id

        print(f"[Simulator] 在点击位置生成敌人: {enemy.name} at ({x:.1f}, {y:.1f})",
              file=sys.stderr)

    def handle_events(self):
        """处理事件"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            elif event.type == pygame.MOUSEBUTTONDOWN:
                # 鼠标点击处理
                if self.mouse_mark_mode and event.button == 1:  # 左键
                    mouse_x, mouse_y = pygame.mouse.get_pos()

                    # 确保点击在仿真区域内
                    if mouse_y < HEIGHT:
                        # 直接使用鼠标坐标（Pygame 坐标已经是仿真坐标）
                        self.spawn_enemy_at_position(mouse_x, mouse_y)

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_r:
                    self.spawn_random_enemy()
                elif event.key == pygame.K_c:
                    self.clear_enemies()
                elif event.key == pygame.K_l:
                    self.toggle_chase_line()
                elif event.key == pygame.K_m:
                    self.toggle_mouse_mark_mode()
                elif event.key in [pygame.K_1, pygame.K_2, pygame.K_3,
                                  pygame.K_4, pygame.K_5, pygame.K_6,
                                  pygame.K_7, pygame.K_8, pygame.K_9]:
                    # 数字键选择敌人
                    num = event.key - pygame.K_0
                    enemy = self.enemy_manager.get_enemy(str(num))
                    if enemy:
                        self.selected_enemy_id = enemy.id
                        print(f"[Simulator] 选择敌人: {enemy.name}", file=sys.stderr)

    def update(self):
        """更新状态"""
        # 更新机器人
        self.robot.update()

        # 更新敌人
        robot_pos = self.robot.get_position()
        self.enemy_manager.update(robot_pos=robot_pos)

        # 处理动作队列
        try:
            if not self.action_queue.empty():
                action = self.action_queue.get_nowait()
                self.robot.execute_action(action)
        except:
            pass

        # ROS回调
        self.action_queue.spin_once(timeout_sec=0.001)

    def draw_info_panel(self):
        """绘制信息面板"""
        panel_y = HEIGHT
        panel_rect = pygame.Rect(0, panel_y, WIDTH, INFO_PANEL_HEIGHT)

        # 背景
        pygame.draw.rect(self.screen, INFO_BG_COLOR, panel_rect)
        pygame.draw.line(self.screen, INFO_BORDER_COLOR,
                        (0, panel_y), (WIDTH, panel_y), 2)

        # 字体
        title_font = pygame.font.Font(None, 28)
        info_font = pygame.font.Font(None, 24)

        # 标题
        title = title_font.render("状态信息", True, (50, 50, 50))
        self.screen.blit(title, (20, panel_y + 10))

        # 机器人信息
        robot_pos = self.robot.get_position()
        info_texts = [
            f"机器人位置: ({robot_pos['x']:.1f}, {robot_pos['y']:.1f})  "
            f"角度: {robot_pos['angle']:.1f}°",
            f"敌人数量: {len(self.enemy_manager.enemies)}  "
            f"选中: {self.selected_enemy_id or '无'}"
        ]

        for i, text in enumerate(info_texts):
            surface = info_font.render(text, True, (80, 80, 80))
            self.screen.blit(surface, (20, panel_y + 45 + i * 25))

        # 操作提示
        if self.mouse_mark_mode:
            hint_texts = ["[鼠标标记模式] 点击生成敌人  [M]退出"]
            hint_color = (200, 100, 50)  # 橙色
        else:
            hint_texts = ["[R]生成 [C]清除 [M]标记 [L]连线 [ESC]退出"]
            hint_color = (120, 120, 120)

        hint_surface = info_font.render(hint_texts[0], True, hint_color)
        self.screen.blit(hint_surface, (WIDTH - 350, panel_y + 10))

    def draw(self):
        """绘制场景"""
        # 主仿真区域
        sim_surface = self.screen.subsurface((0, 0, WIDTH, HEIGHT))
        sim_surface.fill((255, 255, 255))
        draw_grid(sim_surface)

        # 绘制追击线
        if self.show_chase_line and self.selected_enemy_id:
            enemy = self.enemy_manager.get_enemy(self.selected_enemy_id)
            if enemy:
                self.enemy_manager.draw_line_to_robot(
                    sim_surface,
                    self.robot.get_position(),
                    self.selected_enemy_id,
                    CHASE_LINE_COLOR
                )

        # 绘制敌人
        self.enemy_manager.draw_all(sim_surface)

        # 绘制机器人
        self.robot.draw(sim_surface)

        # 鼠标标记模式：绘制十字准星
        if self.mouse_mark_mode:
            mouse_x, mouse_y = pygame.mouse.get_pos()

            # 绘制十字线
            pygame.draw.line(sim_surface, (200, 100, 50),
                           (mouse_x, 0), (mouse_x, HEIGHT), 1)
            pygame.draw.line(sim_surface, (200, 100, 50),
                           (0, mouse_y), (WIDTH, mouse_y), 1)

            # 绘制坐标提示
            font = pygame.font.Font(None, 20)
            coord_text = f"({mouse_x}, {mouse_y})"
            text_surface = font.render(coord_text, True, (200, 100, 50))
            sim_surface.blit(text_surface, (mouse_x + 10, mouse_y + 10))

        # 绘制信息面板
        self.draw_info_panel()

        # 更新显示
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
            self.action_queue.shutdown()
            pygame.quit()
            print("[Simulator] 已关闭", file=sys.stderr)


def main():
    """主函数"""
    simulator = EnhancedSimulator()

    # 初始生成一个敌人
    simulator.spawn_random_enemy()

    # 运行
    simulator.run()


if __name__ == "__main__":
    main()
