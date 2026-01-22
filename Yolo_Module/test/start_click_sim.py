#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
一键启动仿真器 + 鼠标点击生成敌人

运行此脚本后：
1. 自动启动仿真器
2. 自动进入鼠标标记模式
3. 点击地图任意位置生成敌人
4. 按 ESC 退出
"""

import sys
import os
import pygame
from pathlib import Path

# Reconfigure stdout to use UTF-8 encoding on Windows
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

# 添加项目路径（从 test/ 向上三级到项目根目录）
project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

# 导入必要模块
from Sim_Module.sim2d.simulator import Robot, draw_grid, WIDTH, HEIGHT
from Test_Module.enemy_manager import EnemyManager

# 常量
INFO_PANEL_HEIGHT = 80
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRID_COLOR = (220, 220, 220)
GRID_SPACING = 50


class ClickSimulator:
    """点击生成敌人仿真器"""

    def __init__(self):
        print("="*60)
        print("鼠标点击仿真器")
        print("="*60)
        print("操作说明:")
        print("  - 点击地图: 在点击位置生成敌人")
        print("  - 按 C: 清除所有敌人")
        print("  - 按 ESC: 退出")
        print("="*60)
        print()

        # 初始化 Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT + INFO_PANEL_HEIGHT))
        pygame.display.set_caption("鼠标点击仿真器 - 点击生成敌人")
        self.clock = pygame.time.Clock()

        # 创建机器人和敌人管理器
        self.robot = Robot(WIDTH // 2, HEIGHT // 2)
        self.enemy_manager = EnemyManager(bounds=(WIDTH, HEIGHT))

        # 状态
        self.running = True
        self.mouse_x = 0
        self.mouse_y = 0

    def handle_events(self):
        """处理事件"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            elif event.type == pygame.MOUSEBUTTONDOWN:
                # 鼠标点击
                if event.button == 1:  # 左键
                    mouse_x, mouse_y = pygame.mouse.get_pos()

                    # 确保点击在仿真区域内
                    if mouse_y < HEIGHT:
                        self.spawn_enemy_at(mouse_x, mouse_y)

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_c:
                    self.clear_enemies()

    def spawn_enemy_at(self, x, y):
        """在指定位置生成敌人"""
        enemy = self.enemy_manager.spawn_enemy(
            x=x,
            y=y,
            color=(255, 50, 50),
            move_mode="static",
            name=f"敌人{self.enemy_manager.next_id}"
        )

        print(f"[生成敌人] {enemy.name} at ({x}, {y})")

    def clear_enemies(self):
        """清除所有敌人"""
        count = len(self.enemy_manager.enemies)
        self.enemy_manager.clear_all()
        print(f"[清除敌人] 删除了 {count} 个敌人")

    def update(self):
        """更新状态"""
        # 更新机器人
        self.robot.update()

        # 更新敌人
        robot_pos = {'x': self.robot.x, 'y': self.robot.y, 'angle': self.robot.angle}
        self.enemy_manager.update(robot_pos=robot_pos)

        # 更新鼠标位置
        self.mouse_x, self.mouse_y = pygame.mouse.get_pos()

    def draw(self):
        """绘制场景"""
        # 主仿真区域
        sim_surface = self.screen.subsurface((0, 0, WIDTH, HEIGHT))
        sim_surface.fill(WHITE)
        draw_grid(sim_surface)

        # 绘制鼠标十字线
        if self.mouse_y < HEIGHT:
            pygame.draw.line(sim_surface, (200, 100, 50),
                           (self.mouse_x, 0), (self.mouse_x, HEIGHT), 1)
            pygame.draw.line(sim_surface, (200, 100, 50),
                           (0, self.mouse_y), (WIDTH, self.mouse_y), 1)

            # 显示坐标
            font = pygame.font.Font(None, 24)
            coord_text = f"({self.mouse_x}, {self.mouse_y})"
            text_surface = font.render(coord_text, True, (200, 100, 50))
            sim_surface.blit(text_surface, (self.mouse_x + 10, self.mouse_y + 10))

        # 绘制敌人
        self.enemy_manager.draw_all(sim_surface)

        # 绘制机器人
        self.robot.draw(sim_surface)

        # 绘制信息面板
        self.draw_info_panel()

        # 更新显示
        pygame.display.flip()

    def draw_info_panel(self):
        """绘制信息面板"""
        panel_y = HEIGHT
        panel_rect = pygame.Rect(0, panel_y, WIDTH, INFO_PANEL_HEIGHT)

        # 背景
        pygame.draw.rect(self.screen, (240, 240, 240), panel_rect)
        pygame.draw.line(self.screen, (180, 180, 180),
                        (0, panel_y), (WIDTH, panel_y), 2)

        # 字体
        title_font = pygame.font.Font(None, 32)
        info_font = pygame.font.Font(None, 28)

        # 标题
        title = title_font.render("鼠标点击生成敌人", True, (50, 50, 50))
        self.screen.blit(title, (20, panel_y + 10))

        # 统计信息
        info_text = f"敌人: {len(self.enemy_manager.enemies)} 个"
        info_surface = info_font.render(info_text, True, (80, 80, 80))
        self.screen.blit(info_surface, (20, panel_y + 45))

        # 操作提示
        hint_text = "点击地图生成  [C]清除  [ESC]退出"
        hint_surface = info_font.render(hint_text, True, (120, 120, 120))
        self.screen.blit(hint_surface, (WIDTH - 350, panel_y + 25))

    def run(self):
        """运行仿真器"""
        try:
            while self.running:
                self.handle_events()
                self.update()
                self.draw()
                self.clock.tick(60)

        except KeyboardInterrupt:
            print("\n[仿真器] 被用户中断")
        finally:
            pygame.quit()
            print("[仿真器] 已关闭")


def main():
    """主函数"""
    print("\n正在启动仿真器...\n")

    simulator = ClickSimulator()
    simulator.run()


if __name__ == "__main__":
    main()
