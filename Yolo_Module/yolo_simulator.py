#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
追击功能测试仿真器

简化版仿真器，用于测试追击功能：
- 鼠标点击生成敌人
- 显示机器人位置和敌人位置
- 支持通过 ROS2 接收追击命令
"""

import sys
import os
import pygame
import math

# 添加项目路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 导入原有仿真器的 Robot 类
from Sim_Module.sim2d.simulator import Robot, draw_grid, WIDTH, HEIGHT

# 导入敌人管理器
from Test_Module.enemy_manager import EnemyManager

# 导入 ROS 通信
from ros_topic_comm import get_shared_queue, set_robot_state, set_enemy_positions, publish_robot_state

# 中文字体
_TITLE_FONT = None
_INFO_FONT = None
_SMALL_FONT = None


def get_chinese_font(size):
    """获取支持中文的字体"""
    font_paths = [
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK.ttc",
        "/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    ]

    for font_path in font_paths:
        if os.path.exists(font_path):
            try:
                return pygame.font.Font(font_path, size)
            except:
                continue

    return pygame.font.SysFont("WenQuanYi Micro Hei,SimHei,Noto Sans CJK SC", size)


def init_fonts():
    """初始化字体"""
    global _TITLE_FONT, _INFO_FONT, _SMALL_FONT
    _TITLE_FONT = get_chinese_font(28)
    _INFO_FONT = get_chinese_font(24)
    _SMALL_FONT = get_chinese_font(20)


class YoloSimulator:
    """追击仿真器"""

    def __init__(self):
        pygame.init()
        init_fonts()

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
        from ros_topic_comm import get_enemy_remove_subscriber
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

        # 发布机器人状态到 ROS 话题（跨进程）
        publish_robot_state(robot_pos['x'], robot_pos['y'], robot_pos['angle'])

        # 更新敌人位置（给 chase 模块使用）
        enemies = self.enemy_manager.get_all_enemies()
        enemy_positions = [
            {'id': e.id, 'x': e.x, 'y': e.y}
            for e in enemies
        ]
        set_enemy_positions(enemy_positions)

        # 调试：每60帧打印一次
        if hasattr(self, '_frame_count'):
            self._frame_count += 1
        else:
            self._frame_count = 0

        if self._frame_count % 60 == 0 and len(enemy_positions) > 0:
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

        # 绘制信息
        self._draw_info()

        pygame.display.flip()

    def _draw_info(self):
        """绘制信息"""
        robot_pos = {'x': self.robot.x, 'y': self.robot.y, 'angle': self.robot.angle}
        info_texts = [
            f"敌人: {len(self.enemy_manager.enemies)}"
        ]

        y = 10
        for text in info_texts:
            surface = _INFO_FONT.render(text, True, (80, 80, 80))
            self.screen.blit(surface, (10, y))
            y += 30

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
    simulator = YoloSimulator()
    simulator.run()


if __name__ == "__main__":
    main()
