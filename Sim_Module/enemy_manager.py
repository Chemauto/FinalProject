#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
敌人管理模块

负责敌人生成、移动和状态管理
"""

import random
import math
from typing import Dict, List, Optional
from dataclasses import dataclass, field


@dataclass
class Enemy:
    """敌人类"""

    id: str
    x: float
    y: float
    color: tuple = (255, 50, 50)  # 默认红色
    size: int = 15
    name: str = ""

    # 移动模式
    move_mode: str = "static"  # static, random, flee, pattern
    move_speed: float = 0.0
    move_timer: float = 0.0

    def __post_init__(self):
        if not self.name:
            self.name = f"Enemy_{self.id}"

    def get_position(self) -> Dict[str, float]:
        """获取当前位置"""
        return {"x": self.x, "y": self.y, "id": self.id}

    def move(self, mode: str = None, robot_pos: Dict = None,
             bounds: tuple = (800, 600)):
        """
        移动敌人

        Args:
            mode: 移动模式
            robot_pos: 机器人位置（用于flee模式）
            bounds: 边界 (width, height)
        """
        mode = mode or self.move_mode

        if mode == "static":
            return  # 不移动

        elif mode == "random":
            # 随机游走
            if random.random() < 0.1:  # 10%概率改变方向
                angle = random.uniform(0, 2 * math.pi)
                self.move_speed = random.uniform(0.5, 2.0)

            self.x += math.cos(angle) * self.move_speed
            self.y += math.sin(angle) * self.move_speed

        elif mode == "flee" and robot_pos:
            # 远离机器人
            dx = self.x - robot_pos['x']
            dy = self.y - robot_pos['y']
            dist = math.sqrt(dx**2 + dy**2)

            if dist > 0:
                flee_speed = 1.5
                self.x += (dx / dist) * flee_speed
                self.y += (dy / dist) * flee_speed

        elif mode == "pattern":
            # 圆形运动模式
            self.move_timer += 0.05
            center_x, center_y = 400, 300
            radius = 100
            self.x = center_x + radius * math.cos(self.move_timer)
            self.y = center_y + radius * math.sin(self.move_timer)

        # 边界检查
        self.x = max(self.size, min(bounds[0] - self.size, self.x))
        self.y = max(self.size, min(bounds[1] - self.size, self.y))

    def draw(self, screen):
        """绘制敌人"""
        import pygame

        # 绘制外圈
        pygame.draw.circle(screen, self.color,
                         (int(self.x), int(self.y)), self.size, 3)

        # 绘制内圈
        pygame.draw.circle(screen, (*self.color, 100),
                         (int(self.x), int(self.y)), self.size - 5)

        # 绘制ID标签
        font = pygame.font.Font(None, 20)
        text = font.render(self.name, True, (50, 50, 50))
        text_rect = text.get_rect(center=(int(self.x), int(self.y)))
        screen.blit(text, text_rect)


class EnemyManager:
    """敌人管理器"""

    def __init__(self, bounds: tuple = (800, 600)):
        """
        初始化敌人管理器

        Args:
            bounds: 边界 (width, height)
        """
        self.bounds = bounds
        self.enemies: Dict[str, Enemy] = {}
        self.next_id = 1

    def spawn_enemy(self, x: float = None, y: float = None,
                   color: tuple = None, move_mode: str = "static",
                   name: str = "") -> Enemy:
        """
        生成新敌人

        Args:
            x, y: 坐标（随机如果未指定）
            color: 颜色
            move_mode: 移动模式
            name: 名称

        Returns:
            创建的敌人对象
        """
        # 随机位置
        if x is None:
            x = random.uniform(50, self.bounds[0] - 50)
        if y is None:
            y = random.uniform(50, self.bounds[1] - 50)

        # 默认颜色
        if color is None:
            color = (
                random.randint(100, 255),
                random.randint(50, 150),
                random.randint(50, 150)
            )

        enemy_id = str(self.next_id)
        self.next_id += 1

        enemy = Enemy(
            id=enemy_id,
            x=x,
            y=y,
            color=color,
            move_mode=move_mode,
            name=name or f"敌人{enemy_id}"
        )

        self.enemies[enemy_id] = enemy

        print(f"[EnemyManager] 生成敌人: {enemy.name} at ({x:.1f}, {y:.1f})",
              file=sys.stderr)

        return enemy

    def spawn_random_enemies(self, count: int,
                            move_modes: List[str] = None) -> List[Enemy]:
        """
        随机生成多个敌人

        Args:
            count: 数量
            move_modes: 允许的移动模式

        Returns:
            敌人列表
        """
        if move_modes is None:
            move_modes = ["static", "random", "flee"]

        enemies = []
        for _ in range(count):
            mode = random.choice(move_modes)
            enemy = self.spawn_enemy(move_mode=mode)
            enemies.append(enemy)

        return enemies

    def get_enemy(self, enemy_id: str) -> Optional[Enemy]:
        """获取指定敌人"""
        return self.enemies.get(enemy_id)

    def get_all_enemies(self) -> List[Enemy]:
        """获取所有敌人"""
        return list(self.enemies.values())

    def get_nearest_enemy(self, robot_pos: Dict[str, float]) -> Optional[Enemy]:
        """
        获取最近的敌人

        Args:
            robot_pos: 机器人位置 {'x': x, 'y': y}

        Returns:
            最近的敌人对象
        """
        if not self.enemies:
            return None

        nearest = None
        min_dist = float('inf')

        for enemy in self.enemies.values():
            dist = math.sqrt(
                (enemy.x - robot_pos['x'])**2 +
                (enemy.y - robot_pos['y'])**2
            )
            if dist < min_dist:
                min_dist = dist
                nearest = enemy

        return nearest

    def remove_enemy(self, enemy_id: str) -> bool:
        """移除敌人"""
        # 确保 enemy_id 是字符串
        enemy_id_str = str(enemy_id)

        if enemy_id_str in self.enemies:
            del self.enemies[enemy_id_str]
            print(f"[EnemyManager] 移除敌人: {enemy_id_str}", file=sys.stderr)
            return True
        else:
            print(f"[EnemyManager] 移除失败: 找不到敌人ID {enemy_id_str}, 现有: {list(self.enemies.keys())}", file=sys.stderr)
            return False

    def clear_all(self):
        """清除所有敌人"""
        self.enemies.clear()
        print("[EnemyManager] 清除所有敌人", file=sys.stderr)

    def update(self, robot_pos: Dict = None):
        """
        更新所有敌人状态

        Args:
            robot_pos: 机器人位置（用于flee模式）
        """
        for enemy in self.enemies.values():
            enemy.move(robot_pos=robot_pos, bounds=self.bounds)

    def draw_all(self, screen):
        """绘制所有敌人"""
        for enemy in self.enemies.values():
            enemy.draw(screen)

    def draw_line_to_robot(self, screen, robot_pos: Dict, enemy_id: str = None,
                          color: tuple = (100, 100, 100)):
        """
        绘制机器人到敌人的连线

        Args:
            screen: Pygame屏幕
            robot_pos: 机器人位置
            enemy_id: 敌人ID（None=所有敌人）
            color: 连线颜色
        """
        import pygame

        enemies = [self.enemies[enemy_id]] if enemy_id else self.enemies.values()

        for enemy in enemies:
            # 绘制虚线效果
            start = (int(robot_pos['x']), int(robot_pos['y']))
            end = (int(enemy.x), int(enemy.y))

            # 绘制连线
            pygame.draw.line(screen, color, start, end, 2)

            # 绘制距离标记
            dist = math.sqrt(
                (enemy.x - robot_pos['x'])**2 +
                (enemy.y - robot_pos['y'])**2
            )
            dist_m = dist / 100.0  # 像素转米

            mid_x = (start[0] + end[0]) // 2
            mid_y = (start[1] + end[1]) // 2

            font = pygame.font.Font(None, 24)
            text = font.render(f"{dist_m:.1f}m", True, (80, 80, 80))
            screen.blit(text, (mid_x + 5, mid_y))


# 导入sys
import sys
