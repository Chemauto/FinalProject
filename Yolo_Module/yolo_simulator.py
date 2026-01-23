#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO 训练数据生成仿真器

功能：
1. 自动随机撒点生成敌人
2. 截图保存
3. 生成 YOLO 格式标注文件
4. 可视化标注（画框）

YOLO 标注格式（归一化）：
class_id center_x center_y width height
"""

import sys
import os
import random
import json

# 添加项目路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

import pygame
from Sim_Module.enemy_manager import EnemyManager

# --- Constants ---
WIDTH, HEIGHT = 800, 600
WHITE = (250, 250, 250)
BLACK = (0, 0, 0)
GRID_COLOR = (220, 220, 220)
GRID_SPACING = 50

# 敌人参数（与仿真器一致）
ENEMY_RADIUS = 15
ENEMY_COLOR = (255, 50, 50)

# 类别ID（YOLO格式）
CLASS_ID = 0  # 敌人类


def draw_grid(screen):
    """绘制网格背景"""
    for x in range(0, WIDTH, GRID_SPACING):
        pygame.draw.line(screen, GRID_COLOR, (x, 0), (x, HEIGHT))
    for y in range(0, HEIGHT, GRID_SPACING):
        pygame.draw.line(screen, GRID_COLOR, (0, y), (WIDTH, y))


def draw_enemy_with_bbox(screen, enemy, show_bbox=True, bbox_color=(0, 255, 0), bbox_width=2):
    """
    绘制敌人及其边界框

    Args:
        screen: Pygame屏幕
        enemy: 敌人对象
        show_bbox: 是否显示边界框
        bbox_color: 边界框颜色
        bbox_width: 边界框线宽
    """
    x, y = int(enemy.x), int(enemy.y)
    r = enemy.size

    # 绘制敌人（圆形）
    pygame.draw.circle(screen, enemy.color, (x, y), r, 3)
    pygame.draw.circle(screen, (*enemy.color[:3], 100), (x, y), r - 5)

    # 绘制边界框（外接正方形）
    if show_bbox:
        bbox_rect = pygame.Rect(x - r, y - r, 2 * r, 2 * r)
        pygame.draw.rect(screen, bbox_color, bbox_rect, bbox_width)


def enemy_to_yolo_label(enemy, img_width, img_height):
    """
    将敌人位置转换为 YOLO 格式标注

    YOLO 格式：class_id center_x center_y width height
    所有坐标归一化到 [0, 1]

    Args:
        enemy: 敌人对象
        img_width: 图片宽度
        img_height: 图片高度

    Returns:
        str: YOLO 格式标注字符串
    """
    # 圆形的外接正方形边界框
    x_center = enemy.x
    y_center = enemy.y
    bbox_width = 2 * enemy.size
    bbox_height = 2 * enemy.size

    # 归一化到 [0, 1]
    x_norm = x_center / img_width
    y_norm = y_center / img_height
    w_norm = bbox_width / img_width
    h_norm = bbox_height / img_height

    # YOLO 格式：class_id center_x center_y width height
    return f"{CLASS_ID} {x_norm:.6f} {y_norm:.6f} {w_norm:.6f} {h_norm:.6f}"


def generate_yolo_dataset(output_dir, num_samples=100, min_enemies=1, max_enemies=5,
                          show_bbox=False, save_bbox_viz=True):
    """
    生成 YOLO 训练数据集

    Args:
        output_dir: 输出目录
        num_samples: 生成的样本数量
        min_enemies: 每张图最少敌人数
        max_enemies: 每张图最多敌人数
        show_bbox: 是否显示边界框（在原始截图中）
        save_bbox_viz: 是否保存带边界框的可视化图片
    """
    # 创建输出目录
    images_dir = os.path.join(output_dir, "images")
    labels_dir = os.path.join(output_dir, "labels")
    bbox_viz_dir = os.path.join(output_dir, "bbox_viz")

    os.makedirs(images_dir, exist_ok=True)
    os.makedirs(labels_dir, exist_ok=True)
    if save_bbox_viz:
        os.makedirs(bbox_viz_dir, exist_ok=True)

    # 初始化 Pygame
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("YOLO Training Data Generator")
    clock = pygame.time.Clock()

    # 创建敌人管理器
    enemy_manager = EnemyManager(bounds=(WIDTH, HEIGHT))

    # 生成数据
    print(f"[YOLO Gen] 开始生成 {num_samples} 个样本...")
    print(f"[YOLO Gen] 输出目录: {output_dir}")

    metadata = []

    for i in range(num_samples):
        # 清除之前的敌人
        enemy_manager.clear_all()

        # 随机生成敌人
        num_enemies = random.randint(min_enemies, max_enemies)
        for _ in range(num_enemies):
            # 随机位置（留出边界框的边距）
            margin = ENEMY_RADIUS + 10
            x = random.uniform(margin, WIDTH - margin)
            y = random.uniform(margin, HEIGHT - margin)

            enemy_manager.spawn_enemy(x=x, y=y, color=ENEMY_COLOR, move_mode="static")

        # 绘制场景
        screen.fill(WHITE)
        draw_grid(screen)

        enemies = enemy_manager.get_all_enemies()

        # 绘制敌人（可选：显示边界框）
        for enemy in enemies:
            draw_enemy_with_bbox(screen, enemy, show_bbox=show_bbox)

        pygame.display.flip()

        # 生成文件名
        filename = f"sample_{i:05d}"

        # 保存截图（原始图，可选是否带框）
        img_path = os.path.join(images_dir, f"{filename}.png")
        pygame.image.save(screen, img_path)

        # 生成 YOLO 标注文件
        label_path = os.path.join(labels_dir, f"{filename}.txt")
        with open(label_path, 'w') as f:
            for enemy in enemies:
                label_line = enemy_to_yolo_label(enemy, WIDTH, HEIGHT)
                f.write(label_line + '\n')

        # 保存带边界框的可视化图片
        if save_bbox_viz:
            # 重新绘制，这次带明显的边界框
            screen.fill(WHITE)
            draw_grid(screen)
            for enemy in enemies:
                draw_enemy_with_bbox(screen, enemy, show_bbox=True,
                                   bbox_color=(0, 255, 0), bbox_width=3)
            pygame.display.flip()

            bbox_path = os.path.join(bbox_viz_dir, f"{filename}.png")
            pygame.image.save(screen, bbox_path)

        # 记录元数据
        sample_meta = {
            "filename": filename,
            "image_path": img_path,
            "label_path": label_path,
            "num_enemies": len(enemies),
            "enemies": [
                {"id": e.id, "x": e.x, "y": e.y}
                for e in enemies
            ]
        }
        metadata.append(sample_meta)

        # 打印进度
        if (i + 1) % 10 == 0:
            print(f"[YOLO Gen] 已生成 {i + 1}/{num_samples} 样本")

        # 处理事件（防止窗口卡死）
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print(f"[YOLO Gen] 用户中断，已生成 {i + 1} 个样本")
                pygame.quit()
                return metadata

    # 保存元数据
    metadata_path = os.path.join(output_dir, "metadata.json")
    with open(metadata_path, 'w', encoding='utf-8') as f:
        json.dump(metadata, f, indent=2, ensure_ascii=False)

    print(f"[YOLO Gen] 完成！共生成 {num_samples} 个样本")
    print(f"[YOLO Gen] 图片保存至: {images_dir}")
    print(f"[YOLO Gen] 标注保存至: {labels_dir}")
    if save_bbox_viz:
        print(f"[YOLO Gen] 可视化保存至: {bbox_viz_dir}")
    print(f"[YOLO Gen] 元数据: {metadata_path}")

    pygame.quit()
    return metadata


def print_yolo_format_example():
    """打印 YOLO 格式说明"""
    print("\n" + "=" * 60)
    print("YOLO 训练数据格式说明")
    print("=" * 60)
    print("\n1. 图片文件:")
    print("   - 路径: data/images/sample_00000.png")
    print("   - 格式: PNG, 尺寸 800x600")
    print("   - 内容: 仿真器截图，包含敌人和网格背景")
    print("\n2. 标注文件:")
    print("   - 路径: data/labels/sample_00000.txt")
    print("   - 格式: 每行一个目标")
    print("   - 格式: class_id center_x center_y width height")
    print("   - 所有值归一化到 [0, 1]")
    print("\n3. 坐标转换:")
    print("   - 敌人是圆形，半径 15 像素")
    print("   - 边界框是外接正方形 (30x30 像素)")
    print("   - center_x = enemy.x / image_width")
    print("   - center_y = enemy.y / image_height")
    print("   - width = (2 * radius) / image_width = 30 / 800")
    print("   - height = (2 * radius) / image_height = 30 / 600")
    print("\n4. 示例标注文件:")
    print("   0 0.500000 0.400000 0.037500 0.050000")
    print("   0 0.750000 0.600000 0.037500 0.050000")
    print("\n5. 类别:")
    print("   - 0: 敌人 (enemy)")
    print("=" * 60 + "\n")


def main():
    """主函数"""
    print_yolo_format_example()

    # 配置参数
    OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "data")
    NUM_SAMPLES = 100
    MIN_ENEMIES = 1
    MAX_ENEMIES = 5

    # 生成数据集
    metadata = generate_yolo_dataset(
        output_dir=OUTPUT_DIR,
        num_samples=NUM_SAMPLES,
        min_enemies=MIN_ENEMIES,
        max_enemies=MAX_ENEMIES,
        show_bbox=False,  # 原始图不显示边界框
        save_bbox_viz=True  # 保存带框的可视化图
    )

    # 打印统计信息
    print("\n" + "=" * 60)
    print("数据集统计")
    print("=" * 60)
    total_enemies = sum(m["num_enemies"] for m in metadata)
    print(f"总样本数: {len(metadata)}")
    print(f"总目标数: {total_enemies}")
    print(f"平均每张图目标数: {total_enemies / len(metadata):.2f}")
    print("=" * 60)


if __name__ == "__main__":
    main()
