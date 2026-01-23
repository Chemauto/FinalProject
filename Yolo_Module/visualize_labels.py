#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO 标注可视化工具

功能：
1. 读取 YOLO 格式标注文件
2. 在图片上绘制边界框
3. 验证标注是否正确
"""

import os
import sys
import argparse

# 添加项目路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

import pygame


def yolo_label_to_bbox(label_line, img_width, img_height):
    """
    将 YOLO 标注转换为边界框坐标

    Args:
        label_line: YOLO 格式标注 "class_id center_x center_y width height"
        img_width: 图片宽度
        img_height: 图片高度

    Returns:
        dict: {"class_id": int, "bbox": (x1, y1, x2, y2)}
    """
    parts = label_line.strip().split()
    if len(parts) != 5:
        return None

    class_id = int(parts[0])
    center_x = float(parts[1]) * img_width
    center_y = float(parts[2]) * img_height
    width = float(parts[3]) * img_width
    height = float(parts[4]) * img_height

    # 计算边界框左上角和右下角
    x1 = int(center_x - width / 2)
    y1 = int(center_y - height / 2)
    x2 = int(center_x + width / 2)
    y2 = int(center_y + height / 2)

    return {"class_id": class_id, "bbox": (x1, y1, x2, y2)}


def visualize_labels(image_path, label_path, output_path=None, show=True):
    """
    可视化 YOLO 标注

    Args:
        image_path: 图片路径
        label_path: 标注文件路径
        output_path: 输出图片路径（可选）
        show: 是否显示窗口
    """
    if not os.path.exists(image_path):
        print(f"[Error] 图片不存在: {image_path}")
        return

    # 加载图片
    img = pygame.image.load(image_path)
    img_width, img_height = img.get_size()

    # 读取标注
    if not os.path.exists(label_path):
        print(f"[Warning] 标注文件不存在: {label_path}")
        annotations = []
    else:
        with open(label_path, 'r') as f:
            lines = f.readlines()
        annotations = [yolo_label_to_bbox(line, img_width, img_height)
                      for line in lines if line.strip()]
        annotations = [a for a in annotations if a is not None]

    # 绘制
    screen = pygame.display.set_mode((img_width, img_height))
    screen.blit(img, (0, 0))

    # 颜色定义
    colors = [
        (0, 255, 0),    # 绿色
        (255, 0, 0),    # 蓝色
        (0, 0, 255),    # 红色
        (255, 255, 0),  # 青色
        (255, 0, 255),  # 品红
        (0, 255, 255),  # 黄色
    ]

    # 绘制边界框
    font = pygame.font.Font(None, 24)

    for i, ann in enumerate(annotations):
        class_id = ann["class_id"]
        x1, y1, x2, y2 = ann["bbox"]
        color = colors[class_id % len(colors)]

        # 绘制矩形框
        pygame.draw.rect(screen, color, (x1, y1, x2 - x1, y2 - y1), 3)

        # 绘制类别标签
        label_text = f"Class {class_id}"
        text_surface = font.render(label_text, True, color)
        screen.blit(text_surface, (x1, y1 - 25))

    # 保存结果
    if output_path:
        pygame.image.save(screen, output_path)
        print(f"[Vis] 已保存可视化结果: {output_path}")

    # 显示窗口
    if show:
        pygame.display.set_caption(f"YOLO Label Viewer - {len(annotations)} objects")
        pygame.display.flip()

        waiting = True
        while waiting:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    waiting = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        waiting = False
                    elif event.key == pygame.K_SPACE:
                        waiting = False

        pygame.quit()


def visualize_directory(images_dir, labels_dir, output_dir=None, show=False):
    """
    批量可视化整个目录

    Args:
        images_dir: 图片目录
        labels_dir: 标注目录
        output_dir: 输出目录（可选）
        show: 是否显示窗口
    """
    os.makedirs(output_dir, exist_ok=True) if output_dir else None

    image_files = [f for f in os.listdir(images_dir) if f.endswith(('.png', '.jpg', '.jpeg'))]
    print(f"[Vis] 找到 {len(image_files)} 张图片")

    for i, img_file in enumerate(image_files):
        img_path = os.path.join(images_dir, img_file)
        label_file = os.path.splitext(img_file)[0] + ".txt"
        label_path = os.path.join(labels_dir, label_file)

        output_path = None
        if output_dir:
            output_path = os.path.join(output_dir, f"viz_{img_file}")

        print(f"[Vis] 处理 [{i+1}/{len(image_files)}]: {img_file}")

        visualize_labels(img_path, label_path, output_path, show=show)


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description="YOLO 标注可视化工具")
    parser.add_argument("--images", type=str, help="图片目录或单张图片路径")
    parser.add_argument("--labels", type=str, help="标注目录或单张标注路径")
    parser.add_argument("--output", type=str, help="输出目录")
    parser.add_argument("--show", action="store_true", help="显示窗口")
    parser.add_argument("--single", action="store_true", help="单张图片模式")

    args = parser.parse_args()

    # 默认路径
    data_dir = os.path.join(os.path.dirname(__file__), "data")
    default_images = os.path.join(data_dir, "images")
    default_labels = os.path.join(data_dir, "labels")
    default_output = os.path.join(data_dir, "visualized")

    if args.single:
        # 单张图片模式
        img_path = args.images or os.path.join(default_images, "sample_00000.png")
        label_path = args.labels or os.path.join(default_labels, "sample_00000.txt")
        output_path = args.output or os.path.join(default_output, "sample_00000.png")

        visualize_labels(img_path, label_path, output_path, show=args.show)

    else:
        # 批量模式
        images_dir = args.images or default_images
        labels_dir = args.labels or default_labels
        output_dir = args.output or default_output

        visualize_directory(images_dir, labels_dir, output_dir, show=args.show)


if __name__ == "__main__":
    main()
