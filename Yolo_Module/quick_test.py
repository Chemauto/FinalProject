#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO 模块快速测试

快速验证 YOLO 标注格式是否正确
"""

import sys
import os

# 添加项目路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from Yolo_Module.yolo_simulator import enemy_to_yolo_label
from Sim_Module.enemy_manager import Enemy


def print_divider(char="=", length=60):
    """打印分隔线"""
    print(char * length)


def test_label_conversion():
    """测试标注转换"""
    print_divider()
    print("YOLO 标注格式测试")
    print_divider()

    # 测试数据
    test_cases = [
        {"x": 400, "y": 300, "size": 15, "desc": "屏幕中心"},
        {"x": 100, "y": 100, "size": 15, "desc": "左上角"},
        {"x": 700, "y": 500, "size": 15, "desc": "右下角"},
        {"x": 400, "y": 150, "size": 15, "desc": "上方中部"},
    ]

    img_width, img_height = 800, 600

    print(f"\n图片尺寸: {img_width} x {img_height}")
    print(f"敌人半径: 15 像素")
    print(f"边界框大小: 30 x 30 像素\n")

    for i, tc in enumerate(test_cases, 1):
        enemy = Enemy(id=f"test_{i}", x=tc["x"], y=tc["y"], size=tc["size"])
        yolo_label = enemy_to_yolo_label(enemy, img_width, img_height)

        print(f"测试 {i}: {tc['desc']}")
        print(f"  原始坐标: ({enemy.x}, {enemy.y})")
        print(f"  YOLO 标注: {yolo_label}")

        # 解析验证
        parts = yolo_label.split()
        class_id, cx, cy, w, h = int(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])

        # 转换回像素坐标
        cx_px = cx * img_width
        cy_px = cy * img_height
        w_px = w * img_width
        h_px = h * img_height

        print(f"  验证 - 中心: ({cx_px:.1f}, {cy_px:.1f}), 尺寸: {w_px:.1f}x{h_px:.1f}")

        # 验证范围
        assert 0 <= cx <= 1 and 0 <= cy <= 1 and 0 <= w <= 1 and 0 <= h <= 1, "坐标超出范围"
        print(f"  ✓ 通过\n")

    print_divider()
    print("所有测试通过！")
    print_divider()


def print_format_guide():
    """打印格式说明"""
    print("\n" + "=" * 60)
    print("YOLO 训练数据格式说明")
    print("=" * 60)

    print("\n1. 标注文件格式 (每行一个目标):")
    print("   class_id center_x center_y width height")
    print("   所有值归一化到 [0, 1]")

    print("\n2. 示例:")
    print("   0 0.500000 0.500000 0.037500 0.050000")
    print("   │ │         │         │         │")
    print("   │ │         │         │         └─ height (归一化)")
    print("   │ │         │         └───────────── width (归一化)")
    print("   │ │         └───────────────────────── center_y (归一化)")
    print("   │ └───────────────────────────────────── center_x (归一化)")
    print("   └────────────────────────────────────────── class_id (0=enemy)")

    print("\n3. 坐标转换公式:")
    print("   center_x = enemy_x / image_width")
    print("   center_y = enemy_y / image_height")
    print("   width = (2 * radius) / image_width")
    print("   height = (2 * radius) / image_height")

    print("\n4. 对于本系统:")
    print("   image_width = 800, image_height = 600")
    print("   enemy radius = 15 像素")
    print("   bbox size = 30 x 30 像素")
    print("   width = 30 / 800 = 0.0375")
    print("   height = 30 / 600 = 0.05")

    print("\n5. 目录结构:")
    print("   Yolo_Module/data/")
    print("   ├── images/           # 原始截图 (.png)")
    print("   ├── labels/           # 标注文件 (.txt)")
    print("   └── bbox_viz/         # 可视化图片 (.png)")

    print("\n6. 文件命名:")
    print("   images/sample_00000.png  ->  labels/sample_00000.txt")
    print("   images/sample_00001.png  ->  labels/sample_00001.txt")
    print("   ...")

    print("=" * 60 + "\n")


def main():
    """主函数"""
    print_format_guide()
    test_label_conversion()

    print("\n下一步:")
    print("1. 生成训练数据: python3 Yolo_Module/yolo_simulator.py")
    print("2. 可视化标注: python3 Yolo_Module/visualize_labels.py")
    print("3. 训练 YOLO 模型 (使用 ultralytics)")
    print("4. 使用模型检测: python3 Yolo_Module/yolo_detector.py")
    print()


if __name__ == "__main__":
    main()
