#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO 模块测试脚本

测试功能：
1. 训练数据生成
2. 标注格式验证
3. 可视化测试
"""

import sys
import os

# 添加项目路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from Yolo_Module.yolo_simulator import enemy_to_yolo_label
from Sim_Module.enemy_manager import Enemy


def test_yolo_label_format():
    """测试 YOLO 标注格式"""
    print("=" * 60)
    print("测试 YOLO 标注格式转换")
    print("=" * 60)

    # 创建测试敌人
    enemy = Enemy(
        id="test_1",
        x=400.0,  # 屏幕中心
        y=300.0,
        size=15    # 半径 15 像素
    )

    # 图片尺寸
    img_width, img_height = 800, 600

    # 转换为 YOLO 格式
    yolo_label = enemy_to_yolo_label(enemy, img_width, img_height)

    print(f"\n敌人位置: ({enemy.x}, {enemy.y})")
    print(f"敌人半径: {enemy.size} 像素")
    print(f"边界框大小: {2 * enemy.size} x {2 * enemy.size} 像素")
    print(f"\nYOLO 标注: {yolo_label}")

    # 解析并验证
    parts = yolo_label.split()
    class_id = int(parts[0])
    center_x = float(parts[1])
    center_y = float(parts[2])
    width = float(parts[3])
    height = float(parts[4])

    # 验证范围
    assert 0 <= center_x <= 1, "center_x 超出范围"
    assert 0 <= center_y <= 1, "center_y 超出范围"
    assert 0 <= width <= 1, "width 超出范围"
    assert 0 <= height <= 1, "height 超出范围"

    # 验证值
    expected_x = enemy.x / img_width
    expected_y = enemy.y / img_height
    expected_w = (2 * enemy.size) / img_width
    expected_h = (2 * enemy.size) / img_height

    print(f"\n预期值:")
    print(f"  center_x: {expected_x:.6f} (实际: {center_x:.6f})")
    print(f"  center_y: {expected_y:.6f} (实际: {center_y:.6f})")
    print(f"  width: {expected_w:.6f} (实际: {width:.6f})")
    print(f"  height: {expected_h:.6f} (实际: {height:.6f})")

    assert abs(center_x - expected_x) < 0.0001, "center_x 不匹配"
    assert abs(center_y - expected_y) < 0.0001, "center_y 不匹配"
    assert abs(width - expected_w) < 0.0001, "width 不匹配"
    assert abs(height - expected_h) < 0.0001, "height 不匹配"

    print("\n✓ YOLO 标注格式测试通过")


def test_label_file_format():
    """测试标注文件格式"""
    print("\n" + "=" * 60)
    print("测试标注文件格式")
    print("=" * 60)

    # 示例标注内容
    example_labels = [
        "0 0.500000 0.500000 0.037500 0.050000",
        "0 0.750000 0.250000 0.037500 0.050000",
    ]

    print("\n示例标注文件内容:")
    for line in example_labels:
        print(f"  {line}")

    # 解析并验证
    img_width, img_height = 800, 600

    print("\n解析后的边界框（像素坐标）:")
    for i, line in enumerate(example_labels):
        parts = line.split()
        class_id = int(parts[0])
        cx, cy, w, h = map(float, parts[1:])

        # 转换为像素坐标
        cx_px = cx * img_width
        cy_px = cy * img_height
        w_px = w * img_width
        h_px = h * img_height

        # 计算边界框
        x1 = int(cx_px - w_px / 2)
        y1 = int(cy_px - h_px / 2)
        x2 = int(cx_px + w_px / 2)
        y2 = int(cy_px + h_px / 2)

        print(f"  目标 {i + 1}:")
        print(f"    类别: {class_id}")
        print(f"    中心: ({cx_px:.1f}, {cy_px:.1f})")
        print(f"    尺寸: {w_px:.1f} x {h_px:.1f}")
        print(f"    边界框: ({x1}, {y1}) -> ({x2}, {y2})")

    print("\n✓ 标注文件格式测试通过")


def test_data_generation():
    """测试数据生成"""
    print("\n" + "=" * 60)
    print("测试数据生成")
    print("=" * 60)

    from Yolo_Module.yolo_simulator import generate_yolo_dataset

    # 生成少量测试数据
    test_output_dir = os.path.join(
        os.path.dirname(__file__),
        "..", "data", "test"
    )

    print(f"\n生成测试数据到: {test_output_dir}")

    metadata = generate_yolo_dataset(
        output_dir=test_output_dir,
        num_samples=5,
        min_enemies=1,
        max_enemies=3,
        show_bbox=False,
        save_bbox_viz=True
    )

    # 验证文件生成
    images_dir = os.path.join(test_output_dir, "images")
    labels_dir = os.path.join(test_output_dir, "labels")
    bbox_viz_dir = os.path.join(test_output_dir, "bbox_viz")

    assert os.path.exists(images_dir), "images 目录不存在"
    assert os.path.exists(labels_dir), "labels 目录不存在"
    assert os.path.exists(bbox_viz_dir), "bbox_viz 目录不存在"

    image_files = [f for f in os.listdir(images_dir) if f.endswith('.png')]
    label_files = [f for f in os.listdir(labels_dir) if f.endswith('.txt')]

    assert len(image_files) == 5, f"图片数量不正确: {len(image_files)}"
    assert len(label_files) == 5, f"标注数量不正确: {len(label_files)}"

    print(f"\n生成的文件:")
    print(f"  图片: {len(image_files)} 张")
    print(f"  标注: {len(label_files)} 个")
    print(f"  元数据: {len(metadata)} 条")

    # 验证标注文件格式
    for label_file in label_files:
        label_path = os.path.join(labels_dir, label_file)
        with open(label_path, 'r') as f:
            lines = f.readlines()

        for line in lines:
            parts = line.strip().split()
            assert len(parts) == 5, f"标注格式错误: {line}"

            class_id = int(parts[0])
            assert class_id == 0, f"类别 ID 应该为 0: {class_id}"

            cx, cy, w, h = map(float, parts[1:])
            assert 0 <= cx <= 1, f"center_x 超出范围: {cx}"
            assert 0 <= cy <= 1, f"center_y 超出范围: {cy}"
            assert 0 <= w <= 1, f"width 超出范围: {w}"
            assert 0 <= h <= 1, f"height 超出范围: {h}"

    print("\n✓ 数据生成测试通过")


def main():
    """运行所有测试"""
    print("\n" + "=" * 60)
    print("YOLO 模块测试")
    print("=" * 60)

    try:
        test_yolo_label_format()
        test_label_file_format()
        test_data_generation()

        print("\n" + "=" * 60)
        print("所有测试通过！")
        print("=" * 60)

    except Exception as e:
        print(f"\n✗ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
