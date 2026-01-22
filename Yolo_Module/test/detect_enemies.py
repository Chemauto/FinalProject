#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO 敌人检测示例

使用 YOLO 检测图像或截图中的敌人（人员）。
"""

import sys
import os

# 添加项目路径
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, project_root)

# 直接导入模块
from yolo_detector import YoloDetector
from screen_capture import ScreenCapture


def detect_from_image(image_path, detect_persons=True):
    """
    从图像文件检测敌人

    Args:
        image_path: 图像文件路径
        detect_persons: 是否只检测人员（True）或所有目标（False）
    """
    import cv2

    # 初始化检测器
    print("正在初始化 YOLO 检测器...", file=sys.stderr)
    detector = YoloDetector(model_name="yolov8n.pt")

    # 读取图像
    print(f"正在读取图像: {image_path}", file=sys.stderr)
    image = cv2.imread(image_path)
    if image is None:
        print(f"错误: 无法读取图像 {image_path}", file=sys.stderr)
        return

    # 转换为 RGB
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # 检测
    print("正在检测目标...", file=sys.stderr)
    if detect_persons:
        detections = detector.detect_person(image_rgb)
        print(f"✅ 检测到 {len(detections)} 个敌人", file=sys.stderr)
    else:
        detections = detector.detect(image_rgb)
        print(f"✅ 检测到 {len(detections)} 个目标", file=sys.stderr)

    # 输出检测结果
    print("\n检测结果:")
    print("=" * 60)
    for i, det in enumerate(detections):
        print(f"\n目标 {i + 1}:")
        print(f"  类别: {det['class']}")
        print(f"  置信度: {det['confidence']:.2f}")
        print(f"  中心坐标: ({det['center'][0]:.1f}, {det['center'][1]:.1f})")
        x1, y1, x2, y2 = det['box']
        print(f"  边界框: ({x1:.1f}, {y1:.1f}) - ({x2:.1f}, {y2:.1f})")

    # 绘制检测结果并保存
    output_path = "/tmp/detection_result.jpg"
    for det in detections:
        x1, y1, x2, y2 = [int(v) for v in det['box']]
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"{det['class']} {det['confidence']:.2f}"
        cv2.putText(image, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

    cv2.imwrite(output_path, image)
    print(f"\n✅ 检测结果已保存到: {output_path}")


def detect_from_screen(x, y, width, height, detect_persons=True):
    """
    从屏幕截图检测敌人

    Args:
        x, y, width, height: 屏幕区域
        detect_persons: 是否只检测人员
    """
    import cv2

    # 初始化
    print("正在初始化...", file=sys.stderr)
    detector = YoloDetector(model_name="yolov8n.pt")
    capturer = ScreenCapture()

    # 截图
    print(f"正在截取屏幕区域: ({x}, {y}, {width}, {height})", file=sys.stderr)
    image_array = capturer.capture_region(x, y, width, height)

    # 保存截图
    screenshot_path = "/tmp/screenshot.jpg"
    ScreenCapture.save_image(image_array, screenshot_path)

    # 检测
    print("正在检测目标...", file=sys.stderr)
    if detect_persons:
        detections = detector.detect_person(image_array)
        print(f"✅ 检测到 {len(detections)} 个敌人", file=sys.stderr)
    else:
        detections = detector.detect(image_array)
        print(f"✅ 检测到 {len(detections)} 个目标", file=sys.stderr)

    # 输出检测结果
    print("\n检测结果:")
    print("=" * 60)
    for i, det in enumerate(detections):
        print(f"\n目标 {i + 1}:")
        print(f"  类别: {det['class']}")
        print(f"  置信度: {det['confidence']:.2f}")
        print(f"  中心坐标: ({det['center'][0]:.1f}, {det['center'][1]:.1f})")
        x1, y1, x2, y2 = det['box']
        print(f"  边界框: ({x1:.1f}, {y1:.1f}) - ({x2:.1f}, {y2:.1f})")

    # 绘制检测结果
    image_bgr = cv2.cvtColor(image_array, cv2.COLOR_RGB2BGR)
    for det in detections:
        x1, y1, x2, y2 = [int(v) for v in det['box']]
        cv2.rectangle(image_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"{det['class']} {det['confidence']:.2f}"
        cv2.putText(image_bgr, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

    # 保存结果
    output_path = "/tmp/detection_result.jpg"
    cv2.imwrite(output_path, image_bgr)
    print(f"\n✅ 检测结果已保存到: {output_path}")


def main():
    """主函数"""
    print("=" * 60, file=sys.stderr)
    print("YOLO 敌人检测工具", file=sys.stderr)
    print("=" * 60, file=sys.stderr)

    if len(sys.argv) < 2:
        print("\n使用方法:", file=sys.stderr)
        print("  1. 从图像文件检测:", file=sys.stderr)
        print("     python detect_enemies.py image <图像路径> [all]", file=sys.stderr)
        print("", file=sys.stderr)
        print("  2. 从屏幕截图检测:", file=sys.stderr)
        print("     python detect_enemies.py screen <x> <y> <width> <height> [all]", file=sys.stderr)
        print("", file=sys.stderr)
        print("示例:", file=sys.stderr)
        print("  python detect_enemies.py image /path/to/image.jpg", file=sys.stderr)
        print("  python detect_enemies.py image /path/to/image.jpg all  # 检测所有目标", file=sys.stderr)
        print("  python detect_enemies.py screen 100 100 800 600", file=sys.stderr)
        print("  python detect_enemies.py screen 100 100 800 600 all  # 检测所有目标", file=sys.stderr)
        print("", file=sys.stderr)
        print("说明:", file=sys.stderr)
        print("  - 默认只检测人员（敌人）", file=sys.stderr)
        print("  - 添加 'all' 参数检测所有目标", file=sys.stderr)
        return

    mode = sys.argv[1]

    try:
        if mode == "image":
            # 从图像检测
            if len(sys.argv) < 3:
                print("错误: 请提供图像路径", file=sys.stderr)
                return

            image_path = sys.argv[2]
            detect_persons = len(sys.argv) < 4 or sys.argv[3] != "all"
            detect_from_image(image_path, detect_persons)

        elif mode == "screen":
            # 从屏幕检测
            if len(sys.argv) < 6:
                print("错误: 请提供屏幕区域 x y width height", file=sys.stderr)
                return

            x = int(sys.argv[2])
            y = int(sys.argv[3])
            width = int(sys.argv[4])
            height = int(sys.argv[5])
            detect_persons = len(sys.argv) < 7 or sys.argv[6] != "all"
            detect_from_screen(x, y, width, height, detect_persons)

        else:
            print(f"错误: 未知模式 '{mode}'", file=sys.stderr)
            print("请使用 'image' 或 'screen'", file=sys.stderr)

    except Exception as e:
        print(f"\n错误: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
