#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO 实时检测工具

持续截取屏幕区域并检测敌人。
"""

import sys
import os
import time
import cv2

# 添加项目路径
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, project_root)

from yolo_detector import YoloDetector
from screen_capture import ScreenCapture


class RealtimeDetector:
    """实时检测器"""

    def __init__(self, x, y, width, height, detect_persons=True, conf_threshold=0.25):
        """
        初始化实时检测器

        Args:
            x, y, width, height: 屏幕截取区域
            detect_persons: 是否只检测人员
            conf_threshold: 置信度阈值
        """
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.detect_persons = detect_persons
        self.conf_threshold = conf_threshold

        # 初始化检测器和截图工具
        print("正在初始化 YOLO 检测器...", file=sys.stderr)
        self.detector = YoloDetector(model_name="yolov8n.pt")
        self.capturer = ScreenCapture()

        self.running = False

    def detect_once(self, show_result=False):
        """
        执行一次检测

        Args:
            show_result: 是否显示检测结果窗口

        Returns:
            检测到的目标列表
        """
        # 截图
        image_array = self.capturer.capture_region(
            self.x, self.y, self.width, self.height
        )

        # 检测
        if self.detect_persons:
            detections = self.detector.detect_person(image_array, self.conf_threshold)
        else:
            detections = self.detector.detect(image_array, self.conf_threshold)

        # 显示结果
        if show_result and len(detections) > 0:
            image_bgr = cv2.cvtColor(image_array, cv2.COLOR_RGB2BGR)
            for det in detections:
                x1, y1, x2, y2 = [int(v) for v in det['box']]
                cv2.rectangle(image_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{det['class']} {det['confidence']:.2f}"
                cv2.putText(image_bgr, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow("YOLO Detection", image_bgr)
            cv2.waitKey(1)

        return detections

    def run(self, interval=0.1, verbose=True, show_window=False):
        """
        持续运行检测

        Args:
            interval: 检测间隔（秒）
            verbose: 是否打印检测结果
            show_window: 是否显示检测窗口
        """
        self.running = True

        print(f"\n开始实时检测...", file=sys.stderr)
        print(f"区域: ({self.x}, {self.y}, {self.width}, {self.height})", file=sys.stderr)
        print(f"模式: {'只检测人员' if self.detect_persons else '检测所有目标'}", file=sys.stderr)
        print(f"间隔: {interval}秒", file=sys.stderr)
        print(f"按 Ctrl+C 停止\n", file=sys.stderr)

        try:
            frame_count = 0
            while self.running:
                frame_count += 1

                # 检测
                detections = self.detect_once(show_result=show_window)

                # 打印结果
                if verbose and len(detections) > 0:
                    print(f"[帧 {frame_count}] 检测到 {len(detections)} 个目标:", file=sys.stderr)
                    for i, det in enumerate(detections):
                        center_x, center_y = det['center']
                        print(f"  {i+1}. {det['class']} ({det['confidence']:.2f}) "
                              f"at ({center_x:.1f}, {center_y:.1f})", file=sys.stderr)

                # 等待
                time.sleep(interval)

        except KeyboardInterrupt:
            print("\n\n停止检测", file=sys.stderr)
        finally:
            self.running = False
            if show_window:
                cv2.destroyAllWindows()


def main():
    """主函数"""
    print("=" * 60, file=sys.stderr)
    print("YOLO 实时检测工具", file=sys.stderr)
    print("=" * 60, file=sys.stderr)

    if len(sys.argv) < 5:
        print("\n使用方法:", file=sys.stderr)
        print("  python realtime_detect.py <x> <y> <width> <height> [选项]", file=sys.stderr)
        print("", file=sys.stderr)
        print("参数:", file=sys.stderr)
        print("  x, y, width, height  - 屏幕截取区域", file=sys.stderr)
        print("", file=sys.stderr)
        print("选项:", file=sys.stderr)
        print("  --all              - 检测所有目标（默认只检测人员）", file=sys.stderr)
        print("  --interval <秒>    - 检测间隔，默认 0.1 秒", file=sys.stderr)
        print("  --quiet            - 不打印检测结果", file=sys.stderr)
        print("  --show             - 显示检测窗口", file=sys.stderr)
        print("  --conf <阈值>      - 置信度阈值，默认 0.25", file=sys.stderr)
        print("", file=sys.stderr)
        print("示例:", file=sys.stderr)
        print("  # 实时检测屏幕区域 100,100,800,600", file=sys.stderr)
        print("  python realtime_detect.py 100 100 800 600", file=sys.stderr)
        print("", file=sys.stderr)
        print("  # 检测所有目标，间隔 0.5 秒", file=sys.stderr)
        print("  python realtime_detect.py 100 100 800 600 --all --interval 0.5", file=sys.stderr)
        print("", file=sys.stderr)
        print("  # 显示检测窗口", file=sys.stderr)
        print("  python realtime_detect.py 100 100 800 600 --show", file=sys.stderr)
        print("", file=sys.stderr)
        print("说明:", file=sys.stderr)
        print("  - 使用 xdotool 获取窗口位置:", file=sys.stderr)
        print("    xdotool getwindowfocus getwindowgeometry", file=sys.stderr)
        return

    # 解析参数
    x = int(sys.argv[1])
    y = int(sys.argv[2])
    width = int(sys.argv[3])
    height = int(sys.argv[4])

    # 选项
    detect_persons = "--all" not in sys.argv
    interval = 0.1
    verbose = "--quiet" not in sys.argv
    show_window = "--show" in sys.argv
    conf_threshold = 0.25

    # 解析高级选项
    i = 5
    while i < len(sys.argv):
        if sys.argv[i] == "--interval" and i + 1 < len(sys.argv):
            interval = float(sys.argv[i + 1])
            i += 2
        elif sys.argv[i] == "--conf" and i + 1 < len(sys.argv):
            conf_threshold = float(sys.argv[i + 1])
            i += 2
        else:
            i += 1

    # 创建检测器
    detector = RealtimeDetector(
        x, y, width, height,
        detect_persons=detect_persons,
        conf_threshold=conf_threshold
    )

    # 运行
    detector.run(
        interval=interval,
        verbose=verbose,
        show_window=show_window
    )


if __name__ == "__main__":
    main()
