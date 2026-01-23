#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO 敌人位置 ROS 发布器

功能：
1. 截取仿真器屏幕
2. 使用 YOLO 检测敌人位置
3. 发布到 ROS 话题 /robot/yolo_enemies

使用方式：
1. 先启动仿真器
2. 运行此脚本：python3 Yolo_Module/yolo_publisher.py
"""

import sys
import os
import json
import time

# 添加项目路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from yolo_detector import YoloDetector


# 屏幕尺寸（与仿真器一致）
WIDTH, HEIGHT = 800, 600


class YoloEnemyPublisher:
    """YOLO 检测结果的 ROS 发布器"""

    def __init__(self, model_path: str = None, conf_threshold: float = 0.5,
                 publish_rate: float = 1.0):
        """
        初始化发布器

        Args:
            model_path: YOLO 模型路径
            conf_threshold: 置信度阈值
            publish_rate: 发布频率（Hz）
        """
        # 初始化 ROS
        rclpy.init()
        self.node = Node('yolo_enemy_publisher')

        # 创建发布者
        self.publisher = self.node.create_publisher(
            String,
            '/robot/yolo_enemies',
            10
        )
        print("[YoloPublisher] ROS 话题发布器已创建: /robot/yolo_enemies", file=sys.stderr)

        # 创建 YOLO 检测器
        self.detector = YoloDetector(model_path, conf_threshold)

        # 发布频率
        self.publish_rate = publish_rate
        self.publish_interval = 1.0 / publish_rate

        # 屏幕捕获
        self.screen_size = (WIDTH, HEIGHT)

        print(f"[YoloPublisher] 初始化完成，发布频率: {publish_rate} Hz", file=sys.stderr)

    def capture_and_detect(self) -> list:
        """
        捕获屏幕并进行 YOLO 检测

        Returns:
            检测结果列表 [{"id": str, "x": float, "y": float, "conf": float}, ...]
        """
        try:
            # 截取屏幕（使用 X11 方式）
            # 注意：这需要 X11 环境，可能需要设置 DISPLAY 环境变量
            import subprocess

            # 使用 import 懒加载，避免在没有显示器的环境失败
            try:
                from Xlib import display
                from Xlib.ext import xinput
                from PIL import Image, ImageDraw
                import numpy as np

                # 获取屏幕
                d = display.Display()
                screen = d.screen()
                root = screen.root

                # 获取屏幕尺寸
                width = root.get_geometry().width
                height = root.get_geometry().height

                # 截图
                raw = root.get_image(0, 0, width, height, ZPixmap, 0xffffffff)
                image = Image.frombytes("RGB", (width, height), raw.data, "raw", "BGRX")

                # 转换为 pygame surface
                mode = image.mode
                size = image.size
                data = image.tobytes()
                pygame_surface = pygame.image.fromstring(data, size, mode)

                # YOLO 检测
                detections = self.detector.detect_from_screenshot(pygame_surface)

                return detections

            except ImportError:
                print("[YoloPublisher] Xlib 不可用，使用模拟数据", file=sys.stderr)
                return []

        except Exception as e:
            print(f"[YoloPublisher] 屏幕捕获失败: {e}", file=sys.stderr)
            return []

    def publish_detections(self, detections: list):
        """
        发布检测结果到 ROS 话题

        Args:
            detections: 检测结果列表
        """
        try:
            # 转换为标准格式（兼容原有敌人位置格式）
            enemy_positions = [
                {
                    "id": det["id"],
                    "x": det["x"],
                    "y": det["y"]
                }
                for det in detections
            ]

            msg = String()
            msg.data = json.dumps(enemy_positions, ensure_ascii=False)
            self.publisher.publish(msg)

            if len(enemy_positions) > 0:
                print(f"[YoloPublisher] 发布 {len(enemy_positions)} 个敌人位置: {enemy_positions}", file=sys.stderr)

        except Exception as e:
            print(f"[YoloPublisher] 发布失败: {e}", file=sys.stderr)

    def run(self, duration: float = None):
        """
        运行发布器

        Args:
            duration: 运行时长（秒），None 表示无限运行
        """
        print("[YoloPublisher] 开始运行...", file=sys.stderr)
        print("[YoloPublisher] 按 Ctrl+C 停止", file=sys.stderr)

        start_time = time.time()
        last_publish_time = 0

        try:
            while True:
                current_time = time.time()

                # 检查运行时长
                if duration and (current_time - start_time) >= duration:
                    break

                # 按频率发布
                if (current_time - last_publish_time) >= self.publish_interval:
                    # 捕获并检测
                    detections = self.capture_and_detect()

                    # 发布结果
                    self.publish_detections(detections)

                    last_publish_time = current_time

                # ROS 回调
                rclpy.spin_once(self.node, timeout_sec=0.01)

                # 短暂休眠
                time.sleep(0.01)

        except KeyboardInterrupt:
            print("\n[YoloPublisher] 被用户中断", file=sys.stderr)
        finally:
            self.shutdown()

    def shutdown(self):
        """关闭发布器"""
        print("[YoloPublisher] 关闭中...", file=sys.stderr)
        self.node.destroy_node()
        rclpy.shutdown()
        print("[YoloPublisher] 已关闭", file=sys.stderr)


def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description="YOLO 敌人位置 ROS 发布器")
    parser.add_argument("--model", type=str, help="YOLO 模型路径")
    parser.add_argument("--conf", type=float, default=0.5, help="置信度阈值")
    parser.add_argument("--rate", type=float, default=1.0, help="发布频率 (Hz)")
    parser.add_argument("--duration", type=float, help="运行时长（秒）")

    args = parser.parse_args()

    # 创建发布器
    publisher = YoloEnemyPublisher(
        model_path=args.model,
        conf_threshold=args.conf,
        publish_rate=args.rate
    )

    # 运行
    publisher.run(duration=args.duration)


if __name__ == "__main__":
    main()
