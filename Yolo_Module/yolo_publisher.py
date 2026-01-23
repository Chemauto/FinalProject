#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO 敌人位置 ROS 发布器

功能：
1. 自动获取仿真器窗口位置
2. 截取仿真器屏幕
3. 使用 YOLO 检测敌人位置
4. 发布到 ROS 话题 /robot/yolo_enemies

使用方式：
1. 先启动仿真器
2. 运行此脚本：python3 Yolo_Module/yolo_publisher.py
"""

import os
os.environ['YOLO_OFFLINE'] = 'True'

import json
import time
import cv2
import mss
import numpy as np
import subprocess
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO


def get_window_position():
    """
    使用 xwininfo 获取仿真器窗口位置

    Returns:
        {"top": int, "left": int, "width": int, "height": int} 或 None
    """
    try:
        result = subprocess.run(
            ['xwininfo', '-name', '追击功能测试 - 2D Robot Simulator'],
            capture_output=True, text=True, timeout=5
        )
        if result.returncode == 0:
            geometry = {}
            for line in result.stdout.split('\n'):
                if 'Absolute upper-left X:' in line:
                    match = re.search(r'(\d+)', line)
                    if match:
                        geometry['left'] = int(match.group(1))
                elif 'Absolute upper-left Y:' in line:
                    match = re.search(r'(\d+)', line)
                    if match:
                        geometry['top'] = int(match.group(1))
                elif 'Width:' in line:
                    match = re.search(r'(\d+)', line)
                    if match:
                        geometry['width'] = int(match.group(1))
                elif 'Height:' in line:
                    match = re.search(r'(\d+)', line)
                    if match:
                        geometry['height'] = int(match.group(1))
            if all(key in geometry for key in ['top', 'left', 'width', 'height']):
                print(f"[YoloPublisher] ✓ 自动检测到仿真器窗口位置: {geometry}")
                return geometry
    except Exception as e:
        print(f"[YoloPublisher] ✗ 获取窗口位置失败: {e}")

    print("[YoloPublisher] 使用默认窗口位置")
    return {"top": 0, "left": 0, "width": 800, "height": 600}


class YoloEnemyPublisher:
    """YOLO 检测结果的 ROS 发布器"""

    def __init__(self, model_path: str = None, conf_threshold: float = 0.85,
                 publish_rate: float = 1.0, show_result: bool = True):
        """
        初始化发布器

        Args:
            model_path: YOLO 模型路径
            conf_threshold: 置信度阈值
            publish_rate: 发布频率（Hz）
            show_result: 是否显示检测结果图片
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
        print(f"[YoloPublisher] ROS 话题发布器已创建: /robot/yolo_enemies")

        # 加载 YOLO 模型
        model_path = model_path or "/home/xcj/work/FinalProject/Yolo_Module/best.pt"
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold
        print(f"[YoloPublisher] YOLO 模型加载: {model_path}")

        # 自动获取窗口位置
        self.monitor = get_window_position()

        # 截屏
        self.sct = mss.mss()

        # 发布频率
        self.publish_rate = publish_rate
        self.publish_interval = 1.0 / publish_rate
        print(f"[YoloPublisher] 初始化完成，发布频率: {publish_rate} Hz")

        # 是否显示结果
        self.show_result = show_result

    def capture_and_detect(self) -> tuple:
        """
        捕获屏幕并进行 YOLO 检测

        Returns:
            (检测结果列表, 带检测框的图片)
        """
        try:
            # 截屏
            img = np.array(self.sct.grab(self.monitor))[:, :, :3]

            # YOLO 检测
            results = self.model.predict(
                source=img,
                conf=self.conf_threshold,
                verbose=False,
                device="cpu"
            )[0]

            # 转换检测结果为标准格式
            detections = []
            for i, box in enumerate(results.boxes):
                # 获取边界框中心点坐标（相对于截屏区域）
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                center_x = float((x1 + x2) / 2)
                center_y = float((y1 + y2) / 2)

                # 转换为屏幕绝对坐标（加上窗口偏移）
                screen_x = center_x + self.monitor["left"]
                screen_y = center_y + self.monitor["top"]

                detections.append({
                    "id": f"yolo_{i}",
                    "x": round(screen_x, 2),
                    "y": round(screen_y, 2)
                })

            # 生成带检测框的图片
            annotated_img = results.plot()

            return detections, annotated_img

        except Exception as e:
            print(f"[YoloPublisher] 检测失败: {e}")
            return [], None

    def publish_detections(self, detections: list):
        """
        发布检测结果到 ROS 话题

        Args:
            detections: 检测结果列表
        """
        try:
            msg = String()
            msg.data = json.dumps(detections, ensure_ascii=False)
            self.publisher.publish(msg)

            if len(detections) > 0:
                print(f"[YoloPublisher] 已发布 {len(detections)} 个敌人位置: {detections}")

        except Exception as e:
            print(f"[YoloPublisher] 发布失败: {e}")

    def run(self, duration: float = None):
        """
        运行发布器

        Args:
            duration: 运行时长（秒），None 表示无限运行
        """
        print("[YoloPublisher] 开始运行...")
        print("[YoloPublisher] 按 Ctrl+C 停止\n")

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
                    detections, annotated_img = self.capture_and_detect()

                    # 发布结果
                    self.publish_detections(detections)

                    # 显示检测结果
                    if self.show_result and annotated_img is not None:
                        cv2.imshow("YOLO Detection", annotated_img)
                        cv2.waitKey(1)

                    last_publish_time = current_time

                # ROS 回调
                rclpy.spin_once(self.node, timeout_sec=0.01)

                # 短暂休眠
                time.sleep(0.01)

        except KeyboardInterrupt:
            print("\n[YoloPublisher] 被用户中断")
        finally:
            cv2.destroyAllWindows()
            self.shutdown()

    def shutdown(self):
        """关闭发布器"""
        print("[YoloPublisher] 关闭中...")
        self.node.destroy_node()
        rclpy.shutdown()
        print("[YoloPublisher] 已关闭")


def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description="YOLO 敌人位置 ROS 发布器")
    parser.add_argument("--model", type=str, help="YOLO 模型路径")
    parser.add_argument("--conf", type=float, default=0.5, help="置信度阈值")
    parser.add_argument("--rate", type=float, default=1.0, help="发布频率 (Hz)")
    parser.add_argument("--duration", type=float, help="运行时长（秒）")
    parser.add_argument("--no-show", action="store_true", help="不显示检测结果图片")

    args = parser.parse_args()

    # 创建发布器
    publisher = YoloEnemyPublisher(
        model_path=args.model,
        conf_threshold=args.conf,
        publish_rate=args.rate,
        show_result=not args.no_show
    )

    # 运行
    publisher.run(duration=args.duration)


if __name__ == "__main__":
    main()
