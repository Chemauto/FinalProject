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

import os
os.environ['YOLO_OFFLINE'] = 'True'

import sys
import json
import time
import mss
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO


# 截屏区域配置
MONITOR = {"top": 100, "left": 100, "width": 1000, "height": 800}


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
        print(f"[YoloPublisher] ROS 话题发布器已创建: /robot/yolo_enemies")

        # 加载 YOLO 模型
        model_path = model_path or "/home/xcj/work/FinalProject/Yolo_Module/best.pt"
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold
        print(f"[YoloPublisher] YOLO 模型加载: {model_path}")

        # 截屏
        self.sct = mss.mss()
        print(f"[YoloPublisher] 检测区域: {MONITOR}")

        # 发布频率
        self.publish_rate = publish_rate
        self.publish_interval = 1.0 / publish_rate
        print(f"[YoloPublisher] 初始化完成，发布频率: {publish_rate} Hz")

    def capture_and_detect(self) -> list:
        """
        捕获屏幕并进行 YOLO 检测

        Returns:
            检测结果列表 [{"id": str, "x": float, "y": float}, ...]
        """
        try:
            # 截屏
            img = np.array(self.sct.grab(MONITOR))[:, :, :3]

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
                # 获取边界框中心点坐标
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                center_x = float((x1 + x2) / 2)
                center_y = float((y1 + y2) / 2)

                detections.append({
                    "id": f"yolo_{i}",
                    "x": center_x,
                    "y": center_y
                })

            return detections

        except Exception as e:
            print(f"[YoloPublisher] 检测失败: {e}")
            return []

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
                    detections = self.capture_and_detect()

                    # 发布结果
                    self.publish_detections(detections)

                    last_publish_time = current_time

                # ROS 回调
                rclpy.spin_once(self.node, timeout_sec=0.01)

                # 短暂休眠
                time.sleep(0.01)

        except KeyboardInterrupt:
            print("\n[YoloPublisher] 被用户中断")
        finally:
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
