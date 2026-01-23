#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO 目标检测模块

功能：
1. 加载训练好的 YOLO 模型
2. 对仿真器截图进行检测
3. 返回敌人位置列表

依赖：ultralytics (YOLOv8/YOLOv11)
安装：pip install ultralytics
"""

import sys
import os
import json
from typing import List, Dict, Optional, Tuple

# 添加项目路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

import pygame


class YoloDetector:
    """YOLO 目标检测器"""

    def __init__(self, model_path: str = None, conf_threshold: float = 0.5):
        """
        初始化 YOLO 检测器

        Args:
            model_path: 模型路径（.pt 文件）
            conf_threshold: 置信度阈值
        """
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.model = None

        if model_path and os.path.exists(model_path):
            self.load_model(model_path)
        else:
            print("[YoloDetector] 模型未指定或不存在，使用模拟模式", file=sys.stderr)

    def load_model(self, model_path: str):
        """
        加载 YOLO 模型

        Args:
            model_path: 模型路径
        """
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            print(f"[YoloDetector] 模型已加载: {model_path}", file=sys.stderr)
        except ImportError:
            print("[YoloDetector] 未安装 ultralytics，使用模拟模式", file=sys.stderr)
            print("[YoloDetector] 安装: pip install ultralytics", file=sys.stderr)
        except Exception as e:
            print(f"[YoloDetector] 模型加载失败: {e}", file=sys.stderr)

    def detect_from_screenshot(self, screen: pygame.Surface) -> List[Dict]:
        """
        从 Pygame 屏幕检测敌人

        Args:
            screen: Pygame 屏幕

        Returns:
            检测结果列表 [{"id": str, "x": float, "y": float, "conf": float}, ...]
        """
        if self.model is None:
            return self._mock_detect(screen)

        # 将 Pygame 屏幕转换为图片
        import io
        import tempfile

        # 保存到临时文件
        with tempfile.NamedTemporaryFile(suffix='.png', delete=False) as tmp:
            pygame.image.save(screen, tmp.name)
            tmp_path = tmp.name

        try:
            # 使用 YOLO 检测
            results = self.model(tmp_path, conf=self.conf_threshold, verbose=False)

            detections = []
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # 获取边界框中心点
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    x_center = (x1 + x2) / 2
                    y_center = (y1 + y2) / 2
                    conf = float(box.conf)

                    detections.append({
                        "id": f"yolo_{len(detections)}",
                        "x": x_center,
                        "y": y_center,
                        "conf": conf
                    })

            return detections
        finally:
            # 删除临时文件
            try:
                os.unlink(tmp_path)
            except:
                pass

    def detect_from_file(self, image_path: str) -> List[Dict]:
        """
        从图片文件检测敌人

        Args:
            image_path: 图片路径

        Returns:
            检测结果列表
        """
        if self.model is None:
            return self._mock_detect_from_file(image_path)

        try:
            results = self.model(image_path, conf=self.conf_threshold, verbose=False)

            detections = []
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    x_center = (x1 + x2) / 2
                    y_center = (y1 + y2) / 2
                    conf = float(box.conf)

                    detections.append({
                        "id": f"yolo_{len(detections)}",
                        "x": x_center,
                        "y": y_center,
                        "conf": conf
                    })

            return detections
        except Exception as e:
            print(f"[YoloDetector] 检测失败: {e}", file=sys.stderr)
            return []

    def _mock_detect(self, screen: pygame.Surface) -> List[Dict]:
        """
        模拟检测（当没有模型时使用）

        从屏幕上直接读取敌人的真实位置（用于测试）
        """
        # 这里返回空列表，实际应该从仿真器获取真实位置
        # 暂时返回模拟数据用于测试
        return []

    def _mock_detect_from_file(self, image_path: str) -> List[Dict]:
        """
        模拟检测（从文件）
        """
        # 尝试从同名的 .txt 文件读取真实标注
        label_path = image_path.rsplit('.', 1)[0] + '.txt'
        if os.path.exists(label_path):
            detections = []
            img = pygame.image.load(image_path)
            img_width, img_height = img.get_size()

            with open(label_path, 'r') as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) == 5:
                        class_id = int(parts[0])
                        center_x_norm = float(parts[1])
                        center_y_norm = float(parts[2])

                        # 转换为像素坐标
                        x_center = center_x_norm * img_width
                        y_center = center_y_norm * img_height

                        detections.append({
                            "id": f"label_{len(detections)}",
                            "x": x_center,
                            "y": y_center,
                            "conf": 1.0  # 标注文件没有置信度，设为1.0
                        })

            return detections
        return []


def draw_detections(screen: pygame.Surface, detections: List[Dict],
                   color: Tuple[int, int, int] = (0, 255, 0), width: int = 2):
    """
    在屏幕上绘制检测结果

    Args:
        screen: Pygame 屏幕
        detections: 检测结果列表
        color: 边界框颜色
        width: 边界框线宽
    """
    font = pygame.font.Font(None, 24)

    for det in detections:
        x, y = int(det['x']), int(det['y'])

        # 绘制边界框（假设大小为 30x30，与训练数据一致）
        bbox_size = 30
        rect = pygame.Rect(x - bbox_size // 2, y - bbox_size // 2, bbox_size, bbox_size)
        pygame.draw.rect(screen, color, rect, width)

        # 绘制标签
        conf = det.get('conf', 0)
        label_text = f"{det['id']} ({conf:.2f})"
        text_surface = font.render(label_text, True, color)
        screen.blit(text_surface, (x - bbox_size // 2, y - bbox_size // 2 - 20))


def main():
    """测试 YOLO 检测器"""
    print("[YoloDetector] 测试模式", file=sys.stderr)

    # 创建检测器（无模型，使用模拟模式）
    detector = YoloDetector()

    # 测试从文件检测
    data_dir = os.path.join(os.path.dirname(__file__), "data", "images")
    if os.path.exists(data_dir):
        test_images = [f for f in os.listdir(data_dir) if f.endswith('.png')][:3]

        for img_file in test_images:
            img_path = os.path.join(data_dir, img_file)
            print(f"[YoloDetector] 检测: {img_file}", file=sys.stderr)

            detections = detector.detect_from_file(img_path)
            print(f"[YoloDetector] 检测到 {len(detections)} 个目标", file=sys.stderr)
            for det in detections:
                print(f"  - {det['id']}: ({det['x']:.1f}, {det['y']:.1f}) conf={det['conf']:.2f}")


if __name__ == "__main__":
    main()
