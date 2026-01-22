#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO 目标检测器

使用 ultralytics YOLOv8 进行目标检测。
"""

import sys
import numpy as np


class YoloDetector:
    """YOLO 检测器"""

    def __init__(self, model_name="yolov8n.pt"):
        """
        初始化 YOLO 检测器

        Args:
            model_name: YOLO 模型名称 (yolov8n.pt, yolov8s.pt, yolov8m.pt, etc.)
        """
        self.model = None
        self.model_name = model_name
        self._lazy_init()

    def _lazy_init(self):
        """延迟初始化 YOLO 模型"""
        if self.model is None:
            try:
                from ultralytics import YOLO
                self.model = YOLO(self.model_name)
                print(f"[YOLO] 模型加载成功: {self.model_name}", file=sys.stderr)
            except ImportError:
                print("[YOLO] 错误: 未安装 ultralytics", file=sys.stderr)
                print("[YOLO] 请运行: pip install ultralytics", file=sys.stderr)
                raise
            except Exception as e:
                print(f"[YOLO] 模型加载失败: {e}", file=sys.stderr)
                raise

    def detect(self, image_array, conf_threshold=0.25):
        """
        检测图像中的目标

        Args:
            image_array: numpy 数组格式的图像 (RGB)
            conf_threshold: 置信度阈值 (默认 0.25)

        Returns:
            检测到的目标列表
            [{"center": (x, y), "confidence": float, "class": str, "box": (x1, y1, x2, y2)}]
        """
        self._lazy_init()

        # YOLO 推理
        results = self.model(image_array, verbose=False, conf=conf_threshold)

        detections = []
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    # 获取边界框坐标
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                    # 计算中心坐标
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2

                    # 获取置信度和类别
                    confidence = float(box.conf[0].cpu().numpy())
                    class_id = int(box.cls[0].cpu().numpy())
                    class_name = self.model.names[class_id]

                    detections.append({
                        "center": (center_x, center_y),
                        "confidence": confidence,
                        "class": class_name,
                        "box": (x1, y1, x2, y2)
                    })

        return detections

    def detect_person(self, image_array, conf_threshold=0.25):
        """
        检测图像中的人员

        Args:
            image_array: numpy 数组格式的图像 (RGB)
            conf_threshold: 置信度阈值

        Returns:
            检测到的人员列表
        """
        all_detections = self.detect(image_array, conf_threshold)

        # 过滤出人员类别 (class_id == 0)
        person_detections = [
            det for det in all_detections if det["class"] == "person"
        ]

        return person_detections

    def get_model_info(self):
        """获取模型信息"""
        if self.model is None:
            return None

        return {
            "model_name": self.model_name,
            "classes": self.model.names,
            "num_classes": len(self.model.names)
        }


def main():
    """测试函数"""
    import cv2

    print("[测试] YOLO 检测器", file=sys.stderr)

    # 初始化检测器
    detector = YoloDetector(model_name="yolov8n.pt")

    # 显示模型信息
    info = detector.get_model_info()
    print(f"[模型] {info['model_name']}", file=sys.stderr)
    print(f"[类别] 共 {info['num_classes']} 类", file=sys.stderr)

    # 测试图像（如果有）
    test_image = "/tmp/test_image.jpg"
    try:
        # 创建一个简单的测试图像
        test_array = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)
        cv2.imwrite(test_image, test_array)

        # 读取并检测
        image = cv2.imread(test_image)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        detections = detector.detect(image_rgb)
        print(f"[检测] 找到 {len(detections)} 个目标", file=sys.stderr)

    except Exception as e:
        print(f"[错误] 测试失败: {e}", file=sys.stderr)


if __name__ == "__main__":
    main()
