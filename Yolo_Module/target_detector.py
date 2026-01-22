#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
目标检测器模块

提供多种目标检测方式的统一接口
"""

import sys
import asyncio
from abc import ABC, abstractmethod
from typing import List, Dict, Optional, Callable
from dataclasses import dataclass
from enum import Enum


class DetectionMethod(Enum):
    """检测方法枚举"""
    MOUSE = "mouse"           # 鼠标点击
    KEYBOARD = "keyboard"     # 键盘输入
    YOLO = "yolo"            # YOLO 视觉识别


@dataclass
class DetectedTarget:
    """检测到的目标"""
    id: str
    x: float
    y: float
    confidence: float = 1.0
    class_name: str = "enemy"
    method: str = "unknown"

    def to_dict(self) -> Dict:
        """转换为字典"""
        return {
            "id": self.id,
            "x": self.x,
            "y": self.y,
            "confidence": self.confidence,
            "class": self.class_name,
            "method": self.method
        }


class TargetDetector(ABC):
    """目标检测器基类"""

    def __init__(self, method: DetectionMethod):
        self.method = method
        self.last_detection: Optional[List[DetectedTarget]] = None
        self.detection_count = 0

    @abstractmethod
    async def detect(self) -> List[DetectedTarget]:
        """
        检测目标

        Returns:
            检测到的目标列表
        """
        pass

    def get_last_detection(self) -> Optional[List[DetectedTarget]]:
        """获取最后一次检测结果"""
        return self.last_detection

    def format_results(self) -> List[Dict]:
        """格式化检测结果为字典列表"""
        if not self.last_detection:
            return []
        return [target.to_dict() for target in self.last_detection]


class MouseInputDetector(TargetDetector):
    """
    鼠标输入检测器

    通过鼠标点击在仿真器上标记敌人位置

    使用方法：
    1. 在仿真器中按 'M' 进入标记模式
    2. 点击地图位置
    3. 自动生成敌人
    """

    def __init__(self, on_click_callback: Callable[[float, float], None] = None):
        super().__init__(DetectionMethod.MOUSE)
        self.on_click_callback = on_click_callback
        self.click_queue = asyncio.Queue()
        self.next_id = 1

    async def detect(self) -> List[DetectedTarget]:
        """
        等待鼠标点击输入

        Returns:
            检测到的目标（从点击队列获取）
        """
        if self.click_queue.empty():
            return []

        x, y = await self.click_queue.get()

        target = DetectedTarget(
            id=str(self.next_id),
            x=x,
            y=y,
            confidence=1.0,
            class_name="enemy",
            method="mouse"
        )

        self.next_id += 1
        self.detection_count += 1
        self.last_detection = [target]

        print(f"[MouseInputDetector] 检测到目标 at ({x}, {y})", file=sys.stderr)

        # 回调通知
        if self.on_click_callback:
            self.on_click_callback(x, y)

        return [target]

    def add_click(self, x: float, y: float):
        """
        添加点击位置（由仿真器调用）

        Args:
            x, y: 点击坐标（仿真坐标系）
        """
        self.click_queue.put_nowait((x, y))
        print(f"[MouseInputDetector] 添加点击: ({x}, {y})", file=sys.stderr)


class KeyboardInputDetector(TargetDetector):
    """
    键盘输入检测器

    通过键盘输入坐标来设置敌人位置

    使用方法：
    1. 程序提示输入坐标
    2. 输入格式：x,y 或 x y
    3. 自动生成敌人
    """

    def __init__(self):
        super().__init__(DetectionMethod.KEYBOARD)
        self.next_id = 1

    async def detect(self) -> List[DetectedTarget]:
        """
        等待键盘输入

        Returns:
            检测到的目标（从键盘输入获取）
        """
        print("\n[KeyboardInputDetector] 请输入敌人坐标", file=sys.stderr)
        print("格式: x,y 或 x y (例如: 700,300 或 700 300)", file=sys.stderr)
        print("输入 'q' 取消", file=sys.stderr)

        # 在同步环境中获取输入
        loop = asyncio.get_event_loop()
        user_input = await loop.run_in_executor(None, input, "> ")

        # 取消
        if user_input.lower() == 'q':
            print("[KeyboardInputDetector] 已取消", file=sys.stderr)
            return []

        # 解析输入
        try:
            # 移除空格
            user_input = user_input.replace(' ', '')

            # 分割
            if ',' in user_input:
                parts = user_input.split(',')
            else:
                parts = user_input.split(',')

            if len(parts) != 2:
                raise ValueError("需要两个坐标值")

            x = float(parts[0])
            y = float(parts[1])

            # 验证范围
            if not (0 <= x <= 800 and 0 <= y <= 600):
                print(f"[警告] 坐标 ({x}, {y}) 超出范围 (800x600)", file=sys.stderr)

            target = DetectedTarget(
                id=str(self.next_id),
                x=x,
                y=y,
                confidence=1.0,
                class_name="enemy",
                method="keyboard"
            )

            self.next_id += 1
            self.detection_count += 1
            self.last_detection = [target]

            print(f"[KeyboardInputDetector] ✓ 目标已设置: ({x}, {y})", file=sys.stderr)

            return [target]

        except ValueError as e:
            print(f"[KeyboardInputDetector] ✗ 输入错误: {e}", file=sys.stderr)
            return []

        except Exception as e:
            print(f"[KeyboardInputDetector] ✗ 错误: {e}", file=sys.stderr)
            return []


class YOLODetector(TargetDetector):
    """
    YOLO 目标检测器（预留接口）

    使用 YOLO 模型从仿真器截图中检测敌人位置

    TODO:
    - 实现 YOLO 模型加载
    - 实现截图功能
    - 实现检测结果解析
    - 实现坐标映射
    """

    def __init__(self, model_path: str = "yolov8n.pt"):
        super().__init__(DetectionMethod.YOLO)
        self.model_path = model_path
        self.model = None
        self.next_id = 1
        self._initialized = False

    def _initialize(self):
        """初始化 YOLO 模型"""
        if self._initialized:
            return

        try:
            from ultralytics import YOLO
            print(f"[YOLODetector] 加载模型: {self.model_path}", file=sys.stderr)
            self.model = YOLO(self.model_path)
            self._initialized = True
            print("[YOLODetector] ✓ 模型加载成功", file=sys.stderr)
        except ImportError:
            print("[YOLODetector] ✗ 未安装 ultralytics", file=sys.stderr)
            print("  请运行: pip install ultralytics", file=sys.stderr)
            raise
        except Exception as e:
            print(f"[YOLODetector] ✗ 模型加载失败: {e}", file=sys.stderr)
            raise

    async def detect(self) -> List[DetectedTarget]:
        """
        使用 YOLO 检测目标

        Returns:
            检测到的目标列表
        """
        # 初始化模型
        if not self._initialized:
            try:
                self._initialize()
            except Exception as e:
                print(f"[YOLODetector] 无法初始化模型: {e}", file=sys.stderr)
                return []

        # 截取仿真器画面
        from .screen_capture import capture_simulator
        screenshot = capture_simulator()

        if screenshot is None:
            print("[YOLODetector] 截图失败", file=sys.stderr)
            return []

        # YOLO 检测
        try:
            results = self.model(screenshot, verbose=False)

            targets = []
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for i, box in enumerate(boxes):
                        # 获取边界框中心点
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        center_x = (x1 + x2) / 2
                        center_y = (y1 + y2) / 2

                        # 获取置信度
                        confidence = float(box.conf[0].cpu().numpy())

                        # 获取类别
                        class_id = int(box.cls[0].cpu().numpy())
                        class_name = self.model.names[class_id] if self.model.names else f"class_{class_id}"

                        # 过滤低置信度
                        if confidence < 0.5:
                            continue

                        target = DetectedTarget(
                            id=str(self.next_id),
                            x=center_x,
                            y=center_y,
                            confidence=confidence,
                            class_name=class_name,
                            method="yolo"
                        )

                        targets.append(target)
                        self.next_id += 1

            self.detection_count += 1
            self.last_detection = targets

            print(f"[YOLODetector] 检测到 {len(targets)} 个目标", file=sys.stderr)

            return targets

        except Exception as e:
            print(f"[YOLODetector] ✗ 检测失败: {e}", file=sys.stderr)
            return []


# 工厂函数

def create_detector(method: DetectionMethod, **kwargs) -> TargetDetector:
    """
    创建目标检测器

    Args:
        method: 检测方法
        **kwargs: 检测器特定参数

    Returns:
        检测器实例

    Examples:
        # 鼠标输入
        detector = create_detector(DetectionMethod.MOUSE)

        # 键盘输入
        detector = create_detector(DetectionMethod.KEYBOARD)

        # YOLO 检测
        detector = create_detector(DetectionMethod.YOLO, model_path="yolov8n.pt")
    """
    if method == DetectionMethod.MOUSE:
        return MouseInputDetector(**kwargs)
    elif method == DetectionMethod.KEYBOARD:
        return KeyboardInputDetector(**kwargs)
    elif method == DetectionMethod.YOLO:
        return YOLODetector(**kwargs)
    else:
        raise ValueError(f"不支持的检测方法: {method}")


# 便捷函数

async def detect_with_mouse(on_click_callback=None) -> List[Dict]:
    """使用鼠标输入检测目标"""
    detector = create_detector(DetectionMethod.MOUSE, on_click_callback=on_click_callback)
    targets = await detector.detect()
    return detector.format_results()


async def detect_with_keyboard() -> List[Dict]:
    """使用键盘输入检测目标"""
    detector = create_detector(DetectionMethod.KEYBOARD)
    targets = await detector.detect()
    return detector.format_results()


async def detect_with_yolo(model_path="yolov8n.pt") -> List[Dict]:
    """使用 YOLO 检测目标"""
    detector = create_detector(DetectionMethod.YOLO, model_path=model_path)
    targets = await detector.detect()
    return detector.format_results()
