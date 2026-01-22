"""
YOLO 目标检测模块

提供多种目标检测方式：
- 鼠标点击输入（过渡方案）
- 键盘坐标输入（过渡方案）
- YOLO 视觉识别（最终目标）
"""

from .target_detector import (
    TargetDetector,
    MouseInputDetector,
    KeyboardInputDetector,
    YOLODetector
)

__all__ = [
    'TargetDetector',
    'MouseInputDetector',
    'KeyboardInputDetector',
    'YOLODetector',
]

__version__ = '1.0.0'
