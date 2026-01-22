#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO 模块

提供 YOLO 目标检测、屏幕截图和坐标映射功能。
"""

from .yolo_detector import YoloDetector
from .screen_capture import ScreenCapture
from .coordinate_mapper import CoordinateMapper

__all__ = [
    "YoloDetector",
    "ScreenCapture",
    "CoordinateMapper",
]
