# -*- coding: utf-8 -*-
"""
Adapters for Robot Control
支持不同的通信框架：Dora, ROS1, ROS2等
"""
from .base_adapter import BaseAdapter
from .dora_adapter import DoraAdapter
from .ros1_adapter import ROS1Adapter
from .ros2_adapter import ROS2Adapter

__all__ = ["BaseAdapter", "DoraAdapter", "ROS1Adapter", "ROS2Adapter"]
