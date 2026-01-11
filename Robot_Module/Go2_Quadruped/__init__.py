#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Go2 Quadruped Robot Module

Unitree Go2 四足机器人模块
"""

class Go2Robot:
    """Unitree Go2 四足机器人"""

    def __init__(self):
        self.name = "Go2_Quadruped"
        self.type = "quadruped"

    def __repr__(self):
        return f"Go2Robot(name={self.name}, type={self.type})"
