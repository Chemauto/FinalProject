"""
Go2 Quadruped Robot - Unitree Go2 四足机器人

配置文件: robot_config.yaml
技能定义: skills/go2_skills.py
"""

from .go2_robot import Go2Robot
from .skills import *

__version__ = '1.0.0'
__all__ = ['Go2Robot']
