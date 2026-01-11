"""
Sim 2D Robot - 2D仿真机器人

配置文件: robot_config.yaml
技能定义: skills/sim_2d_skills.py
"""

from .sim_2d_robot import Sim2DRobot
from .skills import *

__version__ = '1.0.0'
__all__ = ['Sim2DRobot']
