"""
Robot Module - 机器人模块 (核心!)

功能:
- 定义每种机器人的配置和技能
- 机器人特定的提示词
- 机器人特定的控制器

每个机器人子模块包含:
- robot_config.yaml (机器人配置)
- skills/ (技能定义)
- controllers/ (控制器)
- prompts/ (机器人专用提示词)

现有机器人:
- Go2_Quadruped/ (Unitree Go2四足机器人)
- Sim_2D/ (2D仿真机器人)

添加新机器人:
1. 创建 Robot_Module/NewRobot/ 文件夹
2. 添加 robot_config.yaml
3. 添加 skills/newrobot_skills.py
4. 添加 prompts/ (可选)
"""

from .Go2_Quadruped import Go2Robot
from .Sim_2D import Sim2DRobot

__version__ = '1.0.0'
__all__ = ['Go2Robot', 'Sim2DRobot']
