"""
Go2 Quadruped Robot Skills

技能定义用于Unitree Go2四足机器人
"""

from .go2_skills import (
    skill_move_forward,
    skill_move_backward,
    skill_rotate_left,
    skill_rotate_right,
    skill_stop,
    skill_move_omni
)

__all__ = [
    'skill_move_forward',
    'skill_move_backward',
    'skill_rotate_left',
    'skill_rotate_right',
    'skill_stop',
    'skill_move_omni'
]
