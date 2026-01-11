#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sim 2D Robot - 2D仿真机器人 (TurtleBot等)
"""

import yaml
from pathlib import Path
from typing import Dict, Any, List


class Sim2DRobot:
    """2D仿真机器人类"""

    def __init__(self):
        """初始化2D仿真机器人"""
        self.config = self._load_config()
        self.name = self.config['robot']['name']
        self.type = self.config['robot']['type']

    def _load_config(self) -> Dict[str, Any]:
        """加载机器人配置"""
        config_path = Path(__file__).parent / "robot_config.yaml"

        if config_path.exists():
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        else:
            # 默认配置
            return {
                'robot': {
                    'name': 'Sim2D',
                    'type': '2d_simulation'
                },
                'skills': [
                    'move_forward',
                    'move_backward',
                    'rotate',
                    'stop'
                ]
            }

    def get_skills(self) -> List[str]:
        """获取可用技能列表"""
        return self.config.get('skills', [])

    def get_communication_methods(self) -> List[str]:
        """获取支持的通信方式"""
        return self.config.get('communication', ['ROS2'])

    def execute_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """
        执行机器人命令

        Args:
            command: 命令字典 {'action': ..., 'parameters': ...}

        Returns:
            执行结果
        """
        action = command.get('action')
        parameters = command.get('parameters', {})

        return {
            'robot': self.name,
            'action': action,
            'parameters': parameters,
            'status': 'executing'
        }
