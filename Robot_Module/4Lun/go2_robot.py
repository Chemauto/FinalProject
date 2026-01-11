#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Go2 Robot - Unitree Go2 四足机器人
"""

import yaml
from pathlib import Path
from typing import Dict, Any, List


class Go2Robot:
    """Unitree Go2四足机器人类"""

    def __init__(self):
        """初始化Go2机器人"""
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
                    'name': 'Go2',
                    'type': 'quadruped'
                },
                'skills': [
                    'move_forward',
                    'move_backward',
                    'rotate_left',
                    'rotate_right',
                    'stop'
                ]
            }

    def get_skills(self) -> List[str]:
        """获取可用技能列表"""
        return self.config.get('skills', [])

    def get_communication_methods(self) -> List[str]:
        """获取支持的通信方式"""
        return self.config.get('communication', ['ROS2'])

    def get_prompt_files(self) -> Dict[str, str]:
        """获取提示词文件路径"""
        return self.config.get('prompts', {})

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

        # 这里会调用实际的技能函数
        # 具体实现在skills/go2_skills.py中

        return {
            'robot': self.name,
            'action': action,
            'parameters': parameters,
            'status': 'executing'
        }
