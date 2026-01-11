#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MCP Module - Model Context Protocol 中间连接件

功能:
- 接收LLM和VLM的输入
- 从Robot_Module寻找底层技能
- 完成skill注册
- 连接上层(LLM/VLM)和下层(Middle通信层)
"""

import importlib
import inspect
import yaml
from pathlib import Path
from typing import Dict, List, Any, Callable


class SkillRegistry:
    """技能注册表 - 管理所有可用技能"""

    def __init__(self):
        """初始化技能注册表"""
        self.skills: Dict[str, Callable] = {}
        self.skill_info: Dict[str, Dict[str, Any]] = {}

    def register_from_robot_module(self, robot_name: str):
        """
        从Robot_Module加载技能

        Args:
            robot_name: 机器人名称 (如 'Go2_Quadruped', 'Sim_2D')
        """
        try:
            # 动态导入机器人模块
            robot_skills_module = importlib.import_module(
                f'Robot_Module.{robot_name}.skills'
            )

            # 查找所有skill_*函数
            for name, obj in inspect.getmembers(robot_skills_module):
                if inspect.isfunction(obj) and name.startswith('skill_'):
                    skill_name = name[6:]  # 移除'skill_'前缀
                    self.skills[skill_name] = obj

                    # 获取技能信息
                    doc = inspect.getdoc(obj) or f"Skill: {skill_name}"
                    params = inspect.signature(obj).parameters

                    self.skill_info[skill_name] = {
                        'function': obj,
                        'description': doc,
                        'parameters': list(params.keys()),
                        'robot': robot_name
                    }

                    print(f"✅ [注册] {robot_name}/{skill_name}")

        except ImportError as e:
            print(f"⚠️  [警告] 无法加载 {robot_name} 的技能: {e}")

    def get_skill(self, skill_name: str) -> Callable:
        """获取技能函数"""
        if skill_name not in self.skills:
            raise ValueError(f"技能 '{skill_name}' 未找到")
        return self.skills[skill_name]

    def list_skills(self) -> List[str]:
        """列出所有已注册技能"""
        return list(self.skills.keys())

    def get_skill_info(self, skill_name: str) -> Dict[str, Any]:
        """获取技能信息"""
        return self.skill_info.get(skill_name, {})

    def get_all_skills_info(self) -> Dict[str, Dict[str, Any]]:
        """获取所有技能信息"""
        return self.skill_info


class MCPBridge:
    """
    MCP桥接层 - 连接LLM/VLM和底层技能

    这是MCP模块的核心,作为中间件:
    - 接收来自LLM_Module的规划结果
    - 接收来自VLM_Module的感知结果
    - 从Robot_Module加载和调用技能
    - 通过Middle_Module执行实际控制
    """

    def __init__(self):
        """初始化MCP桥接层"""
        self.skill_registry = SkillRegistry()
        self.robot_configs = {}
        self.loaded_robots = []

    def load_robot(self, robot_name: str):
        """
        加载机器人配置和技能

        Args:
            robot_name: 机器人名称
        """
        if robot_name in self.loaded_robots:
            print(f"⏭️  [跳过] {robot_name} 已加载")
            return

        # 加载机器人配置
        config_path = Path(f'Robot_Module/{robot_name}/robot_config.yaml')
        if config_path.exists():
            with open(config_path, 'r') as f:
                self.robot_configs[robot_name] = yaml.safe_load(f)

        # 注册技能
        self.skill_registry.register_from_robot_module(robot_name)
        self.loaded_robots.append(robot_name)

        print(f"✅ [加载] {robot_name} 完成")

    def load_all_robots(self):
        """加载所有可用机器人"""
        robot_modules = Path('Robot_Module').glob('*/')
        for robot_dir in robot_modules:
            if robot_dir.is_dir():
                robot_name = robot_dir.name
                # 检查是否有robot_config.yaml
                if (robot_dir / 'robot_config.yaml').exists():
                    self.load_robot(robot_name)

    def get_available_skills(self) -> List[str]:
        """获取所有可用技能列表"""
        return self.skill_registry.list_skills()

    def get_mcp_tools_definition(self) -> List[Dict[str, Any]]:
        """
        获取MCP工具定义
        用于提供给LLM的工具列表
        """
        tools = []
        for skill_name, skill_info in self.skill_registry.get_all_skills_info().items():
            tool = {
                'type': 'function',
                'function': {
                    'name': skill_name,
                    'description': skill_info['description'],
                    'parameters': {
                        'type': 'object',
                        'properties': {}
                    }
                }
            }

            # 添加参数信息
            for param_name in skill_info['parameters']:
                if param_name != 'self':
                    tool['function']['parameters']['properties'][param_name] = {
                        'type': 'string',
                        'description': f'{param_name} parameter'
                    }

            tools.append(tool)

        return tools

    def execute_skill(self, skill_name: str, **kwargs) -> Dict[str, Any]:
        """
        执行技能

        Args:
            skill_name: 技能名称
            **kwargs: 技能参数

        Returns:
            执行结果
        """
        try:
            skill_func = self.skill_registry.get_skill(skill_name)
            result = skill_func(**kwargs)

            return {
                'success': True,
                'skill': skill_name,
                'result': result
            }
        except Exception as e:
            return {
                'success': False,
                'skill': skill_name,
                'error': str(e)
            }

    def process_llm_input(self, llm_output: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        处理LLM输出并执行技能

        Args:
            llm_output: LLM输出,包含规划的任务列表

        Returns:
            执行结果列表
        """
        results = []

        # 如果是规划结果,包含多个子任务
        if 'tasks' in llm_output:
            tasks = llm_output['tasks']
            for task in tasks:
                task_type = task.get('type', 'unknown')
                task_desc = task.get('task', '')

                # 根据任务类型调用相应技能
                result = self._execute_task_by_type(task_type, task_desc)
                results.append(result)
        else:
            # 单个任务
            result = self._execute_task_by_type(
                llm_output.get('type', 'unknown'),
                llm_output.get('task', '')
            )
            results.append(result)

        return results

    def _execute_task_by_type(self, task_type: str, task_desc: str) -> Dict[str, Any]:
        """
        根据任务类型执行

        Args:
            task_type: 任务类型
            task_desc: 任务描述

        Returns:
            执行结果
        """
        # 这里可以根据task_type映射到具体技能
        # 简化实现: 直接执行task_desc作为技能名
        return self.execute_skill(task_desc)

    def process_vlm_input(self, vlm_output: Dict[str, Any]) -> Dict[str, Any]:
        """
        处理VLM输出并生成控制指令

        Args:
            vlm_output: VLM感知结果

        Returns:
            控制指令
        """
        # 根据VLM感知结果生成控制指令
        # 例如: 检测到障碍物 -> 停止/绕行
        scene_type = vlm_output.get('scene_type', '')
        obstacles = vlm_output.get('obstacles', [])

        if obstacles:
            return {
                'action': 'stop',
                'reason': '检测到障碍物',
                'obstacles': obstacles
            }

        return {
            'action': 'continue',
            'scene': scene_type
        }


# 便捷函数
def create_mcp_bridge(robots: List[str] = None) -> MCPBridge:
    """
    创建并初始化MCP桥接层

    Args:
        robots: 要加载的机器人列表,默认加载所有

    Returns:
        初始化好的MCPBridge实例
    """
    bridge = MCPBridge()

    if robots is None:
        # 加载所有可用机器人
        bridge.load_all_robots()
    else:
        # 加载指定机器人
        for robot in robots:
            bridge.load_robot(robot)

    return bridge


if __name__ == '__main__':
    # 测试代码
    print("=== MCP Module 测试 ===\n")

    # 创建桥接层
    bridge = create_mcp_bridge(['Sim_2D', 'Go2_Quadruped'])

    print(f"\n可用技能: {bridge.get_available_skills()}")

    # 测试技能执行
    result = bridge.execute_skill('move_forward', distance=1.0, speed=0.2)
    print(f"\n执行结果: {result}")
