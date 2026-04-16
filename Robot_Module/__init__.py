"""
Robot_Module - MCP 工具函数注册中心

基于 FastMCP 的模块化设计，主要组件:
- agent_tools.py: 顶层注册中心，组装 Action / Vision / robot_act
- module/example.py: 示例模块（添加新功能的参考模板）
"""

__version__ = "3.0.0"

from .agent_tools import (
    mcp,
    register_all_modules,
    get_skill_function,
    get_tool_definitions,
)

__all__ = [
    "mcp",
    "register_all_modules",
    "get_skill_function",
    "get_tool_definitions"
]
