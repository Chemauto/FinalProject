"""
Robot_Module - MCP 工具函数注册中心

基于 FastMCP 的模块化设计，主要组件:
- skill.py: MCP 服务器入口，注册所有模块
- module/base.py: 底盘控制模块（移动、旋转、停止）
- module/example.py: 示例模块（添加新功能的参考模板）
"""

__version__ = "3.0.0"

from .skill import (
    mcp,
    register_all_modules,
    get_skill_function,
    get_tool_definitions
)

__all__ = [
    "mcp",
    "register_all_modules",
    "get_skill_function",
    "get_tool_definitions"
]
