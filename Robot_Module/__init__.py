"""
Robot_Module - MCP 工具函数注册中心

基于 FastMCP 的模块化设计。
"""

__version__ = "4.0.0"

from .tools import (
    mcp,
    register_all,
    get_skill_function,
    get_tool_definitions,
)

__all__ = [
    "mcp",
    "register_all",
    "get_skill_function",
    "get_tool_definitions",
]
