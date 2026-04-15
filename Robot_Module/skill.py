"""
FinalProject 机器人技能统一入口。
"""

import asyncio
import signal
import sys
from pathlib import Path

from mcp.server.fastmcp import FastMCP

_current_dir = Path(__file__).parent
_project_root = _current_dir.parent
sys.path.insert(0, str(_current_dir))
sys.path.insert(0, str(_project_root))

mcp = FastMCP("robot")

_tool_registry = {}
_modules_registered = False

AGENT_TOOL_NAMES = {"vlm_observe", "robot_act"}
VISION_TOOL_NAMES = {"vlm_observe"}
ACTION_TOOL_NAMES = {
    "walk",
    "navigation",
    "nav_climb",
    "climb_align",
    "climb",
    "push_box",
    "way_select",
}

from agent_tools import register_tools as register_agent_tools
from module.Action.navigation import register_tools as register_navigation_tools
from module.Vision.vlm import register_tools as register_vision_tools


def get_skill_function(name: str):
    return _tool_registry.get(name)


def get_tool_definitions(allowed_names: set[str] | None = None):
    async def _get_tools():
        tools_list = await mcp.list_tools()
        tools = []
        for tool in tools_list:
            if allowed_names and tool.name not in allowed_names:
                continue
            tools.append(
                {
                    "type": "function",
                    "function": {
                        "name": tool.name,
                        "description": tool.description if hasattr(tool, "description") else "",
                        "parameters": tool.inputSchema
                        if hasattr(tool, "inputSchema")
                        else {"type": "object", "properties": {}, "required": []},
                    },
                }
            )
        return tools

    return asyncio.run(_get_tools())


def get_agent_tool_definitions():
    return get_tool_definitions(AGENT_TOOL_NAMES)


def get_vision_tool_definitions():
    return get_tool_definitions(VISION_TOOL_NAMES)


def get_action_tool_definitions():
    return get_tool_definitions(ACTION_TOOL_NAMES)


def signal_handler(signum, frame):
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


def register_all_modules():
    global _modules_registered
    if _modules_registered:
        return

    _tool_registry.update(register_navigation_tools(mcp))
    _tool_registry.update(register_vision_tools(mcp))
    _tool_registry.update(register_agent_tools(mcp))
    _modules_registered = True


if __name__ == "__main__":
    register_all_modules()
    mcp.run(transport="stdio")
