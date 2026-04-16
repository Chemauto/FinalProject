from __future__ import annotations

from .vlm_observe import register_tool


def register_tools(mcp):
    return register_tool(mcp)
