from __future__ import annotations

from .vlm_observe import register_tools as _register_tools


def register_tools(mcp):
    return _register_tools(mcp)
