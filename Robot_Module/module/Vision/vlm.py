from __future__ import annotations

import json
import sys

from VLM_Module.vlm_core import VLMCore


async def _vlm_observe_impl(image_path: str = "") -> dict:
    """获取当前环境观测。只观察，不执行动作。"""
    vlm = VLMCore()
    resolved_image = vlm.image_source.get_image(image_path or None)
    visual_context = vlm.describe_structured(str(resolved_image))
    scene_facts = VLMCore.build_scene_facts(visual_context)
    return {
        "status": "success",
        "image_source": str(resolved_image),
        "visual_context": visual_context,
        "scene_facts": scene_facts,
        "uncertainties": visual_context.get("uncertainties", []),
    }


def register_tools(mcp):
    """
    注册视觉感知工具函数到 MCP 服务器。

    Args:
        mcp: FastMCP 服务器实例
    """

    @mcp.tool()
    async def vlm_observe(image_path: str = "") -> str:
        """获取当前环境观测。只观察，不执行动作。"""
        return json.dumps(await _vlm_observe_impl(image_path=image_path), ensure_ascii=False)

    print("[Vision/vlm.py:register_tools] 视觉模块已注册 (1 个工具)", file=sys.stderr)

    return {
        "vlm_observe": vlm_observe,
    }
