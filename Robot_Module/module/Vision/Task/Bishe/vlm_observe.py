from __future__ import annotations

import json
import sys

from VLM_Module.vlm_core import VLMCore


async def execute_vlm_observe(image_path: str = "") -> dict[str, object]:
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


def register_tool(mcp):
    @mcp.tool()
    async def vlm_observe(image_path: str = "") -> str:
        return json.dumps(await execute_vlm_observe(image_path=image_path), ensure_ascii=False)

    print("[Vision/Task/Bishe] 视觉模块已注册 (1 个工具)", file=sys.stderr)
    return {"vlm_observe": vlm_observe}

