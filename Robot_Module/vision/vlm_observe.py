"""vlm 视觉技能 - 调用 Data_Module 获取结构化视觉描述，合并 Hardware_Module 实时状态。"""

import json
import sys

from Data_Module.vlm import VLMCore
from Hardware_Module import get_state


async def execute_vlm_observe(image_path: str = "") -> dict:
    """调用 VLM 获取视觉描述，合并实时状态返回。"""
    vlm = VLMCore()
    resolved_image = vlm.image_source.get_image(image_path or None)

    # 视觉语义
    visual_context = vlm.describe_structured(str(resolved_image))
    scene_facts = VLMCore.build_scene_facts(visual_context)

    # 通用实时状态
    try:
        live_state = get_state()
    except Exception:
        live_state = {"connected": False, "task_type": None, "observation": {}, "runtime": {}}

    # 合并视觉 + object_facts
    object_facts = None
    observation = live_state.get("observation") or {}
    environment = observation.get("environment") or {}
    runtime = live_state.get("runtime") or {}
    objects = environment.get("obstacles") or runtime.get("scene_objects") or []
    if objects:
        object_facts = {"objects": objects}
    merged = VLMCore.merge_scene_facts(scene_facts, object_facts)

    return {
        "status": "success",
        "image_source": str(resolved_image),
        "visual_context": visual_context,
        "scene_facts": merged,
        "env_state": {
            "connected": bool(live_state.get("connected")),
            "scene_id": environment.get("scene_id"),
            "agent_position": observation.get("agent_position"),
            "goal": environment.get("goal"),
            "objects_count": len(objects),
            "summary": merged.get("summary"),
        },
    }


def register_tools(mcp):

    @mcp.tool()
    async def vlm(image_path: str = "") -> str:
        """观察当前环境，返回视觉描述和实时状态。

        Args:
            image_path: 图片路径，留空使用默认摄像头

        Returns:
            视觉观察结果 JSON 字符串
        """
        result = await execute_vlm_observe(image_path=image_path)
        return json.dumps(result, ensure_ascii=False)

    print("[Vision/Bishe] 视觉模块已注册 (1 个工具)", file=sys.stderr)
    return {"vlm": vlm}
