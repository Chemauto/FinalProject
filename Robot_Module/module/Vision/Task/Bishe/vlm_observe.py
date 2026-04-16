from __future__ import annotations

import json
import sys

from Comm_Module import get_state
from VLM_Module.vlm_core import VLMCore


def _shorten_text(text: object, limit: int = 150) -> str:
    content = str(text or "").strip()
    if len(content) <= limit:
        return content
    return content[: max(limit - 1, 0)].rstrip("，。；、 ") + "…"


def _build_visual_description(visual_payload: dict[str, object]) -> str:
    segments: list[str] = []
    ground = str(visual_payload.get("ground") or "").strip()
    left_side = str(visual_payload.get("left_side") or "").strip()
    right_side = str(visual_payload.get("right_side") or "").strip()
    front_area = str(visual_payload.get("front_area") or "").strip()
    obstacles = visual_payload.get("obstacles") or []
    uncertainties = visual_payload.get("uncertainties") or []

    if ground and ground != "unknown":
        segments.append(f"当前环境地面为{ground.rstrip('。')}")
    if left_side and left_side != "unknown" and right_side and right_side != "unknown":
        segments.append(f"{left_side.rstrip('。')}，{right_side.rstrip('。')}")
    else:
        if left_side and left_side != "unknown":
            segments.append(left_side.rstrip("。"))
        if right_side and right_side != "unknown":
            segments.append(right_side.rstrip("。"))
    if front_area and front_area != "unknown":
        segments.append(f"前方{front_area.rstrip('。')}")
    if isinstance(obstacles, list) and obstacles:
        obstacle_text = "、".join(str(item).strip() for item in obstacles if str(item).strip())
        if obstacle_text:
            segments.append(f"当前画面主要可见{obstacle_text}")
    elif front_area and front_area != "unknown":
        segments.append("画面中未见独立的平台、箱子或其他明显障碍")
    if isinstance(uncertainties, list) and uncertainties:
        uncertainty_text = "；".join(str(item).strip() for item in uncertainties[:2] if str(item).strip())
        if uncertainty_text:
            segments.append(f"仍需进一步确认的是：{uncertainty_text}")

    description = "。".join(item.rstrip("。") for item in segments if item).strip()
    if not description:
        description = "当前画面环境信息不足，无法形成稳定视觉描述。"
    if not description.endswith("。"):
        description += "。"
    return _shorten_text(description, limit=150)


def _build_alignment_slot(slot_name: str, asset: object) -> dict[str, object] | None:
    if not isinstance(asset, dict):
        return None

    name = str(asset.get("name") or "").strip() or "unknown"
    size = asset.get("size") or []
    height_m = round(float(size[2]), 3) if isinstance(size, list) and len(size) >= 3 else 0.0
    if slot_name == "box":
        side = "center"
        label = "中间"
        object_type = "box"
    elif slot_name == "platform_1":
        side = "left"
        label = "左侧"
        object_type = "platform"
    else:
        side = "right"
        label = "右侧"
        object_type = "platform"

    if "low" in name:
        summary = f"{label}约{height_m:.1f}米低平台"
    elif "high" in name:
        summary = f"{label}约{height_m:.1f}米高平台"
    elif slot_name == "box":
        summary = f"{label}约{height_m:.1f}米箱子"
    else:
        summary = f"{label}目标物 {name}"

    return {
        "name": name,
        "type": object_type,
        "side": side,
        "height_m": height_m,
        "summary": summary,
    }


def _build_envtest_alignment(state: dict[str, object]) -> dict[str, object]:
    observation = state.get("observation") or {}
    environment = observation.get("environment") or {}
    alignment = environment.get("envtest_alignment")
    if not isinstance(alignment, dict):
        runtime = state.get("runtime") or {}
        alignment = runtime.get("envtest_alignment") or {}
    if not isinstance(alignment, dict):
        alignment = {}
    return {
        "platform_1": _build_alignment_slot("platform_1", alignment.get("platform_1")),
        "platform_2": _build_alignment_slot("platform_2", alignment.get("platform_2")),
        "box": _build_alignment_slot("box", alignment.get("box")),
    }


def _build_env_state(scene_facts: dict[str, object], state: dict[str, object], envtest_alignment: dict[str, object]) -> dict[str, object]:
    observation = state.get("observation") or {}
    environment = observation.get("environment") or {}
    action_result = observation.get("action_result") or {}
    runtime = state.get("runtime") or {}
    objects = environment.get("obstacles") or runtime.get("scene_objects") or []
    object_facts = None
    if objects:
        object_facts = {
            "constraints": {"max_climb_height_m": VLMCore.MAX_CLIMB_HEIGHT_M},
            "objects": objects,
        }
    merged_scene_facts = VLMCore.merge_scene_facts(scene_facts, object_facts)
    payload = {
        "connected": bool(state.get("connected")),
        "scene_id": environment.get("scene_id"),
        "agent_position": observation.get("agent_position"),
        "goal": environment.get("goal"),
        "skill": runtime.get("skill") or action_result.get("skill"),
        "model_use": runtime.get("model_use") or action_result.get("model_use"),
        "start": runtime.get("start") if runtime.get("start") is not None else action_result.get("start"),
        "objects_count": len(objects),
        "envtest_alignment": envtest_alignment,
        "summary": merged_scene_facts.get("summary"),
        "scene_facts": merged_scene_facts,
        "uncertainties": merged_scene_facts.get("uncertainties", []),
    }
    if objects:
        payload["objects"] = objects
    return payload


async def execute_vlm_observe(image_path: str = "") -> dict[str, object]:
    vlm = VLMCore()
    resolved_image = vlm.image_source.get_image(image_path or None)
    visual_context = vlm.describe_structured(str(resolved_image))
    scene_facts = VLMCore.build_scene_facts(visual_context)
    try:
        live_state = get_state()
    except Exception:
        live_state = {"connected": False, "task_type": None, "observation": {}, "runtime": {}}
    envtest_alignment = _build_envtest_alignment(live_state)
    env_state = _build_env_state(scene_facts, live_state, envtest_alignment)
    display_visual_context = {
        "description": _build_visual_description(visual_context),
        "envtest_alignment": envtest_alignment,
    }
    return {
        "status": "success",
        "image_source": str(resolved_image),
        "visual_context": display_visual_context,
        "visual_details": visual_context,
        "scene_facts": scene_facts,
        "env_state": env_state,
        "uncertainties": env_state.get("uncertainties", visual_context.get("uncertainties", [])),
    }


def register_tool(mcp):
    @mcp.tool()
    async def vlm(image_path: str = "") -> str:
        return json.dumps(await execute_vlm_observe(image_path=image_path), ensure_ascii=False)

    print("[Vision/Task/Bishe] 视觉模块已注册 (1 个工具)", file=sys.stderr)
    return {"vlm": vlm}
