"""Rule-based planner overrides for known scene geometry."""

from __future__ import annotations

import re
from typing import Any

from .parsing import build_fallback_task, normalize_task


def apply_rule_overrides(
    tasks: list[dict],
    meta: dict[str, object],
    *,
    user_input: str,
    scene_facts: dict[str, Any] | None,
    object_facts: dict[str, Any] | None,
    on_event=None,
) -> tuple[list[dict], dict[str, object]]:
    geometry = _detect_box_assisted_geometry(scene_facts, object_facts)
    if geometry and _should_override_box_assisted_plan(tasks, user_input):
        tasks, summary = _build_box_assisted_plan(geometry)
        updated_meta = dict(meta)
        updated_meta["selected_plan_id"] = "rule_override_box_assist"
        updated_meta["summary"] = summary
        updated_meta["scene_assessment"] = (
            f"检测到可移动箱子 {geometry['box_id']} 与高台 {geometry['platform_id']} 形成辅助登台几何关系。"
        )
        msg = "⚠️  [规则修正] 检测到箱子辅助登台场景，已修正为 push_box -> climb_align -> climb -> nav_climb"
        if on_event:
            on_event("rule_override", {"message": msg})
        else:
            print(msg)
        return tasks, updated_meta

    climbable = _detect_climbable_obstacle(scene_facts, object_facts)
    if climbable and _is_navigation_request(user_input):
        tasks, summary = _build_climbable_obstacle_plan(climbable)
        updated_meta = dict(meta)
        updated_meta["selected_plan_id"] = "rule_override_climbable_obstacle"
        updated_meta["summary"] = summary
        updated_meta["scene_assessment"] = (
            f"检测到可攀爬高台 {climbable['platform_id']} ({climbable['platform_height']}m) 阻挡路径。"
        )
        msg = f"⚠️  [规则修正] 检测到可攀爬高台场景，已修正为 way_select -> nav_climb"
        if on_event:
            on_event("rule_override", {"message": msg})
        else:
            print(msg)
        return tasks, updated_meta

    return tasks, meta


def build_rule_fallback(
    *,
    user_input: str,
    scene_facts: dict[str, Any] | None,
    object_facts: dict[str, Any] | None,
) -> tuple[list[dict], dict[str, object]]:
    geometry = _detect_box_assisted_geometry(scene_facts, object_facts)
    if geometry and _is_navigation_request(user_input):
        tasks, summary = _build_box_assisted_plan(geometry)
        return tasks, {
            "scene_assessment": "规则回退：箱子辅助登台场景",
            "candidate_plans": [],
            "selected_plan_id": "fallback_box_assist",
            "summary": summary,
        }
    climbable = _detect_climbable_obstacle(scene_facts, object_facts)
    if climbable and _is_navigation_request(user_input):
        tasks, summary = _build_climbable_obstacle_plan(climbable)
        return tasks, {
            "scene_assessment": "规则回退：可攀爬高台场景",
            "candidate_plans": [],
            "selected_plan_id": "fallback_climbable_obstacle",
            "summary": summary,
        }
    return [build_fallback_task(user_input)], {
        "scene_assessment": "",
        "candidate_plans": [],
        "selected_plan_id": "fallback_plan",
        "summary": "未生成明确任务，保持用户原始动作意图",
    }


def _is_navigation_request(user_input: str) -> bool:
    return bool(re.search(r"(导航|前往|去往|到达|到.*点|move to|go to|navigate)", str(user_input or ""), re.IGNORECASE))


def _should_override_box_assisted_plan(tasks: list[dict], user_input: str) -> bool:
    if not _is_navigation_request(user_input):
        return False
    function_chain = [str(task.get("function", "")) for task in tasks]
    return function_chain[:4] != ["push_box", "climb_align", "climb", "nav_climb"]


def _planner_scene(scene_facts: dict[str, Any] | None) -> dict[str, Any]:
    if not isinstance(scene_facts, dict):
        return {}
    nested = scene_facts.get("scene_facts")
    return nested if isinstance(nested, dict) else scene_facts


def _object_height(obj: dict[str, Any] | None) -> float:
    if not obj:
        return 0.0
    size = obj.get("size") or [0.0, 0.0, 0.0]
    try:
        return round(float(size[2]), 3)
    except (TypeError, ValueError, IndexError):
        return 0.0


def _platform_side(platform: dict[str, Any]) -> str:
    center = platform.get("center") or [0.0, 0.0, 0.0]
    return "left" if float(center[1]) >= 0 else "right"


def _detect_climbable_obstacle(
    scene_facts: dict[str, Any] | None,
    object_facts: dict[str, Any] | None,
) -> dict[str, object] | None:
    if scene_facts is None or object_facts is None:
        return None

    constraints = object_facts.get("constraints") or {}
    climb_limit = float(constraints.get("max_climb_height_m", 0.3))
    objects = object_facts.get("objects") or []
    platforms = [obj for obj in objects if str(obj.get("type", "")).lower() == "platform"]
    if not platforms:
        return None

    left_platforms = [p for p in platforms if _platform_side(p) == "left"]
    right_platforms = [p for p in platforms if _platform_side(p) == "right"]
    if not left_platforms or not right_platforms:
        return None

    climbable_platforms = [p for p in platforms if 0 < _object_height(p) <= climb_limit]
    if not climbable_platforms:
        return None

    best = min(climbable_platforms, key=_object_height)
    platform_height = _object_height(best)
    center = best.get("center") or [0.0, 0.0, 0.0]
    side = "left" if float(center[1]) >= 0 else "right"
    lateral_distance = abs(float(center[1]))

    return {
        "side": side,
        "platform_id": str(best.get("id", "platform")),
        "platform_height": platform_height,
        "lateral_distance": round(lateral_distance + 0.1, 3),
        "platform_center": center,
        "platform_size": best.get("size") or [0.0, 0.0, 0.0],
    }


def _build_climbable_obstacle_plan(climbable: dict[str, object]) -> tuple[list[dict], str]:
    side = str(climbable["side"])
    side_label = "左侧" if side == "left" else "右侧"
    platform_id = str(climbable["platform_id"])
    platform_height = float(climbable["platform_height"])
    height_label = f"{platform_height:.1f}".rstrip("0").rstrip(".")

    return (
        [
            normalize_task(
                {
                    "step": 1,
                    "task": f"调用 way_select，切换到{side_label}路线靠近高台 {platform_id}",
                    "type": "路线选择",
                    "function": "way_select",
                    "reason": f"前方有{height_label}米高台阻挡，需先切换到{side_label}路线。",
                },
                1,
            ),
            normalize_task(
                {
                    "step": 2,
                    "task": f"使用 nav_climb 导航并攀爬到目标点",
                    "type": "导航攀爬",
                    "function": "nav_climb",
                    "reason": f"高台高{height_label}米，nav_climb 可一步完成导航和攀爬。",
                },
                2,
            ),
        ],
        f"检测到可攀爬高台({height_label}m)，执行 way_select -> nav_climb",
    )


def _detect_box_assisted_geometry(
    scene_facts: dict[str, Any] | None,
    object_facts: dict[str, Any] | None,
) -> dict[str, object] | None:
    if not scene_facts or not object_facts:
        return None

    planner_scene = _planner_scene(scene_facts)
    constraints = object_facts.get("constraints") or planner_scene.get("constraints") or {}
    climb_limit = float(constraints.get("max_climb_height_m", 0.3))
    objects = object_facts.get("objects") or []
    boxes = [obj for obj in objects if obj.get("movable") and str(obj.get("type", "")).lower() == "box"]
    platforms = [obj for obj in objects if str(obj.get("type", "")).lower() == "platform"]
    if not boxes or not platforms:
        return None

    support_box = min(boxes, key=_object_height)
    target_platform = max(platforms, key=_object_height)
    box_height = _object_height(support_box)
    platform_height = _object_height(target_platform)
    remaining_height = round(platform_height - box_height, 3)
    if not (0 < box_height <= climb_limit < platform_height and 0 < remaining_height <= climb_limit):
        return None

    route_options = planner_scene.get("route_options") or []
    blocked = {str(item.get("direction", "")).lower() for item in route_options if str(item.get("status", "")).lower() == "blocked"}
    if route_options and not {"left", "right"}.issubset(blocked):
        return None

    center = support_box.get("center") or target_platform.get("center") or [0.0, 0.0, 0.0]
    side = "left" if float(center[1]) >= 0 else "right"
    return {
        "side": side,
        "box_id": str(support_box.get("id", "box")),
        "box_height": box_height,
        "platform_id": str(target_platform.get("id", "platform")),
        "platform_height": platform_height,
        "remaining_height": remaining_height,
    }


def _build_box_assisted_plan(box_plan: dict[str, object]) -> tuple[list[dict], str]:
    side = str(box_plan["side"])
    side_label = "左侧" if side == "left" else "右侧"
    box_id = str(box_plan["box_id"])
    platform_id = str(box_plan["platform_id"])
    remaining_height = float(box_plan["remaining_height"])
    remaining_height_label = f"{remaining_height:.1f}".rstrip("0").rstrip(".")
    return (
        [
            normalize_task(
                {
                    "step": 1,
                    "task": f"先调用 push_box，将箱子 {box_id} 推到高台 {platform_id} 旁边",
                    "type": "推箱子",
                    "function": "push_box",
                    "reason": f"{side_label}箱子可作为辅助台阶，应先完成推箱。",
                },
                1,
            ),
            normalize_task(
                {
                    "step": 2,
                    "task": f"调用 climb_align，对正到箱子 {box_id} 后方的攀爬起点",
                    "type": "攀爬对正",
                    "function": "climb_align",
                    "reason": "推箱完成后需要先对正，再执行 climb。",
                },
                2,
            ),
            normalize_task(
                {
                    "step": 3,
                    "task": f"利用已推到位的箱子辅助攀爬约{remaining_height_label}米到高台 {platform_id}",
                    "type": "攀爬",
                    "function": "climb",
                    "reason": f"借助箱子后，剩余高差约 {remaining_height_label} 米，单次 climb 可登台。",
                },
                3,
            ),
            normalize_task(
                {
                    "step": 4,
                    "task": "使用 nav_climb 导航前往目标点",
                    "type": "导航攀爬",
                    "function": "nav_climb",
                    "reason": "越过高差后，使用 nav_climb 导航到终点。",
                },
                4,
            ),
        ],
        "根据结构化物体几何信息，应执行 push_box -> climb_align -> climb -> nav_climb",
    )

