"""Bishe rule-based planner: box-assist navigation.

Moved from LLM_Module/llm_core.py HighLevelPlanner._build_rule_based_plan().
"""

from __future__ import annotations

import re
from typing import Any

from ._bishe_helpers import (
    coerce_vec3,
    object_height,
    select_support_box,
    select_target_platform,
    infer_route_side,
)


def _is_navigation_request(user_input: str) -> bool:
    text = str(user_input or "").strip().lower()
    return bool(re.search(r"(导航|前往|去往|到达|到.*点|move to|go to|navigate)", text))


def _all_routes_blocked(planner_scene_facts: dict[str, Any]) -> bool:
    route_options = planner_scene_facts.get("route_options")
    if not isinstance(route_options, list) or not route_options:
        return False
    statuses = [str(item.get("status", "")).lower() for item in route_options if isinstance(item, dict)]
    return bool(statuses) and all(status == "blocked" for status in statuses)


def _normalize_task(task: dict, default_step: int) -> dict:
    return {
        "step": task.get("step", default_step),
        "task": task.get("task", ""),
        "type": task.get("type", "未分类"),
        "function": task.get("function", "待LLM决定"),
        "reason": task.get("reason", "未提供规划依据"),
    }


def plan_box_assist_navigation(
    user_input: str,
    scene_facts: dict[str, Any] | None,
    object_facts: dict[str, Any] | None,
) -> tuple[list[dict], dict[str, Any]] | None:
    """Rule planner for box-assist navigation scenarios.

    Returns (tasks, meta) if this planner handles the input, or None.
    """
    if not isinstance(scene_facts, dict) or "robot_state" not in scene_facts:
        return None
    if not _is_navigation_request(user_input):
        return None

    planner_scene_facts = scene_facts.get("scene_facts") or {}
    robot_state = scene_facts.get("robot_state") or {}
    constraints = scene_facts.get("constraints") or (object_facts or {}).get("constraints") or {}
    objects = scene_facts.get("objects") or (object_facts or {}).get("objects") or []
    if not isinstance(objects, list) or not objects:
        return None

    goal = coerce_vec3(robot_state.get("goal")) or coerce_vec3((object_facts or {}).get("navigation_goal"))
    robot_pose = coerce_vec3(robot_state.get("robot_pose")) or coerce_vec3((object_facts or {}).get("robot_pose"))
    if not goal or not robot_pose:
        return None

    support_box = select_support_box(objects)
    target_platform = select_target_platform(objects)
    climb_limit = float(constraints.get("max_climb_height_m", 0.3))
    box_height = object_height(support_box)
    platform_height = object_height(target_platform)
    remaining_height = round(platform_height - box_height, 3)

    if not support_box or not target_platform:
        return None
    if platform_height <= climb_limit or box_height <= 0 or box_height > climb_limit or remaining_height <= 0 or remaining_height > climb_limit:
        return None
    if not _all_routes_blocked(planner_scene_facts):
        return None

    platform_center = coerce_vec3(target_platform.get("center")) or [0.0, 0.0, 0.0]
    if goal[0] <= platform_center[0]:
        return None

    platform_id = str(target_platform.get("id") or "platform_1")
    box_id = str(support_box.get("id") or "box")
    summary = f"目标点位于高台障碍之后，需先推箱子辅助登台，再前往目标点 ({goal[0]:g}, {goal[1]:g}, {goal[2]:g})。"
    tasks = [
        _normalize_task(
            {
                "step": 1,
                "task": f"将箱子 {box_id} 推到平台 {platform_id} 旁边",
                "type": "物体交互",
                "function": "push_box",
                "reason": f"平台高度 {platform_height:.2f} 米超过最大攀爬高度 {climb_limit:.2f} 米，需要先利用 {box_id} 作为辅助台阶。",
            },
            1,
        ),
        _normalize_task(
            {
                "step": 2,
                "task": f"爬上箱子 {box_id}",
                "type": "攀爬",
                "function": "climb",
                "reason": f"{box_id} 高度 {box_height:.2f} 米，不超过最大攀爬高度，可作为第一步登高。",
            },
            2,
        ),
        _normalize_task(
            {
                "step": 3,
                "task": f"从箱子 {box_id} 爬上平台 {platform_id}",
                "type": "攀爬",
                "function": "climb",
                "reason": f"借助 {box_id} 后，剩余高差 {remaining_height:.2f} 米，不超过最大攀爬高度，可继续登上 {platform_id}。",
            },
            3,
        ),
        _normalize_task(
            {
                "step": 4,
                "task": f"导航至目标坐标 ({goal[0]:g}, {goal[1]:g}, {goal[2]:g})",
                "type": "导航",
                "function": "navigation",
                "reason": f"登台后再前往目标点 ({goal[0]:g}, {goal[1]:g}, {goal[2]:g})，避免在地面阶段被高台阻挡。",
            },
            4,
        ),
    ]
    meta = {
        "scene_assessment": f"检测到高台高度 {platform_height:.2f} 米、辅助箱子高度 {box_height:.2f} 米，满足箱子辅助登台条件。",
        "candidate_plans": [],
        "selected_plan_id": "box_assist_nav_plan",
        "summary": summary,
    }
    return tasks, meta
