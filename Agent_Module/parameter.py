#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Agent_Module/parameter.py — object_facts 加载 + 参数提取。"""

from __future__ import annotations

import json
from math import hypot
from pathlib import Path
import re
from typing import Any

DEFAULT_CONSTRAINTS: dict[str, Any] = {}


# ── Normalization helpers ──────────────────────────────────────────────


def _normalize_bool(value: Any, default: bool) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "1", "yes", "y", "是"}:
            return True
        if normalized in {"false", "0", "no", "n", "否"}:
            return False
    return default


def _normalize_number(value: Any, field_name: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{field_name} 必须是数字，收到: {value!r}") from error


def _normalize_vec3(value: Any, field_name: str) -> list[float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{field_name} 必须是长度为 3 的数组")
    return [_normalize_number(item, field_name) for item in value]


def _normalize_object(payload: dict[str, Any], index: int) -> dict[str, Any]:
    object_id = str(payload.get("id", "")).strip()
    if not object_id:
        raise ValueError(f"objects[{index}].id 不能为空")
    object_type = str(payload.get("type", "")).strip() or "unknown"
    return {
        "id": object_id,
        "type": object_type,
        "center": _normalize_vec3(payload.get("center"), f"objects[{index}].center"),
        "size": _normalize_vec3(payload.get("size"), f"objects[{index}].size"),
        "movable": _normalize_bool(payload.get("movable"), default=False),
    }


def _normalize_constraints(payload: dict[str, Any]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in payload.items():
        if isinstance(value, bool):
            result[key] = value
        elif isinstance(value, (int, float)):
            result[key] = float(value)
        else:
            result[key] = value
    return result


def normalize_object_facts(payload: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(payload, dict):
        raise ValueError("object_facts 顶层必须是 JSON 对象")
    constraints_payload = payload.get("constraints") or {}
    if constraints_payload and not isinstance(constraints_payload, dict):
        raise ValueError("constraints 必须是 JSON 对象")
    objects_payload = payload.get("objects") or []
    if not isinstance(objects_payload, list):
        raise ValueError("objects 必须是数组")
    constraints = _normalize_constraints(constraints_payload)
    return {
        "navigation_goal": _normalize_vec3(payload.get("navigation_goal"), "navigation_goal"),
        "robot_pose": _normalize_vec3(payload.get("robot_pose"), "robot_pose"),
        "constraints": constraints,
        "objects": [_normalize_object(item, index) for index, item in enumerate(objects_payload)],
    }


# ── Public API ─────────────────────────────────────────────────────────


def load_object_facts(path: str | Path | None) -> dict[str, Any] | None:
    if path is None:
        return None
    file_path = Path(path)
    if not file_path.exists():
        return None
    payload = json.loads(file_path.read_text(encoding="utf-8"))
    return normalize_object_facts(payload)


class ParameterCalculator:

    def annotate_tasks(
        self,
        tasks: list[dict[str, Any]],
        object_facts: dict[str, Any] | None,
    ) -> list[dict[str, Any]]:
        if not tasks or not object_facts:
            return tasks

        objects = object_facts.get("objects") or []
        constraints = object_facts.get("constraints") or {}
        climb_limit = float(constraints.get("max_climb_height_m", 0.3))
        navigation_goal = self._coerce_vec3(object_facts.get("navigation_goal"))
        current_pose = self._coerce_vec3(object_facts.get("robot_pose")) or [0.0, 0.0, 0.0]

        support_box = self._select_support_box(objects)
        target_platform = self._select_target_platform(objects)
        route_side = self._infer_route_side(support_box or target_platform)
        current_route_side = route_side
        plan_uses_push_box = any(task.get("function") == "push_box" for task in tasks)
        total_climb_steps = sum(1 for task in tasks if task.get("function") == "climb")
        climb_count = 0

        box_height = self._object_height(support_box)
        platform_height = self._object_height(target_platform)
        remaining_height = round(platform_height - box_height, 3)
        can_use_box_assist = bool(
            support_box
            and target_platform
            and 0 < box_height <= climb_limit
            and platform_height > climb_limit
            and 0 < remaining_height <= climb_limit
        )

        annotated_tasks = []
        for task in tasks:
            annotated = dict(task)
            function_name = task.get("function")
            parameter_context = dict(task.get("parameter_context") or {})

            selected_support_box = self._select_box_for_task(
                task=task,
                objects=objects,
                preferred_side=current_route_side,
                default_box=support_box,
                current_pose=current_pose,
            )
            selected_platform = self._select_platform_for_task(
                task=task,
                objects=objects,
                preferred_side=current_route_side,
                default_platform=target_platform,
                current_pose=current_pose,
            )
            selected_box_height = self._object_height(selected_support_box)
            selected_platform_height = self._object_height(selected_platform)
            selected_remaining_height = round(selected_platform_height - selected_box_height, 3)
            selected_can_use_box_assist = bool(
                plan_uses_push_box
                and selected_support_box
                and selected_platform
                and 0 < selected_box_height <= climb_limit
                and selected_platform_height > climb_limit
                and 0 < selected_remaining_height <= climb_limit
            )

            if function_name == "push_box" and (selected_can_use_box_assist or can_use_box_assist):
                active_box = selected_support_box or support_box
                active_platform = selected_platform or target_platform
                if active_box and active_platform:
                    parameter_context.update(
                        {
                            "support_object": active_box["id"],
                            "target_object": active_platform["id"],
                            "auto_target_position_xyz": self._build_adjacent_ground_position(active_box, active_platform),
                        }
                    )
                    annotated["parameter_context"] = parameter_context
                    annotated["calculated_parameters"] = {
                        "box_height": self._object_height(active_box),
                        "target_position": "auto",
                    }
                    current_pose = list(parameter_context["auto_target_position_xyz"])

            elif function_name == "climb_align" and selected_platform:
                climb_align_goal = None
                if selected_can_use_box_assist and selected_support_box:
                    climb_align_goal = self._build_climb_align_goal(selected_support_box, selected_platform)

                parameter_context.update(
                    {
                        "target_object": selected_platform["id"],
                        "support_object": (selected_support_box or {}).get("id"),
                    }
                )
                if climb_align_goal:
                    parameter_context["climb_align_goal_xyz"] = climb_align_goal

                calc_params = {
                    "stage": selected_platform["id"],
                    "target": selected_platform["id"],
                }
                if climb_align_goal:
                    calc_params["goal_command"] = ",".join(str(v) for v in climb_align_goal)

                annotated["parameter_context"] = parameter_context
                annotated["calculated_parameters"] = calc_params

                if climb_align_goal:
                    current_pose = list(climb_align_goal)

            elif function_name == "climb" and selected_platform:
                climb_count += 1
                if selected_can_use_box_assist and selected_support_box and total_climb_steps <= 1:
                    parameter_context.update(
                        {
                            "support_object": selected_support_box["id"],
                            "target_object": selected_platform["id"],
                            "relative_height_m": selected_remaining_height,
                        }
                    )
                    annotated["parameter_context"] = parameter_context
                    annotated["calculated_parameters"] = {
                        "height": selected_remaining_height,
                        "stage": selected_platform["id"],
                        "target": selected_platform["id"],
                    }
                    current_pose = self._estimate_pose_after_climb(current_pose, selected_platform, selected_remaining_height)
                    current_route_side = self._infer_route_side(selected_platform) or current_route_side
                else:
                    parameter_context.update(
                        {
                            "target_object": selected_platform["id"],
                            "relative_height_m": selected_platform_height,
                        }
                    )
                    annotated["parameter_context"] = parameter_context
                    annotated["calculated_parameters"] = {
                        "height": selected_platform_height,
                        "stage": selected_platform["id"],
                        "target": selected_platform["id"],
                    }
                    current_pose = self._estimate_pose_after_climb(current_pose, selected_platform, selected_platform_height)
                    current_route_side = self._infer_route_side(selected_platform) or current_route_side

            elif function_name == "way_select":
                way_side = self._infer_way_side_from_task(task, objects)
                if way_side:
                    lateral_dist = self._estimate_lateral_distance(way_side, objects)
                    parameter_context.update({
                        "route_side": way_side,
                    })
                    annotated["parameter_context"] = parameter_context
                    annotated["calculated_parameters"] = {
                        "direction": way_side,
                        "lateral_distance": lateral_dist,
                        "target": selected_platform["id"] if selected_platform else "目标点",
                    }
                    # way_select 后 x 基本不变，y 偏移到平台侧面
                    if selected_platform:
                        plat_center = selected_platform.get("center") or [0.0, 0.0, 0.0]
                        current_pose = [current_pose[0], float(plat_center[1]), current_pose[2]]

            elif function_name in {"navigation", "nav_climb"}:
                navigation_parameters = self._build_navigation_parameters(navigation_goal)
                if navigation_parameters:
                    parameter_context.update(
                        {
                            "start_pose_xyz": [round(value, 3) for value in current_pose],
                            "goal_pose_xyz": navigation_goal,
                        }
                    )
                    annotated["parameter_context"] = parameter_context
                    annotated["calculated_parameters"] = navigation_parameters
                    current_pose = list(navigation_goal)

            elif function_name == "walk":
                walk_parameters = self._build_walk_parameters(task, current_pose, navigation_goal)
                if walk_parameters:
                    parameter_context.update(
                        {
                            "start_pose_xyz": [round(value, 3) for value in current_pose],
                            "goal_pose_xyz": navigation_goal,
                        }
                    )
                    annotated["parameter_context"] = parameter_context
                    annotated["calculated_parameters"] = walk_parameters
                    current_pose = list(navigation_goal)

            annotated_tasks.append(annotated)

        return annotated_tasks

    @classmethod
    def _build_walk_parameters(
        cls,
        task: dict[str, Any],
        current_pose: list[float],
        navigation_goal: list[float] | None,
    ) -> dict[str, Any] | None:
        if not navigation_goal:
            return None
        distance = round(hypot(navigation_goal[0] - current_pose[0], navigation_goal[1] - current_pose[1]), 3)
        distance = max(0.05, distance)
        route_side = cls._infer_walk_route_label(task, current_pose, navigation_goal)
        return {
            "route_side": route_side,
            "distance": distance,
            "target": cls._format_navigation_target(navigation_goal),
        }

    @classmethod
    def _build_navigation_parameters(cls, navigation_goal: list[float] | None) -> dict[str, Any] | None:
        if not navigation_goal:
            return None
        return {
            "goal": cls._format_navigation_goal_command(navigation_goal),
            "target": cls._format_navigation_target(navigation_goal),
        }

    @staticmethod
    def _coerce_vec3(value: Any) -> list[float] | None:
        if not isinstance(value, list) or len(value) < 3:
            return None
        try:
            return [float(value[0]), float(value[1]), float(value[2])]
        except (TypeError, ValueError):
            return None

    @classmethod
    def _estimate_pose_after_climb(
        cls,
        current_pose: list[float],
        stage_obj: dict[str, Any] | None,
        fallback_height: float,
    ) -> list[float]:
        if not stage_obj:
            return [round(current_pose[0], 3), round(current_pose[1], 3), round(current_pose[2] + fallback_height, 3)]
        stage_center = cls._coerce_vec3(stage_obj.get("center"))
        stage_size = cls._coerce_vec3(stage_obj.get("size"))
        if not stage_center:
            return [round(current_pose[0], 3), round(current_pose[1], 3), round(current_pose[2] + fallback_height, 3)]
        top_z = stage_center[2] + stage_size[2] if stage_size else current_pose[2] + fallback_height
        return [round(stage_center[0], 3), round(stage_center[1], 3), round(top_z, 3)]

    @classmethod
    def _infer_walk_route_label(
        cls,
        task: dict[str, Any],
        current_pose: list[float],
        navigation_goal: list[float],
    ) -> str:
        task_text = f"{task.get('task', '')} {task.get('reason', '')}".lower()
        if "左侧" in task_text or "left" in task_text:
            return "左侧"
        if "右侧" in task_text or "right" in task_text:
            return "右侧"
        lateral_delta = navigation_goal[1] - current_pose[1]
        if abs(lateral_delta) <= 0.15:
            return "前方"
        return "左侧" if lateral_delta > 0 else "右侧"

    @staticmethod
    def _format_navigation_target(navigation_goal: list[float]) -> str:
        x, y, z = navigation_goal
        return f"目标点[{x:g}, {y:g}, {z:g}]"

    @staticmethod
    def _format_navigation_goal_command(navigation_goal: list[float]) -> str:
        x, y, z = navigation_goal
        return f"[{x:g}, {y:g}, {z:g}]"

    @staticmethod
    def _object_height(obj: dict[str, Any] | None) -> float:
        if not obj:
            return 0.0
        size = obj.get("size") or [0.0, 0.0, 0.0]
        return round(float(size[2]), 3)

    @staticmethod
    def _select_support_box(objects: list[dict[str, Any]]) -> dict[str, Any] | None:
        boxes = [obj for obj in objects if obj.get("movable") and str(obj.get("type", "")).lower() == "box"]
        if not boxes:
            return None
        return min(boxes, key=lambda item: float((item.get("size") or [0.0, 0.0, 0.0])[2]))

    @staticmethod
    def _select_target_platform(objects: list[dict[str, Any]]) -> dict[str, Any] | None:
        platforms = [obj for obj in objects if str(obj.get("type", "")).lower() == "platform"]
        if not platforms:
            return None
        return max(platforms, key=lambda item: float((item.get("size") or [0.0, 0.0, 0.0])[2]))

    @classmethod
    def _select_platform_for_task(
        cls,
        task: dict[str, Any],
        objects: list[dict[str, Any]],
        preferred_side: str | None,
        default_platform: dict[str, Any] | None,
        current_pose: list[float] | None,
    ) -> dict[str, Any] | None:
        explicit_platform = cls._match_object_from_task(task, objects, object_type="platform")
        if explicit_platform:
            return explicit_platform
        task_side = cls._infer_task_side(task) or preferred_side
        if task_side:
            side_platform = cls._select_platform_on_side(objects, task_side, current_pose)
            if side_platform:
                return side_platform
        return default_platform

    @classmethod
    def _select_box_for_task(
        cls,
        task: dict[str, Any],
        objects: list[dict[str, Any]],
        preferred_side: str | None,
        default_box: dict[str, Any] | None,
        current_pose: list[float] | None,
    ) -> dict[str, Any] | None:
        explicit_box = cls._match_object_from_task(task, objects, object_type="box")
        if explicit_box:
            return explicit_box
        task_side = cls._infer_task_side(task) or preferred_side
        if task_side:
            side_box = cls._select_box_on_side(objects, task_side, current_pose)
            if side_box:
                return side_box
        return default_box

    @classmethod
    def _select_platform_on_side(
        cls,
        objects: list[dict[str, Any]],
        side: str,
        current_pose: list[float] | None,
    ) -> dict[str, Any] | None:
        candidates = [
            obj for obj in objects
            if str(obj.get("type", "")).lower() == "platform" and cls._infer_route_side(obj) == side
        ]
        if not candidates:
            return None
        if not current_pose:
            return min(candidates, key=lambda item: abs(float((item.get("center") or [0.0, 0.0, 0.0])[0])))
        return min(candidates, key=lambda item: cls._planar_distance_to_object(current_pose, item))

    @classmethod
    def _select_box_on_side(
        cls,
        objects: list[dict[str, Any]],
        side: str,
        current_pose: list[float] | None,
    ) -> dict[str, Any] | None:
        candidates = [
            obj for obj in objects
            if str(obj.get("type", "")).lower() == "box"
            and obj.get("movable")
            and cls._infer_route_side(obj) == side
        ]
        if not candidates:
            return None
        if not current_pose:
            return min(candidates, key=lambda item: abs(float((item.get("center") or [0.0, 0.0, 0.0])[0])))
        return min(candidates, key=lambda item: cls._planar_distance_to_object(current_pose, item))

    @classmethod
    def _match_object_from_task(
        cls,
        task: dict[str, Any],
        objects: list[dict[str, Any]],
        object_type: str,
    ) -> dict[str, Any] | None:
        task_text = f"{task.get('task', '')} {task.get('reason', '')}".lower()
        for obj in objects:
            if str(obj.get("type", "")).lower() != object_type:
                continue
            object_id = str(obj.get("id", "")).lower()
            if object_id and object_id in task_text:
                return obj
        return None

    @staticmethod
    def _infer_task_side(task: dict[str, Any]) -> str | None:
        task_text = f"{task.get('task', '')} {task.get('reason', '')}".lower()
        if "左侧" in task_text and "右侧" not in task_text:
            return "left"
        if "右侧" in task_text and "左侧" not in task_text:
            return "right"
        return None

    @staticmethod
    def _planar_distance_to_object(current_pose: list[float], obj: dict[str, Any]) -> float:
        center = obj.get("center") or [0.0, 0.0, 0.0]
        return hypot(float(center[0]) - current_pose[0], float(center[1]) - current_pose[1])

    @staticmethod
    def _infer_route_side(obj: dict[str, Any] | None) -> str | None:
        if not obj:
            return None
        center = obj.get("center") or [0.0, 0.0, 0.0]
        return "left" if float(center[1]) >= 0 else "right"

    @staticmethod
    def _infer_way_side_from_task(task: dict[str, Any], objects: list[dict[str, Any]]) -> str | None:
        task_text = f"{task.get('task', '')} {task.get('reason', '')}".lower()
        if "左侧" in task_text and "右侧" not in task_text:
            return "left"
        if "右侧" in task_text and "左侧" not in task_text:
            return "right"
        # Fallback: use first climbable platform side
        platforms = [obj for obj in objects if str(obj.get("type", "")).lower() == "platform"]
        for p in platforms:
            center = p.get("center") or [0.0, 0.0, 0.0]
            return "left" if float(center[1]) >= 0 else "right"
        return None

    @staticmethod
    def _estimate_lateral_distance(side: str, objects: list[dict[str, Any]]) -> float:
        for obj in objects:
            if str(obj.get("type", "")).lower() != "platform":
                continue
            center = obj.get("center") or [0.0, 0.0, 0.0]
            obj_side = "left" if float(center[1]) >= 0 else "right"
            if obj_side == side:
                return round(abs(float(center[1])) + 0.1, 3)
        return 0.5

    @classmethod
    def _build_climb_align_goal(
        cls,
        support_box: dict[str, Any],
        target_platform: dict[str, Any],
    ) -> list[float] | None:
        """计算 climb_align 的导航目标：推到位的箱子背后（背离平台一侧）。"""
        pushed_pos = cls._build_adjacent_ground_position(support_box, target_platform)
        box_size = support_box.get("size") or [0.0, 0.0, 0.0]
        platform_center = target_platform.get("center") or [0.0, 0.0, 0.0]

        dx = float(platform_center[0]) - pushed_pos[0]
        if abs(dx) < 0.01:
            return None

        direction = 1.0 if dx > 0 else -1.0
        align_x = pushed_pos[0] - direction * (float(box_size[0]) / 2 + 0.35)
        align_y = pushed_pos[1]
        align_z = 0.0

        return [round(align_x, 3), round(align_y, 3), round(align_z, 3)]

    @staticmethod
    def _build_adjacent_ground_position(
        support_box: dict[str, Any],
        target_platform: dict[str, Any],
    ) -> list[float]:
        box_center = support_box.get("center") or [0.0, 0.0, 0.0]
        box_size = support_box.get("size") or [0.0, 0.0, 0.0]
        platform_center = target_platform.get("center") or [0.0, 0.0, 0.0]
        platform_size = target_platform.get("size") or [0.0, 0.0, 0.0]
        x = round(float(platform_center[0]) - float(platform_size[0]) / 2 - float(box_size[0]) / 2, 3)
        y = round(float(box_center[1]), 3)
        z = round(float(box_center[2]), 3)
        return [x, y, z]
