#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Data_Module/params.py — 参数计算器（仅几何计算）。"""

from __future__ import annotations

from typing import Any


class ParameterCalculator:
    """只做需要精确代码计算的几何参数。walk/way_select/climb 等直接透传 LLM 参数。"""

    def annotate_tasks(
        self,
        tasks: list[dict[str, Any]],
        object_facts: dict[str, Any] | None,
    ) -> list[dict[str, Any]]:
        if not tasks:
            return tasks

        objects = (object_facts or {}).get("objects") or []

        for task in tasks:
            function_name = task.get("function")
            params = dict(task.get("params") or {})

            if function_name in {"navigation", "nav_climb"}:
                goal = self._get_navigation_goal(object_facts)
                if goal:
                    params.setdefault("goal", self._format_goal(goal))
                    params.setdefault("target", self._format_target(goal))

            elif function_name == "push_box":
                box = self._select_support_box(objects)
                platform = self._select_target_platform(objects)
                if box and platform:
                    params.setdefault("target_position", "auto")
                    params.setdefault("box_height", self._object_height(box))

            elif function_name == "climb_align":
                box = self._select_support_box(objects)
                platform = self._select_target_platform(objects)
                if platform:
                    params.setdefault("stage", platform.get("id", ""))
                    params.setdefault("target", platform.get("id", ""))
                if box and platform:
                    align_goal = self._build_climb_align_goal(box, platform)
                    if align_goal:
                        params.setdefault("goal_command", ",".join(str(v) for v in align_goal))

            elif function_name == "climb":
                platform = self._select_target_platform(objects)
                if platform:
                    params.setdefault("stage", platform.get("id", ""))
                    params.setdefault("target", platform.get("id", ""))

            # walk / way_select: 直接透传，不改

            task["params"] = params

        return tasks

    # ── 简单取值 ──────────────────────────────────────────────────

    @staticmethod
    def _get_navigation_goal(object_facts: dict[str, Any] | None) -> list[float] | None:
        if not object_facts:
            return None
        goal = object_facts.get("navigation_goal")
        if isinstance(goal, list) and len(goal) >= 2:
            return [float(goal[0]), float(goal[1]), float(goal[2]) if len(goal) > 2 else 0.0]
        return None

    @staticmethod
    def _format_goal(goal: list[float]) -> str:
        return f"[{goal[0]:g}, {goal[1]:g}, {goal[2]:g}]"

    @staticmethod
    def _format_target(goal: list[float]) -> str:
        return f"目标点[{goal[0]:g}, {goal[1]:g}, {goal[2]:g}]"

    # ── 物体选择 ──────────────────────────────────────────────────

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

    # ── 几何计算 ──────────────────────────────────────────────────

    @classmethod
    def _build_climb_align_goal(
        cls,
        support_box: dict[str, Any],
        target_platform: dict[str, Any],
    ) -> list[float] | None:
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
