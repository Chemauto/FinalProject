from __future__ import annotations

from math import hypot
from typing import Any


class ParameterCalculator:
    DEFAULT_LATERAL_DISTANCE = 0.5
    WALK_DISTANCE_EPSILON = 0.05

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

        climb_count = 0
        annotated_tasks = []

        for task in tasks:
            annotated = dict(task)
            function_name = task.get("function")
            parameter_context = dict(task.get("parameter_context") or {})

            if function_name == "way_select" and route_side:
                parameter_context.update(
                    {
                        "route_side": route_side,
                        "route_side_label": self._side_label(route_side),
                    }
                )
                annotated["parameter_context"] = parameter_context
                annotated["calculated_parameters"] = {
                    "direction": route_side,
                    "lateral_distance": self.DEFAULT_LATERAL_DISTANCE,
                    "target": "前方目标点",
                }
                current_pose = self._estimate_pose_after_way_select(
                    current_pose=current_pose,
                    route_side=route_side,
                    lateral_distance=self.DEFAULT_LATERAL_DISTANCE,
                    anchor_obj=support_box or target_platform,
                )
            elif function_name == "push_box" and can_use_box_assist:
                target_position = self._build_adjacent_ground_position(support_box, target_platform)
                parameter_context.update(
                    {
                        "support_object": support_box["id"],
                        "target_object": target_platform["id"],
                        "target_position_xyz": target_position,
                    }
                )
                annotated["parameter_context"] = parameter_context
                annotated["calculated_parameters"] = {
                    "box_height": box_height,
                    "target_position": str(target_position),
                }
                current_pose = [target_position[0], target_position[1], current_pose[2]]
            elif function_name == "climb" and target_platform:
                climb_count += 1
                if can_use_box_assist and climb_count == 1:
                    parameter_context.update(
                        {
                            "support_object": support_box["id"],
                            "target_object": support_box["id"],
                            "relative_height_m": box_height,
                        }
                    )
                    annotated["parameter_context"] = parameter_context
                    annotated["calculated_parameters"] = {
                        "height": box_height,
                        "stage": support_box["id"],
                        "target": support_box["id"],
                    }
                    current_pose = self._estimate_pose_after_climb(current_pose, support_box, box_height)
                elif can_use_box_assist and climb_count >= 2:
                    parameter_context.update(
                        {
                            "support_object": support_box["id"],
                            "target_object": target_platform["id"],
                            "relative_height_m": remaining_height,
                        }
                    )
                    annotated["parameter_context"] = parameter_context
                    annotated["calculated_parameters"] = {
                        "height": remaining_height,
                        "stage": target_platform["id"],
                        "target": target_platform["id"],
                    }
                    current_pose = self._estimate_pose_after_climb(current_pose, target_platform, remaining_height)
                else:
                    parameter_context.update(
                        {
                            "target_object": target_platform["id"],
                            "relative_height_m": platform_height,
                        }
                    )
                    annotated["parameter_context"] = parameter_context
                    annotated["calculated_parameters"] = {
                        "height": platform_height,
                        "stage": target_platform["id"],
                        "target": target_platform["id"],
                    }
                    current_pose = self._estimate_pose_after_climb(current_pose, target_platform, platform_height)
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
        distance = max(cls.WALK_DISTANCE_EPSILON, distance)
        route_side = cls._infer_walk_route_label(task, current_pose, navigation_goal)
        return {
            "route_side": route_side,
            "distance": distance,
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
    def _estimate_pose_after_way_select(
        cls,
        current_pose: list[float],
        route_side: str,
        lateral_distance: float,
        anchor_obj: dict[str, Any] | None,
    ) -> list[float]:
        target_y = current_pose[1] + lateral_distance if route_side == "left" else current_pose[1] - lateral_distance
        anchor_center = cls._coerce_vec3((anchor_obj or {}).get("center"))
        if anchor_center:
            if route_side == "left":
                target_y = max(anchor_center[1], target_y)
            else:
                target_y = min(anchor_center[1], target_y)
        return [round(current_pose[0], 3), round(target_y, 3), round(current_pose[2], 3)]

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
    def _object_height(obj: dict[str, Any] | None) -> float:
        if not obj:
            return 0.0
        size = obj.get("size") or [0.0, 0.0, 0.0]
        return round(float(size[2]), 3)

    @staticmethod
    def _select_support_box(objects: list[dict[str, Any]]) -> dict[str, Any] | None:
        boxes = [
            obj
            for obj in objects
            if obj.get("movable") and str(obj.get("type", "")).lower() == "box"
        ]
        if not boxes:
            return None
        return min(boxes, key=lambda item: float((item.get("size") or [0.0, 0.0, 0.0])[2]))

    @staticmethod
    def _select_target_platform(objects: list[dict[str, Any]]) -> dict[str, Any] | None:
        platforms = [
            obj
            for obj in objects
            if str(obj.get("type", "")).lower() == "platform"
        ]
        if not platforms:
            return None
        return max(platforms, key=lambda item: float((item.get("size") or [0.0, 0.0, 0.0])[2]))

    @staticmethod
    def _infer_route_side(obj: dict[str, Any] | None) -> str | None:
        if not obj:
            return None
        center = obj.get("center") or [0.0, 0.0, 0.0]
        return "left" if float(center[1]) >= 0 else "right"

    @staticmethod
    def _side_label(side: str) -> str:
        return "左侧" if side == "left" else "右侧"

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
        y = round(float(platform_center[1]), 3)
        z = round(float(box_center[2]), 3)
        return [x, y, z]
