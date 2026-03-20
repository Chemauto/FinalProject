from __future__ import annotations

from typing import Any


class ParameterCalculator:
    DEFAULT_LATERAL_DISTANCE = 0.5

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
        support_box = self._select_support_box(objects)
        target_platform = self._select_target_platform(objects)
        route_side = self._infer_route_side(support_box or target_platform)

        box_height = self._object_height(support_box)
        platform_height = self._object_height(target_platform)
        can_use_box_assist = bool(
            support_box
            and target_platform
            and 0 < box_height <= climb_limit
            and platform_height > climb_limit
            and 0 < round(platform_height - box_height, 3) <= climb_limit
        )
        climb_count = 0
        annotated_tasks = []

        for task in tasks:
            annotated = dict(task)
            function_name = task.get("function")

            if function_name == "way_select" and route_side:
                annotated["parameter_context"] = {
                    "route_side": route_side,
                    "route_side_label": self._side_label(route_side),
                }
                annotated["calculated_parameters"] = {
                    "direction": route_side,
                    "lateral_distance": self.DEFAULT_LATERAL_DISTANCE,
                    "target": "前方目标点",
                }
            elif function_name == "push_box" and can_use_box_assist:
                target_position = self._build_adjacent_ground_position(support_box, target_platform)
                annotated["parameter_context"] = {
                    "support_object": support_box["id"],
                    "target_object": target_platform["id"],
                    "target_position_xyz": target_position,
                }
                annotated["calculated_parameters"] = {
                    "box_height": box_height,
                    "target_position": str(target_position),
                }
            elif function_name == "climb" and target_platform:
                climb_count += 1
                if can_use_box_assist and climb_count == 1:
                    annotated["parameter_context"] = {
                        "support_object": support_box["id"],
                        "target_object": support_box["id"],
                        "relative_height_m": box_height,
                    }
                    annotated["calculated_parameters"] = {
                        "height": box_height,
                        "stage": support_box["id"],
                        "target": support_box["id"],
                    }
                elif can_use_box_assist and climb_count >= 2:
                    remaining_height = round(platform_height - box_height, 3)
                    annotated["parameter_context"] = {
                        "support_object": support_box["id"],
                        "target_object": target_platform["id"],
                        "relative_height_m": remaining_height,
                    }
                    annotated["calculated_parameters"] = {
                        "height": remaining_height,
                        "stage": target_platform["id"],
                        "target": target_platform["id"],
                    }
                else:
                    annotated["parameter_context"] = {
                        "target_object": target_platform["id"],
                        "relative_height_m": platform_height,
                    }
                    annotated["calculated_parameters"] = {
                        "height": platform_height,
                        "stage": target_platform["id"],
                        "target": target_platform["id"],
                    }

            annotated_tasks.append(annotated)

        return annotated_tasks

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
