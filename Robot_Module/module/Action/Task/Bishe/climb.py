from __future__ import annotations

import json
import math
from typing import Any

from Excu_Module.executor import wait_skill_feedback
from Excu_Module.runtime import (
    DEFAULT_TARGET,
    log_skill,
    read_env_float,
    resolve_feedback_backend,
    speak,
)
from Excu_Module.skill_base import SkillBase
from Excu_Module.state import (
    build_motion_validation_base,
    extract_robot_pose,
    minimum_verified_motion,
    round_pose,
)
from Robot_Module.module.Action.Task.Bishe._bishe_helpers import (
    CLIMB_LIMIT_METERS,
    DEFAULT_CLIMB_EXECUTION_SEC,
    DEFAULT_CLIMB_SPEED_MPS,
    MODEL_USE_CLIMB,
    estimate_pose_after_climb,
    object_height,
    infer_route_side,
)
import re


class ClimbSkill(SkillBase):

    @property
    def name(self) -> str:
        return "climb"

    async def execute(
        self,
        height: float = 0.0,
        stage: str = "高台",
        target: str = DEFAULT_TARGET,
        speech: str = "",
        wait_feedback: bool = True,
    ) -> dict[str, Any]:
        if height <= 0:
            raise ValueError("height 必须大于 0")
        if height > CLIMB_LIMIT_METERS:
            raise ValueError(f"当前 demo 约束为最大攀爬高度 {CLIMB_LIMIT_METERS:.2f} 米，收到 {height:.2f} 米")

        climb_speed = abs(read_env_float("FINALPROJECT_CLIMB_SPEED_MPS", DEFAULT_CLIMB_SPEED_MPS))
        velocity_command = [climb_speed, 0.0, 0.0]
        execution_time_sec = max(0.5, read_env_float("FINALPROJECT_CLIMB_DURATION_SEC", DEFAULT_CLIMB_EXECUTION_SEC))

        speak(speech)
        log_skill(
            "climb",
            f"攀爬{stage}，高度 {height:.2f} 米，目标={target}，速度命令={velocity_command}，预计执行 {execution_time_sec:.2f} 秒",
        )
        feedback = await wait_skill_feedback(
            "climb",
            {
                "height": height,
                "stage": stage,
                "target": target,
                "velocity_command": velocity_command,
            },
            f"已完成{stage}" if "攀爬" in stage else f"已完成{stage}攀爬",
            wait_feedback=wait_feedback,
            model_use=MODEL_USE_CLIMB,
            velocity_command=velocity_command,
            execution_time_sec=execution_time_sec,
        )
        return {
            "skill": "climb",
            "height": height,
            "stage": stage,
            "target": target,
            "speech": speech,
            "action_id": feedback.get("action_id"),
            "execution_feedback": feedback,
            "execution_result": feedback.get("result", {}),
            "control_command": {
                "model_use": MODEL_USE_CLIMB,
                "velocity": velocity_command,
                "estimated_execution_time_sec": round(execution_time_sec, 3),
            },
            "backend": resolve_feedback_backend(feedback),
            "status": "success" if feedback.get("signal") == "SUCCESS" else "failure",
        }

    def check_completion(
        self,
        state: dict[str, Any],
        before_state: dict[str, Any],
        parameters: dict[str, Any],
    ) -> dict[str, Any]:
        before_pose = extract_robot_pose(before_state)
        current_pose = extract_robot_pose(state)
        if current_pose is None or before_pose is None:
            return {"done": False}

        requested_height = max(0.0, float(parameters.get("height") or 0.0))
        minimum_height = minimum_verified_motion(
            requested_height,
            ratio_env="FINALPROJECT_CLIMB_VERIFY_RATIO",
            ratio_default=0.6,
            abs_tol_env="FINALPROJECT_CLIMB_VERIFY_ABS_TOL_M",
            abs_tol_default=0.08,
        )

        delta_z = current_pose[2] - before_pose[2]
        done = delta_z >= minimum_height

        return {
            "done": done,
            "vertical_progress": round(delta_z, 3),
            "minimum_height": round(minimum_height, 3),
            "current_pose": current_pose,
        }

    def validate(
        self,
        parameters: dict[str, Any],
        before_state: dict[str, Any] | None,
        after_state: dict[str, Any] | None,
    ) -> dict[str, Any]:
        validation, before_pose, after_pose = build_motion_validation_base("climb", before_state, after_state)
        if before_pose is None or after_pose is None:
            return validation

        requested_height = max(0.0, float(parameters.get("height") or 0.0))
        minimum_height = minimum_verified_motion(
            requested_height,
            ratio_env="FINALPROJECT_CLIMB_VERIFY_RATIO",
            ratio_default=0.6,
            abs_tol_env="FINALPROJECT_CLIMB_VERIFY_ABS_TOL_M",
            abs_tol_default=0.08,
        )
        delta_x = after_pose[0] - before_pose[0]
        delta_y = after_pose[1] - before_pose[1]
        delta_z = after_pose[2] - before_pose[2]
        planar_distance = math.hypot(delta_x, delta_y)
        meets_requirements = delta_z >= minimum_height
        validation.update(
            {
                "meets_requirements": meets_requirements,
                "delta_pose": round_pose([delta_x, delta_y, delta_z]),
                "planar_distance": round(planar_distance, 3),
                "required_height": round(requested_height, 3),
                "minimum_required_height": round(minimum_height, 3),
                "vertical_progress": round(delta_z, 3),
                "summary": (
                    f"climb 真实状态校验{'通过' if meets_requirements else '失败'}: "
                    f"机器人从 {validation['before_robot_pose']} 移动到 {validation['after_robot_pose']}，"
                    f"高度变化 {delta_z:.3f}m，平面位移 {planar_distance:.3f}m，"
                    f"要求至少抬升 {minimum_height:.3f}m。"
                ),
            }
        )
        return validation

    def normalize_tool_arguments(
        self,
        function_args: dict[str, Any],
        task_description: str,
        previous_result: Any,
    ) -> tuple[dict[str, Any], str | None]:
        """Climb-specific argument correction: split relative heights."""
        requested = self._to_float(function_args.get("height"))
        if requested is None:
            return function_args, None

        candidate_value = self._normalize_relative_numeric_argument(
            requested=requested,
            task_description=task_description,
            previous_result=previous_result,
            upper_bound=CLIMB_LIMIT_METERS,
            context_keyword="箱子",
            candidate_keys=("height", "box_height"),
        )
        if candidate_value is None:
            return function_args, None

        repaired_args = dict(function_args)
        repaired_args["height"] = candidate_value
        message = (
            f"climb 参数 height 从 {requested:.2f} 米修正为 {candidate_value:.2f} 米，"
            f"按单步相对高度执行（上限 {CLIMB_LIMIT_METERS:.2f} 米）"
        )
        return repaired_args, message

    @staticmethod
    def _to_float(value: Any) -> float | None:
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _extract_numeric_values(text: str, unit: str = "米") -> list[float]:
        if not text:
            return []
        return [float(v) for v in re.findall(rf"(\d+(?:\.\d+)?)\s*{unit}", text)]

    @classmethod
    def _extract_previous_numeric_value(
        cls,
        previous_result: Any,
        candidate_keys: tuple[str, ...],
    ) -> float | None:
        if not isinstance(previous_result, dict):
            return None
        for key in candidate_keys:
            value = cls._to_float(previous_result.get(key))
            if value is not None and value > 0:
                return value
        return None

    @classmethod
    def _normalize_relative_numeric_argument(
        cls,
        requested: float,
        task_description: str,
        previous_result: Any,
        upper_bound: float,
        context_keyword: str | None = None,
        candidate_keys: tuple[str, ...] = (),
    ) -> float | None:
        if requested <= upper_bound:
            return None
        numeric_values = cls._extract_numeric_values(task_description)
        previous_value = cls._extract_previous_numeric_value(previous_result, candidate_keys)
        candidate_value = None
        if context_keyword and context_keyword not in task_description:
            return None
        if numeric_values:
            target_value = max(numeric_values)
            if previous_value is not None and target_value > previous_value:
                remaining_value = round(target_value - previous_value, 3)
                if 0 < remaining_value <= upper_bound:
                    candidate_value = remaining_value
            if candidate_value is None and len(numeric_values) >= 2:
                base_value = min(numeric_values)
                remaining_value = round(max(numeric_values) - base_value, 3)
                if 0 < remaining_value <= upper_bound:
                    candidate_value = remaining_value
        return candidate_value

    def calculate_parameters(
        self,
        task: dict[str, Any],
        object_facts: dict[str, Any] | None,
        context: dict[str, Any],
    ) -> dict[str, Any] | None:
        objects = (object_facts or {}).get("objects") or []
        constraints = (object_facts or {}).get("constraints") or {}
        climb_limit = float(constraints.get("max_climb_height_m", CLIMB_LIMIT_METERS))
        current_pose = context.get("current_pose", [0.0, 0.0, 0.0])
        current_route_side = context.get("current_route_side")
        plan_uses_push_box = context.get("plan_uses_push_box", False)
        total_climb_steps = context.get("total_climb_steps", 0)

        support_box = context.get("support_box")
        target_platform = context.get("target_platform")

        from Robot_Module.module.Action.Task.Bishe._bishe_helpers import (
            select_box_for_task,
            select_platform_for_task,
            task_targets_box,
            task_targets_platform,
        )

        selected_support_box = select_box_for_task(task, objects, current_route_side, support_box, current_pose)
        selected_platform = select_platform_for_task(task, objects, current_route_side, target_platform, current_pose)
        selected_box_height = object_height(selected_support_box)
        selected_platform_height = object_height(selected_platform)
        selected_remaining_height = round(selected_platform_height - selected_box_height, 3)
        selected_can_use_box_assist = bool(
            plan_uses_push_box
            and selected_support_box
            and selected_platform
            and 0 < selected_box_height <= climb_limit
            and selected_platform_height > climb_limit
            and 0 < selected_remaining_height <= climb_limit
        )

        if not selected_platform:
            return None

        climb_count = context.get("climb_count", 0) + 1
        context["climb_count"] = climb_count

        if selected_can_use_box_assist and selected_support_box and total_climb_steps <= 1:
            height = selected_remaining_height
            stage_id = selected_platform.get("id", selected_platform.get("id", "高台"))
            context["current_pose"] = estimate_pose_after_climb(current_pose, selected_platform, selected_remaining_height)
            context["current_route_side"] = infer_route_side(selected_platform) or current_route_side
            return {
                "height": height,
                "stage": stage_id,
                "target": stage_id,
            }

        if selected_can_use_box_assist and selected_support_box and (
            task_targets_box(task, selected_support_box) or (climb_count == 1 and not task_targets_platform(task, selected_platform))
        ):
            height = selected_box_height
            stage_id = selected_support_box.get("id", "箱子")
            context["current_pose"] = estimate_pose_after_climb(current_pose, selected_support_box, selected_box_height)
            context["current_route_side"] = infer_route_side(selected_support_box) or current_route_side
            return {
                "height": height,
                "stage": stage_id,
                "target": stage_id,
            }

        if selected_can_use_box_assist and climb_count >= 2:
            height = selected_remaining_height
            stage_id = selected_platform.get("id", "高台")
            context["current_pose"] = estimate_pose_after_climb(current_pose, selected_platform, selected_remaining_height)
            context["current_route_side"] = infer_route_side(selected_platform) or current_route_side
            return {
                "height": height,
                "stage": stage_id,
                "target": stage_id,
            }

        height = selected_platform_height
        stage_id = selected_platform.get("id", "高台")
        context["current_pose"] = estimate_pose_after_climb(current_pose, selected_platform, selected_platform_height)
        context["current_route_side"] = infer_route_side(selected_platform) or current_route_side
        return {
            "height": height,
            "stage": stage_id,
            "target": stage_id,
        }

    def register_tool(self, mcp) -> dict[str, Any]:
        @mcp.tool()
        async def climb(
            height: float,
            stage: str = "高台",
            target: str = DEFAULT_TARGET,
            speech: str = "",
        ) -> str:
            return json.dumps(
                await self.execute(height=height, stage=stage, target=target, speech=speech),
                ensure_ascii=False,
            )

        return {"climb": climb}


_skill = ClimbSkill()
