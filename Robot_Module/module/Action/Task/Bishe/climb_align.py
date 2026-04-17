from __future__ import annotations

import json
from typing import Any

from Comm_Module import get_state
from Excu_Module.executor import wait_skill_feedback
from Excu_Module.runtime import (
    DEFAULT_TARGET,
    estimate_goal_skill_duration,
    hold_idle_stability,
    load_runtime_config,
    log_skill,
    parse_goal_value,
    read_env_float,
    resolve_feedback_backend,
    speak,
)
from Excu_Module.skill_base import SkillBase
from Robot_Module.module.Action.Task.Bishe._bishe_helpers import (
    DEFAULT_NAVIGATION_DURATION_SEC,
    DEFAULT_NAVIGATION_TIMEOUT_MARGIN_SEC,
    DEFAULT_POST_ALIGN_SETTLE_SEC,
    DEFAULT_PRE_CLIMB_SETTLE_SEC,
    MODEL_USE_NAVIGATION,
)


def _load_scene_objects() -> list[dict[str, object]]:
    state = get_state()
    observation = state.get("observation") or {}
    environment = observation.get("environment") or {}
    obstacles = environment.get("obstacles")
    if isinstance(obstacles, list):
        return [item for item in obstacles if isinstance(item, dict)]
    return []


def _match_scene_object(*candidates: str) -> dict[str, object] | None:
    normalized_candidates = {
        str(candidate).strip().lower()
        for candidate in candidates
        if str(candidate).strip()
    }
    if not normalized_candidates:
        return None

    for obj in _load_scene_objects():
        identifiers = {
            str(obj.get("id", "")).strip().lower(),
            str(obj.get("source_name", "")).strip().lower(),
            str(obj.get("name", "")).strip().lower(),
        }
        if identifiers & normalized_candidates:
            return obj
    return None


def _build_climb_staging_goal(stage: str, target: str) -> tuple[list[float], dict[str, object]] | None:
    target_obj = _match_scene_object(stage, target)
    if target_obj is None or str(target_obj.get("type", "")).lower() != "platform":
        return None

    center = target_obj.get("center")
    size = target_obj.get("size")
    if not isinstance(center, list) or len(center) < 2 or not isinstance(size, list) or len(size) < 2:
        return None

    try:
        center_x = float(center[0])
        center_y = float(center[1])
        size_x = float(size[0])
    except (TypeError, ValueError):
        return None

    approach_offset = abs(read_env_float("FINALPROJECT_CLIMB_PREALIGN_OFFSET_M", 0.35))
    goal = [
        round(center_x - size_x / 2.0 - approach_offset, 3),
        round(center_y, 3),
        0.0,
        0.0,
    ]
    return goal, {
        "object_id": str(target_obj.get("id", "")).strip() or target,
        "object_type": str(target_obj.get("type", "")).strip() or "platform",
        "object_center": center,
        "object_size": size,
        "align_mode": "platform_front",
    }


class ClimbAlignSkill(SkillBase):

    @property
    def name(self) -> str:
        return "climb_align"

    async def execute(
        self,
        stage: str = "高台",
        target: str = DEFAULT_TARGET,
        goal_command: list[float] | str | None = None,
        speech: str = "",
        wait_feedback: bool = True,
    ) -> dict[str, Any]:
        resolved_goal = parse_goal_value(goal_command)
        align_meta: dict[str, object] | None = None

        if resolved_goal is None or resolved_goal == "auto":
            staging_goal = _build_climb_staging_goal(stage, target)
            if staging_goal is None:
                raise ValueError(f"无法为目标 {target} 计算攀爬前对正点")
            resolved_goal, align_meta = staging_goal
        else:
            align_meta = {
                "object_id": str(stage).strip() or "climb_staging_point",
                "object_type": "explicit_goal",
                "target_platform_id": str(target).strip() or target,
            }

        align_target = f"{align_meta.get('object_id', 'climb_staging_point')} 前对正点"
        execution_time_sec = estimate_goal_skill_duration(resolved_goal, DEFAULT_NAVIGATION_DURATION_SEC)
        timeout_sec = max(20.0, execution_time_sec + DEFAULT_NAVIGATION_TIMEOUT_MARGIN_SEC)
        config = load_runtime_config()

        if wait_feedback:
            await hold_idle_stability(
                config,
                wait_sec=read_env_float("FINALPROJECT_PRE_CLIMB_SETTLE_SEC", DEFAULT_PRE_CLIMB_SETTLE_SEC),
                reason=f"climb_align 前稳定机身，目标={target}",
            )

        speak(speech)
        log_skill(
            "climb_align",
            f"攀爬前对正到{align_target}，目标命令={resolved_goal}，预计执行 {execution_time_sec:.2f} 秒",
        )
        feedback = await wait_skill_feedback(
            "climb_align",
            {
                "stage": stage,
                "target": target,
                "goal_command": resolved_goal,
                "align_target": align_target,
            },
            f"已完成攀爬前对正到{align_target}",
            wait_feedback=wait_feedback,
            timeout_sec=timeout_sec,
            model_use=MODEL_USE_NAVIGATION,
            goal_command=resolved_goal,
            execution_time_sec=execution_time_sec,
        )

        if wait_feedback and feedback.get("signal") == "SUCCESS":
            await hold_idle_stability(
                config,
                wait_sec=read_env_float("FINALPROJECT_POST_ALIGN_SETTLE_SEC", DEFAULT_POST_ALIGN_SETTLE_SEC),
                reason=f"对正完成后再次稳定机身，目标={target}",
            )

        return {
            "skill": "climb_align",
            "stage": stage,
            "target": target,
            "goal_command": resolved_goal,
            "align_target": align_target,
            "align_meta": align_meta,
            "speech": speech,
            "action_id": feedback.get("action_id"),
            "execution_feedback": feedback,
            "execution_result": feedback.get("result", {}),
            "control_command": {
                "model_use": MODEL_USE_NAVIGATION,
                "goal": resolved_goal,
                "estimated_execution_time_sec": round(execution_time_sec, 3),
                "timeout_sec": round(timeout_sec, 3),
            },
            "backend": resolve_feedback_backend(feedback),
            "status": "success" if feedback.get("signal") == "SUCCESS" else "failure",
        }

    # climb_align uses goal-based arrival check (navigation model).
    # Climb validation (height check) applies via the climb_align name matching
    # the climb validation in state.py. We override check_completion with climb logic.
    def check_completion(
        self,
        state: dict[str, Any],
        before_state: dict[str, Any],
        parameters: dict[str, Any],
    ) -> dict[str, Any]:
        from Excu_Module.state import extract_robot_pose, minimum_verified_motion

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
        import math
        from Excu_Module.state import build_motion_validation_base, minimum_verified_motion, round_pose

        validation, before_pose, after_pose = build_motion_validation_base("climb_align", before_state, after_state)
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
                    f"climb_align 真实状态校验{'通过' if meets_requirements else '失败'}: "
                    f"机器人从 {validation['before_robot_pose']} 移动到 {validation['after_robot_pose']}，"
                    f"高度变化 {delta_z:.3f}m，平面位移 {planar_distance:.3f}m，"
                    f"要求至少抬升 {minimum_height:.3f}m。"
                ),
            }
        )
        return validation

    def calculate_parameters(
        self,
        task: dict[str, Any],
        object_facts: dict[str, Any] | None,
        context: dict[str, Any],
    ) -> dict[str, Any] | None:
        objects = (object_facts or {}).get("objects") or []
        constraints = (object_facts or {}).get("constraints") or {}
        climb_limit = float(constraints.get("max_climb_height_m", 0.3))
        current_route_side = context.get("current_route_side")
        plan_uses_push_box = context.get("plan_uses_push_box", False)

        support_box = context.get("support_box")
        target_platform = context.get("target_platform")
        current_pose = context.get("current_pose", [0.0, 0.0, 0.0])

        from Robot_Module.module.Action.Task.Bishe._bishe_helpers import (
            select_box_for_task,
            select_platform_for_task,
            object_height,
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

        stage_id = selected_platform.get("id", "高台")
        return {
            "stage": stage_id,
            "target": stage_id,
        }

    def register_tool(self, mcp) -> dict[str, Any]:
        @mcp.tool()
        async def climb_align(
            stage: str = "高台",
            target: str = DEFAULT_TARGET,
            goal_command: str = "",
            speech: str = "",
        ) -> str:
            return json.dumps(
                await self.execute(stage=stage, target=target, goal_command=goal_command, speech=speech),
                ensure_ascii=False,
            )

        return {"climb_align": climb_align}


_skill = ClimbAlignSkill()
