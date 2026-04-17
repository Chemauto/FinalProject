from __future__ import annotations

import json
import math
from typing import Any

from Excu_Module.executor import wait_skill_feedback
from Excu_Module.runtime import (
    DEFAULT_TARGET,
    estimate_linear_duration,
    log_skill,
    read_env_float,
    resolve_feedback_backend,
    speak,
)
from Excu_Module.skill_base import SkillBase
from Excu_Module.state import (
    build_motion_validation_base,
    build_missing_validation,
    extract_robot_pose,
    minimum_verified_motion,
    round_pose,
)
from Robot_Module.module.Action.Task.Bishe._bishe_helpers import (
    DEFAULT_WALK_SPEED_MPS,
    MODEL_USE_WALK,
    WALK_DISTANCE_EPSILON,
    coerce_vec3,
    extract_requested_distance,
    format_navigation_target,
    infer_walk_route_label,
)


class WalkSkill(SkillBase):

    @property
    def name(self) -> str:
        return "walk"

    async def execute(
        self,
        route_side: str = "前方",
        distance: float = 1.0,
        target: str = DEFAULT_TARGET,
        speech: str = "",
        wait_feedback: bool = True,
    ) -> dict[str, Any]:
        if distance <= 0:
            raise ValueError("distance 必须大于 0")

        walk_speed = abs(read_env_float("FINALPROJECT_WALK_SPEED_MPS", DEFAULT_WALK_SPEED_MPS))
        velocity_command = [walk_speed, 0.0, 0.0]
        execution_time_sec = estimate_linear_duration(distance, walk_speed)

        speak(speech)
        log_skill(
            "walk",
            f"沿{route_side}路径行走 {distance:.2f} 米，目标={target}，速度命令={velocity_command}，预计执行 {execution_time_sec:.2f} 秒",
        )
        feedback = await wait_skill_feedback(
            "walk",
            {
                "route_side": route_side,
                "distance": distance,
                "target": target,
                "velocity_command": velocity_command,
            },
            f"已完成沿{route_side}路线行走",
            wait_feedback=wait_feedback,
            model_use=MODEL_USE_WALK,
            velocity_command=velocity_command,
            execution_time_sec=execution_time_sec,
        )
        return {
            "skill": "walk",
            "route_side": route_side,
            "distance": distance,
            "target": target,
            "speech": speech,
            "action_id": feedback.get("action_id"),
            "execution_feedback": feedback,
            "execution_result": feedback.get("result", {}),
            "control_command": {
                "model_use": MODEL_USE_WALK,
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

        requested_distance = max(0.0, float(parameters.get("distance") or 0.0))
        minimum_distance = minimum_verified_motion(
            requested_distance,
            ratio_env="FINALPROJECT_WALK_VERIFY_RATIO",
            ratio_default=0.75,
            abs_tol_env="FINALPROJECT_WALK_VERIFY_ABS_TOL_M",
            abs_tol_default=0.15,
        )

        delta_x = current_pose[0] - before_pose[0]
        delta_y = current_pose[1] - before_pose[1]
        planar_distance = math.hypot(delta_x, delta_y)

        route_side = str(parameters.get("route_side") or "前方").strip().lower()
        axis = "x"
        direction_sign = 1.0
        if any(token in route_side for token in ("left", "左")):
            axis = "y"
        elif any(token in route_side for token in ("right", "右")):
            axis = "y"
            direction_sign = -1.0

        signed_progress = (delta_x if axis == "x" else delta_y) * direction_sign
        minimum_direction_progress = max(0.05, minimum_distance * 0.6)
        done = planar_distance >= minimum_distance and signed_progress >= minimum_direction_progress

        return {
            "done": done,
            "planar_distance": round(planar_distance, 3),
            "minimum_distance": round(minimum_distance, 3),
            "current_pose": current_pose,
        }

    def validate(
        self,
        parameters: dict[str, Any],
        before_state: dict[str, Any] | None,
        after_state: dict[str, Any] | None,
    ) -> dict[str, Any]:
        validation, before_pose, after_pose = build_motion_validation_base("walk", before_state, after_state)
        if before_pose is None or after_pose is None:
            return validation

        requested_distance = max(0.0, float(parameters.get("distance") or 0.0))
        minimum_distance = minimum_verified_motion(
            requested_distance,
            ratio_env="FINALPROJECT_WALK_VERIFY_RATIO",
            ratio_default=0.75,
            abs_tol_env="FINALPROJECT_WALK_VERIFY_ABS_TOL_M",
            abs_tol_default=0.15,
        )
        delta_x = after_pose[0] - before_pose[0]
        delta_y = after_pose[1] - before_pose[1]
        delta_z = after_pose[2] - before_pose[2]
        planar_distance = math.hypot(delta_x, delta_y)

        route_side = str(parameters.get("route_side") or "前方").strip().lower()
        axis = "x"
        direction_sign = 1.0
        direction_label = "前方"
        if any(token in route_side for token in ("left", "左")):
            axis = "y"
            direction_label = "左侧"
        elif any(token in route_side for token in ("right", "右")):
            axis = "y"
            direction_sign = -1.0
            direction_label = "右侧"

        signed_progress = (delta_x if axis == "x" else delta_y) * direction_sign
        minimum_direction_progress = max(0.05, minimum_distance * 0.6)
        meets_requirements = planar_distance >= minimum_distance and signed_progress >= minimum_direction_progress
        validation.update(
            {
                "meets_requirements": meets_requirements,
                "delta_pose": round_pose([delta_x, delta_y, delta_z]),
                "planar_distance": round(planar_distance, 3),
                "required_distance": round(requested_distance, 3),
                "minimum_required_distance": round(minimum_distance, 3),
                "direction_axis": axis,
                "direction_label": direction_label,
                "direction_progress": round(signed_progress, 3),
                "summary": (
                    f"walk 真实状态校验{'通过' if meets_requirements else '失败'}: "
                    f"机器人从 {validation['before_robot_pose']} 移动到 {validation['after_robot_pose']}，"
                    f"平面位移 {planar_distance:.3f}m，{direction_label}方向有效位移 {signed_progress:.3f}m，"
                    f"要求至少 {minimum_distance:.3f}m。"
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
        current_pose = context.get("current_pose", [0.0, 0.0, 0.0])
        navigation_goal = context.get("navigation_goal")

        requested_distance = extract_requested_distance(task)
        if requested_distance is not None:
            result = {
                "route_side": "前方",
                "distance": max(WALK_DISTANCE_EPSILON, round(requested_distance, 3)),
                "target": task.get("task", "前方"),
            }
            context["current_pose"] = current_pose
            return result

        if not navigation_goal:
            return None

        distance = round(math.hypot(navigation_goal[0] - current_pose[0], navigation_goal[1] - current_pose[1]), 3)
        distance = max(WALK_DISTANCE_EPSILON, distance)
        route_side = infer_walk_route_label(task, current_pose, navigation_goal)
        context["current_pose"] = list(navigation_goal)
        return {
            "route_side": route_side,
            "distance": distance,
            "target": format_navigation_target(navigation_goal),
        }

    def register_tool(self, mcp) -> dict[str, Any]:
        @mcp.tool()
        async def walk(
            route_side: str = "前方",
            distance: float = 1.0,
            target: str = DEFAULT_TARGET,
            speech: str = "",
        ) -> str:
            return json.dumps(
                await self.execute(route_side=route_side, distance=distance, target=target, speech=speech),
                ensure_ascii=False,
            )

        return {"walk": walk}


_skill = WalkSkill()
