from __future__ import annotations

import json
import math
from typing import Any

from Comm_Module import get_state
from Excu_Module.executor import wait_skill_feedback
from Excu_Module.runtime import (
    DEFAULT_TARGET,
    log_skill,
    normalize_direction,
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
from Excu_Module.runtime import estimate_goal_skill_duration
from Robot_Module.module.Action.Task.Bishe._bishe_helpers import (
    DEFAULT_LATERAL_SPEED_MPS,
    DEFAULT_NAVIGATION_DURATION_SEC,
    DEFAULT_WAY_SELECT_FORWARD_BIAS_M,
    DEFAULT_WAY_SELECT_DURATION_SEC,
    DEFAULT_LATERAL_DISTANCE,
    MODEL_USE_NAVIGATION,
    MODEL_USE_WALK,
    estimate_pose_after_way_select,
    infer_route_side,
    infer_way_select_route_side,
    side_label,
)


def _extract_scene_objects() -> list[dict[str, object]]:
    state = get_state()
    observation = state.get("observation") or {}
    environment = observation.get("environment") or {}
    obstacles = environment.get("obstacles")
    if isinstance(obstacles, list):
        return [item for item in obstacles if isinstance(item, dict)]
    return []


def _extract_robot_pose() -> list[float]:
    state = get_state()
    observation = state.get("observation") or {}
    pose = observation.get("agent_position")
    if not isinstance(pose, list) or len(pose) < 3:
        return [0.0, 0.0, 0.0]
    try:
        return [float(pose[0]), float(pose[1]), float(pose[2])]
    except (TypeError, ValueError):
        return [0.0, 0.0, 0.0]


def _select_side_anchor_y(direction: str) -> float | None:
    direction_sign = 1 if direction == "left" else -1
    candidates: list[tuple[float, float]] = []

    for obj in _extract_scene_objects():
        center = obj.get("center")
        if not isinstance(center, list) or len(center) < 2:
            continue
        try:
            center_x = float(center[0])
            center_y = float(center[1])
        except (TypeError, ValueError):
            continue

        if direction_sign > 0 and center_y >= 0:
            candidates.append((abs(center_x), center_y))
        if direction_sign < 0 and center_y < 0:
            candidates.append((abs(center_x), center_y))

    if not candidates:
        return None
    candidates.sort(key=lambda item: item[0])
    return candidates[0][1]


def _resolve_way_select_navigation_goal(direction: str, lateral_distance: float) -> list[float]:
    robot_x, robot_y, robot_z = _extract_robot_pose()
    forward_bias = read_env_float("FINALPROJECT_WAY_SELECT_FORWARD_BIAS_M", DEFAULT_WAY_SELECT_FORWARD_BIAS_M)
    direction_sign = 1.0 if direction == "left" else -1.0
    anchor_y = _select_side_anchor_y(direction)

    if anchor_y is None:
        target_y = robot_y + direction_sign * lateral_distance
    elif direction == "left":
        target_y = max(anchor_y, robot_y + lateral_distance)
    else:
        target_y = min(anchor_y, robot_y - lateral_distance)

    return [round(robot_x + forward_bias, 3), round(target_y, 3), round(robot_z, 3)]


class WaySelectSkill(SkillBase):

    @property
    def name(self) -> str:
        return "way_select"

    async def execute(
        self,
        direction: str = "left",
        lateral_distance: float = 0.5,
        target: str = DEFAULT_TARGET,
        speech: str = "",
        wait_feedback: bool = True,
    ) -> dict[str, Any]:
        from Excu_Module.runtime import load_runtime_config

        normalized_direction = normalize_direction(direction)
        direction_label = "左侧" if normalized_direction == "left" else "右侧"

        if lateral_distance <= 0:
            raise ValueError("lateral_distance 必须大于 0")
        if not speech:
            speech = f"机器人当前位于中间位置，先切换到{direction_label}路线。"

        config = load_runtime_config()
        control_mode = config.way_select_policy
        goal_command: list[float] | str | None = None
        velocity_command: list[float] | None = None
        model_use = MODEL_USE_WALK

        if control_mode == "navigation":
            model_use = MODEL_USE_NAVIGATION
            goal_command = _resolve_way_select_navigation_goal(normalized_direction, lateral_distance)
            execution_time_sec = estimate_goal_skill_duration(goal_command, DEFAULT_NAVIGATION_DURATION_SEC)
            base_skill_call = {
                "skill": "navigation",
                "goal": goal_command,
                "target": f"{direction_label}路线入口",
            }
        else:
            lateral_speed = abs(DEFAULT_LATERAL_SPEED_MPS)
            signed_lateral_speed = lateral_speed if normalized_direction == "left" else -lateral_speed
            velocity_command = [0.0, signed_lateral_speed, 0.0]
            execution_time_sec = DEFAULT_WAY_SELECT_DURATION_SEC
            base_skill_call = {
                "skill": "walk",
                "route_side": f"{direction_label}路线入口",
                "distance": round(lateral_speed * execution_time_sec, 3),
                "requested_lateral_distance": lateral_distance,
                "target": f"{direction_label}路线入口",
                "velocity_command": velocity_command,
                "execution_time_sec": execution_time_sec,
            }

        speak(speech)
        log_skill(
            "way_select",
            f"选择{direction_label}路线，控制模式={control_mode}，目标={target}，速度命令={velocity_command}，预计执行 {execution_time_sec:.2f} 秒",
        )
        feedback = await wait_skill_feedback(
            "way_select",
            {
                "direction": normalized_direction,
                "lateral_distance": lateral_distance,
                "target": target,
                "control_mode": control_mode,
                "velocity_command": velocity_command,
                "goal_command": goal_command,
            },
            f"已成功切换到{direction_label}路线",
            wait_feedback=wait_feedback,
            model_use=model_use,
            velocity_command=velocity_command,
            goal_command=goal_command,
            execution_time_sec=execution_time_sec,
        )
        return {
            "skill": "way_select",
            "direction": normalized_direction,
            "lateral_distance": lateral_distance,
            "target": target,
            "speech": speech,
            "base_skill_call": base_skill_call,
            "action_id": feedback.get("action_id"),
            "execution_feedback": feedback,
            "execution_result": feedback.get("result", {}),
            "control_command": {
                "model_use": model_use,
                "velocity": velocity_command,
                "goal": goal_command,
                "control_mode": control_mode,
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

        requested_distance = max(0.0, float(parameters.get("lateral_distance") or 0.0))
        minimum_distance = minimum_verified_motion(
            requested_distance,
            ratio_env="FINALPROJECT_WAY_SELECT_VERIFY_RATIO",
            ratio_default=0.7,
            abs_tol_env="FINALPROJECT_WAY_SELECT_VERIFY_ABS_TOL_M",
            abs_tol_default=0.12,
        )

        direction = str(parameters.get("direction") or "").strip().lower()
        direction_sign = 1.0 if direction == "left" else -1.0

        delta_y = current_pose[1] - before_pose[1]
        lateral_progress = delta_y * direction_sign
        planar_distance = math.hypot(
            current_pose[0] - before_pose[0],
            current_pose[1] - before_pose[1],
        )
        minimum_lateral_progress = max(0.05, minimum_distance * 0.8)
        done = planar_distance >= minimum_distance and lateral_progress >= minimum_lateral_progress

        return {
            "done": done,
            "lateral_progress": round(lateral_progress, 3),
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
        validation, before_pose, after_pose = build_motion_validation_base("way_select", before_state, after_state)
        if before_pose is None or after_pose is None:
            return validation

        requested_distance = max(0.0, float(parameters.get("lateral_distance") or 0.0))
        minimum_distance = minimum_verified_motion(
            requested_distance,
            ratio_env="FINALPROJECT_WAY_SELECT_VERIFY_RATIO",
            ratio_default=0.7,
            abs_tol_env="FINALPROJECT_WAY_SELECT_VERIFY_ABS_TOL_M",
            abs_tol_default=0.12,
        )
        direction = str(parameters.get("direction") or "").strip().lower()
        direction_sign = 1.0 if direction == "left" else -1.0
        direction_label = "左侧" if direction_sign > 0 else "右侧"
        delta_x = after_pose[0] - before_pose[0]
        delta_y = after_pose[1] - before_pose[1]
        delta_z = after_pose[2] - before_pose[2]
        planar_distance = math.hypot(delta_x, delta_y)
        lateral_progress = delta_y * direction_sign
        minimum_lateral_progress = max(0.05, minimum_distance * 0.8)
        meets_requirements = planar_distance >= minimum_distance and lateral_progress >= minimum_lateral_progress
        validation.update(
            {
                "meets_requirements": meets_requirements,
                "delta_pose": round_pose([delta_x, delta_y, delta_z]),
                "planar_distance": round(planar_distance, 3),
                "required_lateral_distance": round(requested_distance, 3),
                "minimum_required_distance": round(minimum_distance, 3),
                "lateral_progress": round(lateral_progress, 3),
                "direction_label": direction_label,
                "summary": (
                    f"way_select 真实状态校验{'通过' if meets_requirements else '失败'}: "
                    f"机器人从 {validation['before_robot_pose']} 移动到 {validation['after_robot_pose']}，"
                    f"平面位移 {planar_distance:.3f}m，{direction_label}方向位移 {lateral_progress:.3f}m，"
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
        current_route_side = context.get("current_route_side")
        current_pose = context.get("current_pose", [0.0, 0.0, 0.0])

        objects = (object_facts or {}).get("objects") or []
        from Robot_Module.module.Action.Task.Bishe._bishe_helpers import (
            select_support_box,
            select_target_platform,
        )
        support_box = context.get("support_box") or select_support_box(objects)
        target_platform = context.get("target_platform") or select_target_platform(objects)

        selected_route_side = infer_way_select_route_side(task, current_route_side)
        if not selected_route_side:
            return None

        context["current_pose"] = estimate_pose_after_way_select(
            current_pose=current_pose,
            route_side=selected_route_side,
            lateral_distance=DEFAULT_LATERAL_DISTANCE,
            anchor_obj=support_box or target_platform,
        )
        context["current_route_side"] = selected_route_side
        return {
            "direction": selected_route_side,
            "lateral_distance": DEFAULT_LATERAL_DISTANCE,
            "target": "前方目标点",
        }

    def register_tool(self, mcp) -> dict[str, Any]:
        @mcp.tool()
        async def way_select(
            direction: str,
            lateral_distance: float = 0.5,
            target: str = DEFAULT_TARGET,
            speech: str = "",
        ) -> str:
            return json.dumps(
                await self.execute(
                    direction=direction,
                    lateral_distance=lateral_distance,
                    target=target,
                    speech=speech,
                ),
                ensure_ascii=False,
            )

        return {"way_select": way_select}


_skill = WaySelectSkill()
