from __future__ import annotations

import json
import math
from typing import Any

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
from Excu_Module.state import (
    build_missing_validation,
    build_motion_validation_base,
    extract_box_pose,
    extract_robot_pose,
    extract_scene_objects,
    minimum_verified_motion,
    round_pose,
    summarize_state,
)
from Robot_Module.module.Action.Task.Bishe._bishe_helpers import (
    DEFAULT_PUSH_BOX_DURATION_SEC,
    DEFAULT_POST_PUSH_SETTLE_SEC,
    MODEL_USE_PUSH_BOX,
    build_adjacent_ground_position,
    object_height,
)


class PushBoxSkill(SkillBase):

    @property
    def name(self) -> str:
        return "push_box"

    async def execute(
        self,
        box_height: float = 0.0,
        target_position: str = "高台旁边",
        speech: str = "",
        wait_feedback: bool = True,
    ) -> dict[str, Any]:
        if box_height <= 0:
            raise ValueError("box_height 必须大于 0")

        parsed_goal_command = parse_goal_value(target_position)
        auto_target = parsed_goal_command is None or parsed_goal_command == "auto"
        goal_command = "auto" if auto_target else parsed_goal_command
        target_label = "默认自动目标" if auto_target else str(target_position)
        execution_time_sec = estimate_goal_skill_duration(goal_command, DEFAULT_PUSH_BOX_DURATION_SEC)

        speak(speech)
        log_skill("push_box", f"推动高度 {box_height:.2f} 米的箱子到{target_label}，目标命令={goal_command}")
        feedback = await wait_skill_feedback(
            "push_box",
            {
                "box_height": box_height,
                "target_position": "auto" if auto_target else target_position,
                "goal_command": goal_command,
            },
            f"已完成推箱子到{target_label}",
            wait_feedback=wait_feedback,
            model_use=MODEL_USE_PUSH_BOX,
            goal_command=goal_command,
            execution_time_sec=execution_time_sec,
        )
        status = "success" if feedback.get("signal") == "SUCCESS" else "failure"
        if wait_feedback and status == "success":
            await hold_idle_stability(
                load_runtime_config(),
                wait_sec=read_env_float("FINALPROJECT_POST_PUSH_SETTLE_SEC", DEFAULT_POST_PUSH_SETTLE_SEC),
                reason="push_box 完成后等待姿态和速度收敛",
            )
        return {
            "skill": "push_box",
            "box_height": box_height,
            "target_position": "auto" if auto_target else target_position,
            "speech": speech,
            "action_id": feedback.get("action_id"),
            "execution_feedback": feedback,
            "execution_result": feedback.get("result", {}),
            "control_command": {
                "model_use": MODEL_USE_PUSH_BOX,
                "goal": goal_command,
                "estimated_execution_time_sec": round(execution_time_sec, 3),
            },
            "backend": resolve_feedback_backend(feedback),
            "status": status,
        }

    @staticmethod
    def _estimate_push_expected_distance(
        before_box_pose: list[float],
        state: dict[str, Any],
        parameters: dict[str, Any],
    ) -> float:
        goal_command = parameters.get("goal_command")
        if isinstance(goal_command, list) and len(goal_command) >= 2:
            return math.hypot(
                float(goal_command[0]) - before_box_pose[0],
                float(goal_command[1]) - before_box_pose[1],
            )

        scene_objects = extract_scene_objects(state)
        min_dist = float("inf")
        for obj in scene_objects:
            obj_type = str(obj.get("type", "")).lower()
            if obj_type != "platform":
                continue
            center = obj.get("center")
            size = obj.get("size")
            if not isinstance(center, list) or len(center) < 2:
                continue
            if not isinstance(size, list) or len(size) < 2:
                continue
            try:
                cx, cy = float(center[0]), float(center[1])
                sx, sy = float(size[0]), float(size[1])
                nearest_x = max(cx - sx / 2.0, min(before_box_pose[0], cx + sx / 2.0))
                nearest_y = max(cy - sy / 2.0, min(before_box_pose[1], cy + sy / 2.0))
                dist = math.hypot(before_box_pose[0] - nearest_x, before_box_pose[1] - nearest_y)
                min_dist = min(min_dist, dist)
            except (TypeError, ValueError):
                continue

        return min_dist if min_dist < float("inf") else 0.5

    def check_completion(
        self,
        state: dict[str, Any],
        before_state: dict[str, Any],
        parameters: dict[str, Any],
    ) -> dict[str, Any]:
        before_box = extract_box_pose(before_state)
        current_box = extract_box_pose(state)
        if current_box is None or before_box is None:
            return {"done": False}

        box_delta_x = current_box[0] - before_box[0]
        box_delta_y = current_box[1] - before_box[1]
        box_planar_distance = math.hypot(box_delta_x, box_delta_y)
        arrival_tol = abs(read_env_float("FINALPROJECT_PUSH_BOX_ARRIVAL_TOL_M", 0.1))

        goal_command = parameters.get("goal_command")
        box_goal_dist_xy: float | None = None

        if isinstance(goal_command, list) and len(goal_command) >= 2:
            box_goal_dist_xy = math.hypot(
                current_box[0] - float(goal_command[0]),
                current_box[1] - float(goal_command[1]),
            )
            done = box_goal_dist_xy <= arrival_tol
        else:
            expected_distance = self._estimate_push_expected_distance(before_box, state, parameters)
            minimum_box_motion = minimum_verified_motion(
                expected_distance,
                ratio_env="FINALPROJECT_PUSH_BOX_VERIFY_RATIO",
                ratio_default=0.75,
                abs_tol_env="FINALPROJECT_PUSH_BOX_VERIFY_ABS_TOL_M",
                abs_tol_default=0.15,
            )
            done = box_planar_distance >= minimum_box_motion

        return {
            "done": done,
            "box_planar_distance": round(box_planar_distance, 3),
            "arrival_tol": round(arrival_tol, 3),
            "current_box_pose": current_box,
            "box_goal_dist_xy": round(box_goal_dist_xy, 3) if box_goal_dist_xy is not None else None,
        }

    def validate(
        self,
        parameters: dict[str, Any],
        before_state: dict[str, Any] | None,
        after_state: dict[str, Any] | None,
    ) -> dict[str, Any]:
        before_box = extract_box_pose(before_state)
        after_box = extract_box_pose(after_state)

        if before_box is None or after_box is None:
            validation, before_pose, after_pose = build_motion_validation_base("push_box", before_state, after_state)
            if before_pose is None or after_pose is None:
                return validation

            delta_x = after_pose[0] - before_pose[0]
            delta_y = after_pose[1] - before_pose[1]
            delta_z = after_pose[2] - before_pose[2]
            planar_distance = math.hypot(delta_x, delta_y)
            minimum_robot_motion = abs(read_env_float("FINALPROJECT_PUSH_BOX_VERIFY_MIN_ROBOT_MOVE_M", 0.3))
            meets_requirements = planar_distance >= minimum_robot_motion
            validation.update(
                {
                    "meets_requirements": meets_requirements,
                    "validation_mode": "robot_fallback",
                    "delta_pose": round_pose([delta_x, delta_y, delta_z]),
                    "planar_distance": round(planar_distance, 3),
                    "minimum_robot_motion": round(minimum_robot_motion, 3),
                    "summary": (
                        "push_box 校验(后备-机器人位移): "
                        f"无法获取箱子位置，机器人从 {validation['before_robot_pose']} 移动到 "
                        f"{validation['after_robot_pose']}，平面位移 {planar_distance:.3f}m，"
                        f"最小运动要求 {minimum_robot_motion:.3f}m。"
                    ),
                }
            )
            return validation

        box_delta_x = after_box[0] - before_box[0]
        box_delta_y = after_box[1] - before_box[1]
        box_delta_z = after_box[2] - before_box[2]
        box_planar_distance = math.hypot(box_delta_x, box_delta_y)
        arrival_tol = abs(read_env_float("FINALPROJECT_PUSH_BOX_ARRIVAL_TOL_M", 0.1))

        goal_command = parameters.get("goal_command")
        box_goal_dist_xy: float | None = None
        check_mode = ""

        if isinstance(goal_command, list) and len(goal_command) >= 2:
            box_goal_dist_xy = math.hypot(after_box[0] - float(goal_command[0]), after_box[1] - float(goal_command[1]))
            meets_requirements = box_goal_dist_xy <= arrival_tol
            check_mode = f"目标到达校验: 箱子距目标 {box_goal_dist_xy:.3f}m，容许误差 {arrival_tol:.3f}m"
        else:
            expected_distance = self._estimate_push_expected_distance(before_box, before_state, parameters)
            minimum_box_motion = minimum_verified_motion(
                expected_distance,
                ratio_env="FINALPROJECT_PUSH_BOX_VERIFY_RATIO",
                ratio_default=0.75,
                abs_tol_env="FINALPROJECT_PUSH_BOX_VERIFY_ABS_TOL_M",
                abs_tol_default=0.15,
            )
            meets_requirements = box_planar_distance >= minimum_box_motion
            check_mode = f"位移校验: 箱子位移 {box_planar_distance:.3f}m，预期距离 {expected_distance:.3f}m，最小要求 {minimum_box_motion:.3f}m"

        before_robot = extract_robot_pose(before_state)
        after_robot = extract_robot_pose(after_state)
        robot_info = ""
        if before_robot is not None and after_robot is not None:
            r_delta_x = after_robot[0] - before_robot[0]
            r_delta_y = after_robot[1] - before_robot[1]
            robot_planar = math.hypot(r_delta_x, r_delta_y)
            robot_info = f" 机器人从 {before_robot} 移动到 {after_robot}，平面位移 {robot_planar:.3f}m。"

        summary = (
            f"push_box 真实状态校验{'通过' if meets_requirements else '失败'}: "
            f"箱子从 {before_box} 移动到 {after_box}，{check_mode}。"
        )
        summary += robot_info

        validation: dict[str, Any] = {
            "verified": True,
            "source": "comm_state",
            "validation_mode": "box_primary",
            "before_state": summarize_state(before_state),
            "after_state": summarize_state(after_state),
            "before_robot_pose": before_robot,
            "after_robot_pose": after_robot,
            "before_box_pose": before_box,
            "after_box_pose": after_box,
        }
        validation.update(
            {
                "meets_requirements": meets_requirements,
                "box_delta_pose": round_pose([box_delta_x, box_delta_y, box_delta_z]),
                "box_planar_distance": round(box_planar_distance, 3),
                "arrival_tol": round(arrival_tol, 3),
                "box_goal_dist_xy": round(box_goal_dist_xy, 3) if box_goal_dist_xy is not None else None,
                "summary": summary,
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
        current_pose = context.get("current_pose", [0.0, 0.0, 0.0])
        current_route_side = context.get("current_route_side")

        support_box = context.get("support_box")
        target_platform = context.get("target_platform")

        from Robot_Module.module.Action.Task.Bishe._bishe_helpers import (
            select_box_for_task,
            select_platform_for_task,
        )

        selected_support_box = select_box_for_task(task, objects, current_route_side, support_box, current_pose)
        selected_platform = select_platform_for_task(task, objects, current_route_side, target_platform, current_pose)

        can_use_box_assist = context.get("plan_uses_push_box", False)
        selected_box_height = object_height(selected_support_box)
        selected_platform_height = object_height(selected_platform)
        selected_remaining_height = round(selected_platform_height - selected_box_height, 3)
        selected_can_use_box_assist = bool(
            can_use_box_assist
            and selected_support_box
            and selected_platform
            and 0 < selected_box_height <= climb_limit
            and selected_platform_height > climb_limit
            and 0 < selected_remaining_height <= climb_limit
        )

        active_box = selected_support_box or support_box
        active_platform = selected_platform or target_platform
        if not active_box or not active_platform:
            return None
        if not (selected_can_use_box_assist or can_use_box_assist):
            return None

        target_position = build_adjacent_ground_position(active_box, active_platform)
        context["current_pose"] = [target_position[0], target_position[1], current_pose[2]]
        return {
            "box_height": object_height(active_box),
            "target_position": target_position,
        }

    def register_tool(self, mcp) -> dict[str, Any]:
        @mcp.tool()
        async def push_box(
            box_height: float,
            target_position: str = "高台旁边",
            speech: str = "",
        ) -> str:
            return json.dumps(
                await self.execute(box_height=box_height, target_position=target_position, speech=speech),
                ensure_ascii=False,
            )

        return {"push_box": push_box}


_skill = PushBoxSkill()
