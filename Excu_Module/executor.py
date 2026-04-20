from __future__ import annotations

import asyncio
from typing import Any, Sequence

from .runtime import (
    DEFAULT_STATUS_READY_TIMEOUT_SEC,
    build_feedback,
    estimate_goal_skill_duration,
    load_runtime_config,
    log_skill,
    make_action_id,
    parse_goal_value,
    read_env_float,
    resolve_feedback_backend,
    speak,
    stop_envtest_skill,
    apply_envtest_command,
)

# ── Generic navigation timing defaults ────────────────────────────────
DEFAULT_NAVIGATION_DURATION_SEC = 6.0
DEFAULT_NAVIGATION_TIMEOUT_MARGIN_SEC = 5.0
from .state import (
    build_navigation_validation,
    build_displacement_validation,
    extract_robot_pose,
    extract_status_timestamp,
    summarize_state,
    wait_for_live_state,
    wait_for_navigation_completion,
    wait_for_displacement_completion,
)


async def wait_skill_feedback(
    skill_name: str,
    parameters: dict[str, Any],
    local_success_message: str,
    wait_feedback: bool = True,
    timeout_sec: float = 20.0,
    *,
    model_use: int | None = None,
    velocity_command: Sequence[float] | None = None,
    goal_command: list[float] | str | None = None,
    execution_time_sec: float | None = None,
    task_type: str | None = None,
) -> dict[str, Any]:
    action_id = make_action_id(skill_name)
    config = load_runtime_config(task_type=task_type)
    ready_timeout_sec = max(
        DEFAULT_STATUS_READY_TIMEOUT_SEC,
        read_env_float("FINALPROJECT_STATUS_READY_TIMEOUT_SEC", DEFAULT_STATUS_READY_TIMEOUT_SEC),
    )
    planned_result = {
        "mode": "planned_only",
        "backend": config.backend,
        "task_type": config.task_type,
        "model_use": model_use,
        "parameters": parameters,
        "velocity_command": list(velocity_command) if velocity_command is not None else None,
        "goal_command": goal_command,
        "estimated_execution_time_sec": execution_time_sec,
    }

    if not wait_feedback:
        return {
            "action_id": action_id,
            **build_feedback(skill_name, local_success_message),
            "result": planned_result,
        }

    if model_use is None:
        raise ValueError(f"{skill_name} 缺少执行所需的 model_use")

    before_state = await wait_for_live_state(
        task_type=config.task_type,
        timeout_sec=ready_timeout_sec,
    )
    if before_state is None:
        validation = {
            "verified": False,
            "meets_requirements": False,
            "source": "comm_state",
            "summary": f"{skill_name} 无法开始执行: 缺少实时状态",
        }
        return {
            "action_id": action_id,
            **build_feedback(skill_name, validation["summary"], signal="FAILURE", validation=validation),
            "result": {
                **planned_result,
                "mode": "missing_live_state",
            },
        }

    before_timestamp = extract_status_timestamp(before_state)
    execution_time_sec = max(0.5, float(execution_time_sec or 0.5))

    apply_envtest_command(config, model_use=model_use, velocity=velocity_command, goal=goal_command)
    await asyncio.sleep(config.command_settle_sec)
    apply_envtest_command(config, start=True)

    from .skill_registry import get_navigation_model_uses
    if model_use in get_navigation_model_uses() and isinstance(goal_command, list) and len(goal_command) >= 3:
        arrival_result = await wait_for_navigation_completion(
            task_type=config.task_type,
            goal_command=goal_command,
            timeout_sec=timeout_sec,
        )
        await stop_envtest_skill(config)
        validation = build_navigation_validation(skill_name, goal_command, arrival_result)
        signal = "SUCCESS" if validation.get("verified") and validation.get("meets_requirements") else "FAILURE"
        return {
            "action_id": action_id,
            **build_feedback(
                skill_name,
                str(validation.get("summary") or local_success_message),
                signal=signal,
                validation=validation,
            ),
            "result": {
                **planned_result,
                "mode": "comm_state_navigation",
                "wait_time_sec": arrival_result.get("elapsed_sec"),
                "arrival_check": arrival_result,
                "verification": validation,
            },
        }

    # ── 位移轮询分支：有 velocity_command + distance → 实时检测位移 ──
    displacement_distance = parameters.get("distance")
    if velocity_command is not None and displacement_distance is not None:
        start_pose = extract_robot_pose(before_state)
        if start_pose is not None and float(displacement_distance) > 0:
            direction = None
            if len(velocity_command) >= 2:
                vx, vy = float(velocity_command[0]), float(velocity_command[1])
                if abs(vx) >= abs(vy):
                    direction = "forward" if vx > 0 else "backward"
                else:
                    direction = "left" if vy > 0 else "right"

            disp_result = await wait_for_displacement_completion(
                task_type=config.task_type,
                start_pose=start_pose,
                distance=float(displacement_distance),
                timeout_sec=timeout_sec,
                direction=direction,
            )
            await stop_envtest_skill(config)
            validation = build_displacement_validation(skill_name, float(displacement_distance), disp_result)
            signal = "SUCCESS" if validation.get("verified") and validation.get("meets_requirements") else "FAILURE"
            return {
                "action_id": action_id,
                **build_feedback(
                    skill_name,
                    str(validation.get("summary") or local_success_message),
                    signal=signal,
                    validation=validation,
                ),
                "result": {
                    **planned_result,
                    "mode": "displacement_poll",
                    "wait_time_sec": disp_result.get("elapsed_sec"),
                    "displacement_check": disp_result,
                    "verification": validation,
                },
            }

    # ── 兜底分支：无位移参数，按固定时间等待 ──
    actual_wait_sec = max(timeout_sec, execution_time_sec)
    await asyncio.sleep(actual_wait_sec)
    await stop_envtest_skill(config)
    after_state = await wait_for_live_state(
        task_type=config.task_type,
        timeout_sec=ready_timeout_sec,
        min_timestamp=before_timestamp,
    )

    if after_state is None:
        validation = {
            "verified": False,
            "meets_requirements": False,
            "source": "comm_state",
            "summary": f"{skill_name} 执行后无法获取状态",
            "before_state": summarize_state(before_state),
            "after_state": None,
        }
    else:
        validation = _build_displacement_validation(
            skill_name, before_state, after_state,
            velocity_command=velocity_command,
            execution_time_sec=execution_time_sec,
        )

    validation_ok = validation["verified"] and validation["meets_requirements"]
    signal = "SUCCESS" if validation_ok else "FAILURE"
    summary = validation["summary"]

    return {
        "action_id": action_id,
        **build_feedback(
            skill_name,
            summary,
            signal=signal,
            validation=validation,
        ),
        "result": {
            **planned_result,
            "mode": "comm_state_timeout",
            "verification": validation,
        },
    }


def _build_displacement_validation(
    skill_name: str,
    before_state: dict[str, Any],
    after_state: dict[str, Any],
    *,
    velocity_command: Sequence[float] | None = None,
    execution_time_sec: float | None = None,
) -> dict[str, Any]:
    """根据执行前后位移校验 walk/climb 等非导航技能。"""
    from .state import extract_robot_pose

    before_pose = extract_robot_pose(before_state)
    after_pose = extract_robot_pose(after_state)
    before_summary = summarize_state(before_state)
    after_summary = summarize_state(after_state)

    if before_pose is None or after_pose is None:
        return {
            "verified": True,
            "meets_requirements": True,
            "source": "displacement",
            "summary": f"{skill_name} 已完成执行（缺少位姿数据，无法校验位移）",
            "before_state": before_summary,
            "after_state": after_summary,
        }

    dx = after_pose[0] - before_pose[0]
    dy = after_pose[1] - before_pose[1]
    dz = after_pose[2] - before_pose[2]
    displacement_xy = (dx ** 2 + dy ** 2) ** 0.5

    direction_label = "静止"
    expected_moved = False
    if velocity_command is not None and len(velocity_command) >= 2:
        vx, vy = velocity_command[0], velocity_command[1]
        if abs(vx) > 0.01 or abs(vy) > 0.01:
            expected_moved = True
            if abs(vx) >= abs(vy):
                direction_label = "前进" if vx > 0 else "后退"
            else:
                direction_label = "左移" if vy > 0 else "右移"

    if expected_moved:
        direction_ok = True
        if direction_label == "前进" and dx <= 0:
            direction_ok = False
        elif direction_label == "后退" and dx >= 0:
            direction_ok = False
        elif direction_label == "左移" and dy <= 0:
            direction_ok = False
        elif direction_label == "右移" and dy >= 0:
            direction_ok = False

        if not direction_ok:
            return {
                "verified": True,
                "meets_requirements": False,
                "source": "displacement",
                "summary": f"{skill_name} 方向错误：期望{direction_label}，实际位移 ({dx:.3f}, {dy:.3f}, {dz:.3f})",
                "displacement": {"dx": round(dx, 3), "dy": round(dy, 3), "dz": round(dz, 3), "dist_xy": round(displacement_xy, 3)},
                "direction": direction_label,
                "before_state": before_summary,
                "after_state": after_summary,
            }

    min_displacement_m = 0.02
    meets = displacement_xy >= min_displacement_m or not expected_moved
    summary = f"{skill_name} 已完成执行，位移 ({dx:.3f}, {dy:.3f}, {dz:.3f})m"
    if direction_label != "静止":
        summary = f"{skill_name} {direction_label}完成，位移 {displacement_xy:.3f}m"

    return {
        "verified": True,
        "meets_requirements": meets,
        "source": "displacement",
        "summary": summary,
        "displacement": {"dx": round(dx, 3), "dy": round(dy, 3), "dz": round(dz, 3), "dist_xy": round(displacement_xy, 3)},
        "direction": direction_label,
        "before_state": before_summary,
        "after_state": after_summary,
    }


async def execute_goal_navigation_skill(
    *,
    skill_name: str,
    model_use: int,
    goal_command: list[float] | str,
    target: str,
    speech_text: str = "",
    wait_feedback: bool = True,
    log_label: str = "导航",
    task_type: str | None = None,
) -> dict[str, Any]:
    normalized_goal = parse_goal_value(goal_command)
    if normalized_goal is None or normalized_goal == "auto":
        raise ValueError("goal_command 必须是显式目标点，支持 [x, y, z] / \"x,y,z\" / [x, y, z, yaw] 格式")

    execution_time_sec = estimate_goal_skill_duration(normalized_goal, DEFAULT_NAVIGATION_DURATION_SEC)
    timeout_sec = max(20.0, execution_time_sec + DEFAULT_NAVIGATION_TIMEOUT_MARGIN_SEC)
    speak(speech_text)
    log_skill(skill_name, f"{log_label}到{target}，目标命令={normalized_goal}，预计执行 {execution_time_sec:.2f} 秒")
    feedback = await wait_skill_feedback(
        skill_name,
        {"goal_command": normalized_goal, "target": target},
        f"已完成{log_label}到{target}",
        wait_feedback=wait_feedback,
        timeout_sec=timeout_sec,
        model_use=model_use,
        goal_command=normalized_goal,
        execution_time_sec=execution_time_sec,
        task_type=task_type,
    )
    return {
        "skill": skill_name,
        "goal_command": normalized_goal,
        "target": target,
        "speech": speech_text,
        "action_id": feedback.get("action_id"),
        "execution_feedback": feedback,
        "execution_result": feedback.get("result", {}),
        "control_command": {
            "model_use": model_use,
            "goal": normalized_goal,
            "estimated_execution_time_sec": round(execution_time_sec, 3),
            "timeout_sec": round(timeout_sec, 3),
        },
        "backend": resolve_feedback_backend(feedback),
        "status": "success" if feedback.get("signal") == "SUCCESS" else "failure",
    }
