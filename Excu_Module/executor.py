from __future__ import annotations

import asyncio
from typing import Any, Sequence

from .runtime import (
    DEFAULT_NAVIGATION_DURATION_SEC,
    DEFAULT_NAVIGATION_TIMEOUT_MARGIN_SEC,
    DEFAULT_STATUS_READY_TIMEOUT_SEC,
    NAVIGATION_MODEL_USES,
    build_feedback,
    estimate_goal_skill_duration,
    load_runtime_config,
    log_skill,
    make_action_id,
    parse_goal_value,
    publish_skill_command,
    read_env_float,
    resolve_feedback_backend,
    speak,
    stop_envtest_skill,
    wait_for_execution_feedback,
    apply_envtest_command,
)
from .state import (
    build_navigation_validation,
    extract_status_timestamp,
    validate_live_execution,
    wait_for_live_state,
    wait_for_navigation_completion,
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

    if config.backend == "ros":
        if publish_skill_command is None or wait_for_execution_feedback is None:
            raise RuntimeError("当前环境未安装 ROS2 依赖，无法使用 ros 执行后端")
        action_id = publish_skill_command(skill_name, parameters, action_id=action_id)
        feedback = await wait_for_execution_feedback(action_id, timeout_sec=timeout_sec)
        feedback.setdefault("action_id", action_id)
        feedback.setdefault("skill", skill_name)
        feedback.setdefault("result", {})
        return feedback

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

    if model_use in NAVIGATION_MODEL_USES and isinstance(goal_command, list) and len(goal_command) >= 3:
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

    actual_wait_sec = min(timeout_sec, execution_time_sec)
    await asyncio.sleep(actual_wait_sec)
    await stop_envtest_skill(config)
    after_state = await wait_for_live_state(
        task_type=config.task_type,
        timeout_sec=ready_timeout_sec,
        min_timestamp=before_timestamp,
    )
    validation = validate_live_execution(skill_name, parameters, before_state, after_state)

    if execution_time_sec > timeout_sec:
        return {
            "action_id": action_id,
            **build_feedback(
                skill_name,
                f"{skill_name} 预计执行 {execution_time_sec:.1f}s，超过超时阈值 {timeout_sec:.1f}s",
                signal="FAILURE",
                validation=validation,
            ),
            "result": {
                **planned_result,
                "mode": "execution_timeout",
                "wait_time_sec": actual_wait_sec,
                "verification": validation,
            },
        }

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
            "mode": "comm_state_feedback",
            "wait_time_sec": actual_wait_sec,
            "verification": validation,
        },
    }


async def execute_goal_navigation_skill(
    *,
    skill_name: str,
    model_use: int,
    goal_command: list[float] | str,
    target: str,
    speech_text: str,
    wait_feedback: bool,
    log_label: str,
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
