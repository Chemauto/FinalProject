from __future__ import annotations

import json

from Comm_Module import get_state
from Excu_Module.executor import wait_skill_feedback
from Excu_Module.runtime import (
    DEFAULT_NAVIGATION_DURATION_SEC,
    DEFAULT_NAVIGATION_TIMEOUT_MARGIN_SEC,
    DEFAULT_POST_ALIGN_SETTLE_SEC,
    DEFAULT_PRE_CLIMB_SETTLE_SEC,
    DEFAULT_TARGET,
    MODEL_USE_NAVIGATION,
    estimate_goal_skill_duration,
    hold_idle_stability,
    load_runtime_config,
    log_skill,
    parse_goal_value,
    read_env_float,
    resolve_feedback_backend,
    speak,
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


async def execute_climb_align_skill(
    stage: str = "高台",
    target: str = DEFAULT_TARGET,
    goal_command: list[float] | str | None = None,
    speech: str = "",
    wait_feedback: bool = True,
) -> dict[str, object]:
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


def register_tool(mcp):
    @mcp.tool()
    async def climb_align(
        stage: str = "高台",
        target: str = DEFAULT_TARGET,
        goal_command: str = "",
        speech: str = "",
    ) -> str:
        return json.dumps(
            await execute_climb_align_skill(stage=stage, target=target, goal_command=goal_command, speech=speech),
            ensure_ascii=False,
        )

    return {"climb_align": climb_align}
