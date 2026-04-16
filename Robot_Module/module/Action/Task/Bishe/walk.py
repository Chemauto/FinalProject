from __future__ import annotations

import json

from Excu_Module.executor import wait_skill_feedback
from Excu_Module.runtime import (
    DEFAULT_TARGET,
    DEFAULT_WALK_SPEED_MPS,
    MODEL_USE_WALK,
    estimate_linear_duration,
    log_skill,
    read_env_float,
    resolve_feedback_backend,
    speak,
)


async def execute_walk_skill(
    route_side: str = "前方",
    distance: float = 1.0,
    target: str = DEFAULT_TARGET,
    speech: str = "",
    wait_feedback: bool = True,
) -> dict[str, object]:
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


def register_tool(mcp):
    @mcp.tool()
    async def walk(
        route_side: str = "前方",
        distance: float = 1.0,
        target: str = DEFAULT_TARGET,
        speech: str = "",
    ) -> str:
        return json.dumps(
            await execute_walk_skill(route_side=route_side, distance=distance, target=target, speech=speech),
            ensure_ascii=False,
        )

    return {"walk": walk}
