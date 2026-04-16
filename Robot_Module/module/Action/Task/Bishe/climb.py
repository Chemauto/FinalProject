from __future__ import annotations

import json

from Excu_Module.executor import wait_skill_feedback
from Excu_Module.runtime import (
    CLIMB_LIMIT_METERS,
    DEFAULT_CLIMB_EXECUTION_SEC,
    DEFAULT_CLIMB_SPEED_MPS,
    DEFAULT_TARGET,
    MODEL_USE_CLIMB,
    log_skill,
    read_env_float,
    resolve_feedback_backend,
    speak,
)


async def execute_climb_skill(
    height: float,
    stage: str = "高台",
    target: str = DEFAULT_TARGET,
    speech: str = "",
    wait_feedback: bool = True,
) -> dict[str, object]:
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


def register_tool(mcp):
    @mcp.tool()
    async def climb(
        height: float,
        stage: str = "高台",
        target: str = DEFAULT_TARGET,
        speech: str = "",
    ) -> str:
        return json.dumps(
            await execute_climb_skill(height=height, stage=stage, target=target, speech=speech),
            ensure_ascii=False,
        )

    return {"climb": climb}
