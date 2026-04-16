from __future__ import annotations

import json

from Excu_Module.executor import wait_skill_feedback
from Excu_Module.runtime import (
    DEFAULT_POST_PUSH_SETTLE_SEC,
    DEFAULT_PUSH_BOX_DURATION_SEC,
    MODEL_USE_PUSH_BOX,
    estimate_goal_skill_duration,
    hold_idle_stability,
    load_runtime_config,
    log_skill,
    parse_goal_value,
    read_env_float,
    resolve_feedback_backend,
    speak,
)


async def execute_push_box_skill(
    box_height: float,
    target_position: str = "高台旁边",
    speech: str = "",
    wait_feedback: bool = True,
) -> dict[str, object]:
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


def register_tool(mcp):
    @mcp.tool()
    async def push_box(
        box_height: float,
        target_position: str = "高台旁边",
        speech: str = "",
    ) -> str:
        return json.dumps(
            await execute_push_box_skill(box_height=box_height, target_position=target_position, speech=speech),
            ensure_ascii=False,
        )

    return {"push_box": push_box}
