from __future__ import annotations

import json

from Excu_Module.executor import execute_goal_navigation_skill
from Excu_Module.runtime import DEFAULT_TARGET, MODEL_USE_NAV_CLIMB


async def execute_nav_climb_skill(
    goal_command: list[float] | str,
    target: str = DEFAULT_TARGET,
    speech: str = "",
    wait_feedback: bool = True,
) -> dict[str, object]:
    return await execute_goal_navigation_skill(
        skill_name="nav_climb",
        model_use=MODEL_USE_NAV_CLIMB,
        goal_command=goal_command,
        target=target,
        speech_text=speech,
        wait_feedback=wait_feedback,
        log_label="翻越导航",
    )


def register_tool(mcp):
    @mcp.tool()
    async def nav_climb(
        goal_command: str,
        target: str = DEFAULT_TARGET,
        speech: str = "",
    ) -> str:
        return json.dumps(
            await execute_nav_climb_skill(goal_command=goal_command, target=target, speech=speech),
            ensure_ascii=False,
        )

    return {"nav_climb": nav_climb}
