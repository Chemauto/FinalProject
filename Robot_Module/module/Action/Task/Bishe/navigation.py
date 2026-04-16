from __future__ import annotations

import json

from Excu_Module.executor import execute_goal_navigation_skill
from Excu_Module.runtime import DEFAULT_TARGET, MODEL_USE_NAVIGATION


async def execute_navigation_skill(
    goal_command: list[float] | str,
    target: str = DEFAULT_TARGET,
    speech: str = "",
    wait_feedback: bool = True,
) -> dict[str, object]:
    return await execute_goal_navigation_skill(
        skill_name="navigation",
        model_use=MODEL_USE_NAVIGATION,
        goal_command=goal_command,
        target=target,
        speech_text=speech,
        wait_feedback=wait_feedback,
        log_label="导航",
    )


def register_tool(mcp):
    @mcp.tool()
    async def navigation(
        goal_command: str,
        target: str = DEFAULT_TARGET,
        speech: str = "",
    ) -> str:
        return json.dumps(
            await execute_navigation_skill(goal_command=goal_command, target=target, speech=speech),
            ensure_ascii=False,
        )

    return {"navigation": navigation}
