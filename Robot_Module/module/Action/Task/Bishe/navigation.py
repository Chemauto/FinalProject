from __future__ import annotations

import json
from typing import Any

from Excu_Module.executor import execute_goal_navigation_skill
from Excu_Module.runtime import DEFAULT_TARGET
from Excu_Module.skill_base import SkillBase
from Robot_Module.module.Action.Task.Bishe._bishe_helpers import (
    MODEL_USE_NAVIGATION,
    format_navigation_goal_command,
    format_navigation_target,
)


class NavigationSkill(SkillBase):

    @property
    def name(self) -> str:
        return "navigation"

    async def execute(
        self,
        goal_command: list[float] | str = "",
        target: str = DEFAULT_TARGET,
        speech: str = "",
        wait_feedback: bool = True,
    ) -> dict[str, Any]:
        return await execute_goal_navigation_skill(
            skill_name="navigation",
            model_use=MODEL_USE_NAVIGATION,
            goal_command=goal_command,
            target=target,
            speech_text=speech,
            wait_feedback=wait_feedback,
            log_label="导航",
        )

    # Navigation uses goal-based arrival check (handled by wait_for_navigation_completion in executor),
    # not skill-specific polling. Keep defaults for check_completion and validate.

    def calculate_parameters(
        self,
        task: dict[str, Any],
        object_facts: dict[str, Any] | None,
        context: dict[str, Any],
    ) -> dict[str, Any] | None:
        navigation_goal = context.get("navigation_goal")
        if not navigation_goal:
            return None
        context["current_pose"] = list(navigation_goal)
        return {
            "goal_command": format_navigation_goal_command(navigation_goal),
            "target": format_navigation_target(navigation_goal),
        }

    def register_tool(self, mcp) -> dict[str, Any]:
        @mcp.tool()
        async def navigation(
            goal_command: str,
            target: str = DEFAULT_TARGET,
            speech: str = "",
        ) -> str:
            return json.dumps(
                await self.execute(goal_command=goal_command, target=target, speech=speech),
                ensure_ascii=False,
            )

        return {"navigation": navigation}


_skill = NavigationSkill()
