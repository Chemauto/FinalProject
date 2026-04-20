"""nav_climb 技能 - 导航攀爬，导航到目标后自动攀爬。

设置 model_use=5 (nav_climb)，下发 goal 命令，轮询等待到达。
"""

import json
import sys

from Excu_Module.skill_base import SkillBase


class NavClimbSkill(SkillBase):
    MODEL_USE = 5

    @property
    def name(self) -> str:
        return "nav_climb"

    async def execute(self, goal="", target="目标点", speech="", **kw):
        from Excu_Module.executor import execute_goal_navigation_skill
        from Excu_Module.runtime import parse_goal_value, speak

        goal_command = parse_goal_value(goal)
        if not isinstance(goal_command, list):
            return {"status": "failure", "message": f"无法解析目标坐标: {goal}"}

        speak(speech)
        print(f"[nav_climb] 导航攀爬到 {goal_command}, 目标={target}", file=sys.stderr)
        return await execute_goal_navigation_skill(
            skill_name=self.name,
            goal_command=goal_command,
            target=target,
            speech_text=speech,
            model_use=self.MODEL_USE,
        )


def register_tools(mcp):
    from Excu_Module.skill_registry import register_skill

    skill = NavClimbSkill()
    register_skill(skill)

    @mcp.tool()
    async def nav_climb(goal: str, target: str = "目标点", speech: str = "") -> str:
        """导航到目标位置并攀爬。

        Args:
            goal: 目标坐标，格式为 "x,y,z"
            target: 目标描述
            speech: 语音播报内容
        """
        result = await skill.execute(goal=goal, target=target, speech=speech)
        return json.dumps(result, ensure_ascii=False)

    print("[Action/Task/Bishe] nav_climb 技能已注册", file=sys.stderr)
    return {"nav_climb": nav_climb}
