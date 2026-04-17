"""climb_align 技能 - 攀爬前导航对正。

设置 model_use=4 (navigation)，导航到攀爬目标前方，为后续攀爬做准备。
"""

import json
import sys

from Excu_Module.skill_base import SkillBase


class ClimbAlignSkill(SkillBase):
    MODEL_USE = 4

    @property
    def name(self) -> str:
        return "climb_align"

    async def execute(self, stage="高台", target="目标点", goal_command="", speech="", **kw):
        from Excu_Module.executor import execute_goal_navigation_skill
        from Excu_Module.runtime import parse_goal_value, speak

        resolved = parse_goal_value(goal_command)
        if not isinstance(resolved, list):
            resolved = target

        speak(speech)
        print(f"[climb_align] 攀爬前对正, 目标={resolved}", file=sys.stderr)
        return await execute_goal_navigation_skill(
            skill_name=self.name,
            goal_command=resolved,
            target=target,
            speech_text=speech,
            model_use=self.MODEL_USE,
        )


def register_tools(mcp):
    from Excu_Module.skill_registry import register_skill

    skill = ClimbAlignSkill()
    register_skill(skill)

    @mcp.tool()
    async def climb_align(stage: str = "高台", target: str = "目标点", goal_command: str = "", speech: str = "") -> str:
        """攀爬前导航对正，导航到攀爬目标的前方位置。

        Args:
            stage: 攀爬目标描述，如 "高台"
            target: 目标标识
            goal_command: 目标坐标 "x,y,z"，留空则使用 target
            speech: 语音播报内容
        """
        result = await skill.execute(stage=stage, target=target, goal_command=goal_command, speech=speech)
        return json.dumps(result, ensure_ascii=False)

    print("[Action/Task/Bishe] climb_align 技能已注册", file=sys.stderr)
    return {"climb_align": climb_align}
