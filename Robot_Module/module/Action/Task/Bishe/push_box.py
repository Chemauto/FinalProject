"""push_box 技能 - 推箱子到目标位置。

设置 model_use=3 (push_box)，下发 goal 命令，等待执行完成。
"""

import json
import sys

from Excu_Module.skill_base import SkillBase


class PushBoxSkill(SkillBase):
    MODEL_USE = 3
    DEFAULT_DURATION = 6.0

    @property
    def name(self) -> str:
        return "push_box"

    async def execute(self, box_height=0.15, target_position="auto", speech="", **kw):
        from Excu_Module.executor import wait_skill_feedback
        from Excu_Module.runtime import estimate_goal_skill_duration, parse_goal_value, speak

        if box_height <= 0:
            return {"status": "failure", "message": "box_height 必须大于 0"}

        goal = parse_goal_value(target_position)
        goal_command = goal if goal and goal != "auto" else "auto"
        timeout = estimate_goal_skill_duration(goal_command, self.DEFAULT_DURATION)
        speak(speech)
        print(f"[push_box] 推箱子, 高度={box_height:.2f}m, 目标={goal_command}", file=sys.stderr)

        feedback = await wait_skill_feedback(
            self.name,
            {"box_height": box_height, "target_position": target_position, "goal_command": goal_command},
            "已完成推箱子",
            model_use=self.MODEL_USE,
            goal_command=goal_command,
            execution_time_sec=timeout,
        )
        return self.build_result(feedback, model_use=self.MODEL_USE, goal=goal_command)


def register_tools(mcp):
    from Excu_Module.skill_registry import register_skill

    skill = PushBoxSkill()
    register_skill(skill)

    @mcp.tool()
    async def push_box(box_height: float, target_position: str = "auto", speech: str = "") -> str:
        """推动箱子到指定位置。

        Args:
            box_height: 箱子高度（米）
            target_position: 目标位置，可以是坐标如 "1.8,0,0.1" 或 "auto"
            speech: 语音播报内容
        """
        result = await skill.execute(box_height=box_height, target_position=target_position, speech=speech)
        return json.dumps(result, ensure_ascii=False)

    print("[Action/Task/Bishe] push_box 技能已注册", file=sys.stderr)
    return {"push_box": push_box}
