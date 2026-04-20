"""climb 技能 - 攀爬指定高度。

设置 model_use=2 (climb)，下发速度命令 [vx, 0, 0]，等待执行完成。
"""

import json
import sys

from Excu_Module.skill_base import SkillBase


class ClimbSkill(SkillBase):
    MODEL_USE = 2
    DEFAULT_SPEED = 0.4
    DEFAULT_DURATION = 4.0
    MAX_HEIGHT = 0.35

    @property
    def name(self) -> str:
        return "climb"

    async def execute(self, height=0.15, stage="高台", target="目标点", speech="", **kw):
        from Excu_Module.executor import wait_skill_feedback
        from Excu_Module.runtime import read_env_float, speak

        if height <= 0:
            return {"status": "failure", "message": "height 必须大于 0"}
        if height > self.MAX_HEIGHT:
            return {"status": "failure", "message": f"最大攀爬高度 {self.MAX_HEIGHT:.2f}m"}

        speed = abs(read_env_float("FINALPROJECT_CLIMB_SPEED_MPS", self.DEFAULT_SPEED))
        velocity = [speed, 0.0, 0.0]
        timeout = max(0.5, read_env_float("FINALPROJECT_CLIMB_DURATION_SEC", self.DEFAULT_DURATION))
        speak(speech)
        print(f"[climb] 攀爬{stage}, 高度={height:.2f}m", file=sys.stderr)

        feedback = await wait_skill_feedback(
            self.name,
            {"height": height, "stage": stage, "target": target},
            f"已完成{stage}攀爬",
            model_use=self.MODEL_USE,
            velocity_command=velocity,
            execution_time_sec=timeout,
        )
        return self.build_result(feedback, model_use=self.MODEL_USE, velocity=velocity)


def register_tools(mcp):
    from Excu_Module.skill_registry import register_skill

    skill = ClimbSkill()
    register_skill(skill)

    @mcp.tool()
    async def climb(height: float = 0.15, stage: str = "高台", target: str = "目标点", speech: str = "") -> str:
        """攀爬指定高度的台阶或物体。

        Args:
            height: 攀爬高度（米）
            stage: 攀爬目标描述，如 "高台"、"箱子"
            target: 目标标识
            speech: 语音播报内容
        """
        result = await skill.execute(height=height, stage=stage, target=target, speech=speech)
        return json.dumps(result, ensure_ascii=False)

    print("[Action/Task/Bishe] climb 技能已注册", file=sys.stderr)
    return {"climb": climb}
