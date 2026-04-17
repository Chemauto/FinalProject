"""walk 技能 - 沿指定方向行走指定距离。

设置 model_use=1 (walk)，下发速度命令 [vx, 0, 0]，等待执行完成。
"""

import json
import sys

from Excu_Module.skill_base import SkillBase


class WalkSkill(SkillBase):
    MODEL_USE = 1
    DEFAULT_SPEED = 0.6

    @property
    def name(self) -> str:
        return "walk"

    async def execute(self, route_side="前方", distance=1.0, target="目标点", speech="", **kw):
        from Excu_Module.executor import wait_skill_feedback
        from Excu_Module.runtime import estimate_linear_duration, read_env_float, speak

        if distance <= 0:
            return {"status": "failure", "message": "distance 必须大于 0"}

        speed = abs(read_env_float("FINALPROJECT_WALK_SPEED_MPS", self.DEFAULT_SPEED))
        velocity = [speed, 0.0, 0.0]
        timeout = estimate_linear_duration(distance, speed)
        speak(speech)
        print(f"[walk] 沿{route_side}行走 {distance:.2f}m, 目标={target}", file=sys.stderr)

        feedback = await wait_skill_feedback(
            self.name,
            {"route_side": route_side, "distance": distance, "target": target},
            f"已完成沿{route_side}行走",
            model_use=self.MODEL_USE,
            velocity_command=velocity,
            execution_time_sec=timeout,
        )
        return self.build_result(feedback, model_use=self.MODEL_USE, velocity=velocity)


def register_tools(mcp):
    from Excu_Module.skill_registry import register_skill

    skill = WalkSkill()
    register_skill(skill)

    @mcp.tool()
    async def walk(route_side: str = "前方", distance: float = 1.0, target: str = "目标点", speech: str = "") -> str:
        """沿指定方向行走指定距离。

        Args:
            route_side: 行走方向，如 "前方"、"左侧"、"右侧"
            distance: 行走距离（米）
            target: 目标描述
            speech: 语音播报内容
        """
        result = await skill.execute(route_side=route_side, distance=distance, target=target, speech=speech)
        return json.dumps(result, ensure_ascii=False)

    print("[Action/Task/Bishe] walk 技能已注册", file=sys.stderr)
    return {"walk": walk}
