"""way_select 技能 - 选择左/右路线。

根据控制模式设置 model_use=1(walk) 或 model_use=4(navigation)，
下发横向速度命令或横向目标坐标，完成路线切换。
"""

import json
import sys

from Excu_Module.skill_base import SkillBase


class WaySelectSkill(SkillBase):
    MODEL_USE_WALK = 1
    MODEL_USE_NAV = 4
    DEFAULT_LATERAL_SPEED = 0.3
    DEFAULT_DURATION = 3.0

    @property
    def name(self) -> str:
        return "way_select"

    async def execute(self, direction="left", lateral_distance=0.5, target="目标点", speech="", **kw):
        from Excu_Module.executor import wait_skill_feedback
        from Excu_Module.runtime import load_runtime_config, speak

        dir_lower = str(direction).strip().lower()
        label = "左侧" if dir_lower in ("left", "左", "l") else "右侧"
        if not speech:
            speech = f"切换到{label}路线。"
        if lateral_distance <= 0:
            return {"status": "failure", "message": "lateral_distance 必须大于 0"}

        config = load_runtime_config()
        nav_mode = config.way_select_policy == "navigation"

        if nav_mode:
            model_use = self.MODEL_USE_NAV
            sign = 1.0 if "左" in label else -1.0
            goal = [0.0, sign * lateral_distance, 0.0, 0.0]
            velocity = None
            timeout = 5.0
        else:
            model_use = self.MODEL_USE_WALK
            speed = self.DEFAULT_LATERAL_SPEED
            sign = 1.0 if "左" in label else -1.0
            velocity = [0.0, sign * speed, 0.0]
            goal = None
            timeout = self.DEFAULT_DURATION

        speak(speech)
        print(f"[way_select] 选择{label}路线, 横移={lateral_distance:.2f}m", file=sys.stderr)

        feedback = await wait_skill_feedback(
            self.name,
            {"direction": dir_lower, "lateral_distance": lateral_distance, "target": target},
            f"已切换到{label}路线",
            model_use=model_use,
            velocity_command=velocity,
            goal_command=goal,
            execution_time_sec=timeout,
        )
        return self.build_result(feedback, model_use=model_use, velocity=velocity, goal=goal, direction=dir_lower)


def register_tools(mcp):
    from Excu_Module.skill_registry import register_skill

    skill = WaySelectSkill()
    register_skill(skill)

    @mcp.tool()
    async def way_select(direction: str = "left", lateral_distance: float = 0.5, target: str = "目标点", speech: str = "") -> str:
        """选择行进路线方向（左或右）。

        Args:
            direction: 路线方向，"left" 或 "right"
            lateral_distance: 横向移动距离（米）
            target: 目标描述
            speech: 语音播报内容
        """
        result = await skill.execute(direction=direction, lateral_distance=lateral_distance, target=target, speech=speech)
        return json.dumps(result, ensure_ascii=False)

    print("[Action/Task/Bishe] way_select 技能已注册", file=sys.stderr)
    return {"way_select": way_select}
