"""push_box 技能 - 推箱子到目标位置。

设置 model_use=3 (push_box)，下发 goal 命令，每 0.5 秒轮询检测箱子位置。
箱子位置与目标距离 < 0.05m 立即停止并返回成功。
"""

import asyncio
import json
import math
import sys
import time
from typing import Any

from Excu_Module.skill_base import SkillBase


def _build_push_validation(
    verified: bool,
    summary: str,
    distance: float | None,
    box_pos: list[float] | None,
    goal_pos: list[float] | None,
    elapsed: float,
    threshold: float,
) -> dict[str, Any]:
    """构建 push_box 验证结果字典。"""
    validation: dict[str, Any] = {
        "verified": verified,
        "meets_requirements": verified,
        "source": "comm_state_poll",
        "summary": summary,
        "elapsed_sec": elapsed,
    }
    if distance is not None:
        validation.update({
            "push_box_verified": verified,
            "push_box_distance": distance,
            "push_box_threshold": threshold,
            "box_position": box_pos,
            "goal_position": goal_pos,
        })
    return validation


class PushBoxSkill(SkillBase):
    MODEL_USE = 3
    DEFAULT_DURATION = 6.0
    DEFAULT_TIMEOUT_MARGIN = 5.0
    POLL_INTERVAL = 0.5
    ARRIVAL_THRESHOLD = 0.08

    @property
    def name(self) -> str:
        return "push_box"

    async def execute(self, box_height=0.15, target_position="auto", speech="", **kw):
        from Excu_Module.runtime import (
            estimate_goal_skill_duration,
            parse_goal_value,
            speak,
            load_runtime_config,
            apply_envtest_command,
            stop_envtest_skill,
            make_action_id,
            build_feedback,
            DEFAULT_STATUS_READY_TIMEOUT_SEC,
            read_env_float,
        )
        from Excu_Module.state import (
            wait_for_live_state,
            extract_goal,
            extract_model_use,
            extract_start_flag,
        )

        if box_height <= 0:
            return {"status": "failure", "message": "box_height 必须大于 0"}

        goal = parse_goal_value(target_position)
        goal_command = goal if goal and goal != "auto" else "auto"
        timeout = estimate_goal_skill_duration(goal_command, self.DEFAULT_DURATION)
        deadline_sec = max(timeout, self.DEFAULT_DURATION) + self.DEFAULT_TIMEOUT_MARGIN

        speak(speech)
        print(f"[push_box] 推箱子, 高度={box_height:.2f}m, 目标={goal_command}, 超时={deadline_sec:.1f}s", file=sys.stderr)

        action_id = make_action_id(self.name)
        config = load_runtime_config()
        ready_timeout = max(
            DEFAULT_STATUS_READY_TIMEOUT_SEC,
            read_env_float("FINALPROJECT_STATUS_READY_TIMEOUT_SEC", DEFAULT_STATUS_READY_TIMEOUT_SEC),
        )

        # ── 等待初始状态 ──────────────────────────────────────────
        before_state = await wait_for_live_state(
            task_type=config.task_type,
            timeout_sec=ready_timeout,
        )
        if before_state is None:
            validation = {
                "verified": False,
                "meets_requirements": False,
                "source": "comm_state",
                "summary": f"{self.name} 无法开始执行: 缺少实时状态",
            }
            feedback = {
                "action_id": action_id,
                **build_feedback(self.name, validation["summary"], signal="FAILURE", validation=validation),
            }
            return self.build_result(feedback, model_use=self.MODEL_USE, goal=goal_command)

        # ── 下发命令 ──────────────────────────────────────────────
        apply_envtest_command(config, model_use=self.MODEL_USE, goal=goal_command)
        await asyncio.sleep(config.command_settle_sec)
        apply_envtest_command(config, start=True)

        # ── 轮询检测 ──────────────────────────────────────────────
        start_time = time.time()
        deadline = start_time + deadline_sec
        verified = False
        final_distance = None
        final_box_pos = None
        final_goal_pos = None

        while time.time() < deadline:
            await asyncio.sleep(self.POLL_INTERVAL)

            state = _load_state(config.task_type)
            if not state or not state.get("connected"):
                continue

            goal_pos = extract_goal(state)
            box_pos = self._extract_box_position(state)

            if not goal_pos or len(goal_pos) < 3 or not box_pos:
                # 技能是否已自行停止
                if _skill_stopped(state, self.MODEL_USE):
                    break
                continue

            goal_xyz = [float(goal_pos[0]), float(goal_pos[1]), float(goal_pos[2])]
            box_xyz = [float(box_pos[0]), float(box_pos[1]), float(box_pos[2])]
            distance = math.sqrt(sum((g - b) ** 2 for g, b in zip(goal_xyz, box_xyz)))

            final_distance = round(distance, 4)
            final_box_pos = [round(v, 3) for v in box_xyz]
            final_goal_pos = [round(v, 3) for v in goal_xyz]

            print(f"[push_box] 轮询: 箱子距目标 {distance:.4f}m", file=sys.stderr)

            if distance < self.ARRIVAL_THRESHOLD:
                verified = True
                break

        # ── 停止技能 ──────────────────────────────────────────────
        await stop_envtest_skill(config)
        elapsed = round(time.time() - start_time, 3)

        # ── 构建结果 ──────────────────────────────────────────────
        if verified:
            summary = f"push_box 验证通过: 箱子位置与目标距离 {final_distance}m < {self.ARRIVAL_THRESHOLD}m"
            validation = _build_push_validation(True, summary, final_distance, final_box_pos, final_goal_pos, elapsed, self.ARRIVAL_THRESHOLD)
            feedback = {
                "action_id": action_id,
                **build_feedback(self.name, validation["summary"], signal="SUCCESS", validation=validation),
                "result": {"mode": "comm_state_push_box_poll", "elapsed_sec": elapsed},
            }
        elif final_distance is not None:
            summary = f"push_box 验证失败: 箱子位置与目标距离 {final_distance}m >= {self.ARRIVAL_THRESHOLD}m"
            validation = _build_push_validation(False, summary, final_distance, final_box_pos, final_goal_pos, elapsed, self.ARRIVAL_THRESHOLD)
            feedback = {
                "action_id": action_id,
                **build_feedback(self.name, validation["summary"], signal="FAILURE", validation=validation),
                "result": {"mode": "comm_state_push_box_poll", "elapsed_sec": elapsed},
            }
        else:
            summary = f"push_box 超时: {elapsed}s 内无法获取箱子位置或目标信息"
            validation = _build_push_validation(False, summary, None, None, None, elapsed, self.ARRIVAL_THRESHOLD)
            feedback = {
                "action_id": action_id,
                **build_feedback(self.name, validation["summary"], signal="FAILURE", validation=validation),
                "result": {"mode": "comm_state_push_box_poll", "elapsed_sec": elapsed},
            }

        return self.build_result(feedback, model_use=self.MODEL_USE, goal=goal_command)

    @staticmethod
    def _extract_box_position(state: dict) -> list[float] | None:
        """从 state 的 envtest_alignment 中提取箱子位置。"""
        for prefix in ("runtime", "observation"):
            section = state.get(prefix) or {}
            alignment = (
                section.get("envtest_alignment")
                if prefix == "runtime"
                else (section.get("environment") or {}).get("envtest_alignment")
            )
            if not isinstance(alignment, dict):
                continue
            pos = (alignment.get("box") or {}).get("position")
            if isinstance(pos, list) and len(pos) >= 3:
                return pos
        return None


def _load_state(task_type):
    from Excu_Module.state import load_live_state
    return load_live_state(task_type=task_type)


def _skill_stopped(state, expected_model_use):
    from Excu_Module.state import extract_model_use, extract_start_flag
    start_flag = extract_start_flag(state)
    current_model_use = extract_model_use(state)
    return start_flag is False or (current_model_use is not None and current_model_use != expected_model_use)


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
