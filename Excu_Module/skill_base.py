"""SkillBase - 技能基类。

每个技能继承 SkillBase，实现：
  - name              : 技能标识
  - execute()         : 执行入口
  - register_tool()   : MCP 工具注册

可选覆盖：
  - check_completion(): 轮询完成检测（默认：超时等待）
  - validate()        : 事后状态校验（默认：None）

公共方法：
  - build_result()    : 统一返回结构
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from .runtime import resolve_feedback_backend


class SkillBase(ABC):
    """技能抽象基类。"""

    @property
    @abstractmethod
    def name(self) -> str:
        """技能唯一标识，如 'walk', 'climb', 'push_box'。"""
        ...

    @abstractmethod
    async def execute(self, **kwargs) -> dict[str, Any]:
        """异步执行入口。"""
        ...

    def register_tool(self, mcp) -> dict[str, Any]:
        """注册 MCP 工具。子类可覆盖以自定义注册行为。默认 raise NotImplementedError。"""
        raise NotImplementedError(f"{type(self).__name__} 未实现 register_tool()")

    def check_completion(
        self,
        state: dict[str, Any],
        before_state: dict[str, Any],
        parameters: dict[str, Any],
    ) -> dict[str, Any]:
        """轮询完成检测。默认：超时等待（永远返回未完成）。"""
        return {"done": False}

    def validate(
        self,
        parameters: dict[str, Any],
        before_state: dict[str, Any] | None,
        after_state: dict[str, Any] | None,
    ) -> dict[str, Any] | None:
        """事后状态校验。默认：None（触发通用回退）。"""
        return None

    # ── 公共方法 ────────────────────────────────────────────────────

    def build_result(
        self,
        feedback: dict[str, Any],
        *,
        model_use: int | None = None,
        velocity: list[float] | None = None,
        goal: list[float] | str | None = None,
        estimated_execution_time_sec: float | None = None,
        **extra: Any,
    ) -> dict[str, Any]:
        """统一返回结构。"""
        status = "success" if feedback.get("signal") == "SUCCESS" else "failure"
        result: dict[str, Any] = {
            "skill": self.name,
            "action_id": feedback.get("action_id"),
            "execution_feedback": feedback,
            "execution_result": feedback.get("result", {}),
            "control_command": {
                "model_use": model_use,
                **({"velocity": velocity} if velocity is not None else {}),
                **({"goal": goal} if goal is not None else {}),
                **(
                    {"estimated_execution_time_sec": round(estimated_execution_time_sec, 3)}
                    if estimated_execution_time_sec is not None
                    else {}
                ),
            },
            "backend": resolve_feedback_backend(feedback),
            "status": status,
        }
        result.update(extra)
        return result
