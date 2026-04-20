"""Session state and prompt definitions for the Rich TUI."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


WELCOME_TITLE = "LQN Claw TUI"

AGENT_SYSTEM_PROMPT = """你是一个机器人智能体，不是机器人本体。
你可以直接和用户对话，也可以调用技能。
可用技能只有两类：
- vlm_observe：观察当前环境，只返回观测结果
- robot_act：在需要动作时，调用机器人内部规划与执行链

规则：
- 当你需要执行动作时，你必须在回复中先输出你的思考过程（包括为什么需要观测、打算怎么做），然后在同一次回复中调用工具
- 思考过程应该包括：1) 环境感知需求 2) 动作规划 3) 执行策略
- 涉及在环境中运动（如 walk、navigation、climb 等）时，必须先调用 vlm_observe 观察环境
- `vlm_observe` 会返回 `visual_context` 和 `env_state`；其中 `env_state` 是 VLM 与 Comm 合并后的结构化环境理解，规划时优先参考它
- 调用 robot_act 时，附带一个简短的 agent_thought，说明你为什么要执行这个动作、当前如何理解任务
- 如果已经调用过 vlm_observe，再调用 robot_act 时，应优先把 visual_context 传入 observation_context，把 env_state.scene_facts 传入 scene_facts_json；若没有 env_state，再退回原始 scene_facts
- 回复简洁、自然、直接
"""


@dataclass
class AgentRuntime:
    client: Any
    model: str
    system_prompt: str


@dataclass
class InteractiveSessionState:
    vlm_enabled: bool = True
    recent_inputs: list[str] = field(default_factory=list)
    last_summary: dict[str, Any] = field(default_factory=dict)
    last_sync: dict[str, Any] = field(default_factory=dict)
    last_result: dict[str, Any] = field(default_factory=dict)
    last_observation: dict[str, Any] = field(default_factory=dict)
    messages: list[dict[str, Any]] = field(default_factory=list)

    def reset_runtime_state(self) -> None:
        self.recent_inputs.clear()
        self.last_summary = {}
        self.last_sync = {}
        self.last_result = {}
        self.last_observation = {}
        self.messages = []

    def record_interaction(self, user_input: str, result: dict[str, Any]) -> None:
        self.recent_inputs.append(user_input)
        self.recent_inputs = self.recent_inputs[-5:]
        self.last_result = result
        self.last_summary = result.get("summary") or {}
        self.last_sync = result.get("session_snapshot", {}).get("sync") or {}


def assistant_message_to_dict(message: Any) -> dict[str, Any]:
    tool_calls = []
    for tool_call in getattr(message, "tool_calls", []) or []:
        tool_calls.append(
            {
                "id": tool_call.id,
                "type": "function",
                "function": {
                    "name": tool_call.function.name,
                    "arguments": tool_call.function.arguments,
                },
            }
        )
    return {"role": "assistant", "content": getattr(message, "content", "") or "", "tool_calls": tool_calls}

