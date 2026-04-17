#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Agent_Module/replanner.py — 执行流水线（含重规划）+ TUI 入口。"""

from __future__ import annotations

import io
import json
import logging
import os
import re
import sys
from contextlib import redirect_stderr
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

from openai import OpenAI
from rich.console import Console
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

for logger_name in ("openai", "httpx"):
    logging.getLogger(logger_name).setLevel(logging.WARNING)

for var in ("http_proxy", "https_proxy", "HTTP_PROXY", "HTTPS_PROXY", "ALL_PROXY", "all_proxy", "no_proxy", "NO_PROXY"):
    os.environ.pop(var, None)

project_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(project_root))

try:
    from dotenv import load_dotenv
    env_file = project_root / ".env"
    if env_file.exists():
        load_dotenv(env_file)
except ImportError:
    pass

try:
    from .planner import Planner
    from .executor import TaskExecutor, execute_tool
except ImportError:
    from planner import Planner
    from executor import TaskExecutor, execute_tool


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


# ── PipelineRunner ─────────────────────────────────────────────────────


class PipelineRunner:
    """串联 plan → parameter → execute 的完整流水线，含重规划循环。"""

    def __init__(
        self,
        planner: Planner,
        task_executor: TaskExecutor,
        max_replans: int = 1,
    ):
        self.planner = planner
        self.task_executor = task_executor
        self.max_replans = max_replans

    def run_pipeline(
        self,
        user_input: str,
        agent_thought: str,
        tools: list[dict],
        execute_tool_fn: Callable,
        visual_context: str | None = None,
        scene_facts: dict[str, Any] | None = None,
        object_facts: dict[str, Any] | None = None,
    ) -> list[dict]:
        print("\n" + "█" * 60 + f"\n📥 [用户输入] {user_input}\n" + "█" * 60)
        try:
            results: list[dict[str, Any]] = []
            plan_input = user_input

            for attempt in range(self.max_replans + 1):
                tasks, plan_meta = self.planner.plan_tasks(
                    plan_input, agent_thought, tools, visual_context,
                    scene_facts=scene_facts,
                    object_facts=object_facts,
                )
                if not tasks:
                    break

                active_tasks = self.planner.annotate_tasks(tasks, object_facts)
                task_understanding = self._build_task_understanding(user_input, active_tasks)
                if task_understanding:
                    thought_prefix = f"顶层思考: {agent_thought}\n" if agent_thought else ""
                    print("\n" + "─" * 60 + f"\n👁️ [LLM思考]\n{thought_prefix}{task_understanding}\n" + "─" * 60)

                print("\n" + "█" * 60 + "\n🤖 [下层LLM] 开始执行与工具决策\n" + "█" * 60)
                previous_result = None
                should_replan = False

                for idx, task in enumerate(active_tasks, start=1):
                    print(f"\n【步骤 {idx}/{len(active_tasks)}】")
                    print(f"📌 [规划函数] {task.get('function', '待LLM决定')}")
                    print(f"📝 [规划依据] {task.get('reason', '未提供规划依据')}")
                    result = self.task_executor.execute_single_task(
                        task, tools, execute_tool_fn, previous_result, visual_context,
                    )
                    task_success, assessment_message = self.assess_result(result)
                    result["success"] = task_success
                    if assessment_message:
                        result["assessment_message"] = assessment_message
                    results.append(result)

                    tool_output = result.get("result", {})
                    if task_success and tool_output:
                        previous_result = tool_output.get("result")
                        if assessment_message:
                            print(f"✅ [结果校验] {assessment_message}")
                        continue

                    previous_result = None
                    failure_message = (
                        assessment_message
                        or (result.get("feedback") or {}).get("message")
                        or result.get("error")
                        or "当前步骤执行失败"
                    )
                    if attempt < self.max_replans:
                        print(f"\n🔁 [重规划] {failure_message}")
                        plan_input = f"{user_input}\n上次执行失败：{failure_message}"
                        should_replan = True
                    else:
                        print(f"\n⚠️  [中止] {failure_message}")
                    break

                if not should_replan:
                    break

            print("\n" + "█" * 60 + "\n✅ [执行完成] 任务总结\n" + "█" * 60)
            for idx, result in enumerate(results, 1):
                status = "✅ 成功" if result.get("success") else "❌ 失败"
                action = result.get("action", "未调用")
                task_label = result.get("task", "未记录任务")
                print(f"  {idx}. {task_label} -> {action} - {status}")
            return results
        except Exception as error:
            print(f"\n❌ [错误] {type(error).__name__}: {error}")
            import traceback
            traceback.print_exc()
            return []

    # ── Result assessment ───────────────────────────────────────────────

    def assess_result(self, result: dict[str, Any]) -> tuple[bool, str]:
        feedback = result.get("feedback") or {}
        validation = self._extract_execution_validation(result)

        if validation:
            summary = str(validation.get("summary") or feedback.get("message") or "").strip()
            verified = validation.get("verified")
            meets_requirements = validation.get("meets_requirements")

            if verified is True and meets_requirements is True:
                return True, summary or "动作已通过真实状态校验"
            if verified is False:
                return False, summary or "未获取到真实执行数据，无法确认动作是否满足要求"
            if meets_requirements is False:
                return False, summary or "动作未满足任务要求"

        signal = str(feedback.get("signal") or "").upper()
        message = str(feedback.get("message") or result.get("error") or "当前步骤执行失败").strip()
        return bool(result.get("success")) and signal == "SUCCESS", message

    @staticmethod
    def _extract_execution_validation(result: dict[str, Any]) -> dict[str, Any]:
        feedback = result.get("feedback")
        if isinstance(feedback, dict):
            validation = feedback.get("validation")
            if isinstance(validation, dict):
                return validation

        tool_output = result.get("result")
        if isinstance(tool_output, dict):
            raw_result = tool_output.get("result")
            if isinstance(raw_result, dict):
                execution_feedback = raw_result.get("execution_feedback")
                if isinstance(execution_feedback, dict):
                    validation = execution_feedback.get("validation")
                    if isinstance(validation, dict):
                        return validation
        return {}

    def _build_task_understanding(self, user_input: str, tasks: list[dict]) -> str:
        if not tasks:
            return ""
        summary = self.planner.last_summary or "已完成任务规划"
        function_chain = " -> ".join(task.get("function", "待定") for task in tasks)
        primary_reason = tasks[0].get("reason", "")
        return "\n".join(
            [
                f"环境判断: {self._build_environment_judgment(tasks)}",
                f"任务目标: {user_input}",
                f"技能决策: {summary}",
                f"选择原因: {primary_reason}",
                f"函数链: {function_chain}",
            ]
        )

    @staticmethod
    def _build_environment_judgment(tasks: list[dict]) -> str:
        if not tasks:
            return "未获取到有效环境判断"
        first_task = tasks[0].get("task", "")
        if "。先" in first_task:
            return first_task.split("。先", 1)[0]
        if "，先" in first_task:
            return first_task.split("，先", 1)[0]
        return first_task


# Backward compatibility alias
LLMAgent = PipelineRunner


# ── TUI State & Helpers ────────────────────────────────────────────────


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


def _assistant_message_to_dict(message: Any) -> dict[str, Any]:
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


# ── Agent turn ─────────────────────────────────────────────────────────


def run_agent_turn(
    user_input: str,
    runtime: AgentRuntime,
    tools: list[dict[str, Any]],
    session: InteractiveSessionState,
    console: Console,
    execute_tool_fn: Callable = execute_tool,
) -> dict[str, Any]:
    if not session.messages:
        session.messages = [{"role": "system", "content": runtime.system_prompt}]

    session.messages.append({"role": "user", "content": user_input})
    tool_events = []
    final_reply = ""
    thinking_parts = []
    session_snapshot = {"sync": dict(session.last_sync)}

    robot_act_called = False
    for _ in range(4):
        response = runtime.client.chat.completions.create(
            model=runtime.model,
            messages=session.messages,
            tools=tools,
            tool_choice="auto",
            extra_body={"enable_thinking": False},
        )
        message = response.choices[0].message
        tool_calls = getattr(message, "tool_calls", None) or []
        content = getattr(message, "content", "") or ""

        if not tool_calls:
            final_reply = content
            session.messages.append({"role": "assistant", "content": final_reply})
            break

        if content:
            thinking_parts.append(content)
            _render_thinking_panel(console, content)

        session.messages.append(_assistant_message_to_dict(message))
        for tool_call in tool_calls:
            args = json.loads(tool_call.function.arguments or "{}")
            if tool_call.function.name == "robot_act" and session.last_observation:
                if not args.get("observation_context"):
                    args["observation_context"] = json.dumps(
                        session.last_observation.get("visual_context", {}),
                        ensure_ascii=False,
                    )
                if not args.get("scene_facts_json"):
                    merged_scene_facts = (session.last_observation.get("env_state") or {}).get("scene_facts")
                    args["scene_facts_json"] = json.dumps(
                        merged_scene_facts or session.last_observation.get("scene_facts", {}),
                        ensure_ascii=False,
                    )
            if tool_call.function.name == "robot_act" and not args.get("agent_thought"):
                thought = (content or "").strip()
                if not thought:
                    thought = f"用户要求执行动作：{user_input}"
                    if session.last_observation:
                        thought += "；已有最近一次环境观测"
                args["agent_thought"] = thought
            if tool_call.function.name == "vlm_observe" and not session.vlm_enabled:
                result = {
                    "success": False,
                    "result": {"status": "failure", "message": "当前会话已关闭 VLM"},
                    "tool_name": "vlm_observe",
                    "tool_args": args,
                    "feedback_summary": "当前会话已关闭 VLM",
                }
            else:
                result = execute_tool_fn(tool_call.function.name, args)

            tool_event = {
                "tool_name": result.get("tool_name", tool_call.function.name),
                "tool_args": result.get("tool_args", args),
                "success": result.get("success", False),
                "summary": result.get("feedback_summary", ""),
                "payload": result.get("result", {}),
                "error": result.get("error", ""),
            }
            tool_events.append(tool_event)
            _render_tool_result_panel(console, tool_event)
            if tool_event.get("tool_name") == "vlm_observe":
                _render_env_state_panel(console, tool_event.get("payload", {}))

            payload = result.get("result", result)
            if isinstance(payload, dict):
                session_snapshot = payload.get("session_snapshot", session_snapshot)
                if tool_call.function.name == "vlm_observe" and payload.get("status") == "success":
                    session.last_observation = payload
            session.messages.append(
                {
                    "role": "tool",
                    "tool_call_id": tool_call.id,
                    "content": json.dumps(payload, ensure_ascii=False),
                }
            )

            if tool_call.function.name == "robot_act":
                robot_act_called = True

        if robot_act_called:
            break

    if final_reply:
        console.print(Panel(final_reply, title="Assistant", border_style="bright_cyan", expand=False))

    summary = {
        "tool_calls": len(tool_events),
        "success_count": sum(1 for item in tool_events if item.get("success")),
    }
    return {
        "reply": final_reply,
        "thinking": "\n\n".join(thinking_parts),
        "tool_calls": tool_events,
        "summary": summary,
        "session_snapshot": session_snapshot,
    }


# ── Command handling ───────────────────────────────────────────────────


def render_session_snapshot(session: InteractiveSessionState) -> str:
    return json.dumps(
        {
            "vlm_enabled": session.vlm_enabled,
            "recent_inputs": session.recent_inputs,
            "last_sync": session.last_sync,
            "last_summary": session.last_summary,
        },
        ensure_ascii=False,
        indent=2,
    )


def _current_model_name() -> str:
    return os.getenv("FINALPROJECT_AGENT_MODEL", "qwen3.6-plus")


def _persist_env_setting(key: str, value: str) -> None:
    env_file = project_root / ".env"
    lines = env_file.read_text(encoding="utf-8").splitlines() if env_file.exists() else []
    updated = False
    prefix = f"{key}="
    for index, line in enumerate(lines):
        if line.startswith(prefix):
            lines[index] = f"{prefix}{value}"
            updated = True
            break
    if not updated:
        lines.append(f"{prefix}{value}")
    env_file.write_text("\n".join(lines).rstrip() + "\n", encoding="utf-8")


def handle_command(command: str, session: InteractiveSessionState, llm_builder: Callable) -> dict[str, Any]:
    normalized = command.strip().lower()
    if normalized == "/help":
        return {
            "handled": True,
            "message": (
                "Commands:\n"
                "/help   查看帮助\n"
                "/model  查看/切换模型\n"
                "/tools  查看分层能力结构\n"
                "/reset  重置当前会话上下文\n"
                "/status 查看最近一次会话状态\n"
                "/vlm    开关顶层感知能力\n"
                "/quit   退出程序\n\n"
                "Hierarchy:\n"
                "Top-level Tools:\n"
                "Vision Tool -> Vision Skills\n"
                "Action Tool -> Action Skills"
            ),
        }
    if normalized == "/model":
        return {
            "handled": True,
            "message": f"当前模型: {_current_model_name()}\n用法: /model qwen3.5-plus",
        }
    if normalized.startswith("/model "):
        parts = command.strip().split(maxsplit=1)
        if len(parts) < 2 or not parts[1].strip():
            return {
                "handled": True,
                "message": "用法: /model qwen3.5-plus",
            }
        model_name = parts[1].strip()
        os.environ["FINALPROJECT_AGENT_MODEL"] = model_name
        _persist_env_setting("FINALPROJECT_AGENT_MODEL", model_name)
        agent, tools = llm_builder()
        return {
            "handled": True,
            "message": f"模型已切换并保存为: {model_name}",
            "agent": agent,
            "tools": tools,
        }
    if normalized == "/status":
        return {"handled": True, "message": render_session_snapshot(session)}
    if normalized == "/vlm":
        session.vlm_enabled = not session.vlm_enabled
        return {"handled": True, "message": f"VLM 已切换为{'开启' if session.vlm_enabled else '关闭'}"}
    if normalized == "/reset":
        agent, tools = llm_builder()
        session.reset_runtime_state()
        system_prompt = getattr(agent, "system_prompt", "")
        if system_prompt:
            session.messages = [{"role": "system", "content": system_prompt}]
        return {"handled": True, "message": "会话上下文已重置", "agent": agent, "tools": tools}
    if normalized == "/quit":
        return {"handled": True, "continue_session": False, "message": "Bye!"}
    return {"handled": False}


# ── TUI Rendering ──────────────────────────────────────────────────────


def make_console() -> Console:
    return Console()


def build_tool_summary(agent_tools: list[dict[str, Any]]) -> dict[str, int]:
    from Robot_Module.agent_tools import get_vision_tool_definitions, get_action_tool_definitions
    return {
        "agent_tools": len(agent_tools),
        "vision_skills": len(get_vision_tool_definitions()),
        "action_skills": len(get_action_tool_definitions()),
    }


def show_welcome(console: Console, runtime: AgentRuntime, tools: list[dict[str, Any]], session: InteractiveSessionState) -> None:
    tool_summary = build_tool_summary(tools)
    header = Text()
    header.append("  Model: ", style="dim")
    header.append(runtime.model, style="bold green")
    header.append("\n  API: ", style="dim")
    header.append(str(getattr(runtime.client, "base_url", "unknown")), style="bold cyan")
    header.append("\n  VLM: ", style="dim")
    header.append("ON" if session.vlm_enabled else "OFF", style="bold yellow")
    header.append("\n  Agent Tools: ", style="dim")
    header.append(str(tool_summary["agent_tools"]), style="bold magenta")
    header.append("  (Vision / Action)", style="dim")
    header.append("\n  Vision Skills: ", style="dim")
    header.append(str(tool_summary["vision_skills"]), style="bold magenta")
    header.append("  (registered from Vision module)", style="dim")
    header.append("\n  Action Skills: ", style="dim")
    header.append(str(tool_summary["action_skills"]), style="bold magenta")
    header.append("  (registered from Action module)", style="dim")
    header.append("\n\n")
    header.append("/help /model /tools /reset /status /vlm /quit", style="bold cyan")
    console.print(Panel(header, title=WELCOME_TITLE, border_style="bright_cyan"))


def show_status(console: Console, runtime: AgentRuntime, session: InteractiveSessionState) -> None:
    console.print(
        f"[dim]{runtime.model}[/dim] │ [bold]scene:{session.last_sync.get('scene_id', '-')}[/bold] │ "
        f"[dim]ctx:{len(session.recent_inputs)}[/dim] │ [dim]vlm:{'on' if session.vlm_enabled else 'off'}[/dim]"
    )


def render_stream_line(console: Console, line: str) -> None:
    cleaned = _strip_ansi(line).rstrip()
    if cleaned:
        print(cleaned, file=sys.__stdout__)


def render_tools(console: Console, tools: list[dict[str, Any]]) -> None:
    from Robot_Module.agent_tools import get_vision_tool_definitions, get_action_tool_definitions
    console.print(_build_agent_tool_table())
    console.print(_build_tool_table("Vision Skills", get_vision_tool_definitions()))
    console.print(_build_tool_table("Action Skills", get_action_tool_definitions()))


def render_command_result(console: Console, message: str, title: str = "Command") -> None:
    console.print(Panel(message, title=title, border_style="cyan", expand=False))


# ── Internal render helpers ────────────────────────────────────────────


def _strip_ansi(text: str) -> str:
    return re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", text or "")


def _render_thinking_panel(console: Console, content: str) -> None:
    console.print(Panel(content, title="Thinking", border_style="bright_cyan", expand=False))


def _render_tool_result_panel(console: Console, tool_event: dict[str, Any]) -> None:
    tool_name = tool_event.get("tool_name", "")
    success = tool_event.get("success", False)
    border = "green" if success else "yellow"
    label = "SUCCESS" if success else "FAILURE"

    if tool_name == "robot_act" and isinstance(tool_event.get("payload"), dict):
        _render_robot_act_payload(console, tool_event)
        return

    payload = tool_event.get("payload", {})
    if not isinstance(payload, dict):
        payload = {}

    if tool_name == "vlm_observe" and payload.get("visual_context"):
        visual = payload["visual_context"]
        details = json.dumps(visual, ensure_ascii=False, indent=2)
    else:
        details = json.dumps(payload, ensure_ascii=False, indent=2)

    if tool_event.get("error"):
        details = f"error={tool_event['error']}\n\n{details}"
    console.print(
        Panel(
            details,
            title=f"{tool_name} {label}",
            border_style=border,
            expand=False,
        )
    )


def _render_env_state_panel(console: Console, payload: dict[str, Any]) -> None:
    if not isinstance(payload, dict):
        return
    env_state = payload.get("env_state")
    if not isinstance(env_state, dict) or not env_state:
        return
    compact = {
        "connected": env_state.get("connected"),
        "scene_id": env_state.get("scene_id"),
        "agent_position": env_state.get("agent_position"),
        "goal": env_state.get("goal"),
        "skill": env_state.get("skill"),
        "model_use": env_state.get("model_use"),
        "start": env_state.get("start"),
        "objects_count": env_state.get("objects_count", 0),
        "envtest_alignment": env_state.get("envtest_alignment"),
        "summary": env_state.get("summary"),
        "uncertainties": env_state.get("uncertainties", []),
    }
    console.print(
        Panel(
            json.dumps(compact, ensure_ascii=False, indent=2),
            title="Env State",
            border_style="bright_blue",
            expand=False,
        )
    )


def _render_robot_act_payload(console: Console, tool_call: dict[str, Any]) -> None:
    payload = tool_call.get("payload", {}) if isinstance(tool_call.get("payload", {}), dict) else {}
    summary = payload.get("summary", {})
    summary_text = (
        f"total_tasks={summary.get('total_tasks', 0)}\n"
        f"success_count={summary.get('success_count', 0)}\n"
        f"failure_count={summary.get('failure_count', 0)}"
    )
    if tool_call.get("error"):
        summary_text += f"\nerror={tool_call['error']}"
    console.print(
        Panel(
            summary_text,
            title=f"{tool_call.get('tool_name')} {'SUCCESS' if tool_call.get('success') else 'FAILURE'}",
            border_style="green" if tool_call.get("success") else "yellow",
            expand=False,
        )
    )


def _build_tool_table(title: str, tools: list[dict[str, Any]]) -> Table:
    table = Table(title=title, show_lines=True)
    table.add_column("Tool", style="bold cyan")
    table.add_column("Parameters", style="green")
    table.add_column("Description", style="white")
    for tool in tools:
        func = tool.get("function", {})
        params = func.get("parameters", {}).get("properties", {})
        table.add_row(func.get("name", "-"), ", ".join(params.keys()) or "-", func.get("description", "-"))
    return table


def _build_agent_tool_table() -> Table:
    table = Table(title="Top-level Tools", show_lines=True)
    table.add_column("Tool", style="bold cyan")
    table.add_column("Exposed Skill", style="green")
    table.add_column("Owns Skills", style="white")
    table.add_row("Vision Tool", "vlm_observe", "Vision Skills")
    table.add_row("Action Tool", "robot_act", "Action Skills")
    return table


# ── Build helpers ──────────────────────────────────────────────────────


def build_agent_runtime(client: Any) -> AgentRuntime:
    return AgentRuntime(
        client=client,
        model=os.getenv("FINALPROJECT_AGENT_MODEL", "qwen3.6-plus"),
        system_prompt=AGENT_SYSTEM_PROMPT,
    )


def build_llm_agent() -> tuple[AgentRuntime, list[dict[str, Any]]]:
    from Robot_Module.agent_tools import register_all_modules, get_agent_tool_definitions

    api_key = os.getenv("Test_API_KEY")
    if not api_key:
        print("❌ 错误: 未设置 Test_API_KEY 环境变量", file=sys.stderr)
        sys.exit(1)

    client = OpenAI(
        api_key=api_key,
        base_url=os.getenv("FINALPROJECT_BASE_URL", "https://dashscope.aliyuncs.com/compatible-mode/v1"),
    )
    with redirect_stderr(io.StringIO()):
        register_all_modules()
        tools = get_agent_tool_definitions()
    return build_agent_runtime(client), tools


# ── TUI main ───────────────────────────────────────────────────────────


def main() -> None:
    console = make_console()
    session = InteractiveSessionState()
    runtime, tools = build_llm_agent()
    session.messages = [{"role": "system", "content": runtime.system_prompt}]
    show_welcome(console, runtime, tools, session)

    while True:
        try:
            show_status(console, runtime, session)
            user_input = console.input("[bold cyan]You> [/bold cyan]").strip()
        except (EOFError, KeyboardInterrupt):
            console.print("\n[dim]Bye![/dim]")
            break

        if not user_input:
            continue

        if user_input.startswith("/"):
            if user_input.strip().lower() == "/tools":
                render_tools(console, tools)
                continue
            command_result = handle_command(user_input, session, llm_builder=build_llm_agent)
            if command_result.get("handled"):
                runtime = command_result.get("agent", runtime)
                tools = command_result.get("tools", tools)
                render_command_result(console, command_result.get("message", ""))
                if command_result.get("continue_session") is False:
                    break
                continue

        console.print("[dim]FinalProject is thinking...[/dim]")
        try:
            result = run_agent_turn(
                user_input,
                runtime,
                tools,
                session,
                console,
                execute_tool_fn=lambda name, args: execute_tool(name, args, event_callback=lambda line: render_stream_line(console, line)),
            )
        except Exception as error:
            render_command_result(console, f"请求失败: {error}", title="Error")
            continue

        session.record_interaction(user_input, result)


if __name__ == "__main__":
    main()
