#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

import asyncio
import io
import json
import logging
import os
import re
import sys
from contextlib import redirect_stderr
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from openai import OpenAI
from rich.console import Console
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

for logger_name in ("openai", "httpx"):
    logging.getLogger(logger_name).setLevel(logging.WARNING)

for var in ("http_proxy", "https_proxy", "HTTP_PROXY", "HTTPS_PROXY", "ALL_PROXY", "all_proxy", "no_proxy", "NO_PROXY"):
    os.environ.pop(var, None)

project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

try:
    from dotenv import load_dotenv

    env_file = project_root / ".env"
    if env_file.exists():
        load_dotenv(env_file)
except ImportError:
    pass

from Robot_Module.skill import (
    get_action_tool_definitions,
    get_agent_tool_definitions,
    get_skill_function,
    get_vision_tool_definitions,
    register_all_modules,
)

WELCOME_TITLE = "LQN Claw TUI"
AGENT_SYSTEM_PROMPT = """你是一个机器人智能体，不是机器人本体。
你可以直接和用户对话，也可以调用技能。
可用技能只有两类：
- vlm_observe：观察当前环境，只返回观测结果
- robot_act：在需要动作时，调用机器人内部规划与执行链

规则：
- 一般机器人的执行流程会根据这个任务是否需要观测先判断，如果一个动作执行前是需要和环境交互或者是在环境中运动一般需要先观测再执行
- 需要环境信息但不需要动作时，调用 vlm_observe
- 需要物理动作时，必要时先调用 vlm_observe，再调用 robot_act
- 调用 robot_act 时，附带一个简短的 agent_thought，说明你为什么要执行这个动作、当前如何理解任务
- 如果已经调用过 vlm_observe，再调用 robot_act 时，应优先把 visual_context 传入 observation_context，把 scene_facts 传入 scene_facts_json
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


def _normalize_tool_result(function_name: str, raw_result: Any) -> dict[str, Any]:
    parsed = raw_result
    if isinstance(raw_result, str):
        try:
            parsed = json.loads(raw_result)
        except json.JSONDecodeError:
            parsed = {"raw_result": raw_result}
    if not isinstance(parsed, dict):
        parsed = {"raw_result": parsed}

    status = parsed.get("status", "success")
    feedback = parsed.get("execution_feedback") or {
        "signal": "SUCCESS" if status == "success" else "FAILURE",
        "skill": function_name,
        "message": f"{function_name} 执行{'成功' if status == 'success' else '失败'}",
    }
    return {
        "success": status == "success" and feedback.get("signal") == "SUCCESS",
        "result": parsed,
        "feedback": feedback,
    }


def execute_tool(function_name: str, function_args: dict[str, Any], event_callback=None) -> dict[str, Any]:
    if function_name == "robot_act":
        from Robot_Module.agent_tools import run_robot_act_pipeline

        try:
            observation_context = None
            if function_args.get("observation_context"):
                try:
                    observation_context = json.loads(function_args.get("observation_context") or "null")
                except json.JSONDecodeError:
                    observation_context = {"text": function_args.get("observation_context")}

            scene_facts = None
            if function_args.get("scene_facts_json"):
                try:
                    scene_facts = json.loads(function_args.get("scene_facts_json") or "null")
                except json.JSONDecodeError:
                    scene_facts = None

            result = run_robot_act_pipeline(
                user_intent=function_args.get("user_intent", ""),
                agent_thought=function_args.get("agent_thought", ""),
                observation_context=observation_context,
                scene_facts=scene_facts,
                log_callback=event_callback,
            )
            return {
                "success": result.get("status") == "success",
                "result": result,
                "tool_name": function_name,
                "tool_args": function_args,
                "feedback_summary": f"robot_act finished with {result.get('summary', {}).get('success_count', 0)} success steps",
            }
        except Exception as error:
            return {
                "success": False,
                "error": str(error),
                "tool_name": function_name,
                "tool_args": function_args,
                "feedback_summary": str(error),
            }

    skill_func = get_skill_function(function_name)
    if not skill_func:
        return {
            "success": False,
            "error": f"Unknown tool: {function_name}",
            "tool_name": function_name,
            "tool_args": function_args,
            "feedback_summary": f"{function_name} 未注册",
        }

    try:
        normalized = _normalize_tool_result(function_name, asyncio.run(skill_func(**function_args)))
        return {
            "success": normalized["success"],
            "result": normalized["result"],
            "feedback": normalized["feedback"],
            "tool_name": function_name,
            "tool_args": function_args,
            "feedback_summary": normalized["feedback"].get("message", ""),
        }
    except Exception as error:
        return {
            "success": False,
            "error": str(error),
            "tool_name": function_name,
            "tool_args": function_args,
            "feedback_summary": str(error),
        }


def build_agent_runtime(client: Any) -> AgentRuntime:
    return AgentRuntime(
        client=client,
        model=os.getenv("FINALPROJECT_AGENT_MODEL", "qwen3.6-plus"),
        system_prompt=AGENT_SYSTEM_PROMPT,
    )


def build_llm_agent() -> tuple[AgentRuntime, list[dict[str, Any]]]:
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


def run_agent_turn(
    user_input: str,
    runtime: AgentRuntime,
    tools: list[dict[str, Any]],
    session: InteractiveSessionState,
    execute_tool_fn=execute_tool,
) -> dict[str, Any]:
    if not session.messages:
        session.messages = [{"role": "system", "content": runtime.system_prompt}]

    session.messages.append({"role": "user", "content": user_input})
    tool_events = []
    final_reply = ""
    session_snapshot = {"sync": dict(session.last_sync)}

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
                    args["scene_facts_json"] = json.dumps(
                        session.last_observation.get("scene_facts", {}),
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

            tool_events.append(
                {
                    "tool_name": result.get("tool_name", tool_call.function.name),
                    "tool_args": result.get("tool_args", args),
                    "success": result.get("success", False),
                    "summary": result.get("feedback_summary", ""),
                    "payload": result.get("result", {}),
                    "error": result.get("error", ""),
                }
            )
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

    summary = {
        "tool_calls": len(tool_events),
        "success_count": sum(1 for item in tool_events if item.get("success")),
    }
    return {
        "reply": final_reply,
        "tool_calls": tool_events,
        "summary": summary,
        "session_snapshot": session_snapshot,
    }


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


def handle_command(command: str, session: InteractiveSessionState, llm_builder) -> dict[str, Any]:
    normalized = command.strip().lower()
    if normalized == "/help":
        return {
            "handled": True,
            "message": (
                "Commands:\n"
                "/help   查看帮助\n"
                "/tools  查看分层能力结构\n"
                "/reset  重置当前会话上下文\n"
                "/status 查看最近一次会话状态\n"
                "/vlm    开关顶层感知能力\n"
                "/quit   退出程序\n\n"
                "Hierarchy:\n"
                "Agent Tools:\n"
                "Perception Tool -> Vision Skills\n"
                "Action Tool -> Action Skills"
            ),
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


def make_console() -> Console:
    return Console()


def build_tool_summary(agent_tools: list[dict[str, Any]]) -> dict[str, int]:
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
    header.append("  (Perception / Action)", style="dim")
    header.append("\n  Vision Skills: ", style="dim")
    header.append(str(tool_summary["vision_skills"]), style="bold magenta")
    header.append("  (inside Perception Tool)", style="dim")
    header.append("\n  Action Skills: ", style="dim")
    header.append(str(tool_summary["action_skills"]), style="bold magenta")
    header.append("  (inside Action Tool)", style="dim")
    header.append("\n\n")
    header.append("/help /tools /reset /status /vlm /quit", style="bold cyan")
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
    table = Table(title="Agent Tools", show_lines=True)
    table.add_column("Tool", style="bold cyan")
    table.add_column("Exposed Skill", style="green")
    table.add_column("Owns Skills", style="white")
    table.add_row("Perception Tool", "vlm_observe", "Vision Skills")
    table.add_row("Action Tool", "robot_act", "Action Skills")
    return table


def render_tools(console: Console, tools: list[dict[str, Any]]) -> None:
    console.print(_build_agent_tool_table())
    console.print(_build_tool_table("Vision Skills", get_vision_tool_definitions()))
    console.print(_build_tool_table("Action Skills", get_action_tool_definitions()))


def render_command_result(console: Console, message: str, title: str = "Command") -> None:
    console.print(Panel(message, title=title, border_style="cyan", expand=False))


def _strip_ansi(text: str) -> str:
    return re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", text or "")


def _summarize_robot_act_results(payload: dict[str, Any]) -> str:
    lines = []
    for index, item in enumerate(payload.get("results", []), start=1):
        action = item.get("action", "未调用")
        task = item.get("task", "未记录任务")
        if item.get("success"):
            lines.append(f"{index}. {task} -> {action} [SUCCESS]")
        else:
            error = item.get("error") or item.get("feedback", {}).get("message", "执行失败")
            lines.append(f"{index}. {task} -> {action} [FAILURE] {error}")
    return "\n".join(lines)


def _render_robot_act_payload(console: Console, tool_call: dict[str, Any]) -> None:
    payload = tool_call.get("payload", {}) if isinstance(tool_call.get("payload", {}), dict) else {}
    summary = payload.get("summary", {})
    summary_text = (
        f"args={json.dumps(tool_call.get('tool_args', {}), ensure_ascii=False)}\n\n"
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

    log_text = _strip_ansi(payload.get("log", "")).strip()
    if log_text:
        console.print(Panel(log_text, title="Robot Planning / Execution", border_style="cyan", expand=False))

    result_summary = _summarize_robot_act_results(payload)
    if result_summary:
        console.print(Panel(result_summary, title="Robot Results", border_style="magenta", expand=False))


def render_agent_result(console: Console, result: dict[str, Any]) -> None:
    for tool_call in result.get("tool_calls", []):
        if tool_call.get("tool_name") == "robot_act" and isinstance(tool_call.get("payload"), dict):
            _render_robot_act_payload(console, tool_call)
            continue

        payload = tool_call.get("payload", {})
        details = json.dumps(payload, ensure_ascii=False, indent=2) if isinstance(payload, dict) else str(payload)
        if tool_call.get("error"):
            details = f"error={tool_call['error']}\n\n{details}"
        console.print(
            Panel(
                f"args={json.dumps(tool_call.get('tool_args', {}), ensure_ascii=False)}\n\n{details}",
                title=f"{tool_call.get('tool_name')} {'SUCCESS' if tool_call.get('success') else 'FAILURE'}",
                border_style="green" if tool_call.get("success") else "yellow",
                expand=False,
            )
        )

    reply = result.get("reply") or "(empty response)"
    console.print(Panel(reply, title="Assistant", border_style="bright_cyan", expand=False))


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
                execute_tool_fn=lambda name, args: execute_tool(name, args, event_callback=lambda line: render_stream_line(console, line)),
            )
        except Exception as error:
            render_command_result(console, f"请求失败: {error}", title="Error")
            continue

        render_agent_result(console, result)
        session.record_interaction(user_input, result)


if __name__ == "__main__":
    main()
