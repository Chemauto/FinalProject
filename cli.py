#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cli.py — TUI 交互界面入口。"""

from __future__ import annotations

import io
import json
import logging
import os
import sys
from contextlib import redirect_stderr
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

from openai import OpenAI
from rich.console import Console
from rich.panel import Panel
from rich.table import Table

for logger_name in ("openai", "httpx"):
    logging.getLogger(logger_name).setLevel(logging.WARNING)

for var in ("http_proxy", "https_proxy", "HTTP_PROXY", "HTTPS_PROXY", "ALL_PROXY", "all_proxy", "no_proxy", "NO_PROXY"):
    os.environ.pop(var, None)

project_root = Path(__file__).resolve().parent
sys.path.insert(0, str(project_root))

try:
    from dotenv import load_dotenv
    env_file = project_root / ".env"
    if env_file.exists():
        load_dotenv(env_file)
except ImportError:
    pass

from Robot_Module.tools import (
    register_all,
    get_agent_tool_definitions,
    get_vision_tool_definitions,
    get_action_tool_definitions,
)


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


def execute_tool(function_name: str, function_args: dict[str, Any], event_callback=None) -> dict[str, Any]:
    """工具分发入口。"""
    if function_name == "robot_act":
        from Robot_Module.tools import _run_robot_act_pipeline

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

            with redirect_stderr(io.StringIO()):
                result = _run_robot_act_pipeline(
                    user_intent=function_args.get("user_intent", ""),
                    agent_thought=function_args.get("agent_thought", ""),
                    observation_context=observation_context,
                    scene_facts=scene_facts,
                    on_event=event_callback,
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

    from Robot_Module.tools import get_skill_function
    import asyncio

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
        raw_result = asyncio.run(skill_func(**function_args))
        if isinstance(raw_result, str):
            try:
                parsed = json.loads(raw_result)
            except json.JSONDecodeError:
                parsed = {"raw_result": raw_result}
        else:
            parsed = raw_result
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
            "tool_name": function_name,
            "tool_args": function_args,
            "feedback_summary": feedback.get("message", ""),
        }
    except Exception as error:
        return {
            "success": False,
            "error": str(error),
            "tool_name": function_name,
            "tool_args": function_args,
            "feedback_summary": str(error),
        }


def run_agent_turn(
    user_input: str,
    runtime: AgentRuntime,
    tools: list[dict[str, Any]],
    session: InteractiveSessionState,
    console: Console,
    execute_tool_fn: Callable = execute_tool,
    event_callback=None,
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
                result = execute_tool_fn(tool_call.function.name, args, event_callback=event_callback)

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
        table = Table(show_lines=False, expand=False, title="Commands")
        table.add_column("Command", style="bold cyan", width=8)
        table.add_column("Description", style="white")
        for cmd, desc in [
            ("/help", "查看帮助"),
            ("/model", "查看/切换模型"),
            ("/tools", "查看分层能力结构"),
            ("/status", "查看机器人实时状态"),
            ("/reset", "重置会话上下文"),
            ("/vlm", "开关感知能力"),
            ("/quit", "退出程序"),
        ]:
            table.add_row(cmd, desc)
        return {"handled": True, "rich_table": table}
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
        robot_state = _fetch_robot_state()
        table = Table(show_lines=False, expand=False, title="Status")
        table.add_column("Key", style="dim", width=16)
        table.add_column("Value")
        table.add_row("Model", _current_model_name())
        table.add_row("VLM", "on" if session.vlm_enabled else "off")
        table.add_row("Connected", "[green]ON[/green]" if robot_state.get("connected") else "[red]OFF[/red]")
        table.add_row("Scene", str(robot_state.get("scene_id", "-")))
        pose = robot_state.get("robot_pose") or ["-", "-", "-"]
        table.add_row("Position", f"({pose[0]}, {pose[1]}, {pose[2]})" if isinstance(pose, list) else str(pose))
        table.add_row("Skill", str(robot_state.get("skill", "-")))
        table.add_row("Inputs", str(len(session.recent_inputs)))
        return {"handled": True, "rich_table": table}
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
    return {
        "agent_tools": len(agent_tools),
        "vision_skills": len(get_vision_tool_definitions()),
        "action_skills": len(get_action_tool_definitions()),
    }


def _fetch_robot_state() -> dict[str, Any]:
    """获取机器人实时状态摘要。"""
    try:
        from Hardware_Module import get_state
        from Excu_Module.state import summarize_state
        state = get_state()
        if state and isinstance(state, dict):
            summary = summarize_state(state)
            if summary:
                return summary
    except Exception:
        pass
    return {}


def _make_pipeline_event_handler(console: Console):
    """创建 pipeline 事件回调，用 Rich Panel 渲染。"""
    def on_event(event_type: str, data: dict[str, Any]):
        if event_type == "plan_start":
            console.print(Panel(
                f"[bold]{data['user_input']}[/bold]",
                title="User Intent", border_style="bright_blue", expand=False,
            ))
        elif event_type == "llm_planning":
            console.print("[dim cyan]Planning...[/dim cyan]")
        elif event_type == "plan_done":
            tasks = data.get("tasks", [])
            summary = data.get("summary", "")
            if tasks:
                table = Table(show_lines=False, expand=False)
                table.add_column("#", style="dim", width=3)
                table.add_column("Task", style="white")
                table.add_column("Skill", style="cyan")
                for idx, task in enumerate(tasks, 1):
                    table.add_row(str(idx), task.get("task", "-"), task.get("function", "-"))
                console.print(Panel(
                    table,
                    title=f"Plan ({len(tasks)} steps): {summary}",
                    border_style="bright_blue", expand=False,
                ))
            else:
                console.print(f"[dim]Plan: {summary}[/dim]")
        elif event_type == "plan_thinking":
            thinking = data.get("thinking", "")
            if thinking:
                console.print(Panel(
                    thinking,
                    title="LLM Reasoning", border_style="dim cyan", expand=False,
                ))
        elif event_type == "execute_start":
            console.print("[dim]Executing...[/dim]")
        elif event_type == "step_start":
            idx, total = data["idx"], data["total"]
            func = data["function"]
            params = data.get("params", {})
            reason = data.get("reason", "")
            console.print(f"\n[bold cyan]Step {idx}/{total}[/] [dim]{func}[/]")
            if reason:
                console.print(f"  [dim]reason: {reason}[/dim]")
            if params:
                console.print(f"  [dim]params: {json.dumps(params, ensure_ascii=False)}[/dim]")
        elif event_type == "step_done":
            icon = "OK" if data["success"] else "FAIL"
            style = "green" if data["success"] else "red"
            console.print(f"  [{style}]{icon}:[/] {data['message']}")
        elif event_type == "pipeline_done":
            results = data.get("results", [])
            success = sum(1 for r in results if r.get("success"))
            fail = len(results) - success
            border = "green" if fail == 0 else "yellow"
            console.print(Panel(
                f"Total: {len(results)}  Success: {success}  Fail: {fail}",
                title="Pipeline Summary", border_style=border, expand=False,
            ))
        elif event_type == "pipeline_error":
            console.print(Panel(
                f"[red]{data['error']}[/red]",
                title="Pipeline Error", border_style="red", expand=False,
            ))
        elif event_type == "plan_error":
            console.print(f"[red]Plan error: {data['error']}[/red] [dim]using fallback[/dim]")
        elif event_type == "replan":
            console.print(f"  [yellow]Replan: {data['message']}[/yellow]")
        elif event_type == "rule_override":
            console.print(f"  [yellow]{data['message']}[/yellow]")
    return on_event


def show_welcome(console: Console, runtime: AgentRuntime, tools: list[dict[str, Any]], session: InteractiveSessionState) -> None:
    tool_summary = build_tool_summary(tools)
    table = Table(show_lines=False, expand=False)
    table.add_column("Key", style="dim", width=14)
    table.add_column("Value")
    table.add_row("Model", runtime.model)
    base_url = str(getattr(runtime.client, "base_url", "unknown"))
    if len(base_url) > 40:
        base_url = base_url[:37] + "..."
    table.add_row("API", base_url)
    vlm_style = "green" if session.vlm_enabled else "red"
    table.add_row("VLM", f"[{vlm_style}]{'ON' if session.vlm_enabled else 'OFF'}[/{vlm_style}]")
    table.add_row("Skills", f"Vision:{tool_summary['vision_skills']}  Action:{tool_summary['action_skills']}")
    table.add_row("Commands", "/help /model /tools /reset /status /vlm /quit")
    console.print(Panel(table, title=WELCOME_TITLE, border_style="bright_cyan"))


def show_status(console: Console, runtime: AgentRuntime, session: InteractiveSessionState) -> None:
    robot_state = _fetch_robot_state()
    connected = robot_state.get("connected", False)
    pose = robot_state.get("robot_pose") or ["-", "-", "-"]
    scene = robot_state.get("scene_id", "-")

    conn_icon = "[green]ON[/green]" if connected else "[red]OFF[/red]"
    pose_str = f"({pose[0]}, {pose[1]}, {pose[2]})" if isinstance(pose, list) else str(pose)
    status_line = (
        f"[dim]{runtime.model}[/dim] │ "
        f"Conn:{conn_icon} │ "
        f"Scene:{scene} │ "
        f"Pos:{pose_str} │ "
        f"VLM:{'on' if session.vlm_enabled else 'off'}"
    )
    console.print(status_line)


def render_tools(console: Console, tools: list[dict[str, Any]]) -> None:
    console.print(_build_agent_tool_table())
    console.print(_build_tool_table("Vision Skills", get_vision_tool_definitions()))
    console.print(_build_tool_table("Action Skills", get_action_tool_definitions()))


def render_command_result(console: Console, result: dict[str, Any], title: str = "Command") -> None:
    if result.get("rich_table"):
        console.print(result["rich_table"])
    else:
        console.print(Panel(result.get("message", ""), title=title, border_style="cyan", expand=False))


# ── Internal render helpers ────────────────────────────────────────────


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
    table = Table(show_lines=False, expand=False, title="Env State")
    table.add_column("Key", style="dim")
    table.add_column("Value")
    for key in ("connected", "scene_id", "agent_position", "goal", "skill", "model_use", "start", "objects_count"):
        val = env_state.get(key, "-")
        table.add_row(key, str(val))
    console.print(table)


def _render_robot_act_payload(console: Console, tool_call: dict[str, Any]) -> None:
    payload = tool_call.get("payload", {}) if isinstance(tool_call.get("payload", {}), dict) else {}
    results = payload.get("results", [])

    if results:
        table = Table(show_lines=False, expand=False)
        table.add_column("#", style="dim", width=3)
        table.add_column("Task", style="white")
        table.add_column("Skill", style="cyan")
        table.add_column("Result", width=6)
        for idx, r in enumerate(results, 1):
            icon = "[green]OK[/green]" if r.get("success") else "[red]FAIL[/red]"
            table.add_row(str(idx), r.get("task", "-"), r.get("action", "-"), icon)
    else:
        summary = payload.get("summary", {})
        total = summary.get("total_tasks", 0)
        success = summary.get("success_count", 0)
        fail = summary.get("failure_count", 0)
        table = f"Total: {total}  Success: {success}  Fail: {fail}"

    border = "green" if tool_call.get("success") else "yellow"
    label = "SUCCESS" if tool_call.get("success") else "FAILURE"
    console.print(Panel(table, title=f"robot_act {label}", border_style=border, expand=False))


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
    api_key = os.getenv("Test_API_KEY")
    if not api_key:
        print("❌ 错误: 未设置 Test_API_KEY 环境变量", file=sys.stderr)
        sys.exit(1)

    client = OpenAI(
        api_key=api_key,
        base_url=os.getenv("FINALPROJECT_BASE_URL", "https://dashscope.aliyuncs.com/compatible-mode/v1"),
    )
    with redirect_stderr(io.StringIO()):
        register_all()
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
                render_command_result(console, command_result)
                if command_result.get("continue_session") is False:
                    break
                continue

        console.print("[dim]FinalProject is thinking...[/dim]")
        pipeline_event_handler = _make_pipeline_event_handler(console)
        try:
            result = run_agent_turn(
                user_input,
                runtime,
                tools,
                session,
                console,
                execute_tool_fn=execute_tool,
                event_callback=pipeline_event_handler,
            )
        except Exception as error:
            render_command_result(console, {"message": f"请求失败: {error}"}, title="Error")
            continue

        session.record_interaction(user_input, result)


if __name__ == "__main__":
    main()
