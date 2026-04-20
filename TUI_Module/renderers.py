"""Rich renderers for commands, tool calls, and pipeline events."""

from __future__ import annotations

import json
from typing import Any

from rich.console import Console
from rich.panel import Panel
from rich.table import Table

from Robot_Module.tools import get_action_tool_definitions, get_vision_tool_definitions

from .commands import current_model_name, fetch_robot_state
from .session import AgentRuntime, InteractiveSessionState, WELCOME_TITLE


def make_console() -> Console:
    return Console()


def build_tool_summary(agent_tools: list[dict[str, Any]]) -> dict[str, int]:
    return {
        "agent_tools": len(agent_tools),
        "vision_skills": len(get_vision_tool_definitions()),
        "action_skills": len(get_action_tool_definitions()),
    }


def make_pipeline_event_handler(console: Console):
    """Create the pipeline event callback used by the TUI."""

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
        elif event_type == "step_progress":
            pose = data.get("pose")
            if pose and isinstance(pose, list) and len(pose) >= 2:
                console.print(f"  [dim]→ ({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f})[/dim]")

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
    robot_state = fetch_robot_state()
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


def render_thinking_panel(console: Console, content: str) -> None:
    console.print(Panel(content, title="Thinking", border_style="bright_cyan", expand=False))


def render_tool_result_panel(console: Console, tool_event: dict[str, Any]) -> None:
    tool_name = tool_event.get("tool_name", "")
    success = tool_event.get("success", False)
    border = "green" if success else "yellow"
    label = "SUCCESS" if success else "FAILURE"

    if tool_name == "robot_act" and isinstance(tool_event.get("payload"), dict):
        render_robot_act_payload(console, tool_event)
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


def render_env_state_panel(console: Console, payload: dict[str, Any]) -> None:
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


def render_robot_act_payload(console: Console, tool_call: dict[str, Any]) -> None:
    payload = tool_call.get("payload", {}) if isinstance(tool_call.get("payload", {}), dict) else {}
    results = payload.get("results", [])

    if results:
        table = Table(show_lines=False, expand=False)
        table.add_column("#", style="dim", width=3)
        table.add_column("Task", style="white")
        table.add_column("Skill", style="cyan")
        table.add_column("Result", width=6)
        for idx, result in enumerate(results, 1):
            icon = "[green]OK[/green]" if result.get("success") else "[red]FAIL[/red]"
            table.add_row(str(idx), result.get("task", "-"), result.get("action", "-"), icon)
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

