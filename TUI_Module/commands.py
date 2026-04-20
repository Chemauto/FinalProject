"""Slash command handling for the Rich TUI."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Callable

from rich.table import Table

from .session import InteractiveSessionState


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def current_model_name() -> str:
    return os.getenv("FINALPROJECT_AGENT_MODEL", "qwen3.6-plus")


def persist_env_setting(key: str, value: str) -> None:
    env_file = PROJECT_ROOT / ".env"
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


def fetch_robot_state() -> dict[str, Any]:
    """Return a compact live robot-state summary for status rendering."""
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
            "message": f"当前模型: {current_model_name()}\n用法: /model qwen3.5-plus",
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
        persist_env_setting("FINALPROJECT_AGENT_MODEL", model_name)
        agent, tools = llm_builder()
        return {
            "handled": True,
            "message": f"模型已切换并保存为: {model_name}",
            "agent": agent,
            "tools": tools,
        }
    if normalized == "/status":
        robot_state = fetch_robot_state()
        table = Table(show_lines=False, expand=False, title="Status")
        table.add_column("Key", style="dim", width=16)
        table.add_column("Value")
        table.add_row("Model", current_model_name())
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

