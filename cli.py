#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cli.py — Rich TUI interactive entrypoint."""

from __future__ import annotations

import io
import logging
import os
import sys
from contextlib import redirect_stderr
from pathlib import Path
from typing import Any

from openai import OpenAI

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

from Robot_Module.tools import get_agent_tool_definitions, register_all
from TUI_Module.agent_turn import execute_tool, run_agent_turn
from TUI_Module.commands import handle_command
from TUI_Module.renderers import (
    make_console,
    make_pipeline_event_handler,
    render_command_result,
    render_tools,
    show_status,
    show_welcome,
)
from TUI_Module.session import AGENT_SYSTEM_PROMPT, AgentRuntime, InteractiveSessionState


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
        pipeline_event_handler = make_pipeline_event_handler(console)
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
