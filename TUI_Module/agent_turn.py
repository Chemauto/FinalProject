"""Agent turn execution and tool dispatch for the Rich TUI."""

from __future__ import annotations

import asyncio
import io
import json
from contextlib import redirect_stderr
from typing import Any, Callable

from rich.console import Console
from rich.panel import Panel

from .renderers import (
    render_env_state_panel,
    render_thinking_panel,
    render_tool_result_panel,
)
from .session import AgentRuntime, InteractiveSessionState, assistant_message_to_dict


def execute_tool(function_name: str, function_args: dict[str, Any], event_callback=None) -> dict[str, Any]:
    """Dispatch one model-selected tool call."""
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
            render_thinking_panel(console, content)

        session.messages.append(assistant_message_to_dict(message))
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
            render_tool_result_panel(console, tool_event)
            if tool_event.get("tool_name") == "vlm_observe":
                render_env_state_panel(console, tool_event.get("payload", {}))

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

