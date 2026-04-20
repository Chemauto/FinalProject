"""Runtime helpers for registered MCP tools."""

from __future__ import annotations

import asyncio
import json
import threading
from typing import Any


def normalize_tool_result(function_name: str, raw_result: Any) -> dict[str, Any]:
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
    validation = feedback.get("validation") if isinstance(feedback, dict) else None
    validation_success = True
    if isinstance(validation, dict) and validation:
        verified = validation.get("verified")
        meets_requirements = validation.get("meets_requirements")
        if verified is False or meets_requirements is False:
            validation_success = False
        elif verified is True and meets_requirements is True:
            validation_success = True

    return {
        "success": status == "success" and feedback.get("signal") == "SUCCESS" and validation_success,
        "feedback": feedback,
        "result": parsed,
    }


def run_async_blocking(coro):
    try:
        asyncio.get_running_loop()
    except RuntimeError:
        return asyncio.run(coro)

    result: dict[str, Any] = {}
    error: dict[str, BaseException] = {}

    def _runner():
        try:
            result["value"] = asyncio.run(coro)
        except BaseException as exc:
            error["value"] = exc

    thread = threading.Thread(target=_runner, daemon=True)
    thread.start()
    thread.join()
    if "value" in error:
        raise error["value"]
    return result.get("value")


def execute_registered_tool(
    function_name: str,
    function_args: dict[str, Any],
    tool_registry: dict[str, Any],
) -> dict[str, Any]:
    skill_func = tool_registry.get(function_name)
    if not skill_func:
        return {"success": False, "error": f"Unknown tool: {function_name}"}
    raw_result = run_async_blocking(skill_func(**function_args))
    normalized = normalize_tool_result(function_name, raw_result)
    return {
        "success": normalized["success"],
        "feedback": normalized["feedback"],
        "result": normalized["result"],
    }


def snapshot_tool_definitions(mcp) -> list[dict[str, Any]]:
    async def _get_tools():
        tools_list = await mcp.list_tools()
        tools = []
        for tool in tools_list:
            tools.append(
                {
                    "type": "function",
                    "function": {
                        "name": tool.name,
                        "description": tool.description if hasattr(tool, "description") else "",
                        "parameters": tool.inputSchema
                        if hasattr(tool, "inputSchema")
                        else {"type": "object", "properties": {}, "required": []},
                    },
                }
            )
        return tools

    return asyncio.run(_get_tools())

