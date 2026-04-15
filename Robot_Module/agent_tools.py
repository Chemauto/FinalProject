from __future__ import annotations

import asyncio
import io
import json
import os
import threading
from contextlib import redirect_stdout
from pathlib import Path
from typing import Any

from LLM_Module.llm_core import LLMAgent
from LLM_Module.object_facts_loader import load_object_facts
from VLM_Module.vlm_core import VLMCore
from Comm_Module.Robot.Sim.envtest_status_sync import (
    sync_object_facts_from_live_envtest,
    sync_runtime_overrides_from_user_input,
)
DEFAULT_OBJECT_FACTS_PATH = Path(
    os.getenv("FINALPROJECT_OBJECT_FACTS_PATH", str(Path(__file__).resolve().parents[1] / "config" / "object_facts.json"))
)


class _StreamingBuffer(io.StringIO):
    def __init__(self, log_callback=None):
        super().__init__()
        self._log_callback = log_callback
        self._partial = ""

    def write(self, text: str) -> int:
        count = super().write(text)
        if not self._log_callback or not text:
            return count

        self._partial += text
        while "\n" in self._partial:
            line, self._partial = self._partial.split("\n", 1)
            if line.strip():
                self._log_callback(line)
        return count

    def flush_pending(self) -> None:
        if self._log_callback and self._partial.strip():
            self._log_callback(self._partial)
        self._partial = ""


def _format_available_skills(tools: list[dict[str, Any]]) -> str:
    lines = []
    for tool in tools:
        func = tool.get("function", {})
        params = func.get("parameters", {}).get("properties", {})
        lines.append(f"  - {func.get('name', '')}({', '.join(params.keys())})")
    return "\n".join(lines)


def _load_robot_prompt(tools: list[dict[str, Any]]) -> str:
    import yaml

    prompt_path = Path(__file__).resolve().parents[1] / "LLM_Module" / "prompts" / "highlevel_prompt.yaml"
    data = yaml.safe_load(prompt_path.read_text(encoding="utf-8")) or {}
    return str(data.get("prompt", "")).strip()


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
        "feedback": feedback,
        "result": parsed,
    }


def _execute_registered_tool(function_name: str, function_args: dict[str, Any]) -> dict[str, Any]:
    from Robot_Module.skill import get_skill_function

    skill_func = get_skill_function(function_name)
    if not skill_func:
        return {"success": False, "error": f"Unknown tool: {function_name}"}
    raw_result = _run_async_blocking(skill_func(**function_args))
    normalized = _normalize_tool_result(function_name, raw_result)
    return {
        "success": normalized["success"],
        "feedback": normalized["feedback"],
        "result": normalized["result"],
    }


def _run_async_blocking(coro):
    try:
        asyncio.get_running_loop()
    except RuntimeError:
        return asyncio.run(coro)

    result: dict[str, Any] = {}
    error: dict[str, BaseException] = {}

    def _runner():
        try:
            result["value"] = asyncio.run(coro)
        except BaseException as exc:  # pragma: no cover
            error["value"] = exc

    thread = threading.Thread(target=_runner, daemon=True)
    thread.start()
    thread.join()
    if "value" in error:
        raise error["value"]
    return result.get("value")


def run_robot_act_pipeline(
    user_intent: str,
    agent_thought: str = "",
    observation_context: dict[str, Any] | None = None,
    scene_facts: dict[str, Any] | None = None,
    object_facts_path: str | Path | None = None,
    log_callback=None,
) -> dict[str, Any]:
    from Robot_Module.skill import get_action_tool_definitions, register_all_modules

    register_all_modules()
    path = Path(object_facts_path or DEFAULT_OBJECT_FACTS_PATH)
    try:
        synced_payload = sync_object_facts_from_live_envtest(path, user_input=user_intent)
    except Exception:
        synced_payload = None
        sync_runtime_overrides_from_user_input(path, user_input=user_intent)

    object_facts = load_object_facts(path)
    merged_scene_facts = scene_facts
    visual_context_text = json.dumps(observation_context, ensure_ascii=False) if observation_context else None
    if object_facts is not None:
        merged_scene_facts = VLMCore.merge_scene_facts(scene_facts, object_facts)

    api_key = os.getenv("Test_API_KEY")
    llm_agent = LLMAgent(
        api_key=api_key,
        prompt_path=str(Path(__file__).resolve().parents[1] / "LLM_Module" / "prompts" / "highlevel_prompt.yaml"),
    )
    action_tools = get_action_tool_definitions()
    llm_agent.planning_prompt_template = _load_robot_prompt(action_tools)

    pipeline_stdout = _StreamingBuffer(log_callback=log_callback)
    with redirect_stdout(pipeline_stdout):
        results = llm_agent.run_pipeline(
            user_input=user_intent,
            agent_thought=agent_thought,
            tools=action_tools,
            execute_tool_fn=_execute_registered_tool,
            visual_context=visual_context_text,
            scene_facts=merged_scene_facts,
            object_facts=object_facts,
        )
    pipeline_stdout.flush_pending()

    success_count = sum(1 for item in results if item.get("success"))
    runtime_state = (synced_payload or {}).get("runtime_state") or {}
    return {
        "status": "success" if results else "warning",
        "summary": {
            "total_tasks": len(results),
            "success_count": success_count,
            "failure_count": len(results) - success_count,
        },
        "results": results,
        "log": pipeline_stdout.getvalue().strip(),
        "session_snapshot": {
            "sync": {
                "scene_id": runtime_state.get("scene_id"),
                "model_use": runtime_state.get("model_use"),
                "objects_count": len((synced_payload or {}).get("objects") or []),
            }
        },
    }


async def robot_act(
    user_intent: str,
    agent_thought: str = "",
    observation_context: str = "",
    scene_facts_json: str = "",
) -> dict[str, Any]:
    """根据用户动作目标执行机器人内部规划与动作链。"""
    observation_payload = None
    if observation_context:
        try:
            observation_payload = json.loads(observation_context)
        except json.JSONDecodeError:
            observation_payload = {"text": observation_context}

    scene_facts = None
    if scene_facts_json:
        try:
            scene_facts = json.loads(scene_facts_json)
        except json.JSONDecodeError:
            scene_facts = None

    return run_robot_act_pipeline(
        user_intent=user_intent,
        agent_thought=agent_thought,
        observation_context=observation_payload,
        scene_facts=scene_facts,
    )


def register_tools(mcp):
    mcp.tool()(robot_act)
    return {
        "robot_act": robot_act,
    }
