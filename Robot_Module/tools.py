"""Robot_Module/tools.py — 轻量工具注册中心。"""

from __future__ import annotations

import asyncio
import json
import os
import threading
from pathlib import Path
from typing import Any

from mcp.server.fastmcp import FastMCP

from Data_Module.facts import load_object_facts
from Data_Module.vlm import VLMCore
from Data_Module.params import ParameterCalculator
from Data_Module.context import build_planner_context
from Hardware_Module.registry import (
    sync_object_facts_from_live_data,
    sync_runtime_overrides_from_user_input,
)
from Hardware_Module import get_state
from Planner_Module.planner import Planner
from Excu_Module.pipeline import run_pipeline

from .tasks import register_tools as register_task_tools
from .vision.vlm_observe import register_tools as register_vision_tools

DEFAULT_OBJECT_FACTS_PATH = Path(
    os.getenv("FINALPROJECT_OBJECT_FACTS_PATH", str(Path(__file__).resolve().parents[1] / "config" / "object_facts.json"))
)

mcp = FastMCP("robot")

AGENT_TOOL_NAMES = {"vlm_observe", "robot_act"}

_tool_registry: dict[str, Any] = {}
_tool_definitions: list[dict[str, Any]] = []
_modules_registered = False


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


def _execute_registered_tool(function_name: str, function_args: dict[str, Any]) -> dict[str, Any]:
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
        except BaseException as exc:
            error["value"] = exc

    thread = threading.Thread(target=_runner, daemon=True)
    thread.start()
    thread.join()
    if "value" in error:
        raise error["value"]
    return result.get("value")


def _snapshot_tool_definitions() -> list[dict[str, Any]]:
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


def get_skill_function(name: str):
    return _tool_registry.get(name)


def get_tool_definitions(allowed_names: set[str] | None = None):
    if allowed_names is None:
        return list(_tool_definitions)
    return [tool for tool in _tool_definitions if tool.get("function", {}).get("name") in allowed_names]


def get_agent_tool_definitions():
    return get_tool_definitions(AGENT_TOOL_NAMES)


def get_vision_tool_definitions():
    return get_tool_definitions({"vlm"})


def get_action_tool_definitions():
    action_names = _tool_registry.keys() - {"vlm", "vlm_observe", "robot_act"}
    return get_tool_definitions(action_names)


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

    return _run_robot_act_pipeline(
        user_intent=user_intent,
        agent_thought=agent_thought,
        observation_context=observation_payload,
        scene_facts=scene_facts,
    )


async def vlm_observe(image_path: str = "") -> dict[str, Any]:
    skill_func = get_skill_function("vlm")
    if not skill_func:
        raise RuntimeError("Vision skill 'vlm' 未注册")

    raw_result = await skill_func(image_path=image_path)
    if isinstance(raw_result, str):
        try:
            return json.loads(raw_result)
        except json.JSONDecodeError:
            return {"status": "failure", "message": "vlm 返回了无法解析的结果", "raw_result": raw_result}
    if isinstance(raw_result, dict):
        return raw_result
    return {"status": "failure", "message": "vlm 返回了无效结果", "raw_result": raw_result}


def _run_robot_act_pipeline(
    user_intent: str,
    agent_thought: str = "",
    observation_context: dict[str, Any] | None = None,
    scene_facts: dict[str, Any] | None = None,
    object_facts_path: str | Path | None = None,
    on_event=None,
) -> dict[str, Any]:
    register_all()
    path = Path(object_facts_path or DEFAULT_OBJECT_FACTS_PATH)
    try:
        synced_payload = sync_object_facts_from_live_data(path, user_input=user_intent)
    except Exception:
        synced_payload = None
        try:
            sync_runtime_overrides_from_user_input(path, user_input=user_intent)
        except Exception:
            pass

    object_facts = load_object_facts(path)
    merged_scene_facts = scene_facts
    visual_context_text = json.dumps(observation_context, ensure_ascii=False) if observation_context else None
    if object_facts is not None:
        merged_scene_facts = VLMCore.merge_scene_facts(scene_facts, object_facts)
    planner_context = build_planner_context(merged_scene_facts, object_facts, synced_payload)

    api_key = os.getenv("Test_API_KEY")
    from openai import OpenAI

    client = OpenAI(
        api_key=api_key,
        base_url=os.getenv("FINALPROJECT_BASE_URL", "https://dashscope.aliyuncs.com/compatible-mode/v1"),
    )

    parameter_calculator = ParameterCalculator()
    prompt_path = str(Path(__file__).resolve().parents[1] / "Planner_Module" / "prompts" / "highlevel_prompt.yaml")
    planner = Planner(client=client, prompt_path=prompt_path, parameter_calculator=parameter_calculator)
    action_tools = get_action_tool_definitions()

    # 获取 robot_state 给规划 LLM 参考
    robot_state = {}
    try:
        raw_state = get_state()
        if raw_state and raw_state.get("connected"):
            robot_state = raw_state
    except Exception:
        pass

    import yaml
    data = yaml.safe_load(Path(prompt_path).read_text(encoding="utf-8")) or {}
    planner.planning_prompt_template = str(data.get("prompt", "")).strip()

    results = run_pipeline(
        user_input=user_intent,
        planner=planner,
        execute_tool_fn=_execute_registered_tool,
        tools=action_tools,
        visual_context=visual_context_text,
        scene_facts=planner_context,
        object_facts=object_facts,
        agent_thought=agent_thought,
        robot_state=robot_state,
        max_replans=0,
        on_event=on_event,
    )

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
        "session_snapshot": {
            "sync": {
                "scene_id": runtime_state.get("scene_id"),
                "model_use": runtime_state.get("model_use"),
                "objects_count": len((synced_payload or {}).get("objects") or []),
            }
        },
    }


def register_all():
    """注册所有工具模块。"""
    global _modules_registered, _tool_definitions
    if _modules_registered:
        return

    _tool_registry.update(register_task_tools(mcp))
    _tool_registry.update(register_vision_tools(mcp))
    mcp.tool()(vlm_observe)
    _tool_registry["vlm_observe"] = vlm_observe
    mcp.tool()(robot_act)
    _tool_registry["robot_act"] = robot_act
    _tool_definitions = _snapshot_tool_definitions()
    _modules_registered = True
