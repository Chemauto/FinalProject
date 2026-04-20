"""Robot_Module/tools.py — 轻量工具注册中心。"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from mcp.server.fastmcp import FastMCP

from .pipeline_factory import run_robot_act_pipeline
from .tasks import register_tools as register_task_tools
from .tool_runtime import execute_registered_tool, snapshot_tool_definitions
from .vision.vlm_observe import register_tools as register_vision_tools

mcp = FastMCP("robot")

AGENT_TOOL_NAMES = {"vlm_observe", "robot_act"}

_tool_registry: dict[str, Any] = {}
_tool_definitions: list[dict[str, Any]] = []
_modules_registered = False


def _execute_registered_tool(function_name: str, function_args: dict[str, Any]) -> dict[str, Any]:
    return execute_registered_tool(function_name, function_args, _tool_registry)


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
    return run_robot_act_pipeline(
        user_intent=user_intent,
        agent_thought=agent_thought,
        observation_context=observation_context,
        scene_facts=scene_facts,
        object_facts_path=object_facts_path,
        on_event=on_event,
        ensure_registered=register_all,
        get_action_tool_definitions=get_action_tool_definitions,
        execute_tool_fn=_execute_registered_tool,
    )


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
    _tool_definitions = snapshot_tool_definitions(mcp)
    _modules_registered = True
