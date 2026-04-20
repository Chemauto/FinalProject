"""Factory for assembling and running the robot_act pipeline."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Callable

from Data_Module.context import build_planner_context
from Data_Module.facts import load_object_facts
from Data_Module.params import ParameterCalculator
from Data_Module.vlm import VLMCore
from Excu_Module.pipeline import run_pipeline
from Hardware_Module import get_state
from Hardware_Module.registry import (
    sync_object_facts_from_live_data,
    sync_runtime_overrides_from_user_input,
)
from Planner_Module.planner import Planner


DEFAULT_OBJECT_FACTS_PATH = Path(
    os.getenv("FINALPROJECT_OBJECT_FACTS_PATH", str(Path(__file__).resolve().parents[1] / "config" / "object_facts.json"))
)


def run_robot_act_pipeline(
    *,
    user_intent: str,
    agent_thought: str = "",
    observation_context: dict[str, Any] | None = None,
    scene_facts: dict[str, Any] | None = None,
    object_facts_path: str | Path | None = None,
    on_event=None,
    ensure_registered: Callable[[], None],
    get_action_tool_definitions: Callable[[], list[dict[str, Any]]],
    execute_tool_fn: Callable[[str, dict[str, Any]], dict[str, Any]],
) -> dict[str, Any]:
    ensure_registered()
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
        execute_tool_fn=execute_tool_fn,
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

