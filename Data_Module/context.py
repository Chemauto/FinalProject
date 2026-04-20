#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Data_Module/context.py — 构建 planner 上下文。"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from Hardware_Module import get_state
from Hardware_Module.registry import (
    sync_object_facts_from_live_data,
    sync_runtime_overrides_from_user_input,
)

from .vlm import VLMCore
from .facts import load_object_facts
from .schema import RobotStateSnapshot


DEFAULT_OBJECT_FACTS_PATH = Path(
    __import__("os").getenv(
        "FINALPROJECT_OBJECT_FACTS_PATH",
        str(Path(__file__).resolve().parents[1] / "config" / "object_facts.json"),
    )
)


def build_context(
    user_intent: str = "",
    object_facts_path: str | Path | None = None,
) -> RobotStateSnapshot:
    """构建完整的规划上下文：获取状态 + 同步数据 + 加载 facts。"""
    path = Path(object_facts_path or DEFAULT_OBJECT_FACTS_PATH)

    # 1. 获取硬件状态
    try:
        state = get_state()
    except Exception:
        state = {"connected": False, "task_type": None, "observation": {}, "runtime": {}}

    # 2. 同步 live data
    synced_payload = None
    try:
        synced_payload = sync_object_facts_from_live_data(path, user_input=user_intent)
    except Exception:
        try:
            sync_runtime_overrides_from_user_input(path, user_input=user_intent)
        except Exception:
            pass

    # 3. 加载 object_facts
    object_facts = load_object_facts(path)

    # 4. 合并 scene_facts
    scene_facts = None
    if object_facts is not None:
        scene_facts = VLMCore.merge_scene_facts(None, object_facts)

    return RobotStateSnapshot(
        state=state,
        scene_facts=scene_facts,
        object_facts=object_facts,
        connected=bool(state.get("connected")),
        task_type=state.get("task_type"),
    )


def build_planner_context(
    scene_facts: dict[str, Any] | None,
    object_facts: dict[str, Any] | None,
    synced_payload: dict[str, Any] | None,
) -> dict[str, Any]:
    """构建传给 Planner 的上下文字典。"""
    runtime_state = (synced_payload or {}).get("runtime_state") or {}
    return {
        "robot_state": {
            "scene_id": runtime_state.get("scene_id"),
            "robot_pose": (object_facts or {}).get("robot_pose"),
            "goal": (object_facts or {}).get("navigation_goal") or runtime_state.get("goal"),
            "skill": runtime_state.get("skill"),
            "model_use": runtime_state.get("model_use"),
            "start": runtime_state.get("start"),
        },
        "constraints": (object_facts or {}).get("constraints") or {},
        "objects": (object_facts or {}).get("objects") or [],
        "scene_facts": scene_facts or {},
    }
