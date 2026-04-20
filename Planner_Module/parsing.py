"""Parsing helpers for high-level planner responses."""

from __future__ import annotations

import json
from typing import Any


def format_available_skills(tools: list[dict[str, Any]]) -> str:
    lines = []
    for tool in tools:
        func = tool.get("function", {})
        params = func.get("parameters", {}).get("properties", {})
        lines.append(f"- {func.get('name', '')}({', '.join(params.keys())})")
    return "\n".join(lines)


def parse_plan_payload(content: str | None) -> dict:
    text = (content or "").strip()
    if text.startswith("```"):
        lines = text.splitlines()
        if lines and lines[0].startswith("```"):
            lines = lines[1:]
        if lines and lines[-1].strip() == "```":
            lines = lines[:-1]
        text = "\n".join(lines).strip()
        if text.startswith("json"):
            text = text[4:].strip()
    payload = json.loads(text)
    if not isinstance(payload, dict):
        raise ValueError("高层规划输出不是 JSON 对象")
    return payload


def parse_plan_response(payload: dict) -> tuple[list[dict], dict[str, object]]:
    raw_tasks = payload.get("tasks", [])
    tasks = [normalize_task(task, idx) for idx, task in enumerate(raw_tasks, 1)]
    meta = {
        "scene_assessment": str(payload.get("scene_assessment", "")).strip(),
        "candidate_plans": payload.get("candidate_plans", []) or [],
        "selected_plan_id": str(payload.get("selected_plan_id", "default_plan")).strip() or "default_plan",
        "summary": str(payload.get("summary", "")).strip(),
    }
    return tasks, meta


def normalize_task(task: dict, default_step: int) -> dict:
    return {
        "step": task.get("step", default_step),
        "task": task.get("task", ""),
        "type": task.get("type", "未分类"),
        "function": task.get("function", "待LLM决定"),
        "params": task.get("params") or {},
        "reason": task.get("reason", "未提供规划依据"),
    }


def build_fallback_task(user_input: str) -> dict:
    return normalize_task(
        {"step": 1, "task": user_input, "type": "通用动作", "function": "待LLM决定", "reason": "保持用户原始动作意图，不做特殊场景硬编码修正"},
        1,
    )

