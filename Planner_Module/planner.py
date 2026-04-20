#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Planner_Module/planner.py — 输入和规划（CoT 推理链 + 任务序列生成）。"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

import yaml

from .parsing import format_available_skills, parse_plan_payload, parse_plan_response
from .rule_overrides import apply_rule_overrides, build_rule_fallback


class Planner:
    """高层任务规划器，通过 CoT 推理链生成任务序列。"""

    def __init__(
        self,
        client: Any,
        model: str | None = None,
        prompt_path: str | Path | None = None,
        parameter_calculator: Any | None = None,
    ):
        self.client = client
        self.model = model or os.getenv("FINALPROJECT_AGENT_MODEL", "qwen3.5-plus")
        root = Path(__file__).resolve().parent
        self.prompt_path = Path(prompt_path) if prompt_path else root / "prompts" / "highlevel_prompt.yaml"
        self.parameter_calculator = parameter_calculator
        self.planning_prompt_template = self._load_prompt(self.prompt_path)
        self.last_summary = ""
        self.last_plan_metadata: dict[str, object] = {}

    # ── Prompt loading ──────────────────────────────────────────────────

    @staticmethod
    def _load_prompt(path: Path) -> str:
        if not path.exists():
            return "你是一个机器人任务规划助手。请根据用户输入生成任务序列：{user_input}"
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        return str(data.get("prompt", "")).strip()

    def _load_system_prompt(self) -> dict[str, str]:
        if not self.prompt_path.exists():
            return {
                "system_prompt": "你是一个机器人任务规划助手。输出必须是有效 JSON。",
                "prompt": "请根据用户输入生成任务序列：{user_input}",
            }
        data = yaml.safe_load(self.prompt_path.read_text(encoding="utf-8")) or {}
        return {
            "system_prompt": str(data.get("system_prompt", "")).strip(),
            "prompt": str(data.get("prompt", "")).strip(),
        }

    # ── Public planning API ─────────────────────────────────────────────

    def plan_tasks(
        self,
        user_input: str,
        agent_thought: str,
        tools: list[dict],
        visual_context: str | None = None,
        scene_facts: dict[str, Any] | None = None,
        object_facts: dict[str, Any] | None = None,
        robot_state: dict[str, Any] | None = None,
        on_event=None,
    ) -> tuple[list[dict], dict]:
        """规划任务序列，返回 (tasks, metadata)。"""
        if on_event:
            on_event("llm_planning", {"user_input": user_input})
        else:
            print("\n" + "=" * 60 + "\n🧠 [上层LLM] 任务规划中...\n" + "=" * 60)
        try:
            user_content = self.planning_prompt_template.format(
                available_skills=format_available_skills(tools),
                user_input=user_input,
                agent_thought=agent_thought or "无",
                visual_context=visual_context if visual_context else "无",
                scene_facts=json.dumps(scene_facts or {}, ensure_ascii=False),
                object_facts=json.dumps(object_facts or {}, ensure_ascii=False),
                robot_state=json.dumps(robot_state or {}, ensure_ascii=False, indent=2),
            )
            prompts = self._load_system_prompt()
            completion = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": prompts["system_prompt"]},
                    {"role": "user", "content": user_content},
                ],
                temperature=0.2,
                extra_body={"enable_thinking": False},
            )
            plan = parse_plan_payload(completion.choices[0].message.content)
            tasks, meta = parse_plan_response(plan)
            tasks, meta = apply_rule_overrides(
                tasks,
                meta,
                user_input=user_input,
                scene_facts=scene_facts,
                object_facts=object_facts,
                on_event=on_event,
            )
            if not tasks:
                tasks, meta = build_rule_fallback(
                    user_input=user_input,
                    scene_facts=scene_facts,
                    object_facts=object_facts,
                )

            self._store_plan_meta(meta)
            self._print_plan(tasks, on_event=on_event)
            if on_event:
                on_event("plan_done", {"tasks": tasks, "summary": self.last_summary})
            return tasks, meta
        except Exception as error:
            fallback, meta = build_rule_fallback(
                user_input=user_input,
                scene_facts=scene_facts,
                object_facts=object_facts,
            )
            self.last_summary = str(meta.get("summary", "")).strip()
            self.last_plan_metadata = meta
            if on_event:
                on_event("plan_error", {"error": str(error)})
            else:
                print(f"❌ [规划失败] {error}\n[回退] 使用规则任务链")
            return fallback, dict(self.last_plan_metadata)

    # ── Parameter annotation ────────────────────────────────────────────

    def annotate_tasks(self, tasks: list[dict], object_facts: dict | None) -> list[dict]:
        """参数标注，通过依赖注入的 parameter_calculator 完成。"""
        if not object_facts or self.parameter_calculator is None:
            return tasks
        return self.parameter_calculator.annotate_tasks(tasks, object_facts)

    # ── Internal helpers ────────────────────────────────────────────────

    def _store_plan_meta(self, meta: dict) -> None:
        self.last_summary = str(meta.get("summary", "")).strip() or "已生成任务序列"
        self.last_plan_metadata = meta

    def _print_plan(self, tasks: list[dict], *, on_event=None) -> None:
        if on_event:
            return
        print(f"✅ [规划完成] 共分解为 {len(tasks)} 个子任务\n📋 [任务概述] {self.last_summary}\n\n子任务序列：")
        for task in tasks:
            print(f"  步骤 {task['step']}: {task['task']} ({task['type']})")
            print(f"    函数: {task['function']}")
            params = task.get("params")
            if params:
                print(f"    参数: {json.dumps(params, ensure_ascii=False)}")
            print(f"    依据: {task['reason']}")

# Backward compatibility aliases
LLMAgent = Planner
HighLevelPlanner = Planner
