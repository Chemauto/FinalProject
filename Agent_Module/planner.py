#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Agent_Module/planner.py — 输入和规划（CoT 推理链 + 任务序列生成）。"""

from __future__ import annotations

import json
import os
from pathlib import Path
import re
from typing import Any

import yaml

try:
    from .parameter import ParameterCalculator
except ImportError:
    from parameter import ParameterCalculator


def _format_available_skills(tools: list[dict[str, Any]]) -> str:
    lines = []
    for tool in tools:
        func = tool.get("function", {})
        params = func.get("parameters", {}).get("properties", {})
        lines.append(f"- {func.get('name', '')}({', '.join(params.keys())})")
    return "\n".join(lines)


class Planner:
    """高层任务规划器，通过 CoT 推理链生成任务序列。"""

    def __init__(
        self,
        client: Any,
        model: str | None = None,
        prompt_path: str | Path | None = None,
    ):
        self.client = client
        self.model = model or os.getenv("FINALPROJECT_AGENT_MODEL", "qwen3.5-plus")
        root = Path(__file__).resolve().parent
        self.prompt_path = Path(prompt_path) if prompt_path else root / "prompts" / "highlevel_prompt.yaml"
        self.parameter_calculator = ParameterCalculator()
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
    ) -> tuple[list[dict], dict]:
        """规划任务序列，返回 (tasks, metadata)。

        metadata 包含 CoT 推理链（scene_assessment, candidate_plans, selected_plan_id, summary）。
        """
        print("\n" + "=" * 60 + "\n🧠 [上层LLM] 任务规划中...\n" + "=" * 60)
        try:
            user_content = self.planning_prompt_template.format(
                available_skills=_format_available_skills(tools),
                user_input=user_input,
                agent_thought=agent_thought or "无",
                visual_context=visual_context if visual_context else "无",
                scene_facts=json.dumps(scene_facts or {}, ensure_ascii=False),
                object_facts=json.dumps(object_facts or {}, ensure_ascii=False),
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
            plan = self._parse_plan_payload(completion.choices[0].message.content)
            tasks, meta = self._parse_plan_response(plan)
            tasks, meta = self._apply_rule_overrides(
                tasks,
                meta,
                user_input=user_input,
                scene_facts=scene_facts,
                object_facts=object_facts,
            )
            if not tasks:
                tasks, meta = self._build_rule_fallback(
                    user_input=user_input,
                    scene_facts=scene_facts,
                    object_facts=object_facts,
                )

            self._store_plan_meta(meta)
            self._print_plan(tasks)
            return tasks, meta
        except Exception as error:
            fallback, meta = self._build_rule_fallback(
                user_input=user_input,
                scene_facts=scene_facts,
                object_facts=object_facts,
            )
            self.last_summary = str(meta.get("summary", "")).strip()
            self.last_plan_metadata = meta
            print(f"❌ [规划失败] {error}\n[回退] 使用规则任务链")
            return fallback, dict(self.last_plan_metadata)

    # ── Parameter annotation ────────────────────────────────────────────

    def annotate_tasks(self, tasks: list[dict], object_facts: dict | None) -> list[dict]:
        if not object_facts:
            return tasks
        return self.parameter_calculator.annotate_tasks(tasks, object_facts)

    # ── Internal helpers ────────────────────────────────────────────────

    def _store_plan_meta(self, meta: dict) -> None:
        self.last_summary = str(meta.get("summary", "")).strip() or "已生成任务序列"
        self.last_plan_metadata = meta

    def _print_plan(self, tasks: list[dict]) -> None:
        print(f"✅ [规划完成] 共分解为 {len(tasks)} 个子任务\n📋 [任务概述] {self.last_summary}\n\n子任务序列：")
        for task in tasks:
            print(f"  步骤 {task['step']}: {task['task']} ({task['type']})")
            print(f"    建议函数: {task['function']}")
            print(f"    规划依据: {task['reason']}")

    def _apply_rule_overrides(
        self,
        tasks: list[dict],
        meta: dict[str, object],
        *,
        user_input: str,
        scene_facts: dict[str, Any] | None,
        object_facts: dict[str, Any] | None,
    ) -> tuple[list[dict], dict[str, object]]:
        geometry = self._detect_box_assisted_geometry(scene_facts, object_facts)
        if geometry and self._should_override_box_assisted_plan(tasks, user_input):
            tasks, summary = self._build_box_assisted_plan(geometry)
            updated_meta = dict(meta)
            updated_meta["selected_plan_id"] = "rule_override_box_assist"
            updated_meta["summary"] = summary
            updated_meta["scene_assessment"] = (
                f"检测到可移动箱子 {geometry['box_id']} 与高台 {geometry['platform_id']} 形成辅助登台几何关系。"
            )
            print("⚠️  [规则修正] 检测到箱子辅助登台场景，已修正为 push_box -> climb_align -> climb -> navigation")
            return tasks, updated_meta
        return tasks, meta

    def _build_rule_fallback(
        self,
        *,
        user_input: str,
        scene_facts: dict[str, Any] | None,
        object_facts: dict[str, Any] | None,
    ) -> tuple[list[dict], dict[str, object]]:
        geometry = self._detect_box_assisted_geometry(scene_facts, object_facts)
        if geometry and self._is_navigation_request(user_input):
            tasks, summary = self._build_box_assisted_plan(geometry)
            return tasks, {
                "scene_assessment": "规则回退：箱子辅助登台场景",
                "candidate_plans": [],
                "selected_plan_id": "fallback_box_assist",
                "summary": summary,
            }
        return [self._build_fallback_task(user_input)], {
            "scene_assessment": "",
            "candidate_plans": [],
            "selected_plan_id": "fallback_plan",
            "summary": "未生成明确任务，保持用户原始动作意图",
        }

    @staticmethod
    def _parse_plan_payload(content: str | None) -> dict:
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

    @staticmethod
    def _parse_plan_response(payload: dict) -> tuple[list[dict], dict[str, object]]:
        raw_tasks = payload.get("tasks", [])
        tasks = [Planner._normalize_task(task, idx) for idx, task in enumerate(raw_tasks, 1)]
        meta = {
            "scene_assessment": str(payload.get("scene_assessment", "")).strip(),
            "candidate_plans": payload.get("candidate_plans", []) or [],
            "selected_plan_id": str(payload.get("selected_plan_id", "default_plan")).strip() or "default_plan",
            "summary": str(payload.get("summary", "")).strip(),
        }
        return tasks, meta

    @staticmethod
    def _normalize_task(task: dict, default_step: int) -> dict:
        return {
            "step": task.get("step", default_step),
            "task": task.get("task", ""),
            "type": task.get("type", "未分类"),
            "function": task.get("function", "待LLM决定"),
            "reason": task.get("reason", "未提供规划依据"),
        }

    @staticmethod
    def _build_fallback_task(user_input: str) -> dict:
        return Planner._normalize_task(
            {"step": 1, "task": user_input, "type": "通用动作", "function": "待LLM决定", "reason": "保持用户原始动作意图，不做特殊场景硬编码修正"},
            1,
        )

    @staticmethod
    def _is_navigation_request(user_input: str) -> bool:
        return bool(re.search(r"(导航|前往|去往|到达|到.*点|move to|go to|navigate)", str(user_input or ""), re.IGNORECASE))

    def _should_override_box_assisted_plan(self, tasks: list[dict], user_input: str) -> bool:
        if not self._is_navigation_request(user_input):
            return False
        function_chain = [str(task.get("function", "")) for task in tasks]
        return function_chain[:4] != ["push_box", "climb_align", "climb", "navigation"]

    @staticmethod
    def _planner_scene(scene_facts: dict[str, Any] | None) -> dict[str, Any]:
        if not isinstance(scene_facts, dict):
            return {}
        nested = scene_facts.get("scene_facts")
        return nested if isinstance(nested, dict) else scene_facts

    @staticmethod
    def _object_height(obj: dict[str, Any] | None) -> float:
        if not obj:
            return 0.0
        size = obj.get("size") or [0.0, 0.0, 0.0]
        try:
            return round(float(size[2]), 3)
        except (TypeError, ValueError, IndexError):
            return 0.0

    def _detect_box_assisted_geometry(
        self,
        scene_facts: dict[str, Any] | None,
        object_facts: dict[str, Any] | None,
    ) -> dict[str, object] | None:
        if not scene_facts or not object_facts:
            return None

        planner_scene = self._planner_scene(scene_facts)
        constraints = (object_facts.get("constraints") or planner_scene.get("constraints") or {})
        climb_limit = float(constraints.get("max_climb_height_m", 0.3))
        objects = object_facts.get("objects") or []
        boxes = [obj for obj in objects if obj.get("movable") and str(obj.get("type", "")).lower() == "box"]
        platforms = [obj for obj in objects if str(obj.get("type", "")).lower() == "platform"]
        if not boxes or not platforms:
            return None

        support_box = min(boxes, key=self._object_height)
        target_platform = max(platforms, key=self._object_height)
        box_height = self._object_height(support_box)
        platform_height = self._object_height(target_platform)
        remaining_height = round(platform_height - box_height, 3)
        if not (0 < box_height <= climb_limit < platform_height and 0 < remaining_height <= climb_limit):
            return None

        route_options = planner_scene.get("route_options") or []
        blocked = {str(item.get("direction", "")).lower() for item in route_options if str(item.get("status", "")).lower() == "blocked"}
        if route_options and not {"left", "right"}.issubset(blocked):
            return None

        center = support_box.get("center") or target_platform.get("center") or [0.0, 0.0, 0.0]
        side = "left" if float(center[1]) >= 0 else "right"
        return {
            "side": side,
            "box_id": str(support_box.get("id", "box")),
            "box_height": box_height,
            "platform_id": str(target_platform.get("id", "platform")),
            "platform_height": platform_height,
            "remaining_height": remaining_height,
        }

    def _build_box_assisted_plan(self, box_plan: dict[str, object]) -> tuple[list[dict], str]:
        side = str(box_plan["side"])
        side_label = "左侧" if side == "left" else "右侧"
        box_id = str(box_plan["box_id"])
        platform_id = str(box_plan["platform_id"])
        remaining_height = float(box_plan["remaining_height"])
        remaining_height_label = f"{remaining_height:.1f}".rstrip("0").rstrip(".")
        return (
            [
                self._normalize_task(
                    {
                        "step": 1,
                        "task": f"先调用 push_box，将箱子 {box_id} 推到高台 {platform_id} 旁边",
                        "type": "推箱子",
                        "function": "push_box",
                        "reason": f"{side_label}箱子可作为辅助台阶，应先完成推箱。",
                    },
                    1,
                ),
                self._normalize_task(
                    {
                        "step": 2,
                        "task": f"调用 climb_align，对正到箱子 {box_id} 后方的攀爬起点",
                        "type": "攀爬对正",
                        "function": "climb_align",
                        "reason": "推箱完成后需要先对正，再执行 climb。",
                    },
                    2,
                ),
                self._normalize_task(
                    {
                        "step": 3,
                        "task": f"利用已推到位的箱子辅助攀爬约{remaining_height_label}米到高台 {platform_id}",
                        "type": "攀爬",
                        "function": "climb",
                        "reason": f"借助箱子后，剩余高差约 {remaining_height_label} 米，单次 climb 可登台。",
                    },
                    3,
                ),
                self._normalize_task(
                    {
                        "step": 4,
                        "task": "登上高台后，使用 navigation 前往目标点",
                        "type": "导航",
                        "function": "navigation",
                        "reason": "越过高差后，再用 navigation 精准到终点。",
                    },
                    4,
                ),
            ],
            "根据结构化物体几何信息，应执行 push_box -> climb_align -> climb -> navigation",
        )


# Backward compatibility alias
LLMAgent = Planner
HighLevelPlanner = Planner
