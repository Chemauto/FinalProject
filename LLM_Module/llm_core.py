#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LLM Core - 高层规划与执行流水线核心模块。"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Callable

from openai import OpenAI
import yaml

try:
    from .llm_lowlevel import LowLevelExecutor
    from .parameter_calculator import ParameterCalculator
except ImportError:
    from llm_lowlevel import LowLevelExecutor
    from parameter_calculator import ParameterCalculator


def _format_available_skills(tools: list[dict[str, Any]]) -> str:
    lines = []
    for tool in tools:
        func = tool.get("function", {})
        params = func.get("parameters", {}).get("properties", {})
        lines.append(f"- {func.get('name', '')}({', '.join(params.keys())})")
    return "\n".join(lines)


class HighLevelPlanner:
    """高层 LLM：根据用户意图生成尽量简单的任务序列。"""

    def __init__(self, client, model: str, prompt_path: str | None = None):
        self.client = client
        self.model = model
        root = Path(__file__).resolve().parent
        self.prompt_path = Path(prompt_path) if prompt_path else root / "prompts" / "highlevel_prompt.yaml"
        self.last_summary = ""
        self.last_plan_metadata: dict[str, object] = {}

    def load_prompt(self) -> dict[str, str]:
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

    def plan_tasks(
        self,
        user_input: str,
        planning_prompt: str,
        agent_thought: str = "",
        visual_context: str | None = None,
        scene_facts: dict | None = None,
        object_facts: dict | None = None,
        available_skills: str = "",
    ) -> list[dict]:
        print("\n" + "=" * 60 + "\n🧠 [上层LLM] 任务规划中...\n" + "=" * 60)
        prompts = self.load_prompt()
        try:
            user_content = planning_prompt.format(
                available_skills=available_skills,
                user_input=user_input,
                agent_thought=agent_thought or "无",
                visual_context=visual_context if visual_context else "无",
                scene_facts=json.dumps(scene_facts or {}, ensure_ascii=False),
                object_facts=json.dumps(object_facts or {}, ensure_ascii=False),
            )
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
            tasks, meta = self.parse_plan_response(plan)
            if not tasks:
                tasks = [self._build_fallback_task(user_input)]
                meta["summary"] = meta.get("summary") or "未生成明确任务，保持用户原始动作意图"

            self.last_summary = str(meta.get("summary", "")).strip() or "已生成任务序列"
            self.last_plan_metadata = meta
            print(f"✅ [规划完成] 共分解为 {len(tasks)} 个子任务\n📋 [任务概述] {self.last_summary}\n\n子任务序列：")
            for task in tasks:
                print(f"  步骤 {task['step']}: {task['task']} ({task['type']})")
                print(f"    建议函数: {task['function']}")
                print(f"    规划依据: {task['reason']}")
            return tasks
        except Exception as error:
            fallback = [self._build_fallback_task(user_input)]
            self.last_summary = "规划失败，保持用户原始动作意图"
            self.last_plan_metadata = {
                "scene_assessment": "",
                "candidate_plans": [],
                "selected_plan_id": "fallback_plan",
                "summary": self.last_summary,
            }
            print(f"❌ [规划失败] {error}\n[回退] 按单个通用动作任务处理")
            return fallback

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

    def parse_plan_response(self, payload: dict) -> tuple[list[dict], dict[str, object]]:
        raw_tasks = payload.get("tasks", [])
        tasks = [self._normalize_task(task, idx) for idx, task in enumerate(raw_tasks, 1)]
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

    def _build_fallback_task(self, user_input: str) -> dict:
        return self._normalize_task(
            {
                "step": 1,
                "task": user_input,
                "type": "通用动作",
                "function": "待LLM决定",
                "reason": "保持用户原始动作意图，不做特殊场景硬编码修正",
            },
            1,
        )


class LLMAgent:
    """动作规划与执行代理。"""

    def __init__(
        self,
        api_key: str,
        base_url: str = "https://dashscope.aliyuncs.com/compatible-mode/v1",
        prompt_path: str | None = None,
        client: Any | None = None,
    ):
        self.client = client or OpenAI(api_key=api_key, base_url=base_url)
        self.model = "qwen3.6-plus"
        self.highlevel = HighLevelPlanner(self.client, self.model, prompt_path=prompt_path)
        self.lowlevel = LowLevelExecutor(self.client, self.model)
        self.parameter_calculator = ParameterCalculator()
        self.planning_prompt_template = self.load_prompt(prompt_path)
        self.max_replans = 1

    def load_prompt(self, prompt_path: str | None = None) -> str:
        path = Path(prompt_path) if prompt_path else self.highlevel.prompt_path
        if not path.exists():
            return "你是一个机器人任务规划助手。请根据用户输入生成任务序列：{user_input}"
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        return str(data.get("prompt", "")).strip()

    def plan_tasks(
        self,
        user_input: str,
        agent_thought: str,
        tools: list[dict],
        visual_context: str | None = None,
        scene_facts: dict[str, Any] | None = None,
        object_facts: dict[str, Any] | None = None,
    ) -> list[dict]:
        return self.highlevel.plan_tasks(
            user_input,
            self.planning_prompt_template,
            agent_thought,
            visual_context,
            scene_facts=scene_facts,
            object_facts=object_facts,
            available_skills=_format_available_skills(tools),
        )

    def execute_single_task(
        self,
        task_info: dict,
        tools: list[dict],
        execute_tool_fn: Callable,
        previous_result: Any = None,
        visual_context: str | None = None,
    ) -> dict:
        return self.lowlevel.execute_single_task(task_info, tools, execute_tool_fn, previous_result, visual_context)

    def run_pipeline(
        self,
        user_input: str,
        agent_thought: str,
        tools: list[dict],
        execute_tool_fn: Callable,
        visual_context: str | None = None,
        scene_facts: dict[str, Any] | None = None,
        object_facts: dict[str, Any] | None = None,
    ) -> list[dict]:
        print("\n" + "█" * 60 + f"\n📥 [用户输入] {user_input}\n" + "█" * 60)
        try:
            results: list[dict[str, Any]] = []
            plan_input = user_input

            for attempt in range(self.max_replans + 1):
                tasks = self.plan_tasks(
                    plan_input,
                    agent_thought,
                    tools,
                    visual_context,
                    scene_facts=scene_facts,
                    object_facts=object_facts,
                )
                if not tasks:
                    break

                active_tasks = self._apply_parameter_calculation(tasks, object_facts)
                task_understanding = self._build_task_understanding(user_input, active_tasks)
                if task_understanding:
                    thought_prefix = f"顶层思考: {agent_thought}\n" if agent_thought else ""
                    print("\n" + "─" * 60 + f"\n👁️ [LLM思考]\n{thought_prefix}{task_understanding}\n" + "─" * 60)

                print("\n" + "█" * 60 + "\n🤖 [下层LLM] 开始执行与工具决策\n" + "█" * 60)
                previous_result = None
                should_replan = False

                for idx, task in enumerate(active_tasks, start=1):
                    print(f"\n【步骤 {idx}/{len(active_tasks)}】")
                    print(f"📌 [规划函数] {task.get('function', '待LLM决定')}")
                    print(f"📝 [规划依据] {task.get('reason', '未提供规划依据')}")
                    result = self.execute_single_task(task, tools, execute_tool_fn, previous_result, visual_context)
                    results.append(result)

                    tool_output = result.get("result", {})
                    if result.get("success") and tool_output:
                        previous_result = tool_output.get("result")
                        feedback = result.get("feedback", {})
                        if feedback:
                            print(f"✅ [反馈确认] {feedback.get('skill', '当前技能')} 已返回成功信号，继续下一步")
                        continue

                    previous_result = None
                    failure_message = (result.get("feedback") or {}).get("message") or result.get("error") or "当前步骤执行失败"
                    if attempt < self.max_replans:
                        print(f"\n🔁 [重规划] {failure_message}")
                        plan_input = f"{user_input}\n上次执行失败：{failure_message}"
                        should_replan = True
                    else:
                        print(f"\n⚠️  [中止] {failure_message}")
                    break

                if not should_replan:
                    break

            print("\n" + "█" * 60 + "\n✅ [执行完成] 任务总结\n" + "█" * 60)
            for idx, result in enumerate(results, 1):
                status = "✅ 成功" if result.get("success") else "❌ 失败"
                action = result.get("action", "未调用")
                task_label = result.get("task", "未记录任务")
                print(f"  {idx}. {task_label} -> {action} - {status}")
            return results
        except Exception as error:
            print(f"\n❌ [错误] {type(error).__name__}: {error}")
            import traceback

            traceback.print_exc()
            return []

    def _apply_parameter_calculation(
        self,
        tasks: list[dict],
        object_facts: dict[str, Any] | None,
    ) -> list[dict]:
        if not object_facts:
            return tasks
        return self.parameter_calculator.annotate_tasks(tasks, object_facts)

    def _build_task_understanding(self, user_input: str, tasks: list[dict]) -> str:
        if not tasks:
            return ""

        summary = getattr(self.highlevel, "last_summary", "") or "已完成任务规划"
        function_chain = " -> ".join(task.get("function", "待定") for task in tasks)
        primary_reason = tasks[0].get("reason", "")
        return "\n".join(
            [
                f"环境判断: {self._build_environment_judgment(tasks)}",
                f"任务目标: {user_input}",
                f"技能决策: {summary}",
                f"选择原因: {primary_reason}",
                f"函数链: {function_chain}",
            ]
        )

    @staticmethod
    def _build_environment_judgment(tasks: list[dict]) -> str:
        if not tasks:
            return "未获取到有效环境判断"

        first_task = tasks[0].get("task", "")
        if "。先" in first_task:
            return first_task.split("。先", 1)[0]
        if "，先" in first_task:
            return first_task.split("，先", 1)[0]
        return first_task
