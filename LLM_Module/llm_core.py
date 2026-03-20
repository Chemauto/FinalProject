#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LLM Core - 双层 LLM 架构核心模块。"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable

from openai import OpenAI
import yaml

try:
    from .llm_highlevel import HighLevelPlanner
    from .llm_lowlevel import LowLevelExecutor
    from .parameter_calculator import ParameterCalculator
except ImportError:
    from llm_highlevel import HighLevelPlanner
    from llm_lowlevel import LowLevelExecutor
    from parameter_calculator import ParameterCalculator


class LLMAgent:
    """双层 LLM 代理：上层规划，下层执行。"""

    def __init__(
        self,
        api_key: str,
        base_url: str = "https://dashscope.aliyuncs.com/compatible-mode/v1",
        prompt_path: str | None = None,
    ):
        self.client = OpenAI(api_key=api_key, base_url=base_url)
        self.model = "qwen3-max-2025-09-23"
        self.highlevel = HighLevelPlanner(self.client, self.model, prompt_path=prompt_path)
        self.lowlevel = LowLevelExecutor(self.client, self.model)
        self.parameter_calculator = ParameterCalculator()
        self.planning_prompt_template = self.load_prompt(prompt_path)

    def load_prompt(self, prompt_path: str | None = None) -> str:
        """兼容旧接口：加载上层规划提示词模板。"""
        path = Path(prompt_path) if prompt_path else self.highlevel.prompt_path
        if not path.exists():
            return "你是一个机器人任务规划助手。请将用户的复杂指令分解为简单的子任务。用户输入：{user_input}"
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        return str(data.get("prompt", "")).strip()

    def plan_tasks(
        self,
        user_input: str,
        tools: list[dict],
        visual_context: str | None = None,
        scene_facts: dict[str, Any] | None = None,
        object_facts: dict[str, Any] | None = None,
    ) -> list[dict]:
        return self.highlevel.plan_tasks(
            user_input,
            self.planning_prompt_template,
            visual_context,
            scene_facts=scene_facts,
            object_facts=object_facts,
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
        tools: list[dict],
        execute_tool_fn: Callable,
        visual_context: str | None = None,
        scene_facts: dict[str, Any] | None = None,
        object_facts: dict[str, Any] | None = None,
    ) -> list[dict]:
        print("\n" + "█" * 60 + f"\n📥 [用户输入] {user_input}\n" + "█" * 60)
        try:
            tasks = self.plan_tasks(
                user_input,
                tools,
                visual_context,
                scene_facts=scene_facts,
                object_facts=object_facts,
            )
            if not tasks:
                return []
            active_tasks = self._apply_parameter_calculation(tasks, object_facts)

            task_understanding = self._build_task_understanding(user_input, active_tasks)
            if task_understanding:
                print("\n" + "─" * 60 + f"\n👁️ [LLM思考]\n{task_understanding}\n" + "─" * 60)

            print("\n" + "█" * 60 + "\n🤖 [下层LLM] 开始执行与工具决策\n" + "█" * 60)
            results = []
            previous_result = None
            idx = 0

            while idx < len(active_tasks):
                task = active_tasks[idx]
                print(f"\n【步骤 {idx + 1}/{len(active_tasks)}】")
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
                    idx += 1
                else:
                    previous_result = None

                if not result.get("success"):
                    feedback = result.get("feedback", {})
                    if feedback:
                        print(f"\n⚠️  [中止] 未收到成功反馈: {feedback.get('message', '当前步骤执行失败')}")
                    else:
                        print(f"\n⚠️  [中止] 步骤 {idx + 1} 执行失败，停止后续任务")
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
        """根据用户目标和 LLM 规划结果生成简洁的 LLM 思考输出。"""
        if not tasks:
            return ""

        summary = getattr(self.highlevel, "last_summary", "") or "已完成任务规划"
        environment_judgment = self._build_environment_judgment(tasks)
        function_chain = " -> ".join(task.get("function", "待定") for task in tasks)
        primary_reason = tasks[0].get("reason", "")

        lines = [
            f"环境判断: {environment_judgment}",
            f"任务目标: {user_input}",
            f"技能决策: {summary}",
            f"选择原因: {primary_reason}",
            f"函数链: {function_chain}",
        ]
        return "\n".join(lines)

    @staticmethod
    def _build_environment_judgment(tasks: list[dict]) -> str:
        """根据第一步任务内容提取一条环境判断。"""
        if not tasks:
            return "未获取到有效环境判断"

        first_task = tasks[0].get("task", "")
        if "。先" in first_task:
            return first_task.split("。先", 1)[0]
        if "，先" in first_task:
            return first_task.split("，先", 1)[0]
        return first_task
