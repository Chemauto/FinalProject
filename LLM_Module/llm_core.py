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
except ImportError:
    from llm_highlevel import HighLevelPlanner
    from llm_lowlevel import LowLevelExecutor


class LLMAgent:
    """双层 LLM 代理：上层规划，下层执行。"""

    def __init__(
        self,
        api_key: str,
        base_url: str = "https://dashscope.aliyuncs.com/compatible-mode/v1",
        prompt_path: str | None = None,
    ):
        self.client = OpenAI(api_key=api_key, base_url=base_url)
        self.model = "qwen-plus"
        self.highlevel = HighLevelPlanner(self.client, self.model, prompt_path=prompt_path)
        self.lowlevel = LowLevelExecutor(self.client, self.model)
        self.planning_prompt_template = self.load_prompt(prompt_path)

    def load_prompt(self, prompt_path: str | None = None) -> str:
        """兼容旧接口：加载上层规划提示词模板。"""
        path = Path(prompt_path) if prompt_path else self.highlevel.prompt_path
        if not path.exists():
            return "你是一个机器人任务规划助手。请将用户的复杂指令分解为简单的子任务。用户输入：{user_input}"
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        return str(data.get("prompt", "")).strip()

    def plan_tasks(self, user_input: str, tools: list[dict]) -> list[dict]:
        return self.highlevel.plan_tasks(user_input, self.planning_prompt_template)

    def execute_single_task(
        self,
        task_description: str,
        tools: list[dict],
        execute_tool_fn: Callable,
        previous_result: Any = None,
    ) -> dict:
        return self.lowlevel.execute_single_task(task_description, tools, execute_tool_fn, previous_result)

    def run_pipeline(self, user_input: str, tools: list[dict], execute_tool_fn: Callable) -> list[dict]:
        print("\n" + "█" * 60 + f"\n📥 [用户输入] {user_input}\n" + "█" * 60)
        try:
            tasks = self.plan_tasks(user_input, tools)
            if not tasks:
                return []

            print("\n" + "█" * 60 + "\n🚀 [开始执行] 按顺序执行子任务\n" + "█" * 60)
            results = []
            previous_result = None

            for idx, task in enumerate(tasks, 1):
                print(f"\n【步骤 {idx}/{len(tasks)}】")
                result = self.execute_single_task(task["task"], tools, execute_tool_fn, previous_result)
                results.append(result)

                if result.get("success") and result.get("result"):
                    previous_result = result["result"].get("result")
                else:
                    previous_result = None

                if not result.get("success"):
                    print(f"\n⚠️  [警告] 步骤 {idx} 失败，但继续执行后续任务")

            print("\n" + "█" * 60 + "\n✅ [执行完成] 任务总结\n" + "█" * 60)
            for idx, (task, result) in enumerate(zip(tasks, results), 1):
                status = "✅ 成功" if result.get("success") else "❌ 失败"
                print(f"  {idx}. {task['task']} - {status}")
            return results
        except Exception as error:
            print(f"\n❌ [错误] {type(error).__name__}: {error}")
            import traceback

            traceback.print_exc()
            return []
