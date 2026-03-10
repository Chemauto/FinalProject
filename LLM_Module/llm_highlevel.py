from __future__ import annotations

import json
from pathlib import Path

import yaml


class HighLevelPlanner:
    """上层 LLM：负责把用户输入拆成任务序列。"""

    def __init__(self, client, model: str, prompt_path: str | None = None):
        self.client = client
        self.model = model
        root = Path(__file__).resolve().parent
        self.prompt_path = Path(prompt_path) if prompt_path else root / "prompts" / "highlevel_prompt.yaml"

    def load_prompt(self) -> dict[str, str]:
        if not self.prompt_path.exists():
            return {
                "system_prompt": "你是一个专业的机器人任务规划助手。输出必须是有效的JSON格式。",
                "prompt": "你是一个机器人任务规划助手。请将用户的复杂指令分解为简单的子任务。用户输入：{user_input}",
            }
        data = yaml.safe_load(self.prompt_path.read_text(encoding="utf-8")) or {}
        return {
            "system_prompt": str(data.get("system_prompt", "")).strip(),
            "prompt": str(data.get("prompt", "")).strip(),
        }

    def plan_tasks(self, user_input: str, planning_prompt: str, visual_context: str | None = None) -> list[dict]:
        print("\n" + "=" * 60 + "\n🧠 [上层LLM] 任务规划中...\n" + "=" * 60)
        prompts = self.load_prompt()
        try:
            user_content = planning_prompt.format(
                user_input=user_input,
                visual_context=visual_context if visual_context else "无",
            )

            completion = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": prompts["system_prompt"]},
                    {"role": "user", "content": user_content},
                ],
                temperature=0.3,
                extra_body={"enable_thinking": False},
            )
            response_text = completion.choices[0].message.content.strip()
            if response_text.startswith("```"):
                response_text = response_text.split("```")[1]
                if response_text.startswith("json"):
                    response_text = response_text[4:]

            plan = json.loads(response_text)
            tasks = plan.get("tasks", [])
            summary = plan.get("summary", "")
            print(f"✅ [规划完成] 共分解为 {len(tasks)} 个子任务\n📋 [任务概述] {summary}\n\n子任务序列：")
            for task in tasks:
                print(f"  步骤 {task['step']}: {task['task']} ({task['type']})")
            return tasks
        except Exception as error:
            print(f"❌ [规划失败] {error}\n[回退] 将作为单个任务处理")
            return [{"step": 1, "task": user_input, "type": "综合"}]
