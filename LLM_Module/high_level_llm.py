#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
高层LLM - 任务规划器
负责理解用户意图、生成任务序列、处理环境变化时的重新规划
"""
import os
import yaml
import json
from openai import OpenAI
from typing import List, Dict, Any, Optional


class HighLevelLLM:
    """
    高层LLM：任务规划器

    职责：
    1. 理解用户自然语言指令
    2. 根据环境状态生成任务序列
    3. 当环境变化时重新规划
    4. 管理任务队列
    """

    def __init__(self,
                 api_key: str,
                 base_url: str = "https://dashscope.aliyuncs.com/compatible-mode/v1",
                 model: str = "qwen3-32b",
                 prompt_path: str = None):
        """
        初始化高层LLM

        Args:
            api_key: API密钥
            base_url: API基础URL
            model: 使用的模型名称
            prompt_path: 规划提示词文件路径
        """
        self.client = OpenAI(api_key=api_key, base_url=base_url)
        self.model = model
        self.prompt_path = prompt_path
        self.prompt_template = self._load_prompt_template()

    def _load_prompt_template(self) -> str:
        """从YAML文件加载规划Prompt模板"""
        if not self.prompt_path or not os.path.exists(self.prompt_path):
            print("⚠️  警告: Prompt文件路径未提供或不存在，使用默认内置Prompt")
            return self._get_default_prompt()

        try:
            with open(self.prompt_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                return data.get("prompt", "")
        except Exception as e:
            print(f"❌ 错误: 加载Prompt文件失败: {e}")
            return self._get_default_prompt()

    def _get_default_prompt(self) -> str:
        """获取默认的规划Prompt"""
        return """你是一个机器人任务规划助手。你的职责是将用户的复杂指令分解为简单的、顺序执行的子任务。

可用技能:
{available_skills}

输出格式（JSON）：
{{
  "tasks": [
    {{"step": 1, "task": "子任务描述1", "type": "动作类型"}},
    {{"step": 2, "task": "子任务描述2", "type": "动作类型"}}
  ],
  "summary": "整体任务概述"
}}

用户输入：{user_input}

请将上述指令分解为子任务序列。"""

    def plan_tasks(self,
                   user_input: str,
                   available_skills: List[str],
                   env_state: Optional[Dict[str, Any]] = None) -> List[Dict[str, Any]]:
        """
        根据用户输入和环境状态生成任务序列

        Args:
            user_input: 用户自然语言指令
            available_skills: 可用技能列表
            env_state: 当前环境状态（可选）

        Returns:
            任务序列列表，格式：[{"step": 1, "task": "...", "type": "..."}, ...]
        """
        print("\n" + "="*60)
        print("🧠 [高层LLM] 任务规划中...")
        print("="*60)

        # 构建prompt
        skills_desc = "\n".join([f"  - {skill}" for skill in available_skills])

        # 检查模板是否已经包含available_skills（已由load_dynamic_prompt填充）
        if "{available_skills}" in self.prompt_template:
            # 还未填充，使用传入的skills
            prompt = self.prompt_template.format(
                user_input=user_input,
                available_skills=skills_desc
            )
        else:
            # 已经填充过了，只替换user_input
            prompt = self.prompt_template.format(
                user_input=user_input
            )

        # 添加环境状态信息（如果有）
        if env_state:
            prompt += f"\n\n当前环境状态:\n{json.dumps(env_state, indent=2, ensure_ascii=False)}"

        try:
            completion = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {
                        "role": "system",
                        "content": "你是一个专业的机器人任务规划助手。输出必须是有效的JSON格式。"
                    },
                    {"role": "user", "content": prompt}
                ],
                temperature=0.3,
                extra_body={"enable_thinking": False}
            )

            response_text = completion.choices[0].message.content.strip()

            # 清理markdown代码块标记
            if response_text.startswith("```"):
                response_text = response_text.split("```")[1]
                if response_text.startswith("json"):
                    response_text = response_text[4:]

            plan = json.loads(response_text)
            tasks = plan.get("tasks", [])
            summary = plan.get("summary", "")

            print(f"\n✅ [规划完成] 共分解为 {len(tasks)} 个子任务")
            print(f"📋 [任务概述] {summary}\n")
            print("子任务序列：")
            for task in tasks:
                print(f"  步骤 {task['step']}: {task['task']} ({task['type']})")

            return tasks

        except Exception as e:
            print(f"\n❌ [规划失败] {e}")
            print(f"[回退] 将作为单个任务处理")
            # 回退：将用户输入作为单个任务
            return [{
                "step": 1,
                "task": user_input,
                "type": "综合"
            }]

    def replan_tasks(self,
                     failed_task: Dict[str, Any],
                     env_state: Dict[str, Any],
                     failure_reason: str,
                     original_user_input: str,
                     available_skills: List[str]) -> List[Dict[str, Any]]:
        """
        任务失败时重新规划

        Args:
            failed_task: 失败的任务
            env_state: 当前环境状态
            failure_reason: 失败原因
            original_user_input: 原始用户指令
            available_skills: 可用技能列表

        Returns:
            新的任务序列
        """
        print("\n" + "="*60)
        print("🔄 [高层LLM] 重新规划中...")
        print("="*60)
        print(f"失败任务: {failed_task.get('task', 'Unknown')}")
        print(f"失败原因: {failure_reason}")

        # 构建重新规划的prompt
        replan_prompt = f"""你是一个自适应规划专家。当任务执行失败或环境变化时，你需要重新规划。

原始用户指令: {original_user_input}

失败的任务:
- 步骤: {failed_task.get('step', 'Unknown')}
- 描述: {failed_task.get('task', 'Unknown')}
- 类型: {failed_task.get('type', 'Unknown')}

失败原因: {failure_reason}

当前环境状态:
{json.dumps(env_state, indent=2, ensure_ascii=False)}

可用技能:
{chr(10).join([f'  - {s}' for s in available_skills])}

重新规划策略:
1. 分析失败原因
2. 评估当前环境状态
3. 生成替代方案
4. 考虑用户意图的保持

可能的策略:
- 尝试不同的方法完成相同目标
- 调整任务顺序
- 增加感知任务获取更多信息
- 请求用户澄清或帮助（如果是最后手段）

输出格式（JSON）：
{{
  "strategy": "重新规划策略描述",
  "tasks": [
    {{"step": 1, "task": "新任务描述", "type": "任务类型"}},
    {{"step": 2, "task": "新任务描述", "type": "任务类型"}}
  ],
  "explanation": "重新规划的解释"
}}

请生成新的任务规划:"""

        try:
            completion = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {
                        "role": "system",
                        "content": "你是一个自适应规划专家。输出必须是有效的JSON格式。"
                    },
                    {"role": "user", "content": replan_prompt}
                ],
                temperature=0.4,  # 稍高一点的温度以鼓励创造性
                extra_body={"enable_thinking": False}
            )

            response_text = completion.choices[0].message.content.strip()

            # 清理markdown代码块标记
            if response_text.startswith("```"):
                response_text = response_text.split("```")[1]
                if response_text.startswith("json"):
                    response_text = response_text[4:]

            plan = json.loads(response_text)
            tasks = plan.get("tasks", [])
            strategy = plan.get("strategy", "未提供策略")
            explanation = plan.get("explanation", "")

            print(f"\n✅ [重新规划完成] 策略: {strategy}")
            print(f"📝 [规划说明] {explanation}\n")
            print(f"新生成 {len(tasks)} 个任务:")
            for task in tasks:
                print(f"  步骤 {task['step']}: {task['task']} ({task['type']})")

            return tasks

        except Exception as e:
            print(f"\n❌ [重新规划失败] {e}")
            print(f"[回退] 返回空任务列表")
            return []

    def validate_plan(self, tasks: List[Dict[str, Any]]) -> bool:
        """
        验证生成的任务计划是否有效

        Args:
            tasks: 任务列表

        Returns:
            是否有效
        """
        if not tasks:
            return False

        # 检查基本结构
        for task in tasks:
            if "step" not in task or "task" not in task:
                return False

        return True
