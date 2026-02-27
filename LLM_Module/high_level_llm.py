#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
高层LLM - 任务规划器
负责理解用户意图、生成任务序列、处理环境变化时的重新规划
"""
import os
import sys
import yaml
import json
from openai import OpenAI
from typing import List, Dict, Any, Optional


class HighLevelLLM:
    """
    高层LLM：任务规划器

    职责：
    1. 理解用户自然语言指令
    2. 根据 VLM 环境理解和用户输入生成任务序列
    3. 当环境变化时重新规划
    4. 管理任务队列
    """

    def __init__(self,
                 api_key: str,
                 base_url: str = "https://dashscope.aliyuncs.com/compatible-mode/v1",
                 model: str = "qwen3-32b",
                 prompt_path: str = None,
                 vlm_core: Optional['VLMCore'] = None):
        """
        初始化高层LLM

        Args:
            api_key: API密钥
            base_url: API基础URL
            model: 使用的模型名称（文本LLM）
            prompt_path: 规划提示词文件路径
            vlm_core: VLM核心实例（可选，如果不提供则不使用VLM）
        """
        self.client = OpenAI(api_key=api_key, base_url=base_url)
        self.model = model
        self.prompt_path = prompt_path
        self.vlm_core = vlm_core  # VLM 核心实例
        self.prompt_template = self._load_prompt_template()

        if self.vlm_core:
            print(f"[High-Level LLM] VLM 功能已启用", file=sys.stderr)
        else:
            print(f"[High-Level LLM] VLM 功能未启用", file=sys.stderr)

    def _load_prompt_template(self) -> str:
        """从YAML文件加载规划Prompt模板"""
        if not self.prompt_path or not os.path.exists(self.prompt_path):
            print("⚠️  警告: Prompt文件路径未提供或不存在，使用默认内置Prompt", file=sys.stderr)
            return self._get_default_prompt()

        try:
            with open(self.prompt_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                return data.get("prompt", "")
        except Exception as e:
            print(f"❌ 错误: 加载Prompt文件失败: {e}", file=sys.stderr)
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
                   env_state: Optional[Dict[str, Any]] = None,
                   image_path: Optional[str] = None) -> List[Dict[str, Any]]:
        """
        根据用户输入和环境状态生成任务序列

        Args:
            user_input: 用户自然语言指令
            available_skills: 可用技能列表
            env_state: 当前环境状态（可选）
            image_path: 环境图像路径（可选，用于VLM理解）

        Returns:
            任务序列列表，格式：[{"step": 1, "task": "...", "type": "..."}, ...]
        """
        print("\n" + "="*60, file=sys.stderr)
        print("🧠 [高层LLM] 任务规划中...", file=sys.stderr)
        print("="*60, file=sys.stderr)

        # ==================== VLM 环境理解 ====================
        vlm_understanding = ""

        # 如果没有提供图片，使用默认图片
        use_default_image = False
        if image_path is None and self.vlm_core:
            # 使用默认图片
            image_path = "/home/xcj/work/FinalProject/VLM_Module/assets/red.png"
            use_default_image = True
            print(f"🖼️  [高层LLM] 使用默认图片进行环境理解: {image_path}", file=sys.stderr)

        # 如果有图片（用户提供的或默认的）且 VLM 已初始化
        if image_path and self.vlm_core:
            if not use_default_image:
                print(f"🖼️  [高层LLM] 使用用户图片进行环境理解: {image_path}", file=sys.stderr)

            # 检查图片是否存在
            import os
            if not os.path.exists(image_path):
                print(f"⚠️  [VLM] 图片不存在: {image_path}，跳过环境理解", file=sys.stderr)
            else:
                print(f"🖼️  [高层LLM] 正在分析环境图像...", file=sys.stderr)
                vlm_result = self.vlm_core.analyze_environment(image_path)

                if vlm_result:
                    # 显示 VLM 的分析结果（200字左右）
                    print(f"📷 [VLM 环境分析结果]\n{vlm_result}\n", file=sys.stderr)
                    vlm_understanding = f"【环境观察】\n{vlm_result}"
                    print(f"✅ [高层LLM] VLM 环境理解完成", file=sys.stderr)
                else:
                    print("⚠️  [高层LLM] VLM 环境理解失败，继续使用文本规划", file=sys.stderr)
        # ==========================================================

        # 构建prompt
        skills_desc = "\n".join([f"  - {skill}" for skill in available_skills])

        # 准备用户输入部分（包含VLM理解）
        user_input_section = user_input
        if vlm_understanding:
            user_input_section = f"{vlm_understanding}\n\n【用户指令】\n{user_input}"

        # 检查模板是否已经包含available_skills（已由load_dynamic_prompt填充）
        if "{available_skills}" in self.prompt_template:
            # 还未填充，使用传入的skills
            prompt = self.prompt_template.format(
                user_input=user_input_section,  # 使用增强的输入
                available_skills=skills_desc
            )
        else:
            # 已经填充过了，只替换user_input
            prompt = self.prompt_template.format(
                user_input=user_input_section  # 使用增强的输入
            )

        # 添加环境状态信息（如果有）
        if env_state:
            prompt += f"\n\n当前环境状态:\n{json.dumps(env_state, indent=2, ensure_ascii=False)}"

        try:
            # 使用简化的 prompt，要求在 JSON 中包含 reasoning 字段
            enhanced_prompt = f"""请将用户的指令分解为任务序列。

要求：
1. 必须在 JSON 中包含 "reasoning" 字段，详细说明你的分析过程（300字左右）
2. 然后列出具体的任务序列

输出格式（JSON）：
{{
  "reasoning": "你的思考过程，包括需求分析、技能选择、参数设置等",
  "tasks": [
    {{"step": 1, "task": "子任务描述", "type": "动作类型"}}
  ],
  "summary": "整体任务概述"
}}

{prompt}"""

            # 使用流式调用
            stream = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {
                        "role": "system",
                        "content": "你是一个专业的机器人任务规划助手。输出必须是有效的JSON格式，必须包含reasoning字段说明你的思考过程。"
                    },
                    {"role": "user", "content": enhanced_prompt}
                ],
                temperature=0.3,
                stream=True
            )

            # 收集响应
            response_content = []

            for chunk in stream:
                if chunk.choices:
                    delta = chunk.choices[0].delta
                    if hasattr(delta, 'content') and delta.content:
                        response_content.append(delta.content)

            # 获取完整响应
            full_text = ''.join(response_content).strip()

            # 显示思考过程（如果有 reasoning 字段）
            print(f"\n💭 [High-Level LLM 思考过程]\n", file=sys.stderr)

            try:
                # 尝试直接解析 JSON
                plan = json.loads(full_text)

                # 检查是否有 reasoning 字段
                if 'reasoning' in plan and plan['reasoning']:
                    reasoning = plan['reasoning']
                    # 限制显示长度
                    if len(reasoning) > 300:
                        reasoning = reasoning[:300] + '...'
                    print(f"{reasoning}\n", file=sys.stderr)
                else:
                    print("[提示] 模型未包含 reasoning 字段，下次会提示模型添加", file=sys.stderr)

                # 提取任务和概述
                tasks = plan.get("tasks", [])
                summary = plan.get("summary", "")

            except json.JSONDecodeError:
                # 如果直接解析失败，尝试清理 markdown
                if full_text.startswith("```json"):
                    full_text = full_text.split("```")[1]
                    if full_text.startswith("json"):
                        full_text = full_text[5:]
                elif full_text.startswith("```"):
                    full_text = full_text.split("```")[1]

                full_text = full_text.strip()
                plan = json.loads(full_text)

                tasks = plan.get("tasks", [])
                summary = plan.get("summary", "")

                if 'reasoning' in plan and plan['reasoning']:
                    reasoning = plan['reasoning']
                    if len(reasoning) > 300:
                        reasoning = reasoning[:300] + '...'
                    print(f"{reasoning}\n", file=sys.stderr)

            print(f"{'='*60}\n", file=sys.stderr)

            # 显示任务序列
            print(f"\n✅ [规划完成] 共分解为 {len(tasks)} 个子任务", file=sys.stderr)
            print(f"📋 [任务概述] {summary}\n", file=sys.stderr)
            print("子任务序列：", file=sys.stderr)
            for task in tasks:
                print(f"  步骤 {task['step']}: {task['task']} ({task['type']})", file=sys.stderr)

            return tasks

        except Exception as e:
            print(f"\n❌ [规划失败] {e}", file=sys.stderr)
            print(f"[回退] 将作为单个任务处理", file=sys.stderr)
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
        print("\n" + "="*60, file=sys.stderr)
        print("🔄 [高层LLM] 重新规划中...", file=sys.stderr)
        print("="*60, file=sys.stderr)
        print(f"失败任务: {failed_task.get('task', 'Unknown')}", file=sys.stderr)
        print(f"失败原因: {failure_reason}", file=sys.stderr)

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

            print(f"\n✅ [重新规划完成] 策略: {strategy}", file=sys.stderr)
            print(f"📝 [规划说明] {explanation}\n", file=sys.stderr)
            print(f"新生成 {len(tasks)} 个任务:", file=sys.stderr)
            for task in tasks:
                print(f"  步骤 {task['step']}: {task['task']} ({task['type']})", file=sys.stderr)

            return tasks

        except Exception as e:
            print(f"\n❌ [重新规划失败] {e}", file=sys.stderr)
            print(f"[回退] 返回空任务列表", file=sys.stderr)
            return []
