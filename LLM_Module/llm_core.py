#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LLM Core - 双层LLM架构核心模块
包含任务规划和任务执行的通用逻辑
"""
import os
import sys
import json
import yaml
from openai import OpenAI
from typing import Callable, Dict, List, Any

class LLMAgent:
    """
    双层LLM代理
    - 上层LLM: 任务规划
    - 下层LLM: 任务执行
    """

    def __init__(self, api_key: str, base_url: str = "https://dashscope.aliyuncs.com/compatible-mode/v1", prompt_path: str = None):
        """
        初始化LLM代理
        """
        self.client = OpenAI(api_key=api_key, base_url=base_url)
        self.model = "qwen-plus"
        self.planning_prompt_template = self.load_prompt(prompt_path)

    def load_prompt(self, prompt_path: str) -> str:
        """从YAML文件加载规划Prompt"""
        if not prompt_path or not os.path.exists(prompt_path):
            print("⚠️ 警告: Prompt文件路径未提供或不存在，将使用默认的内置Prompt。")
            return "你是一个机器人任务规划助手。请将用户的复杂指令分解为简单的子任务。用户输入：{user_input}"
        
        try:
            with open(prompt_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                return data.get("prompt", "")
        except Exception as e:
            print(f"❌ 错误: 加载Prompt文件失败: {e}")
            return ""

    def plan_tasks(self, user_input: str, tools: List[Dict]) -> List[Dict]:
        """
        上层LLM：将用户输入分解为子任务序列
        """
        print("\n" + "="*60 + "\n🧠 [上层LLM] 任务规划中...\n" + "="*60)
        
        planning_prompt = self.planning_prompt_template.format(user_input=user_input)

        try:
            completion = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": "你是一个专业的机器人任务规划助手。输出必须是有效的JSON格式。"},
                    {"role": "user", "content": planning_prompt}
                ],
                temperature=0.3,
                extra_body={"enable_thinking": False}
            )
            response_text = completion.choices[0].message.content.strip()
            if response_text.startswith("```"):
                response_text = response_text.split("```")[1]
                if response_text.startswith("json"): response_text = response_text[4:]
            
            plan = json.loads(response_text)
            tasks = plan.get("tasks", [])
            summary = plan.get("summary", "")

            print(f"✅ [规划完成] 共分解为 {len(tasks)} 个子任务\n📋 [任务概述] {summary}\n\n子任务序列：")
            for task in tasks: print(f"  步骤 {task['step']}: {task['task']} ({task['type']})")
            return tasks
        except Exception as e:
            print(f"❌ [规划失败] {e}\n[回退] 将作为单个任务处理")
            return [{"step": 1, "task": user_input, "type": "综合"}]

    def execute_single_task(self, task_description: str, tools: List[Dict], execute_tool_fn: Callable, previous_result: Any = None) -> Dict:
        """
        下层LLM：执行单个子任务

        Args:
            task_description: 任务描述
            tools: 可用工具列表
            execute_tool_fn: 工具执行函数
            previous_result: 上一步的执行结果（如果有）
        """
        import time
        print(f"\n{'─'*50}\n⚙️  [执行中] {task_description}\n{'─'*50}")
        try:
            system_prompt = """你是一个机器人控制助手。根据子任务描述，调用相应的工具函数。

【关键规则 - 强制执行】
1. **追击敌人任务（最重要）**：
   - 如果任务描述包含"追击"字样，必须调用 chase_enemy(enemy_positions=...)
   - enemy_positions 参数必须使用"上一步的结果"中的值
   - 即使上一步的结果是空列表 []，也要传递给 chase_enemy！
   - **绝对禁止**在追击任务中调用 get_enemy_positions()

2. **正确示例**：
   - 任务："获取敌人位置" → 调用 get_enemy_positions()
   - 任务："追击最近的敌人" + 上一步结果='[{"id":"1","x":100,"y":200}]' → 调用 chase_enemy(enemy_positions='[{"id":"1","x":100,"y":200}]')
   - 任务："追击最近的敌人" + 上一步结果='[]' → 调用 chase_enemy(enemy_positions='[]')

3. **错误示例（不要这样做）**：
   - 任务："追击敌人" → 调用 get_enemy_positions()  ❌ 错误！
   - 任务："追击敌人" → 调用 chase_enemy()  ❌ 缺少参数！

4. **图片路径参数**：
   - 如果任务描述中包含文件路径（.png, .jpg），必须将其作为参数传入
   - 调用 detect_color_and_act 时，必须设置 image_path 参数

【执行决策】
- 如果有"上一步的结果"且任务是"追击" → chase_enemy(enemy_positions=上一步的结果)
- 如果任务是"获取敌人位置" → get_enemy_positions()
- 其他任务 → 根据描述调用相应工具

记住：chase_enemy 需要一个 JSON 字符串参数，不要调用它时省略参数！"""

            # 构建用户消息
            user_message = f"执行任务：{task_description}"
            if previous_result is not None:
                user_message += f"\n\n上一步的结果：{previous_result}"
                print(f"[LLM] 上一步结果: {previous_result}", file=sys.stderr)
            else:
                print(f"[LLM] 没有上一步结果", file=sys.stderr)

            completion = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_message}
                ],
                tools=tools,
                tool_choice="auto",
                extra_body={"enable_thinking": False}
            )
            response_message = completion.choices[0].message
            tool_calls = response_message.tool_calls
            if not tool_calls:
                print("[跳过] 无效指令或无法识别的操作")
                return {"success": False, "action": "none", "error": "No tool called"}

            tool_call = tool_calls[0]
            function_name = tool_call.function.name
            function_args = json.loads(tool_call.function.arguments)
            print(f"🔧 [工具调用] {function_name}({function_args})")
            result = execute_tool_fn(function_name, function_args)

            if result and result.get("delay"):
                delay = result["delay"]
                print(f"⏳ [等待] 执行时间: {delay:.1f}秒", end="", flush=True)
                steps = max(1, int(delay))
                for i in range(steps):
                    time.sleep(delay / steps)
                    print(".", end="", flush=True)
                print(" ✅ 完成!")
            return {"success": True, "action": function_name, "task": task_description, "result": result}
        except Exception as e:
            print(f"\n❌ [错误] 执行失败: {e}")
            return {"success": False, "error": str(e), "task": task_description}

    def run_pipeline(self, user_input: str, tools: List[Dict], execute_tool_fn: Callable) -> List[Dict]:
        """
        运行完整的双层LLM流程
        """
        print("\n" + "█"*60 + f"\n📥 [用户输入] {user_input}\n" + "█"*60)
        try:
            tasks = self.plan_tasks(user_input, tools)

            # 如果没有任务，直接返回
            if not tasks:
                return []

            print("\n" + "█"*60 + "\n🚀 [开始执行] 按顺序执行子任务\n" + "█"*60)
            results = []
            previous_result = None

            for idx, task in enumerate(tasks, 1):
                print(f"\n【步骤 {idx}/{len(tasks)}】")
                result = self.execute_single_task(task["task"], tools, execute_tool_fn, previous_result)
                results.append(result)

                # 保存结果供下一步使用
                if result.get("success") and result.get("result"):
                    previous_result = result["result"].get("result")
                else:
                    previous_result = None

                if not result.get("success"):
                    print(f"\n⚠️  [警告] 步骤 {idx} 失败，但继续执行后续任务")

            print("\n" + "█"*60 + "\n✅ [执行完成] 任务总结\n" + "█"*60)
            for idx, (task, result) in enumerate(zip(tasks, results), 1):
                status = "✅ 成功" if result.get("success") else "❌ 失败"
                print(f"  {idx}. {task['task']} - {status}")
            return results
        except Exception as e:
            print(f"\n❌ [错误] {type(e).__name__}: {e}")
            import traceback
            traceback.print_exc()
            return []
		
