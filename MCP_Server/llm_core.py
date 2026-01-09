#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LLM Core - 双层LLM架构核心模块
包含任务规划和任务执行的通用逻辑，可以被Dora、ROS2等不同适配器使用
"""
from openai import OpenAI
from typing import Callable, Dict, List, Any


class LLMAgent:
    """
    双层LLM代理
    - 上层LLM: 任务规划
    - 下层LLM: 任务执行
    """

    def __init__(self, api_key: str, base_url: str = "https://dashscope.aliyuncs.com/compatible-mode/v1"):
        """
        初始化LLM代理

        Args:
            api_key: API密钥
            base_url: API基础URL
        """
        self.client = OpenAI(api_key=api_key, base_url=base_url)
        self.model = "qwen-plus"

    def plan_tasks(self, user_input: str, tools: List[Dict]) -> List[Dict]:
        """
        上层LLM：将用户输入分解为子任务序列

        Args:
            user_input: 用户输入的自然语言指令
            tools: 可用的工具列表

        Returns:
            子任务列表，例如：
            [
                {"step": 1, "task": "向左转90度", "type": "转向"},
                {"step": 2, "task": "向前走1米", "type": "移动"}
            ]
        """
        print("\n" + "="*60)
        print("🧠 [上层LLM] 任务规划中...")
        print("="*60)

        planning_prompt = f"""你是一个机器人任务规划助手。你的职责是将用户的复杂指令分解为简单的、顺序执行的子任务。

规则：
1. 将复杂指令分解为2-5个简单的子任务
2. 每个子任务应该是单一的、原子的动作
3. 子任务必须按照执行顺序排列
4. 使用清晰、简洁的中文描述每个子任务

输出格式（JSON）：
{{
  "tasks": [
    {{"step": 1, "task": "子任务描述1", "type": "动作类型"}},
    {{"step": 2, "task": "子任务描述2", "type": "动作类型"}},
    ...
  ],
  "summary": "整体任务概述"
}}

动作类型包括：转向、移动、抓取、放置、停止

示例：
输入："先左转90度，再往前走1米"
输出：
{{
  "tasks": [
    {{"step": 1, "task": "向左转90度", "type": "转向"}},
    {{"step": 2, "task": "向前走1米", "type": "移动"}}
  ],
  "summary": "左转后前进"
}}

输入："前进50厘米然后向右转45度"
输出：
{{
  "tasks": [
    {{"step": 1, "task": "向前走50厘米", "type": "移动"}},
    {{"step": 2, "task": "向右转45度", "type": "转向"}}
  ],
  "summary": "前进后右转"
}}

用户输入：{user_input}"""

        try:
            completion = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {
                        "role": "system",
                        "content": "你是一个专业的机器人任务规划助手。输出必须是有效的JSON格式。"
                    },
                    {
                        "role": "user",
                        "content": planning_prompt
                    }
                ],
                temperature=0.3
            )

            response_text = completion.choices[0].message.content.strip()

            # 解析JSON
            if response_text.startswith("```"):
                response_text = response_text.split("```")[1]
                if response_text.startswith("json"):
                    response_text = response_text[4:]

            import json
            plan = json.loads(response_text)

            tasks = plan.get("tasks", [])
            summary = plan.get("summary", "")

            print(f"✅ [规划完成] 共分解为 {len(tasks)} 个子任务")
            print(f"📋 [任务概述] {summary}")
            print("\n子任务序列：")
            for task in tasks:
                print(f"  步骤 {task['step']}: {task['task']} ({task['type']})")

            return tasks

        except Exception as e:
            print(f"❌ [规划失败] {e}")
            print("[回退] 将作为单个任务处理")
            return [
                {"step": 1, "task": user_input, "type": "综合"}
            ]

    def execute_single_task(self, task_description: str, tools: List[Dict],
                           execute_tool_fn: Callable) -> Dict:
        """
        下层LLM：执行单个子任务

        Args:
            task_description: 子任务描述
            tools: 可用的工具列表
            execute_tool_fn: 执行工具的回调函数

        Returns:
            执行结果字典
        """
        import time
        print(f"\n{'─'*50}")
        print(f"⚙️  [执行中] {task_description}")
        print(f"{'─'*50}")

        try:
            # 调用下层LLM将子任务转换为工具调用
            completion = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {
                        "role": "system",
                        "content": "你是一个机器人控制助手。根据子任务描述，调用相应的工具函数。"
                    },
                    {
                        "role": "user",
                        "content": f"执行任务：{task_description}"
                    }
                ],
                tools=tools,
                tool_choice="auto"
            )

            response_message = completion.choices[0].message
            tool_calls = response_message.tool_calls

            if not tool_calls:
                print("[跳过] 没有需要执行的动作")
                return {"success": True, "action": "none"}

            # 执行工具调用
            tool_call = tool_calls[0]
            function_name = tool_call.function.name
            function_args = json.loads(tool_call.function.arguments)

            print(f"🔧 [工具调用] {function_name}({function_args})")

            # 调用执行函数
            result = execute_tool_fn(function_name, function_args)

            # 等待执行完成
            if result.get("delay"):
                delay = result["delay"]
                print(f"⏳ [等待] 执行时间: {delay:.1f}秒", end="", flush=True)
                steps = max(1, int(delay))
                for i in range(steps):
                    time.sleep(delay / steps)
                    print(".", end="", flush=True)
                print(" ✅ 完成!")

            return {
                "success": True,
                "action": function_name,
                "task": task_description,
                "result": result
            }

        except Exception as e:
            print(f"\n❌ [错误] 执行失败: {e}")
            return {
                "success": False,
                "error": str(e),
                "task": task_description
            }

    def run_pipeline(self, user_input: str, tools: List[Dict],
                    execute_tool_fn: Callable) -> List[Dict]:
        """
        运行完整的双层LLM流程

        Args:
            user_input: 用户输入
            tools: 可用工具列表
            execute_tool_fn: 执行工具的回调函数

        Returns:
            所有子任务的执行结果列表
        """
        print("\n" + "█"*60)
        print(f"📥 [用户输入] {user_input}")
        print("█"*60)

        try:
            # 阶段1: 上层LLM - 任务规划
            tasks = self.plan_tasks(user_input, tools)

            # 阶段2: 顺序执行每个子任务
            print("\n" + "█"*60)
            print("🚀 [开始执行] 按顺序执行子任务")
            print("█"*60)

            results = []
            for idx, task in enumerate(tasks, 1):
                print(f"\n【步骤 {idx}/{len(tasks)}】")
                result = self.execute_single_task(task["task"], tools, execute_tool_fn)
                results.append(result)

                if not result.get("success"):
                    print(f"\n⚠️  [警告] 步骤 {idx} 失败，但继续执行后续任务")

            # 总结
            print("\n" + "█"*60)
            print("✅ [执行完成] 任务总结")
            print("█"*60)
            for idx, (task, result) in enumerate(zip(tasks, results), 1):
                status = "✅ 成功" if result.get("success") else "❌ 失败"
                print(f"  {idx}. {task['task']} - {status}")

            return results

        except Exception as e:
            print(f"\n❌ [错误] {type(e).__name__}: {e}")
            import traceback
            traceback.print_exc()
            return []


def get_standard_mcp_tools():
    """
    获取标准的MCP工具定义
    所有适配器共享相同的工具定义

    Returns:
        标准的MCP工具列表
    """
    return [
        {
            "type": "function",
            "function": {
                "name": "turn_left",
                "description": "向左转指定角度",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "angle": {
                            "type": "number",
                            "description": "转向角度（度），默认90度",
                            "default": 90
                        }
                    }
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "turn_right",
                "description": "向右转指定角度",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "angle": {
                            "type": "number",
                            "description": "转向角度（度），默认90度",
                            "default": 90
                        }
                    }
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "move_forward",
                "description": "向前移动指定距离",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "distance": {
                            "type": "number",
                            "description": "移动距离数值"
                        },
                        "unit": {
                            "type": "string",
                            "description": "距离单位（m/cm/mm）",
                            "enum": ["m", "cm", "mm"],
                            "default": "m"
                        }
                    },
                    "required": ["distance"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "move_backward",
                "description": "向后移动指定距离",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "distance": {
                            "type": "number",
                            "description": "移动距离数值"
                        },
                        "unit": {
                            "type": "string",
                            "description": "距离单位（m/cm/mm）",
                            "enum": ["m", "cm", "mm"],
                            "default": "m"
                        }
                    },
                    "required": ["distance"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "pick_up",
                "description": "抓取指定物体",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "object_name": {
                            "type": "string",
                            "description": "要抓取的物体名称"
                        }
                    },
                    "required": ["object_name"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "place",
                "description": "放置物体到指定位置",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "object_name": {
                            "type": "string",
                            "description": "物体名称"
                        },
                        "location": {
                            "type": "string",
                            "description": "目标位置"
                        }
                    },
                    "required": ["object_name", "location"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "stop",
                "description": "停止机器人",
                "parameters": {
                    "type": "object",
                    "properties": {}
                }
            }
        },
    ]
