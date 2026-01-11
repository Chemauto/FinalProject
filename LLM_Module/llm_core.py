#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LLM Core - 双层LLM架构核心模块
包含任务规划和任务执行的通用逻辑，可以被Dora、ROS2等不同适配器使用
"""
import os
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
                temperature=0.3
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

    def execute_single_task(self, task_description: str, tools: List[Dict], execute_tool_fn: Callable) -> Dict:
        """
        下层LLM：执行单个子任务
        """
        import time
        print(f"\n{'─'*50}\n⚙️  [执行中] {task_description}\n{'─'*50}")
        try:
            completion = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": "你是一个机器人控制助手。根据子任务描述，调用相应的工具函数。如果无法识别任务或不属于机器人操作，返回空结果。"},
                    {"role": "user", "content": f"执行任务：{task_description}"}
                ],
                tools=tools,
                tool_choice="auto"
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
            for idx, task in enumerate(tasks, 1):
                print(f"\n【步骤 {idx}/{len(tasks)}】")
                result = self.execute_single_task(task["task"], tools, execute_tool_fn)
                results.append(result)
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
		
def get_standard_mcp_tools():
		
    """
		
    获取标准的MCP工具定义
		
    """
		
    # This function is now just a fallback, the main logic uses the YAML files.
		
    return [
		
        {"type": "function", "function": {"name": "turn_left", "description": "向左转指定角度", "parameters": {"type": "object", "properties": {"angle": {"type": "number", "description": "转向角度（度）"}}}}},
		
        {"type": "function", "function": {"name": "turn_right", "description": "向右转指定角度", "parameters": {"type": "object", "properties": {"angle": {"type": "number", "description": "转向角度（度）"}}}}},
		
        {"type": "function", "function": {"name": "move_forward", "description": "向前移动指定距离", "parameters": {"type": "object", "properties": {"distance": {"type": "number"}, "unit": {"type": "string", "enum": ["m", "cm", "mm"]}}, "required": ["distance"]}}},
		
    ]
		

		
def get_gazebo_mcp_tools():
		
    """
		
    获取Gazebo测试机器人的MCP工具定义
		
    """
		
    return [
		
        {
		
            "type": "function",
		
            "function": {
		
                "name": "move",
		
                "description": "Moves the robot in a specified direction and distance, supporting omnidirectional movement.",
		
                "parameters": {
		
                    "type": "object",
		
                    "properties": {
		
                        "direction": {
		
                            "type": "string",
		
                            "description": "The direction to move. Can be 'forward', 'backward', 'left', 'right'.",
		
                            "enum": ["forward", "backward", "left", "right"]
		
                        },
		
                        "distance": {
		
                            "type": "number",
		
                            "description": "The distance to move in meters."
		
                        }
		
                    },
		
                    "required": ["direction", "distance"]
		
                }
		
            }
		
        },
		
        {
		
            "type": "function",
		
            "function": {
		
                "name": "rotate",
		
                "description": "Rotates the robot by a specified angle.",
		
                "parameters": {
		
                    "type": "object",
		
                    "properties": {
		
                        "angle": {
		
                            "type": "number",
		
                            "description": "The angle to rotate in degrees. Positive for counter-clockwise (left), negative for clockwise (right)."
		
                        }
		
                    },
		
                    "required": ["angle"]
		
                }
		
            }
		
        },
		
        {
		
            "type": "function",
		
            "function": {
		
                "name": "stop",
		
                "description": "Stops the robot's movement immediately.",
		
                "parameters": { "type": "object", "properties": {} }
		
            }
		
        }
		
    ]
		