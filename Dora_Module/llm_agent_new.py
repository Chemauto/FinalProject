#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LLM Agent with MCP Support - Dora版本（使用核心模块）
基于 llm_core.py 的简化版本
"""
import sys
import os
import json
import time

# Reconfigure stdout to use UTF-8 encoding on Windows
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

from dotenv import load_dotenv
from dora import Node
from dora import Node

# 导入核心LLM模块
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'MCP_Server'))
from llm_core import LLMAgent, get_standard_mcp_tools

# 加载环境变量
project_root = os.path.abspath(os.path.join(os.getcwd(), '..'))
dotenv_path = os.path.join(project_root, '.env')
load_dotenv(dotenv_path=dotenv_path)

API_KEY = os.getenv("Test_API_KEY")
if not API_KEY:
    raise ValueError("Missing Test_API_KEY")


class DoraLLMNode:
    """Dora双层LLM节点（使用核心模块）"""

    def __init__(self):
        self.node = Node()
        self.llm_agent = LLMAgent(API_KEY)
        self.tools = get_standard_mcp_tools()

    def execute_tool(self, function_name: str, function_args: dict) -> dict:
        """
        执行工具，发送命令到Dora数据流

        Args:
            function_name: 工具名称
            function_args: 工具参数

        Returns:
            执行结果
        """
        # 转换为Dora命令
        command = self.build_dora_command(function_name, function_args)

        if not command:
            return {"success": False, "error": f"Unknown tool: {function_name}"}

        # 发送到Dora
        import pyarrow as pa
        print(f"[Dora] 发送命令: {command}")
        arrow_data = pa.array([command])
        self.node.send_output("command", arrow_data)

        # 计算延迟
        delay = self.get_action_delay(command)
        return {"delay": delay}

    def build_dora_command(self, function_name: str, function_args: dict) -> dict:
        """将工具调用转换为Dora命令"""
        command = {}

        if function_name == "turn_left":
            angle = function_args.get("angle", 90)
            command = {
                "action": "navigate",
                "parameters": {"angle": f"-{angle}deg"}  # Dora坐标系：左转为负
            }
        elif function_name == "turn_right":
            angle = function_args.get("angle", 90)
            command = {
                "action": "navigate",
                "parameters": {"angle": f"{angle}deg"}  # Dora坐标系：右转为正
            }
        elif function_name == "move_forward":
            distance = function_args["distance"]
            unit = function_args.get("unit", "m")
            command = {
                "action": "navigate",
                "parameters": {"direction": "front", "distance": f"{distance}{unit}"}
            }
        elif function_name == "move_backward":
            distance = function_args["distance"]
            unit = function_args.get("unit", "m")
            command = {
                "action": "navigate",
                "parameters": {"direction": "back", "distance": f"{distance}{unit}"}
            }
        elif function_name == "pick_up":
            object_name = function_args.get("object_name", "unknown")
            command = {
                "action": "pick",
                "parameters": {"object": object_name}
            }
        elif function_name == "place":
            object_name = function_args.get("object_name", "unknown")
            location = function_args["location"]
            command = {
                "action": "place",
                "parameters": {"object": object_name, "location": location}
            }
        elif function_name == "stop":
            command = {"action": "stop", "parameters": {}}

        return command

    def get_action_delay(self, command: dict) -> float:
        """根据命令估算执行时间"""
        action = command.get("action")
        params = command.get("parameters", {})

        if action == "navigate":
            if "angle" in params:
                angle_str = params["angle"]
                angle = float(angle_str.replace("deg", "").replace("-", ""))
                return max(1.5, (angle / 90) * 2.0)
            elif "distance" in params:
                distance_str = params["distance"]
                if distance_str.endswith("cm"):
                    distance = float(distance_str.replace("cm", "")) / 100
                elif distance_str.endswith("mm"):
                    distance = float(distance_str.replace("mm", "")) / 1000
                else:
                    distance = float(distance_str.replace("m", ""))
                return max(1.0, distance / 0.5)
            else:
                return 3.0

        elif action in ["pick", "place"]:
            return 2.0

        elif action == "stop":
            return 0.5

        else:
            return 1.0

    def run(self):
        """运行节点"""
        print("LLM_AGENT_MCP: 双层LLM架构初始化完成（使用核心模块）")
        print("  - 基于 llm_core.py")
        print("  - 上层LLM: 任务规划")
        print("  - 下层LLM: 任务执行")
        print("LLM_AGENT_MCP: 等待用户指令...\n")

        for event in self.node:
            if event["type"] != "INPUT":
                continue

            user_input = event["value"][0].as_py()

            # 使用核心LLM模块执行完整流程
            self.llm_agent.run_pipeline(
                user_input=user_input,
                tools=self.tools,
                execute_tool_fn=self.execute_tool
            )


if __name__ == "__main__":
    node = DoraLLMNode()
    node.run()
