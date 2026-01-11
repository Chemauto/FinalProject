#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dora Interactive MCP - 双层LLM架构
使用 LLM_Module 的 LLMAgent 进行任务规划和执行
使用 MCP_Module 获取 Robot_Module 中的技能定义
"""
import sys
import os
import json
from dotenv import load_dotenv

# Import from new modular structure
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from LLM_Module import LLMAgent
from MCP_Module import create_mcp_bridge

# Reconfigure stdout to use UTF-8 encoding on Windows
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

# 加载环境变量
dotenv_path = os.path.join(project_root, '.env')
load_dotenv(dotenv_path=dotenv_path)

API_KEY = os.getenv("Test_API_KEY")
if not API_KEY:
    print("⚠️  警告: 未找到 Test_API_KEY，请检查 .env 文件")

# Dora imports
import pyarrow as pa
from dora import Node


def main():
    """Dora LLM Agent 主函数"""
    if not API_KEY:
        print("❌ 错误: 未配置 Test_API_KEY")
        return

    # 1. 创建 MCP Bridge 并加载机器人技能
    print("\n🔧 初始化 MCP Bridge...")
    mcp_bridge = create_mcp_bridge(['Sim_2D'])

    # 2. 获取技能工具定义 (用于 LLM)
    mcp_tools = mcp_bridge.get_mcp_tools_definition()

    print(f"✅ 已加载 {len(mcp_bridge.get_available_skills())} 个技能")

    # 3. 初始化 LLM Agent
    prompt_path = os.path.join(project_root, 'LLM_Module', 'prompts', 'planning_prompt_2d.yaml')

    # 检查提示词文件是否存在
    if not os.path.exists(prompt_path):
        print(f"⚠️  警告: Prompt文件不存在: {prompt_path}")
        print(f"   将使用默认的内置Prompt")
        prompt_path = None

    llm_agent = LLMAgent(api_key=API_KEY, prompt_path=prompt_path)

    print(f"\n✅ LLM Agent 初始化成功")

    # 4. 初始化 Dora 节点
    node = Node("llm-agent-mcp")

    print(f"✅ Dora 节点已连接")
    print(f"✅ 系统就绪，等待用户输入...\n")

    # 定义工具执行函数
    def execute_tool_fn(skill_name: str, skill_params: dict):
        """
        执行技能

        通过 MCP_Bridge 调用 Robot_Module 中的技能
        然后将结果发送到 Dora 数据流
        """
        # 调用 MCP_Bridge 执行技能
        result = mcp_bridge.execute_skill(skill_name, **skill_params)

        if result.get('success'):
            # 从技能结果中提取 action 和 parameters
            skill_result = result.get('result', {})
            action = skill_result.get('action')
            parameters = skill_result.get('parameters', {})

            # 构造 Dora 输出消息
            dora_output = {
                "action": action,
                "parameters": parameters
            }

            # 发送到 Dora simulator (使用 PyArrow 数组)
            print(f"📤 [Dora] 发送到仿真器: action={action}, params={parameters}")
            node.send_output("command", pa.array([dora_output]))

            # 计算等待时间
            if action in ["turn_left", "turn_right"]:
                angle_str = parameters.get("angle", "90deg")
                angle = float(angle_str.replace("deg", "").replace("-", ""))
                delay = max(1.0, (abs(angle) / 90) * 2.0)
            elif action == "navigate":
                distance_str = parameters.get("distance", "1m")
                if "cm" in distance_str:
                    distance = float(distance_str.replace("cm", "")) / 100
                elif "mm" in distance_str:
                    distance = float(distance_str.replace("mm", "")) / 1000
                else:
                    distance = float(distance_str.replace("m", ""))
                delay = max(1.0, distance / 0.5)
            else:
                delay = 1.0

            return {"status": "success", "delay": delay}
        else:
            raise ValueError(f"技能执行失败: {result.get('error')}")

    # 5. 主循环：处理 Dora 消息
    print("🔄 [Dora] 进入事件循环...")
    print(f"📋 [Dora] 监听输入: user_command")

    while True:
        event = node.next()
        if event is None:
            continue

        if event["type"] == "STOP":
            print("🛑 [Dora] 收到 STOP 事件，正在退出...")
            break

        if event["type"] == "INPUT":
            input_id = event.get("id", "")
            print(f"📥 [Dora] 收到输入: {input_id}")

            if input_id == "user_command":
                try:
                    # 接收来自 input_ui 的数据
                    user_input_data = event["value"][0]
                    user_input = str(user_input_data.as_py())

                    print(f"\n{'█'*60}")
                    print(f"📥 [用户输入] {user_input}")
                    print(f"{'█'*60}")

                    # 使用 LLM Agent 的 pipeline 处理输入
                    llm_agent.run_pipeline(user_input, mcp_tools, execute_tool_fn)
                except Exception as e:
                    print(f"❌ [错误] 处理输入失败: {e}")
                    import traceback
                    traceback.print_exc()


if __name__ == "__main__":
    main()
