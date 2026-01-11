#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 Interactive MCP - 双层LLM架构
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

# 初始化 ROS2
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("❌ ROS2 未安装，请先安装 ROS2 Humble")

# ---------- ROS2 Command Publisher ----------
class Ros2CommandPublisher(Node):
    """ROS2 命令发布节点"""
    def __init__(self):
        super().__init__('ros2_interactive_mcp')
        self.publisher = self.create_publisher(String, '/robot_command', 10)
        self.get_logger().info('[ROS2] Interactive MCP Command Publisher Ready')

    def send_command(self, action: str, parameters: dict):
        """发送命令到 ROS2"""
        command = {"action": action, "parameters": parameters}
        msg = String()
        msg.data = json.dumps(command, ensure_ascii=False)
        self.publisher.publish(msg)
        self.get_logger().info(f'[ROS2] 发送命令: {command}')
        return command

# ---------- Helper Functions ----------
def get_action_delay(action: str, parameters: dict) -> float:
    """根据命令计算执行时间"""
    if action in ["turn_left", "turn_right"]:
        angle_str = parameters.get("angle", "90deg")
        angle = float(angle_str.replace("deg", ""))
        return max(1.5, (abs(angle) / 90) * 2.0)
    elif action == "navigate":
        distance_str = parameters.get("distance", "1m")
        if "cm" in distance_str:
            distance = float(distance_str.replace("cm", "")) / 100
        elif "mm" in distance_str:
            distance = float(distance_str.replace("mm", "")) / 1000
        else:
            distance = float(distance_str.replace("m", ""))
        return max(1.0, distance / 0.5)
    elif action in ["pick", "place"]:
        return 2.0
    return 1.0

# ---------- Interactive Mode ----------
def run_interactive_mode(llm_agent, mcp_tools, ros2_node):
    print("\n" + "█"*60 + "\n🤖 ROS2 Interactive MCP - 双层LLM架构\n" + "█"*60)
    print("  - 上层LLM: 任务规划 (qwen-plus)")
    print("  - 下层LLM: 任务执行 (qwen-plus)")
    print("  - 技能来源: Robot_Module/Sim_2D/skills/")
    print("  - 执行层: ROS2 Humble (2D Simulation)")
    print("="*60)

    # 显示可用技能
    print("\n📋 可用技能:")
    for tool in mcp_tools:
        func = tool.get("function", {})
        name = func.get("name", "unknown")
        desc = func.get("description", "无描述")
        print(f"  - {name}: {desc}")

    print("\n💡 输入示例:")
    print('  - "前进1米"')
    print('  - "先左转90度，再往前走1米"')
    print('  - "前进50厘米然后向右转45度"')
    print('  - 输入 "q" 或 "quit" 退出')
    print("\n" + "="*60)
    print("⌨️  请输入指令...\n")

    while True:
        try:
            user_input = input("\n👤 用户> ").strip()
            if not user_input:
                continue
            if user_input.lower() in ['q', 'quit', 'exit', '退出']:
                print("\n👋 再见!")
                break

            # 定义工具执行函数
            def execute_tool_fn(skill_name: str, skill_params: dict):
                """
                执行技能

                通过 MCP_Bridge 调用 Robot_Module 中的技能
                然后将结果发送到 ROS2
                """
                # 调用 MCP_Bridge 执行技能
                result = mcp_bridge.execute_skill(skill_name, **skill_params)

                if result.get('success'):
                    # 从技能结果中提取 action 和 parameters
                    skill_result = result.get('result', {})
                    action = skill_result.get('action')
                    parameters = skill_result.get('parameters', {})

                    # 发送到 ROS2
                    ros2_node.send_command(action, parameters)

                    # 计算等待时间
                    delay = get_action_delay(action, parameters)
                    return {"status": "success", "delay": delay}
                else:
                    raise ValueError(f"技能执行失败: {result.get('error')}")

            # 使用 LLM Agent 的 pipeline
            llm_agent.run_pipeline(user_input, mcp_tools, execute_tool_fn)

        except KeyboardInterrupt:
            print("\n\n👋 被用户中断，退出...")
            break
        except Exception as e:
            print(f"\n❌ [错误] {e}")
            import traceback
            traceback.print_exc()

def main():
    if not API_KEY:
        print("❌ 错误: 未配置 Test_API_KEY")
        return
    if not ROS2_AVAILABLE:
        print("❌ 错误: ROS2 未安装或无法导入")
        return

    rclpy.init()
    try:
        ros2_node = Ros2CommandPublisher()

        # 1. 创建 MCP Bridge 并加载机器人技能
        print("\n🔧 初始化 MCP Bridge...")
        global mcp_bridge
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

        print(f"\n✅ ROS2 初始化成功")
        print(f"✅ 节点名称: {ros2_node.get_name()}")
        print(f"✅ 发布话题: /robot_command")

        run_interactive_mode(llm_agent, mcp_tools, ros2_node)

    except Exception as e:
        print(f"\n❌ [致命错误] {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
