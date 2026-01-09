#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 Interactive MCP - 双层LLM架构
类似 dora-interactive-mcp.yaml 的 ROS2 版本

上层LLM: 任务规划（将复杂指令分解为子任务）
下层LLM: 执行子任务（调用ROS2工具）
"""
import sys
import os
import json
import time
import asyncio
from dotenv import load_dotenv
from openai import OpenAI

# Reconfigure stdout to use UTF-8 encoding on Windows
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

# 加载环境变量
# 获取项目根目录（FinalProject/）
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
dotenv_path = os.path.join(project_root, '.env')
load_dotenv(dotenv_path=dotenv_path)

API_KEY = os.getenv("Test_API_KEY")
if not API_KEY:
    print("⚠️  警告: 未找到 Test_API_KEY，请检查 .env 文件")
    print("   您可以从 https://dashscope.aliyun.com 获取 API Key")
    # 不退出，允许使用测试模式

# 初始化 ROS2
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("❌ ROS2 未安装，请先安装 ROS2 Humble")

# ---------- OpenAI Client ----------
client = OpenAI(
    api_key=API_KEY or "dummy",
    base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
)

# ---------- MCP Tool Definitions ----------
MCP_TOOLS = [
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


class Ros2CommandPublisher(Node):
    """ROS2 命令发布节点"""

    def __init__(self):
        super().__init__('ros2_interactive_mcp')

        # 发布到 robot_command 话题
        self.publisher = self.create_publisher(String, '/robot_command', 10)

        self.get_logger().info('[ROS2] Interactive MCP Command Publisher Ready')

    def send_command(self, action: str, parameters: dict):
        """
        发送命令到 ROS2

        Args:
            action: 动作名称
            parameters: 参数字典
        """
        command = {
            "action": action,
            "parameters": parameters
        }

        msg = String()
        msg.data = json.dumps(command, ensure_ascii=False)
        self.publisher.publish(msg)

        return command


# ---------- Task Planning (Upper LLM) ----------
def plan_tasks(user_input: str) -> list:
    """
    上层LLM：将用户输入分解为子任务序列

    Args:
        user_input: 用户输入的自然语言指令

    Returns:
        子任务列表
    """
    print("\n" + "="*60)
    print("🧠 [上层LLM] 任务规划中...")
    print("="*60)

    planning_prompt = """你是一个机器人任务规划助手。你的职责是将用户的复杂指令分解为简单的、顺序执行的子任务。

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

用户输入：""" + user_input

    try:
        completion = client.chat.completions.create(
            model="qwen-plus",
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
            temperature=0.3  # 降低随机性，确保输出稳定
        )

        response_text = completion.choices[0].message.content.strip()

        # 解析JSON
        # 移除可能的markdown代码块标记
        if response_text.startswith("```"):
            response_text = response_text.split("```")[1]
            if response_text.startswith("json"):
                response_text = response_text[4:]

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
        # 回退：将整个输入作为单个任务
        return [
            {"step": 1, "task": user_input, "type": "综合"}
        ]


def execute_single_task(task_description: str, ros2_node) -> dict:
    """
    下层LLM：执行单个子任务

    Args:
        task_description: 子任务描述
        ros2_node: ROS2节点实例

    Returns:
        执行结果
    """
    print(f"\n{'─'*50}")
    print(f"⚙️  [执行中] {task_description}")
    print(f"{'─'*50}")

    try:
        # 调用下层LLM将子任务转换为工具调用
        completion = client.chat.completions.create(
            model="qwen-plus",
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
            tools=MCP_TOOLS,
            tool_choice="auto"
        )

        response_message = completion.choices[0].message
        tool_calls = response_message.tool_calls

        if not tool_calls:
            print("[跳过] 没有需要执行的动作")
            return {"success": True, "action": "none"}

        # 执行工具调用（单个任务通常只有一个调用）
        tool_call = tool_calls[0]
        function_name = tool_call.function.name
        function_args = json.loads(tool_call.function.arguments)

        print(f"🔧 [工具调用] {function_name}({function_args})")

        # 构建ROS2命令
        command = build_command_from_tool(function_name, function_args)

        if not command:
            print(f"❌ [错误] 未知工具: {function_name}")
            return {"success": False, "error": f"Unknown tool: {function_name}"}

        # 发送命令到ROS2
        print(f"📤 [ROS2] 发送命令: {command}")
        ros2_node.send_command(command["action"], command["parameters"])

        # 等待动作完成
        delay = get_action_delay_from_command(command)
        print(f"⏳ [等待] 执行时间: {delay:.1f}秒", end="", flush=True)

        # 分段等待显示进度
        steps = max(1, int(delay))
        for i in range(steps):
            time.sleep(delay / steps)
            print(".", end="", flush=True)

        print(" ✅ 完成!")

        return {
            "success": True,
            "action": function_name,
            "task": task_description
        }

    except Exception as e:
        print(f"\n❌ [错误] 执行失败: {e}")
        return {
            "success": False,
            "error": str(e),
            "task": task_description
        }


def build_command_from_tool(function_name: str, function_args: dict) -> dict:
    """
    将工具调用转换为ROS2命令

    Args:
        function_name: 工具名称
        function_args: 工具参数

    Returns:
        ROS2命令字典
    """
    command = {}

    if function_name == "turn_left":
        angle = function_args.get("angle", 90)
        command = {
            "action": "turn_left",
            "parameters": {"angle": f"{angle}deg"}  # 左转：正角度
        }
    elif function_name == "turn_right":
        angle = function_args.get("angle", 90)
        command = {
            "action": "turn_right",
            "parameters": {"angle": f"-{angle}deg"}  # 右转：负角度
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


def get_action_delay_from_command(command: dict) -> float:
    """
    根据命令估算执行时间

    Args:
        command: ROS2命令字典

    Returns:
        延迟时间（秒）
    """
    action = command.get("action")
    params = command.get("parameters", {})

    if action in ["turn_left", "turn_right"]:
        # 转向
        if "angle" in params:
            angle_str = params["angle"]
            angle = float(angle_str.replace("deg", "").replace("-", ""))
            return max(1.5, (angle / 90) * 2.0)
        return 2.0

    elif action == "navigate":
        # 移动
        if "distance" in params:
            distance_str = params["distance"]
            if distance_str.endswith("cm"):
                distance = float(distance_str.replace("cm", "")) / 100
            elif distance_str.endswith("mm"):
                distance = float(distance_str.replace("mm", "")) / 1000
            else:  # m
                distance = float(distance_str.replace("m", ""))
            return max(1.0, distance / 0.5)
        return 2.0

    elif action in ["pick", "place"]:
        return 2.0

    elif action == "stop":
        return 0.5

    else:
        return 1.0


# ---------- Interactive Mode ----------
def run_interactive_mode(ros2_node):
    """
    交互式模式：等待用户输入并执行

    Args:
        ros2_node: ROS2节点实例
    """
    print("\n" + "█"*60)
    print("🤖 ROS2 Interactive MCP - 双层LLM架构")
    print("█"*60)
    print("  - 上层LLM: 任务规划 (qwen-plus)")
    print("  - 下层LLM: 任务执行 (qwen-plus)")
    print("  - 执行层: ROS2 Humble")
    print("="*60)
    print("\n💡 输入示例:")
    print('  - "前进1米"')
    print('  - "先左转90度，再往前走1米"')
    print('  - "前进50厘米然后向右转45度"')
    print('  - "抓取杯子"')
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

            # 显示用户输入
            print("\n" + "█"*60)
            print(f"📥 [用户输入] {user_input}")
            print("█"*60)

            try:
                # 阶段1: 上层LLM - 任务规划
                tasks = plan_tasks(user_input)

                # 阶段2: 顺序执行每个子任务
                print("\n" + "█"*60)
                print("🚀 [开始执行] 按顺序执行子任务")
                print("█"*60)

                results = []
                for idx, task in enumerate(tasks, 1):
                    print(f"\n【步骤 {idx}/{len(tasks)}】")
                    result = execute_single_task(task["task"], ros2_node)
                    results.append(result)

                    # 如果某个任务失败，询问是否继续
                    if not result.get("success"):
                        print(f"\n⚠️  [警告] 步骤 {idx} 失败，但继续执行后续任务")

                # 总结
                print("\n" + "█"*60)
                print("✅ [执行完成] 任务总结")
                print("█"*60)
                for idx, (task, result) in enumerate(zip(tasks, results), 1):
                    status = "✅ 成功" if result.get("success") else "❌ 失败"
                    print(f"  {idx}. {task['task']} - {status}")

            except Exception as e:
                print(f"\n❌ [错误] {type(e).__name__}: {e}")
                import traceback
                traceback.print_exc()

        except KeyboardInterrupt:
            print("\n\n👋 被用户中断，退出...")
            break
        except Exception as e:
            print(f"\n❌ [错误] {e}")


def main():
    """主函数"""
    # 检查API Key
    if not API_KEY:
        print("❌ 错误: 未配置 Test_API_KEY")
        print("\n请按以下步骤配置:")
        print("1. 访问 https://dashscope.aliyun.com 获取 API Key")
        print("2. 在项目根目录创建 .env 文件")
        print("3. 添加内容: Test_API_KEY=你的API密钥")
        return

    # 初始化 ROS2
    if not ROS2_AVAILABLE:
        print("❌ 错误: ROS2 未安装或无法导入")
        print("\n请参考 ROS2_QUICKSTART.md 安装 ROS2 Humble")
        return

    rclpy.init()

    try:
        # 创建 ROS2 节点
        ros2_node = Ros2CommandPublisher()

        print("\n✅ ROS2 初始化成功")
        print(f"✅ 节点名称: {ros2_node.get_name()}")
        print(f"✅ 发布话题: /robot_command")

        # 运行交互式模式
        run_interactive_mode(ros2_node)

    except Exception as e:
        print(f"\n❌ [致命错误] {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理
        rclpy.shutdown()


if __name__ == "__main__":
    main()
