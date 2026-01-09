# -*- coding: utf-8 -*-
"""
MCP Robot Control Server
提供机器人控制的MCP服务器，可以被LLM调用
"""
import sys
import os
import json
from typing import Any, Optional
import asyncio

# Reconfigure stdout to use UTF-8 encoding on Windows
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

# 导入mcp相关模块
try:
    from mcp.server.models import InitializationOptions
    from mcp.server import NotificationOptions, Server
    from mcp.types import (
        Resource,
        Tool,
        TextContent,
        ImageContent,
        EmbeddedResource,
    )
    MCP_AVAILABLE = True
except ImportError:
    MCP_AVAILABLE = False
    print("ERROR: MCP package not installed. Please run: pip install mcp")
    sys.exit(1)

from robot_skills import RobotSkills
from adapters import DoraAdapter, ROS1Adapter

# 加载环境变量
from dotenv import load_dotenv
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
dotenv_path = os.path.join(project_root, '.env')
load_dotenv(dotenv_path=dotenv_path)


# ==================== MCP Server ====================

class RobotControlServer:
    """机器人控制MCP服务器"""

    def __init__(self, adapter_type: str = "dora"):
        """
        初始化MCP服务器

        Args:
            adapter_type: 适配器类型（dora/ros1）
        """
        self.adapter_type = adapter_type
        self.server = Server("robot-control-server")
        self.robot_skills: Optional[RobotSkills] = None
        self.adapter = None

        # 初始化适配器
        self._init_adapter()

        # 注册所有tools
        self._register_tools()

    def _init_adapter(self):
        """初始化通信适配器"""
        if self.adapter_type == "dora":
            self.adapter = DoraAdapter()
            if not self.adapter.is_available():
                print("[MCP Server] WARNING: Dora adapter not available, running in standalone mode")
        elif self.adapter_type == "ros1":
            self.adapter = ROS1Adapter()
            if not self.adapter.is_available():
                print("[MCP Server] WARNING: ROS1 adapter not available, running in standalone mode")
        else:
            print(f"[MCP Server] Unknown adapter type: {self.adapter_type}")
            return

        # 初始化robot skills
        self.robot_skills = RobotSkills(self.adapter)
        print(f"[MCP Server] Initialized with {self.adapter_type} adapter")

    def _register_tools(self):
        """注册所有MCP tools"""

        @self.server.list_tools()
        async def handle_list_tools() -> list[Tool]:
            """列出所有可用的tools"""
            return [
                Tool(
                    name="turn_left",
                    description="向左转指定角度",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "angle": {
                                "type": "number",
                                "description": "转向角度（度），默认90度",
                                "default": 90
                            }
                        }
                    }
                ),
                Tool(
                    name="turn_right",
                    description="向右转指定角度",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "angle": {
                                "type": "number",
                                "description": "转向角度（度），默认90度",
                                "default": 90
                            }
                        }
                    }
                ),
                Tool(
                    name="move_forward",
                    description="向前移动指定距离",
                    inputSchema={
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
                ),
                Tool(
                    name="move_backward",
                    description="向后移动指定距离",
                    inputSchema={
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
                ),
                Tool(
                    name="move_left",
                    description="向左移动指定距离",
                    inputSchema={
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
                ),
                Tool(
                    name="move_right",
                    description="向右移动指定距离",
                    inputSchema={
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
                ),
                Tool(
                    name="navigate_to",
                    description="导航到指定位置",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "location": {
                                "type": "string",
                                "description": "目标位置名称（如kitchen, table）"
                            },
                            "direction": {
                                "type": "string",
                                "description": "相对方向（front/back/left/right）"
                            },
                            "distance": {
                                "type": "string",
                                "description": "距离（如30cm, 1.5m）"
                            }
                        },
                        "required": ["location"]
                    }
                ),
                Tool(
                    name="pick_up",
                    description="抓取指定物体",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "object_name": {
                                "type": "string",
                                "description": "要抓取的物体名称"
                            }
                        },
                        "required": ["object_name"]
                    }
                ),
                Tool(
                    name="place",
                    description="放置物体到指定位置",
                    inputSchema={
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
                ),
                Tool(
                    name="turn_then_move",
                    description="转向然后移动的复合动作",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "turn_angle": {
                                "type": "number",
                                "description": "转向角度（度）"
                            },
                            "move_distance": {
                                "type": "number",
                                "description": "移动距离数值"
                            },
                            "move_unit": {
                                "type": "string",
                                "description": "移动距离单位",
                                "enum": ["m", "cm", "mm"],
                                "default": "m"
                            },
                            "turn_direction": {
                                "type": "string",
                                "description": "转向方向",
                                "enum": ["left", "right"],
                                "default": "left"
                            }
                        },
                        "required": ["turn_angle", "move_distance"]
                    }
                ),
                Tool(
                    name="move_square",
                    description="沿正方形路径移动（用于测试）",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "side_length": {
                                "type": "number",
                                "description": "正方形边长"
                            },
                            "unit": {
                                "type": "string",
                                "description": "距离单位",
                                "enum": ["m", "cm", "mm"],
                                "default": "m"
                            }
                        },
                        "required": ["side_length"]
                    }
                ),
                Tool(
                    name="stop",
                    description="停止机器人",
                    inputSchema={
                        "type": "object",
                        "properties": {}
                    }
                ),
                Tool(
                    name="get_status",
                    description="获取机器人当前状态",
                    inputSchema={
                        "type": "object",
                        "properties": {}
                    }
                ),
                Tool(
                    name="wait",
                    description="等待指定时间",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "seconds": {
                                "type": "number",
                                "description": "等待秒数",
                                "default": 1.0
                            }
                        }
                    }
                ),
            ]

        @self.server.call_tool()
        async def handle_call_tool(name: str, arguments: dict | None) -> list[TextContent | ImageContent | EmbeddedResource]:
            """处理tool调用"""
            if not self.robot_skills:
                return [TextContent(
                    type="text",
                    text="Robot skills not initialized"
                )]

            if arguments is None:
                arguments = {}

            try:
                # 根据tool名称调用对应的skill函数
                if name == "turn_left":
                    result = self.robot_skills.turn_left(**arguments)
                elif name == "turn_right":
                    result = self.robot_skills.turn_right(**arguments)
                elif name == "move_forward":
                    result = self.robot_skills.move_forward(**arguments)
                elif name == "move_backward":
                    result = self.robot_skills.move_backward(**arguments)
                elif name == "move_left":
                    result = self.robot_skills.move_left(**arguments)
                elif name == "move_right":
                    result = self.robot_skills.move_right(**arguments)
                elif name == "navigate_to":
                    result = self.robot_skills.navigate_to(**arguments)
                elif name == "pick_up":
                    # 参数名转换
                    obj_name = arguments.get("object_name")
                    result = self.robot_skills.pick_up(obj_name)
                elif name == "place":
                    result = self.robot_skills.place(
                        arguments.get("object_name"),
                        arguments.get("location")
                    )
                elif name == "turn_then_move":
                    result = self.robot_skills.turn_then_move(**arguments)
                elif name == "move_square":
                    result = self.robot_skills.move_square(**arguments)
                elif name == "stop":
                    result = self.robot_skills.stop()
                elif name == "get_status":
                    result = self.robot_skills.get_status()
                elif name == "wait":
                    result = self.robot_skills.wait(**arguments)
                else:
                    result = {
                        "success": False,
                        "error": f"Unknown tool: {name}"
                    }

                # 返回结果
                return [TextContent(
                    type="text",
                    text=json.dumps(result, ensure_ascii=False, indent=2)
                )]

            except Exception as e:
                return [TextContent(
                    type="text",
                    text=json.dumps({
                        "success": False,
                        "error": str(e),
                        "tool": name,
                        "arguments": arguments
                    }, ensure_ascii=False, indent=2)
                )]

    async def run(self):
        """运行MCP服务器"""
        print("[MCP Server] Starting robot control MCP server...")
        print(f"[MCP Server] Using adapter: {self.adapter_type}")

        # 运行服务器
        async with self.server.run() as server:
            await server.communicate()


# ==================== Main Entry ====================

async def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description="Robot Control MCP Server")
    parser.add_argument(
        "--adapter",
        type=str,
        default="dora",
        choices=["dora", "ros1"],
        help="Communication adapter (default: dora)"
    )

    args = parser.parse_args()

    # 创建并运行服务器
    server = RobotControlServer(adapter_type=args.adapter)
    await server.run()


if __name__ == "__main__":
    asyncio.run(main())
