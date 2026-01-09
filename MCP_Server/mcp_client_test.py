#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MCP Client for ROS2 Robot Control
一个简单的 MCP 客户端，用于连接和测试 ROS2 Robot Control Server
"""
import asyncio
import json
from mcp.client.session import ClientSession
from mcp.client.stdio import stdio_client


class Ros2McpClient:
    """ROS2 MCP 客户端"""

    def __init__(self, server_script_path, adapter="ros2"):
        """
        初始化 MCP 客户端

        Args:
            server_script_path: MCP 服务器脚本路径
            adapter: 适配器类型 (ros1/ros2/dora)
        """
        self.server_script_path = server_script_path
        self.adapter = adapter
        self.session = None

    async def connect(self):
        """连接到 MCP 服务器"""
        print(f"[Client] 正在连接到 MCP 服务器...")

        # 使用 stdio 连接到服务器
        server_params = {
            "command": "python3",
            "args": [
                self.server_script_path,
                "--adapter",
                self.adapter
            ],
            "env": {
                "ROS_DISTRO": "humble",
                "PATH": "/opt/ros/humble/bin:" + asyncio.subprocess.os.environ.get("PATH", ""),
            }
        }

        stdio_transport = await stdio_client(server_params)
        self.stdio, self.write = stdio_transport

        # 创建会话
        self.session = ClientSession(self.stdio, self.write)

        # 初始化
        await self.session.initialize()

        print(f"[Client] ✓ 已连接到服务器")
        print(f"[Client] ✓ 服务器: {self.server_script_path}")
        print(f"[Client] ✓ 适配器: {self.adapter}\n")

    async def list_tools(self):
        """列出所有可用的工具"""
        print("\n" + "="*60)
        print("可用的机器人控制工具:")
        print("="*60)

        tools = await self.session.list_tools()

        for i, tool in enumerate(tools.tools, 1):
            print(f"\n{i}. {tool.name}")
            print(f"   描述: {tool.description}")
            if tool.inputSchema:
                print(f"   参数: {json.dumps(tool.inputSchema, ensure_ascii=False, indent=6)}")

        print("\n" + "="*60)

        return tools.tools

    async def call_tool(self, tool_name, arguments=None):
        """
        调用工具

        Args:
            tool_name: 工具名称
            arguments: 参数字典
        """
        if arguments is None:
            arguments = {}

        print(f"\n[Client] 调用工具: {tool_name}")
        print(f"[Client] 参数: {json.dumps(arguments, ensure_ascii=False, indent=2)}")

        try:
            result = await self.session.call_tool(tool_name, arguments)

            print(f"\n[Client] ✓ 执行成功")
            print(f"[Client] 结果:")

            for content in result.content:
                if hasattr(content, 'text'):
                    print(f"  {content.text}")
                else:
                    print(f"  {content}")

            return result

        except Exception as e:
            print(f"\n[Client] ✗ 执行失败: {e}")
            return None

    async def run_interactive(self):
        """交互式运行"""
        # 连接
        await self.connect()

        # 列出工具
        tools = await self.list_tools()

        # 创建工具名称到索引的映射
        tool_map = {str(i): tool.name for i, tool in enumerate(tools, 1)}
        tool_map.update({tool.name: tool.name for tool in tools})

        print("\n交互式命令模式")
        print("输入工具名称或编号来调用，输入 'q' 退出\n")

        while True:
            try:
                choice = input("请选择工具 (1-{} 或 q): ".format(len(tools))).strip()

                if choice.lower() == 'q':
                    print("\n[Client] 退出")
                    break

                if choice not in tool_map:
                    print(f"❌ 无效选择: {choice}")
                    continue

                tool_name = tool_map[choice]

                # 获取工具的参数定义
                tool = next((t for t in tools if t.name == tool_name), None)
                if not tool:
                    continue

                # 收集参数
                arguments = {}
                if tool.inputSchema and 'properties' in tool.inputSchema:
                    print(f"\n需要参数:")
                    for prop_name, prop_info in tool.inputSchema['properties'].items():
                        required = prop_name in tool.inputSchema.get('required', [])
                        req_text = " (必需)" if required else " (可选)"
                        default = prop_info.get('default', '')

                        prompt = f"  {prop_name}{req_text}"
                        if default:
                            prompt += f" [默认: {default}]"

                        value = input(f"{prompt}: ").strip()

                        if value:
                            # 尝试转换类型
                            prop_type = prop_info.get('type', 'string')
                            if prop_type == 'number':
                                value = float(value)
                            elif prop_type == 'integer':
                                value = int(value)

                            arguments[prop_name] = value
                        elif required:
                            print(f"  ❌ 缺少必需参数: {prop_name}")
                            break
                else:
                    print("  (无需参数)")

                # 调用工具
                if arguments or (tool.inputSchema and not tool.inputSchema.get('required')):
                    await self.call_tool(tool_name, arguments)

            except KeyboardInterrupt:
                print("\n\n[Client] 被用户中断")
                break
            except Exception as e:
                print(f"\n❌ 错误: {e}")

    async def run_test_commands(self):
        """运行测试命令序列"""
        # 连接
        await self.connect()

        # 列出工具
        await self.list_tools()

        print("\n运行测试命令序列...\n")

        # 测试命令
        test_commands = [
            {
                "tool": "move_forward",
                "args": {"distance": 1.0, "unit": "m"}
            },
            {
                "tool": "turn_left",
                "args": {"angle": 90}
            },
            {
                "tool": "move_forward",
                "args": {"distance": 0.5, "unit": "m"}
            },
            {
                "tool": "stop",
                "args": {}
            }
        ]

        for i, cmd in enumerate(test_commands, 1):
            print(f"\n{'='*60}")
            print(f"测试命令 {i}/{len(test_commands)}")
            print(f"{'='*60}")

            await self.call_tool(cmd["tool"], cmd["args"])

            # 等待执行完成
            await asyncio.sleep(2)

        print(f"\n{'='*60}")
        print("测试完成！")
        print(f"{'='*60}\n")

    async def close(self):
        """关闭连接"""
        if self.session:
            await self.session.close()


async def main():
    """主函数"""
    import sys

    server_script = "/home/robot/work/FinalProject/MCP_Server/mcp_robot_server.py"

    print("="*60)
    print("ROS2 MCP 客户端")
    print("="*60)

    client = Ros2McpClient(server_script, adapter="ros2")

    try:
        # 检查命令行参数
        if len(sys.argv) > 1 and sys.argv[1] == "--test":
            # 运行测试序列
            await client.run_test_commands()
        else:
            # 交互式模式
            await client.run_interactive()

    except KeyboardInterrupt:
        print("\n\n[Client] 被用户中断")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        await client.close()


if __name__ == "__main__":
    asyncio.run(main())
