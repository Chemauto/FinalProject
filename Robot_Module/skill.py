"""
FinalProject 机器人技能统一入口 (Robot Skills Entry Point)

基于 RoboOS MCP 架构设计，负责：
1. 初始化 FastMCP 服务器
2. 自动注册工具函数和元数据
3. 管理与仿真器的通信队列

模块列表：
    - base.py: 底盘控制模块（移动、旋转、停止）

添加新功能：
    1. 在模块中编写异步函数（带完整 docstring）
    2. 在 register_tools() 中添加：mcp.tool()(your_function)
"""

import sys
import os
import signal
import atexit
import json
from pathlib import Path
from mcp.server.fastmcp import FastMCP

# 添加当前目录到 Python 路径
_current_dir = Path(__file__).parent
sys.path.insert(0, str(_current_dir))

# ==============================================================================
# 1. 全局变量和初始化（必须在导入模块之前）
# ==============================================================================

# 初始化 FastMCP 服务器
mcp = FastMCP("robot")

# 工具注册表
_tool_registry = {}
_tool_metadata = {}

# ==============================================================================
# 2. 导入各功能模块（此时 mcp 已可用）
# ==============================================================================
from module.base import register_tools as register_base_tools
from module.vision import register_tools as register_vision_tools
from module.example import register_tools as register_example_tools

# ==============================================================================
# 3. 工具注册和元数据提取
# ==============================================================================

def get_skill_function(name: str):
    """获取工具函数"""
    return _tool_registry.get(name)


def get_tool_definitions():
    """获取工具定义（OpenAI function calling 格式）"""
    import asyncio

    # 从 mcp 获取已注册的工具（异步）
    async def _get_tools():
        tools_list = await mcp.list_tools()
        tools = []
        for tool in tools_list:
            tool_def = {
                "type": "function",
                "function": {
                    "name": tool.name,
                    "description": tool.description if hasattr(tool, 'description') else "",
                    "parameters": tool.inputSchema if hasattr(tool, 'inputSchema') else {
                        "type": "object",
                        "properties": {},
                        "required": []
                    }
                }
            }
            tools.append(tool_def)
        return tools

    return asyncio.run(_get_tools())


# ==============================================================================
# 4. 信号处理
# ==============================================================================

def signal_handler(signum, frame):
    """信号处理器 - 处理 Ctrl+C 等信号"""
    print(f"\n[skill.py] 收到信号 {signum}，正在清理资源...", file=sys.stderr)
    sys.exit(0)


atexit.register(lambda: print("[skill.py] 清理资源完成", file=sys.stderr))
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


# ==============================================================================
# 5. 注册所有模块
# ==============================================================================

def register_all_modules():
    """注册所有功能模块到 MCP 服务器"""
    print("=" * 60, file=sys.stderr)
    print("[skill.py] 开始注册机器人技能模块...", file=sys.stderr)
    print("=" * 60, file=sys.stderr)

    # 注册底盘控制模块（返回工具函数映射）
    base_tools = register_base_tools(mcp)
    _tool_registry.update(base_tools)

    # 注册视觉感知模块（返回工具函数映射）
    vision_tools = register_vision_tools(mcp)
    _tool_registry.update(vision_tools)

    # 示例模块
    # example_tools = register_example_tools(mcp)
    # _tool_registry.update(example_tools)

    print("=" * 60, file=sys.stderr)
    print("[skill.py] ✓ 所有模块注册完成", file=sys.stderr)
    print("=" * 60, file=sys.stderr)


# ==============================================================================
# 6. 主函数
# ==============================================================================

if __name__ == "__main__":
    print("\n" + "=" * 60, file=sys.stderr)
    print("[skill.py] FinalProject 机器人技能服务器启动中...", file=sys.stderr)
    print("=" * 60, file=sys.stderr)
    print(f"[skill.py] 工作目录: {os.getcwd()}", file=sys.stderr)
    print(f"[skill.py] Python版本: {sys.version}", file=sys.stderr)
    print("=" * 60 + "\n", file=sys.stderr)

    register_all_modules()

    print("\n[skill.py] MCP 服务器准备就绪，等待工具调用...", file=sys.stderr)
    print("[skill.py] 按 Ctrl+C 停止服务器\n", file=sys.stderr)

    try:
        mcp.run(transport="stdio")
    except KeyboardInterrupt:
        print("\n[skill.py] 服务器已停止", file=sys.stderr)
    except Exception as e:
        print(f"\n[skill.py] ✗ 服务器错误: {e}", file=sys.stderr)
        sys.exit(1)
