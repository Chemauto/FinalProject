"""
示例模块 (Example Module)

这是一个模板文件，展示如何创建新的功能模块。

添加新功能的步骤：
1. 复制本文件并重命名（如 camera.py, sensor.py 等）
2. 修改模块文档字符串
3. 在 register_tools(mcp) 中添加 @mcp.tool() 装饰的工具函数
4. 在 skill.py 中导入并注册
"""


# ==============================================================================
# MCP 注册函数
# ==============================================================================

def register_tools(mcp):
    """
    注册示例模块的工具函数到 MCP 服务器

    Args:
        mcp: FastMCP 服务器实例
    """

    @mcp.tool()
    async def example_tool(param1: str, param2: float = 10.0) -> str:
        """示例工具函数

        这是一个模板函数，展示如何定义新的工具函数。
        复制此文件并修改为你需要的功能。

        Args:
            param1: 第一个参数（字符串）
            param2: 第二个参数（数字，可选）

        Returns:
            操作结果JSON字符串

        Examples:
            example_tool(param1="test", param2=5.0)
        """
        import sys
        import json
        print(f"[example.example_tool] 执行: param1={param1}, param2={param2}", file=sys.stderr)

        result = {
            'status': 'success',
            'message': f'执行完成: {param1}, {param2}',
            'parameters': {'param1': param1, 'param2': param2}
        }

        return json.dumps(result, ensure_ascii=False)

    print("[example.py:register_tools] 示例模块已注册 (1 个工具)", file=sys.stderr)
