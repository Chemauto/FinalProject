"""
示例模块 (Example Module)

这是一个模板文件，展示如何创建新的功能模块。

添加新功能的步骤：
1. 复制本文件并重命名（如 camera.py, sensor.py 等）
2. 修改模块文档字符串
3. 实现具体的工具函数（带完整 docstring）
4. 在 skill.py 中导入并注册

Functions:
    - example_tool: 示例工具函数（参考模板）
"""

import sys
import json
import inspect


# =============================================================================
# 工具函数实现
# ==============================================================================

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
    print(f"[example.example_tool] 执行: param1={param1}, param2={param2}", file=sys.stderr)

    result = {
        'status': 'success',
        'message': f'执行完成: {param1}, {param2}',
        'parameters': {'param1': param1, 'param2': param2}
    }

    return json.dumps(result, ensure_ascii=False)


# =============================================================================
# MCP 注册函数
# ==============================================================================

def _extract_tool_metadata(func):
    """从函数提取工具元数据（OpenAI function calling 格式）"""
    name = func.__name__
    doc = inspect.getdoc(func) or ""

    # 解析函数签名
    sig = inspect.signature(func)
    parameters = {}
    required = []

    for param_name, param in sig.parameters.items():
        param_type = "string"
        if param.annotation != inspect.Parameter.empty:
            if param.annotation == int:
                param_type = "number"
            elif param.annotation == float:
                param_type = "number"
            elif param.annotation == bool:
                param_type = "boolean"

        param_info = {"type": param_type}

        # 从 docstring 中提取参数描述
        if "Args:" in doc:
            for line in doc.split("Args:")[1].split("\n"):
                if f"{param_name}:" in line:
                    parts = line.split(":", 1)
                    if len(parts) == 2:
                        param_desc = parts[1].strip()
                        if param_desc:
                            param_info["description"] = param_desc
                    break

        parameters[param_name] = param_info

        # 检查是否是必需参数
        if param.default == inspect.Parameter.empty:
            required.append(param_name)

    # 构建工具元数据
    metadata = {
        "description": doc.split("\n\n")[0].split("\n")[0] if doc else "",
        "parameters": {
            "type": "object",
            "properties": parameters,
            "required": required
        }
    }

    return name, metadata


def register_tools(mcp, tool_registry=None, tool_metadata=None):
    """
    注册示例模块的工具函数到 MCP 服务器

    这是一个模板函数，展示如何注册工具。

    使用方法：
        1. 编写异步函数（带完整 docstring）
        2. 在此函数中添加：mcp.tool()(your_function)

    Args:
        mcp: FastMCP 服务器实例
        tool_registry: 工具函数注册表（可选）
        tool_metadata: 工具元数据注册表（可选）
    """
    # 要注册的工具函数列表
    tools = [example_tool]

    for func in tools:
        # 注册到 FastMCP
        mcp.tool()(func)

        # 提取并存储元数据（用于 LLM function calling）
        if tool_registry is not None and tool_metadata is not None:
            name, metadata = _extract_tool_metadata(func)
            tool_registry[name] = func
            tool_metadata[name] = metadata

    print(f"[example.py:register_tools] 示例模块已注册 ({len(tools)} 个工具)", file=sys.stderr)
