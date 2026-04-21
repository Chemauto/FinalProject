from mcp.server.fastmcp import FastMCP
from Executor.skills import Nav, walk, Push, climb as climb_skill

mcp = FastMCP("robot")
#创建MCP实例

@mcp.tool()
def nav(x: float, y: float, z: float) -> str:
    """导航到目标坐标"""
    return Nav(x, y, z)

@mcp.tool()
def push(x: float, y: float, z: float) -> str:
    """把箱子推到目标坐标"""
    return Push(x, y, z)

@mcp.tool()
def climb(height: float) -> str:
    """攀爬指定高度，最高0.3m"""
    return climb_skill(height)

@mcp.tool()
def walk_skill(direction: str, v: float) -> str:
    """按方向和速度移动，direction可选front/back/left/right"""
    return walk(direction, v)
#注册4个技能到MCP，climb重命名为climb_skill避免和装饰器函数同名

def get_tool_definitions():
    return [
        {
            "type": "function",
            "function": {
                "name": "nav",
                "description": "导航到目标坐标",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "x": {"type": "number"},
                        "y": {"type": "number"},
                        "z": {"type": "number"},
                    },
                    "required": ["x", "y", "z"],
                },
            },
        },
        {
            "type": "function",
            "function": {
                "name": "push",
                "description": "把箱子推到目标坐标",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "x": {"type": "number"},
                        "y": {"type": "number"},
                        "z": {"type": "number"},
                    },
                    "required": ["x", "y", "z"],
                },
            },
        },
        {
            "type": "function",
            "function": {
                "name": "climb",
                "description": "攀爬指定高度，最高0.3m",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "height": {"type": "number"},
                    },
                    "required": ["height"],
                },
            },
        },
        {
            "type": "function",
            "function": {
                "name": "walk_skill",
                "description": "按方向和速度移动，direction可选front/back/left/right",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "direction": {"type": "string"},
                        "v": {"type": "number"},
                    },
                    "required": ["direction", "v"],
                },
            },
        },
    ]
#返回工具定义给OpenAI API

def call_tool(name, args, emit=None):
    TOOLS = {
        "nav": Nav,
        "walk_skill": walk,
        "push": Push,
        "climb": climb_skill,
    }
    if name not in TOOLS:
        return f"未知工具: {name}"
    return TOOLS[name](**args, emit=emit)
#根据工具名调用对应技能，emit透传给技能函数
