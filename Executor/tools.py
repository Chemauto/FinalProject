from mcp.server.fastmcp import FastMCP
from Executor.skills import Nav, NavClimb, walk, Push, climb as climb_skill
from Vision import observe_environment

mcp = FastMCP("robot")
#创建MCP实例

@mcp.tool()
def nav(x: float, y: float, z: float) -> dict:
    """导航到目标坐标"""
    return Nav(x, y, z)
#注册导航工具

@mcp.tool()
def nav_climb(x: float, y: float, z: float) -> dict:
    """导航到目标坐标并攀爬无法绕开的可通过障碍物"""
    return NavClimb(x, y, z)
#注册导航攀爬工具

@mcp.tool()
def push(x: float, y: float, z: float) -> dict:
    """把箱子推到目标坐标"""
    return Push(x, y, z)
#注册推箱子工具

@mcp.tool()
def climb(height: float) -> dict:
    """攀爬指定高度，最高0.3m"""
    return climb_skill(height)
#注册攀爬工具

@mcp.tool()
def walk_skill(direction: str, v: float = 0.5, distance: float = 0.0) -> dict:
    """按方向、速度和目标距离移动，可前后行走或左右侧移避让单侧障碍"""
    return walk(direction, v, distance)
#注册行走工具

@mcp.tool()
def observe(image_path: str = "") -> dict:
    """观察当前环境，返回结构化视觉事实"""
    return observe_environment(image_path)
#注册视觉观察工具

def get_tool_definitions():
    return [
        {
            "type": "function",
            "function": {
                "name": "observe",
                "description": "观察当前环境，返回结构化视觉事实",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "image_path": {"type": "string"},
                    },
                },
            },
        },
        {
            "type": "function",
            "function": {
                "name": "nav",
                "description": "导航到目标坐标；路径可通行或可通过普通绕行时使用",
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
                "name": "nav_climb",
                "description": "导航到目标坐标并攀爬无法绕开的可通过障碍物；只有左右都受阻、普通导航或侧移绕行不可行，或用户明确要求攀爬时使用",
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
                "description": "按方向、速度和目标距离移动，direction可选front/back/left/right，v是速度且默认0.5，distance是目标距离且单位米；可用left/right侧移避让单侧障碍物",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "direction": {"type": "string"},
                        "v": {"type": "number", "description": "速度，默认0.5", "default": 0.5},
                        "distance": {"type": "number", "description": "目标距离，单位米"},
                    },
                    "required": ["direction", "distance"],
                },
            },
        },
    ]
#返回工具定义给OpenAI API

def call_tool(name, args, emit=None):
    TOOLS = {
        "observe": observe_environment,
        "nav": Nav,
        "nav_climb": NavClimb,
        "walk_skill": walk,
        "push": Push,
        "climb": climb_skill,
    }
    if name not in TOOLS:
        return f"未知工具: {name}"
    return TOOLS[name](**args, emit=emit)
#根据工具名调用对应技能，emit透传给技能函数
